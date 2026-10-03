#include "infer.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "infer";

#ifndef INFER_ARENA_BYTES
#define INFER_ARENA_BYTES (80 * 1024)    /* host TFLM measured 53,632 B used (64-bit); ESP-NN adds scratch */
#endif

/* internal SRAM: PSRAM is several times slower for activations */
alignas(16) static uint8_t s_arena[INFER_ARENA_BYTES];
static tflite::MicroInterpreter *s_interp;
static TfLiteTensor *s_in_img, *s_in_tel, *s_in_h;
static TfLiteTensor *s_out_logits, *s_out_pos, *s_out_h;
static bool s_img_fast;   /* input scale 1, zero point -128 -> int8 = pixel ^ 0x80 */
static int s_channels;    /* 2 = current, previous; 3 = + thermal */

static int8_t q8(float v, const TfLiteTensor *t)
{
    long q = lroundf(v / t->params.scale) + t->params.zero_point;
    return (int8_t)(q < -128 ? -128 : q > 127 ? 127 : q);
}

static float dq8(int8_t v, const TfLiteTensor *t)
{
    return (v - t->params.zero_point) * t->params.scale;
}

static const char *in_name(const tflite::Model *m, int i)
{
    const auto *sg = m->subgraphs()->Get(0);
    const auto *tn = sg->tensors()->Get(sg->inputs()->Get(i));
    return tn->name() ? tn->name()->c_str() : "";
}

void infer_h_reset(int8_t *h)
{
    memset(h, s_in_h ? q8(0.f, s_in_h) : 0, MODEL_STATE_DIM);
}

bool infer_uses_thermal(void) { return s_channels == 3; }

size_t infer_arena_used(void) { return s_interp ? s_interp->arena_used_bytes() : 0; }

esp_err_t infer_init(void)
{
    const tflite::Model *model = tflite::GetModel(g_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "schema %lu != %d", (unsigned long)model->version(), TFLITE_SCHEMA_VERSION);
        return ESP_FAIL;
    }
    /* op list printed by training/export.py; keep in sync with model_meta.h */
    static tflite::MicroMutableOpResolver<18> r;
    r.AddConv2D(); r.AddDepthwiseConv2D(); r.AddFullyConnected(); r.AddMean();
    r.AddReshape(); r.AddTranspose(); r.AddSoftmax(); r.AddConcatenation();
    r.AddAdd(); r.AddSub(); r.AddMul(); r.AddLogistic(); r.AddTanh();
    r.AddQuantize(); r.AddDequantize(); r.AddPack(); r.AddMinimum(); r.AddMaximum();

    static tflite::MicroInterpreter interp(model, r, s_arena, sizeof s_arena);
    s_interp = &interp;
    if (interp.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed (arena %u B too small or op missing)",
                 (unsigned)sizeof s_arena);
        return ESP_FAIL;
    }

    /* inputs by name, outputs by shape: converter output names are not stable */
    for (size_t i = 0; i < interp.inputs_size(); i++) {
        const char *n = in_name(model, i);
        TfLiteTensor *t = interp.input(i);
        if (strstr(n, "image")) s_in_img = t;
        else if (strstr(n, "telemetry")) s_in_tel = t;
        else if (strstr(n, "h_in")) s_in_h = t;
    }
    for (size_t i = 0; i < interp.outputs_size(); i++) {
        TfLiteTensor *t = interp.output(i);
        int d = t->dims->data[t->dims->size - 1];
        if (d == MODEL_NUM_CLASSES) s_out_logits = t;
        else if (d == 2) s_out_pos = t;
        else if (d == MODEL_STATE_DIM) s_out_h = t;
    }
    s_channels = s_in_img ? (int)(s_in_img->bytes / (MODEL_IN * MODEL_IN)) : 0;
    if (!s_in_img || !s_in_tel || !s_in_h || !s_out_logits || !s_out_pos || !s_out_h ||
        s_in_img->type != kTfLiteInt8 || (s_channels != 2 && s_channels != 3) ||
        s_in_img->bytes != (size_t)(MODEL_IN * MODEL_IN * s_channels) ||
        s_in_tel->bytes != TEL_FEATURES || s_in_h->bytes != MODEL_STATE_DIM) {
        ESP_LOGE(TAG, "model tensors do not match model_meta.h");
        return ESP_FAIL;
    }
    /* converter gives scale ~0.999999 for a 0..255 input; treat as 1 */
    s_img_fast = fabsf(s_in_img->params.scale - 1.0f) < 1e-3f && s_in_img->params.zero_point == -128;
    ESP_LOGI(TAG, "ready: arena %u / %u B, %d image channels, img fast path %d, h in %.5f/%ld out %.5f/%ld",
             (unsigned)interp.arena_used_bytes(), (unsigned)sizeof s_arena, s_channels, s_img_fast,
             s_in_h->params.scale, (long)s_in_h->params.zero_point,
             s_out_h->params.scale, (long)s_out_h->params.zero_point);
    return ESP_OK;
}

esp_err_t infer_run(const uint8_t *cur, const uint8_t *prev, int stride, const uint8_t *thermal,
                    const float tel[TEL_FEATURES], int8_t *h, infer_out_t *out)
{
    if (!s_interp) return ESP_ERR_INVALID_STATE;
    if (!prev) prev = cur;                               /* no previous image: use cur twice */

    /* image: NHWC, channel 0 = current, 1 = previous, 2 = thermal (0 = no thermal frame) */
    int8_t *img = s_in_img->data.int8;
    const bool th = s_channels == 3;
    for (int y = 0; y < MODEL_IN; y++) {
        const uint8_t *c = cur + y * stride, *p = prev + y * stride;
        const uint8_t *t = thermal ? thermal + y * MODEL_IN : NULL;
        if (s_img_fast) {
            for (int x = 0; x < MODEL_IN; x++) {
                *img++ = (int8_t)(c[x] ^ 0x80);
                *img++ = (int8_t)(p[x] ^ 0x80);
                if (th) *img++ = (int8_t)((t ? t[x] : 0) ^ 0x80);
            }
        } else {
            for (int x = 0; x < MODEL_IN; x++) {
                *img++ = q8(c[x], s_in_img);
                *img++ = q8(p[x], s_in_img);
                if (th) *img++ = q8(t ? t[x] : 0, s_in_img);
            }
        }
    }
    for (int i = 0; i < TEL_FEATURES; i++) s_in_tel->data.int8[i] = q8(tel[i], s_in_tel);
    memcpy(s_in_h->data.int8, h, MODEL_STATE_DIM);

    int64_t t0 = esp_timer_get_time();
    if (s_interp->Invoke() != kTfLiteOk) return ESP_FAIL;
    out->invoke_us = (uint32_t)(esp_timer_get_time() - t0);

    /* h_out and h_in have different quantisation: requantise, never memcpy */
    const float k = s_out_h->params.scale / s_in_h->params.scale;
    for (int i = 0; i < MODEL_STATE_DIM; i++) {
        long v = lroundf((s_out_h->data.int8[i] - s_out_h->params.zero_point) * k) +
                 s_in_h->params.zero_point;
        h[i] = (int8_t)(v < -128 ? -128 : v > 127 ? 127 : v);
    }

    /* calibrated probabilities */
    float z[MODEL_NUM_CLASSES], mx = -1e30f, sum = 0.f;
    for (int i = 0; i < MODEL_NUM_CLASSES; i++) {
        z[i] = dq8(s_out_logits->data.int8[i], s_out_logits) / MODEL_TEMPERATURE;
        if (z[i] > mx) mx = z[i];
    }
    out->top = 0;
    for (int i = 0; i < MODEL_NUM_CLASSES; i++) { z[i] = expf(z[i] - mx); sum += z[i]; }
    for (int i = 0; i < MODEL_NUM_CLASSES; i++) {
        out->prob[i] = z[i] / sum;
        if (out->prob[i] > out->prob[out->top]) out->top = i;
    }
    out->pos[0] = dq8(s_out_pos->data.int8[0], s_out_pos);
    out->pos[1] = dq8(s_out_pos->data.int8[1], s_out_pos);
    return ESP_OK;
}
