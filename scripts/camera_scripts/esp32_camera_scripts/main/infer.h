#pragma once
/*
 * Situation Selector step model wrapper (TFLite Micro). C interface.
 * Stateless: the caller owns the recurrent state h and the previous image, so the same
 * model can serve many tiles, each with its own memory.
 */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "model_meta.h"
#include "proto.h"

#define MODEL_IN 96   /* model input is MODEL_IN x MODEL_IN pixels */

typedef struct {
    float prob[MODEL_NUM_CLASSES];  /* calibrated: softmax(logits / T) */
    float pos[2];                   /* target offset inside the window, [-1, 1] */
    int top;                        /* argmax class */
    uint32_t invoke_us;
} infer_out_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t infer_init(void);
/* cur / prev: top-left pixel of a MODEL_IN x MODEL_IN window inside 8-bit gray images whose
 * rows are `stride` bytes apart (stride = image width). prev may be NULL: cur is used twice.
 * thermal: MODEL_IN x MODEL_IN bytes for the third channel (scan_thermal_tile()), or NULL when
 * there is no thermal frame; ignored by a model with two image channels.
 * tel: telemetry_features() output. h: MODEL_STATE_DIM values, read and updated in place. */
esp_err_t infer_run(const uint8_t *cur, const uint8_t *prev, int stride, const uint8_t *thermal,
                    const float tel[TEL_FEATURES], int8_t *h, infer_out_t *out);
/* true if the model has a thermal image channel */
bool infer_uses_thermal(void);
/* h = "no memory" (the int8 code of 0.0); call for a new tile, a new mission, a long gap */
void infer_h_reset(int8_t *h);
size_t infer_arena_used(void);

#ifdef __cplusplus
}
#endif
