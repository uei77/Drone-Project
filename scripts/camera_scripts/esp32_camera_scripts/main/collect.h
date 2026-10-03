#pragma once
/*
 * Data-collection build: streams every 320x240 gray frame + raw telemetry over Wi-Fi UDP
 * to the laptop recorder (training/recorder.py). Enable with COLLECT_MODE 1 in config.h.
 * Never fly a collect build as the vision build: Wi-Fi costs CPU and RAM.
 *
 * Packet (little endian), one frame = n_chunks packets of COLLECT_SLICE image bytes:
 *   u32 magic 'DLV2' | u32 frame_id | u8 chunk | u8 n_chunks | u16 payload_len |
 *   u16 width | u16 height | u32 esp_ms | u32 tel_age_ms | telemetry_t (raw) | payload
 */
#include <stdint.h>

#define COLLECT_MAGIC   0x32564C44u   /* "DLV2" */
#define COLLECT_SLICE   1200          /* image bytes per packet (fits one Wi-Fi frame) */
#define COLLECT_PORT    5005

void collect_run(void);   /* never returns */
