#pragma once
/*
 * Board pin map. The build target picks the board:
 *   idf.py set-target esp32    -> AI-Thinker ESP32-CAM
 *   idf.py set-target esp32s3  -> Freenove ESP32-S3-WROOM CAM (same camera pins as ESP32-S3-EYE)
 * Another S3 camera board: change only the ESP32-S3 block below.
 * The S3 pins are from the board's published pin map and have NOT been tried on hardware yet.
 */
#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32            /* AI-Thinker ESP32-CAM */
#define BOARD_NAME        "AI-Thinker ESP32-CAM"
#define CAM_PIN_PWDN      32
#define CAM_PIN_RESET     -1
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD      26
#define CAM_PIN_SIOC      27
#define CAM_PIN_D7        35
#define CAM_PIN_D6        34
#define CAM_PIN_D5        39
#define CAM_PIN_D4        36
#define CAM_PIN_D3        21
#define CAM_PIN_D2        19
#define CAM_PIN_D1        18
#define CAM_PIN_D0        5
#define CAM_PIN_VSYNC     25
#define CAM_PIN_HREF      23
#define CAM_PIN_PCLK      22
#define FC_TX_PIN         14
#define FC_RX_PIN         15          /* strapping pin: FC TX line must idle high at boot */

#elif CONFIG_IDF_TARGET_ESP32S3        /* Freenove ESP32-S3-WROOM CAM / ESP32-S3-EYE pin map */
#define BOARD_NAME        "ESP32-S3 CAM (Freenove / S3-EYE pin map)"
#define CAM_PIN_PWDN      -1
#define CAM_PIN_RESET     -1
#define CAM_PIN_XCLK      15
#define CAM_PIN_SIOD      4
#define CAM_PIN_SIOC      5
#define CAM_PIN_D7        16
#define CAM_PIN_D6        17
#define CAM_PIN_D5        18
#define CAM_PIN_D4        12
#define CAM_PIN_D3        10
#define CAM_PIN_D2        8
#define CAM_PIN_D1        9
#define CAM_PIN_D0        11
#define CAM_PIN_VSYNC     6
#define CAM_PIN_HREF      7
#define CAM_PIN_PCLK      13
#define FC_TX_PIN         47          /* free header pins on the Freenove board; check your board */
#define FC_RX_PIN         21

#else
#error "Unsupported target: add a pin map to board.h"
#endif
