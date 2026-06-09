#pragma once

#include "hm01b0.h"

// ESP32 camera-driver based adapter. Requires esp_camera component available
// in the build (ESP-IDF camera component / Arduino-ESP32 camera). This adapter
// will initialize the camera peripheral and provide a capture function that
// fills a raw buffer (grayscale).

hm01b0_status_e hm01b0_camera_init(hm01b0_cfg_t* cfg, void* arg);
hm01b0_status_e hm01b0_camera_deinit(hm01b0_cfg_t* cfg, void* arg);
hm01b0_status_e hm01b0_camera_capture_frame(hm01b0_cfg_t* cfg, uint8_t* buf, uint32_t buf_len, uint32_t* out_len, uint32_t timeout_ms);
