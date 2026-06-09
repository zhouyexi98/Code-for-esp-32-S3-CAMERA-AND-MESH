#pragma once

#include "hm01b0.h"
#include <driver/gpio.h>

// Pin mapping provided by user (change here if needed)
// D1-IO10,D3-IO11,VSYNC-IO12,TRIG-IO13,MCLK-IO18,
// D2-IO14, D4-IO15, D7-IO21,D0-IO17,HSYNC-IO16, PCLK-IO33,
// INT-IO35,D6-IO36, D5-IO34,SCL-IO38,SDA-IO37
#ifndef HM01B0_D1_PIN
#define HM01B0_D1_PIN ((gpio_num_t)10)
#endif
#ifndef HM01B0_D3_PIN
#define HM01B0_D3_PIN ((gpio_num_t)11)
#endif
#ifndef HM01B0_VSYNC_PIN
#define HM01B0_VSYNC_PIN ((gpio_num_t)12)
#endif
#ifndef HM01B0_TRIG_PIN
#define HM01B0_TRIG_PIN ((gpio_num_t)13)
#endif
#ifndef HM01B0_MCLK_PIN
#define HM01B0_MCLK_PIN ((gpio_num_t)18)
#endif
#ifndef HM01B0_D2_PIN
#define HM01B0_D2_PIN ((gpio_num_t)14)
#endif
#ifndef HM01B0_D4_PIN
#define HM01B0_D4_PIN ((gpio_num_t)15)
#endif
#ifndef HM01B0_D7_PIN
#define HM01B0_D7_PIN ((gpio_num_t)21)
#endif
#ifndef HM01B0_D0_PIN
#define HM01B0_D0_PIN ((gpio_num_t)17)
#endif
#ifndef HM01B0_HSYNC_PIN
#define HM01B0_HSYNC_PIN ((gpio_num_t)16)
#endif
#ifndef HM01B0_PCLK_PIN
#define HM01B0_PCLK_PIN ((gpio_num_t)33)
#endif
#ifndef HM01B0_INT_PIN
#define HM01B0_INT_PIN ((gpio_num_t)35)
#endif
#ifndef HM01B0_D6_PIN
#define HM01B0_D6_PIN ((gpio_num_t)36)
#endif
#ifndef HM01B0_D5_PIN
#define HM01B0_D5_PIN ((gpio_num_t)34)
#endif
#ifndef HM01B0_SCL_PIN
#define HM01B0_SCL_PIN ((gpio_num_t)38)
#endif
#ifndef HM01B0_SDA_PIN
#define HM01B0_SDA_PIN ((gpio_num_t)37)
#endif

// Data pins array in bit order (D0 LSB .. D7 MSB) used by read macros
#define HM01B0_DATA_PIN_D0 HM01B0_D0_PIN
#define HM01B0_DATA_PIN_D1 HM01B0_D1_PIN
#define HM01B0_DATA_PIN_D2 HM01B0_D2_PIN
#define HM01B0_DATA_PIN_D3 HM01B0_D3_PIN
#define HM01B0_DATA_PIN_D4 HM01B0_D4_PIN
#define HM01B0_DATA_PIN_D5 HM01B0_D5_PIN
#define HM01B0_DATA_PIN_D6 HM01B0_D6_PIN
#define HM01B0_DATA_PIN_D7 HM01B0_D7_PIN

// I2C port is handled by Wire (Arduino wrapper)

// If set to 1, the platform will temporarily disable MCLK while performing
// I2C register writes. Some modules require MCLK to remain active; leave as
// 0 to keep MCLK enabled during I2C operations.
#ifndef HM01B0_DISABLE_MCLK_DURING_I2C
#define HM01B0_DISABLE_MCLK_DURING_I2C 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

hm01b0_status_e hm01b0_esp32_init(hm01b0_cfg_t* cfg, void* arg);
hm01b0_status_e hm01b0_esp32_deinit(hm01b0_cfg_t* cfg, void* arg);
hm01b0_status_e hm01b0_esp32_write(hm01b0_cfg_t* cfg, uint16_t reg, uint8_t *data, uint32_t len, void* arg);
hm01b0_status_e hm01b0_esp32_read(hm01b0_cfg_t* cfg, uint16_t reg, uint8_t *data, uint32_t len, void* arg);
hm01b0_status_e hm01b0_esp32_mclk(hm01b0_cfg_t* cfg, bool enable, void* arg);
// Set MCLK frequency (Hz). Returns HM01B0_ERR_OK on success.
hm01b0_status_e hm01b0_esp32_set_mclk_freq(hm01b0_cfg_t* cfg, uint32_t freq_hz, void* arg);
hm01b0_status_e hm01b0_esp32_trig(hm01b0_cfg_t* cfg, bool enable, void* arg);

// Capture helpers
// Read one frame into buffer (buffer must be at least HM01B0_PIXEL_X_NUM * HM01B0_PIXEL_Y_NUM bytes)
hm01b0_status_e hm01b0_esp32_read_frame(hm01b0_cfg_t* cfg, uint8_t* buf, uint32_t buf_len, uint32_t timeout_ms);

// Read a single row into a buffer; used by frame reader. start_ms and timeout_ms
// are forwarded to avoid long busy-waits that trigger the watchdog.
hm01b0_status_e hm01b0_esp32_read_row(uint8_t* buf, uint32_t maxlen, uint32_t start_ms, uint32_t timeout_ms);

// Helper to populate the configuration with the ESP32 platform interface
void hm01b0_populate_cfg(hm01b0_cfg_t* cfg);

// Diagnostic: sample MCLK/VSYNC/HSYNC/PCLK for `ms` milliseconds and log counts
void hm01b0_esp32_dump_signals(uint32_t ms);

#ifdef __cplusplus
}
#endif

