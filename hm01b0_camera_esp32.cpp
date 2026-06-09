#include "hm01b0_camera_esp32.h"
#include "hm01b0.h"
#include "sdkconfig.h"

#if __has_include("esp_camera.h")
#include "esp_camera.h"

#include "hm01b0_esp32.h"
#include "esp_log.h"
#include "esp_err.h"
#include <cstring>
#include <driver/ledc.h>
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

static const char *TAG_CAM = "hm01b0_cam_esp32";

static bool s_camera_inited = false;

hm01b0_status_e hm01b0_camera_init(hm01b0_cfg_t* cfg, void* arg){
  (void)cfg; (void)arg;
  if(s_camera_inited) return HM01B0_ERR_OK;
  camera_config_t config = {};
  // User must ensure pins mapping fits HM01B0 connection; this is an example
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  // 8-bit Mode: Use all D0-D7 pins
  config.pin_d0 = HM01B0_D0_PIN;
  config.pin_d1 = HM01B0_D1_PIN;
  config.pin_d2 = HM01B0_D2_PIN;
  config.pin_d3 = HM01B0_D3_PIN;
  config.pin_d4 = HM01B0_D4_PIN;
  config.pin_d5 = HM01B0_D5_PIN;
  config.pin_d6 = HM01B0_D6_PIN;
  config.pin_d7 = HM01B0_D7_PIN;
  config.pin_xclk = HM01B0_MCLK_PIN;
  config.pin_pclk = HM01B0_PCLK_PIN;
  config.pin_vsync = HM01B0_VSYNC_PIN;
  config.pin_href = HM01B0_HSYNC_PIN;
  config.pin_sccb_sda = HM01B0_SDA_PIN;
  config.pin_sccb_scl = HM01B0_SCL_PIN;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  // prefer to use 4MHz XCLK for HM01B0 on this board (matches MCLK we drive)
  // Keep manual MCLK enabled during init to ensure probe succeeds
  // hm01b0_esp32_mclk(cfg, false, NULL);
  config.pin_xclk = HM01B0_MCLK_PIN;
  config.xclk_freq_hz = 4000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE; // prefer GRAYSCALE / RAW
  
  // Use QCIF (176x144) to allocate a larger buffer (25KB) than QQVGA (19KB).
  // The HM01B0 outputs 162x120 (19440 bytes) in its default state (Native 324x244 / 2).
  // QQVGA buffer (19200) is too small, causing overflow and image tearing.
  // QCIF buffer is large enough to hold the full 162x120 frame.
  config.frame_size = FRAMESIZE_QCIF; 
  
  // config.jpeg_quality = 10; // Not used for GRAYSCALE
  config.fb_count = 1; // Use single buffer to prevent sync issues during debug
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK) {
    ESP_LOGE(TAG_CAM, "esp_camera_init failed: %s", esp_err_to_name(err));
    return HM01B0_ERR_INIT;
  }
  
  // Force MCLK back on via LEDC, in case esp_camera_init disabled it or configured it for LCD_CAM but didn't start it.
  // HM01B0 needs continuous MCLK.
  // First disable to reset internal state tracking (s_current_mclk_hz), then enable.
  hm01b0_esp32_mclk(cfg, false, NULL);
  hm01b0_esp32_mclk(cfg, true, NULL);

  // Invert VSYNC input signal using GPIO matrix.
  // HM01B0 VSYNC is Active High (Pulse High), but esp_camera driver usually expects Active Low (Pulse Low).
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    esp_rom_gpio_connect_in_signal(config.pin_vsync, CAM_V_SYNC_IDX, true);
    // Revert to Inverted PCLK (Standard for ESP32)
    esp_rom_gpio_connect_in_signal(config.pin_pclk, CAM_PCLK_IDX, true);
  #else
    esp_rom_gpio_connect_in_signal(config.pin_vsync, I2S0I_V_SYNC_IDX, true);
    esp_rom_gpio_connect_in_signal(config.pin_pclk, I2S0I_WS_IN_IDX, true);
  #endif

  s_camera_inited = true;
  return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_camera_deinit(hm01b0_cfg_t* cfg, void* arg){
  (void)cfg; (void)arg;
  if(!s_camera_inited) return HM01B0_ERR_OK;
  esp_camera_deinit();
  s_camera_inited = false;
  return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_camera_capture_frame(hm01b0_cfg_t* cfg, uint8_t* buf, uint32_t buf_len, uint32_t* out_len, uint32_t timeout_ms){
  (void)cfg; (void)timeout_ms;
  if(!s_camera_inited) return HM01B0_ERR_INIT;
  camera_fb_t *fb = esp_camera_fb_get();
  if(!fb) return HM01B0_ERR;
  
  // Calculate expected size based on actual frame length or sensor status
  // Default to QVGA if unknown
  size_t expected = (size_t)HM01B0_PIXEL_X_NUM * (size_t)HM01B0_PIXEL_Y_NUM;
  
  // Check if we are in QQVGA mode (approx check based on length)
  if (fb->len == 19200 || fb->len == 19764) {
      expected = fb->len;
  }
  // Check for QCIF (176x144 = 25344)
  if (fb->len == 25344) {
      expected = fb->len;
  }

  // Allow partial frames if user requested "just return data"
  size_t copy_len = (fb->len < expected) ? fb->len : expected;
  if(copy_len > buf_len) copy_len = buf_len;

  memcpy(buf, fb->buf, copy_len);
  if(out_len) {
      *out_len = (uint32_t)copy_len;
      ESP_LOGI(TAG_CAM, "Updated out_len to %u", (unsigned)*out_len);
  } else {
      ESP_LOGW(TAG_CAM, "out_len is NULL");
  }
  
  if(fb->len != expected){
      ESP_LOGW(TAG_CAM, "Frame size mismatch! Expected: %u, Got: %u. Copied %u bytes.", (unsigned)expected, (unsigned)fb->len, (unsigned)copy_len);
      // return HM01B0_ERR; // Do not fail, just warn
  }
  
  esp_camera_fb_return(fb);
  return HM01B0_ERR_OK;
}

#else

// esp_camera not available: provide stubs
hm01b0_status_e hm01b0_camera_init(hm01b0_cfg_t* cfg, void* arg){ (void)cfg; (void)arg; return HM01B0_ERR_UNIMPLEMENTED; }
hm01b0_status_e hm01b0_camera_deinit(hm01b0_cfg_t* cfg, void* arg){ (void)cfg; (void)arg; return HM01B0_ERR_UNIMPLEMENTED; }
hm01b0_status_e hm01b0_camera_capture_frame(hm01b0_cfg_t* cfg, uint8_t* buf, uint32_t buf_len, uint32_t* out_len, uint32_t timeout_ms){ (void)cfg; (void)buf; (void)buf_len; (void)out_len; (void)timeout_ms; return HM01B0_ERR_UNIMPLEMENTED; }

#endif
