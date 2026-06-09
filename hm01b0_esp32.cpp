#include "hm01b0_esp32.h"
#include "hm01b0.h"
#include <Arduino.h>
// #include <Wire.h> // Removed to avoid conflict with driver/i2c.h
#include "hm01b0_script.h"
#include <esp_task_wdt.h>
#include "driver/i2c.h"
#define HM01B0_I2C_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

#include "soc/ledc_struct.h"
#include <driver/ledc.h>
// MCPWM fallback removed to avoid dependency on legacy/new MCPWM APIs
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#if 0
#include <driver/rmt.h>
#endif
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

static const uint8_t hm_i2c_addr = HM01B0_DEFAULT_ADDRESS; // 0x24
// cached current MCLK frequency (0 == not configured)
static uint32_t s_current_mclk_hz = 0;
// track whether MCPWM is used for MCLK
// local spinlock for critical sampling sections
static portMUX_TYPE s_hm01b0_lock = portMUX_INITIALIZER_UNLOCKED;

hm01b0_status_e hm01b0_esp32_init(hm01b0_cfg_t* cfg, void* arg){
  (void)cfg; (void)arg;
  
  // Init I2C using ESP-IDF Legacy Driver
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = HM01B0_SDA_PIN;
  conf.scl_io_num = HM01B0_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = HM01B0_I2C_FREQ_HZ;
  
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  
  // configure trigger pin
  pinMode(HM01B0_TRIG_PIN, OUTPUT);
  digitalWrite(HM01B0_TRIG_PIN, LOW);

  // ensure MCLK pin drive capability is strong
  gpio_set_drive_capability((gpio_num_t)HM01B0_MCLK_PIN, GPIO_DRIVE_CAP_3);

  // configure parallel data and sync pins as inputs
  pinMode(HM01B0_D0_PIN, INPUT);
  pinMode(HM01B0_D1_PIN, INPUT);
  pinMode(HM01B0_D2_PIN, INPUT);
  pinMode(HM01B0_D3_PIN, INPUT);
  pinMode(HM01B0_D4_PIN, INPUT);
  pinMode(HM01B0_D5_PIN, INPUT);
  pinMode(HM01B0_D6_PIN, INPUT);
  pinMode(HM01B0_D7_PIN, INPUT);

  pinMode(HM01B0_HSYNC_PIN, INPUT);
  pinMode(HM01B0_VSYNC_PIN, INPUT);
  pinMode(HM01B0_PCLK_PIN, INPUT);
  pinMode(HM01B0_INT_PIN, INPUT);
  return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_esp32_deinit(hm01b0_cfg_t* cfg, void* arg){
  (void)cfg; (void)arg;
  // uninstall I2C driver
  digitalWrite(HM01B0_TRIG_PIN, LOW);
  i2c_driver_delete(I2C_MASTER_NUM);

  // optional: set pins to input
  pinMode(HM01B0_TRIG_PIN, INPUT);
  // pinMode(HM01B0_MCLK_PIN, INPUT); // Keep MCLK running to prevent sensor reset
  pinMode(HM01B0_D0_PIN, INPUT);
  pinMode(HM01B0_D1_PIN, INPUT);
  pinMode(HM01B0_D2_PIN, INPUT);
  pinMode(HM01B0_D3_PIN, INPUT);
  pinMode(HM01B0_D4_PIN, INPUT);
  pinMode(HM01B0_D5_PIN, INPUT);
  pinMode(HM01B0_D6_PIN, INPUT);
  pinMode(HM01B0_D7_PIN, INPUT);
  (void)0;
  return HM01B0_ERR_OK;
}

// I2C write: HM01B0 registers are 16-bit addresses followed by data bytes
hm01b0_status_e hm01b0_esp32_write(hm01b0_cfg_t* cfg, uint16_t reg, uint8_t *data, uint32_t len, void* arg){
  (void)cfg; (void)arg;
  if(len > 256) return HM01B0_ERR_PARAMS;
  
  #if HM01B0_DISABLE_MCLK_DURING_I2C
  hm01b0_esp32_mclk(cfg, false, NULL);
  #endif

  uint8_t buffer[258];
  buffer[0] = (reg >> 8) & 0xFF;
  buffer[1] = reg & 0xFF;
  memcpy(&buffer[2], data, len);

  esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, hm_i2c_addr, buffer, len + 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  #if HM01B0_DISABLE_MCLK_DURING_I2C
  hm01b0_esp32_mclk(cfg, true, NULL);
  #endif

  if(err != ESP_OK){
    ESP_LOGW("hm01b0_esp32", "I2C write failed: %s", esp_err_to_name(err));
    return HM01B0_ERR_I2C;
  }
  return HM01B0_ERR_OK;
}

hm01b0_status_e hm01b0_esp32_read(hm01b0_cfg_t* cfg, uint16_t reg, uint8_t *data, uint32_t len, void* arg){
  (void)cfg; (void)arg;
  
  uint8_t reg_addr[2];
  reg_addr[0] = (reg >> 8) & 0xFF;
  reg_addr[1] = reg & 0xFF;

  esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, hm_i2c_addr, reg_addr, 2, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  if(err != ESP_OK) return HM01B0_ERR_I2C;
  
  return HM01B0_ERR_OK;
}

// MCLK: use LEDC to generate square wave on HM01B0_MCLK_PIN
hm01b0_status_e hm01b0_esp32_mclk(hm01b0_cfg_t* cfg, bool enable, void* arg){
  (void)cfg; (void)arg;
  const int ledc_channel = LEDC_CHANNEL_0;
  if(enable){
    // Use the helper to set default 4MHz, but skip reconfiguration if already set
    if(s_current_mclk_hz == 4000000) return HM01B0_ERR_OK;
    hm01b0_status_e st = hm01b0_esp32_set_mclk_freq(cfg, 4000000, arg);
    if(st == HM01B0_ERR_OK) s_current_mclk_hz = 4000000;
    return st;
  }else{
    // detach only if previously configured
    if(s_current_mclk_hz != 0){
      ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, 0);
      ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, 0);
    }
    // reset pin to default (input) to avoid residual alternate functions
    gpio_reset_pin((gpio_num_t)HM01B0_MCLK_PIN);
    // no MCPWM fallback in this build; just reset state
    s_current_mclk_hz = 0;
    return HM01B0_ERR_OK;
  }
}

// Set MCLK frequency on HM01B0_MCLK_PIN using LEDC. Returns HM01B0_ERR_OK on success.
hm01b0_status_e hm01b0_esp32_set_mclk_freq(hm01b0_cfg_t* cfg, uint32_t freq_hz, void* arg){
  (void)cfg; (void)arg;
  const int ledc_channel = LEDC_CHANNEL_0;
  const int ledc_timer = LEDC_TIMER_0;
  // Ensure previous configuration is stopped and pin reset to GPIO to avoid conflicts
  if(s_current_mclk_hz != 0){
    ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, 0);
  }
  gpio_reset_pin((gpio_num_t)HM01B0_MCLK_PIN);
  pinMode(HM01B0_MCLK_PIN, OUTPUT);

  // First try the commonly-working resolution (4) to avoid many failing attempts
  {
    int res = 4;
    ledc_timer_config_t tc = {};
    tc.duty_resolution = (ledc_timer_bit_t)res;
    tc.freq_hz = freq_hz;
    tc.speed_mode = LEDC_LOW_SPEED_MODE;
    tc.timer_num = (ledc_timer_t)ledc_timer;
    if(ledc_timer_config(&tc) == ESP_OK){
      ledc_channel_config_t cc = {};
      cc.channel = (ledc_channel_t)ledc_channel;
      uint32_t max_duty = (1u << tc.duty_resolution) - 1u;
      cc.duty = (max_duty + 1u) / 2u;
      cc.gpio_num = HM01B0_MCLK_PIN;
      cc.speed_mode = LEDC_LOW_SPEED_MODE;
      cc.timer_sel = (ledc_timer_t)ledc_timer;
      if(ledc_channel_config(&cc) == ESP_OK){
        ESP_LOGI("hm01b0_esp32", "MCLK enabled: LOW_SPEED_MODE %u Hz res=%d duty=%u/%u", (unsigned)tc.freq_hz, res, (unsigned)cc.duty, (unsigned)max_duty);
        s_current_mclk_hz = freq_hz;
        return HM01B0_ERR_OK;
      }
    }
  }

  // Try small resolutions 3..1 as a last-ditch before MCPWM
  for(int res = 3; res >= 1; --res){
    ledc_timer_config_t tc = {};
    tc.duty_resolution = (ledc_timer_bit_t)res;
    tc.freq_hz = freq_hz;
    tc.speed_mode = LEDC_LOW_SPEED_MODE;
    tc.timer_num = (ledc_timer_t)ledc_timer;
    if(ledc_timer_config(&tc) != ESP_OK) continue;

    ledc_channel_config_t cc = {};
    cc.channel = (ledc_channel_t)ledc_channel;
    uint32_t max_duty = (1u << tc.duty_resolution) - 1u;
    cc.duty = (max_duty + 1u) / 2u;
    cc.gpio_num = HM01B0_MCLK_PIN;
    cc.speed_mode = LEDC_LOW_SPEED_MODE;
    cc.timer_sel = (ledc_timer_t)ledc_timer;
    if(ledc_channel_config(&cc) != ESP_OK) continue;
    ESP_LOGI("hm01b0_esp32", "MCLK enabled: LOW_SPEED_MODE %u Hz res=%d duty=%u/%u", (unsigned)tc.freq_hz, res, (unsigned)cc.duty, (unsigned)max_duty);
    s_current_mclk_hz = freq_hz;
    return HM01B0_ERR_OK;
  }

  ESP_LOGW("hm01b0_esp32", "LEDC couldn't configure requested frequency %u. Consider MCPWM fallback. Using best available resolution=%d", (unsigned)freq_hz, 4);
  ESP_LOGW("hm01b0_esp32", "No MCPWM fallback available; cannot configure MCLK %u Hz", (unsigned)freq_hz);
  return HM01B0_ERR_MCLK;
}

hm01b0_status_e hm01b0_esp32_trig(hm01b0_cfg_t* cfg, bool enable, void* arg){
  (void)cfg; (void)arg;
  digitalWrite(HM01B0_TRIG_PIN, enable ? HIGH : LOW);
  return HM01B0_ERR_OK;
}

// Provide a platform interface instance the C driver expects
static hm01b0_if_t s_esp_if = {
  .init = hm01b0_esp32_init,
  .write = hm01b0_esp32_write,
  .read = hm01b0_esp32_read,
  .mclk = hm01b0_esp32_mclk,
  .trig = hm01b0_esp32_trig,
  .deinit = hm01b0_esp32_deinit,
  .arg = NULL
};

// Helper to populate cfg
void hm01b0_populate_cfg(hm01b0_cfg_t* cfg){
  cfg->interface = &s_esp_if;
}

// Diagnostic helper: sample MCLK/VSYNC/HSYNC/PCLK for `ms` milliseconds and log edge counts
void hm01b0_esp32_dump_signals(uint32_t ms){
  const uint32_t interval_ms = ms ? ms : 100;
  uint32_t start = millis();
  int mclk_count = 0, vsync_count = 0, hsync_count = 0, pclk_count = 0;
  int prev_mclk = gpio_get_level(HM01B0_MCLK_PIN);
  int prev_vsync = gpio_get_level(HM01B0_VSYNC_PIN);
  int prev_hsync = gpio_get_level(HM01B0_HSYNC_PIN);
  int prev_pclk = gpio_get_level(HM01B0_PCLK_PIN);
  while((millis() - start) < interval_ms){
    int v = gpio_get_level(HM01B0_MCLK_PIN);
    if(v && !prev_mclk) mclk_count++;
    prev_mclk = v;
    v = gpio_get_level(HM01B0_VSYNC_PIN);
    if(v && !prev_vsync) vsync_count++;
    prev_vsync = v;
    v = gpio_get_level(HM01B0_HSYNC_PIN);
    if(v && !prev_hsync) hsync_count++;
    prev_hsync = v;
    v = gpio_get_level(HM01B0_PCLK_PIN);
    if(v && !prev_pclk) pclk_count++;
    prev_pclk = v;
    ets_delay_us(100); // 0.1 ms polling
  }
  ESP_LOGI("hm01b0_esp32", "Signal sample %ums: MCLK edges=%d VSYNC edges=%d HSYNC edges=%d PCLK edges=%d", interval_ms, mclk_count, vsync_count, hsync_count, pclk_count);
}

// Read one byte from parallel D0..D7 on PCLK rising edge (returns -1 on timeout)
// Read one byte from parallel D0..D7 on PCLK edge (returns -1 on timeout)
// pclk_rising: true => sample on rising edge (low->high). false => sample on falling edge (high->low)
// Fast parallel byte read using GPIO input register to sample D0..D7.
// Returns -1 on timeout, otherwise 0..255 value.
static inline IRAM_ATTR int read_parallel_byte(uint32_t start_ms, uint32_t timeout_ms, bool pclk_rising){
  // Wait for the chosen PCLK edge by polling gpio input level.
  uint32_t now;
  if(pclk_rising){
    // wait for PCLK low -> high
    while((((uint64_t)GPIO.in >> HM01B0_PCLK_PIN) & 1u)){
      now = millis();
      if(timeout_ms && (now - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "PCLK stayed HIGH until timeout (start=%u now=%u)", start_ms, now);
        return -1;
      }
      // short yield
      ets_delay_us(10);
      esp_task_wdt_reset();
    }
    while((((uint64_t)GPIO.in >> HM01B0_PCLK_PIN) & 1u) == 0){
      now = millis();
      if(timeout_ms && (now - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "PCLK stayed LOW until timeout (start=%u now=%u)", start_ms, now);
        return -1;
      }
      ets_delay_us(10);
      esp_task_wdt_reset();
    }
  } else {
    // falling edge: wait for high -> low
    while((((uint64_t)GPIO.in >> HM01B0_PCLK_PIN) & 1u) == 0){
      now = millis();
      if(timeout_ms && (now - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "PCLK stayed LOW until timeout (start=%u now=%u)", start_ms, now);
        return -1;
      }
      ets_delay_us(10);
      esp_task_wdt_reset();
    }
    while((((uint64_t)GPIO.in >> HM01B0_PCLK_PIN) & 1u)){
      now = millis();
      if(timeout_ms && (now - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "PCLK stayed HIGH until timeout (start=%u now=%u)", start_ms, now);
        return -1;
      }
      ets_delay_us(10);
      esp_task_wdt_reset();
    }
  }

  // Read all D0..D7 from GPIO input register in one go. Lock only briefly around the register read.
  portENTER_CRITICAL(&s_hm01b0_lock);
  uint32_t gpio_in = GPIO.in;
  portEXIT_CRITICAL(&s_hm01b0_lock);
  uint64_t gpio_in64 = (uint64_t)gpio_in;
  uint8_t b = 0;
  b |= ((gpio_in64 >> HM01B0_D0_PIN) & 1u) ? (1u << 0) : 0;
  b |= ((gpio_in64 >> HM01B0_D1_PIN) & 1u) ? (1u << 1) : 0;
  b |= ((gpio_in64 >> HM01B0_D2_PIN) & 1u) ? (1u << 2) : 0;
  b |= ((gpio_in64 >> HM01B0_D3_PIN) & 1u) ? (1u << 3) : 0;
  b |= ((gpio_in64 >> HM01B0_D4_PIN) & 1u) ? (1u << 4) : 0;
  b |= ((gpio_in64 >> HM01B0_D5_PIN) & 1u) ? (1u << 5) : 0;
  b |= ((gpio_in64 >> HM01B0_D6_PIN) & 1u) ? (1u << 6) : 0;
  b |= ((gpio_in64 >> HM01B0_D7_PIN) & 1u) ? (1u << 7) : 0;
  return (int)b;
}

// Simple demo: read a single row into buffer until HSYNC toggles (not used in init)
// read a row with configurable HSYNC active polarity and PCLK edge
IRAM_ATTR hm01b0_status_e hm01b0_esp32_read_row(uint8_t* buf, uint32_t maxlen, uint32_t start_ms, uint32_t timeout_ms, bool hsync_active_high, bool pclk_rising){
  uint32_t idx = 0;
  // wait for HSYNC active (assuming HSYNC is active HIGH)
  if(hsync_active_high){
    while(((GPIO.in >> HM01B0_HSYNC_PIN) & 1u) == 0){
      if(timeout_ms && (millis() - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "HSYNC didn't go HIGH before timeout (start=%u now=%u)", start_ms, millis());
        return HM01B0_ERR_PARAMS;
      }
      ets_delay_us(50);
      esp_task_wdt_reset();
    }
    // while HSYNC remains high, sample PCLK
    while(((GPIO.in >> HM01B0_HSYNC_PIN) & 1u) != 0){
      int val = read_parallel_byte(start_ms, timeout_ms, pclk_rising);
      if(val < 0) return HM01B0_ERR_PARAMS;
      if(idx < maxlen){
        buf[idx++] = (uint8_t)val;
      }
      ets_delay_us(5);
      esp_task_wdt_reset();
    }
  } else {
    while(((GPIO.in >> HM01B0_HSYNC_PIN) & 1u) != 0){
      if(timeout_ms && (millis() - start_ms) > timeout_ms){
        ESP_LOGW("hm01b0_esp32", "HSYNC didn't go LOW before timeout (start=%u now=%u)", start_ms, millis());
        return HM01B0_ERR_PARAMS;
      }
      ets_delay_us(50);
      esp_task_wdt_reset();
    }
    while(((GPIO.in >> HM01B0_HSYNC_PIN) & 1u) == 0){
      int val = read_parallel_byte(start_ms, timeout_ms, pclk_rising);
      if(val < 0) return HM01B0_ERR_PARAMS;
      if(idx < maxlen){
        buf[idx++] = (uint8_t)val;
      }
      ets_delay_us(5);
      esp_task_wdt_reset();
    }
  }
  return HM01B0_ERR_OK;
}

IRAM_ATTR hm01b0_status_e hm01b0_esp32_read_frame(hm01b0_cfg_t* cfg, uint8_t* buf, uint32_t buf_len, uint32_t timeout_ms){
  (void)cfg;
  if(buf == NULL) return HM01B0_ERR_PARAMS;

  const uint32_t expected = (uint32_t)HM01B0_PIXEL_X_NUM * (uint32_t)HM01B0_PIXEL_Y_NUM;
  if(buf_len < expected) return HM01B0_ERR_PARAMS;

  uint32_t idx = 0;
  uint32_t start = millis();

  // We'll try combinations of VSYNC polarity, HSYNC polarity and PCLK edge if the default fails.
  const bool vsync_options[2] = { true, false };
  const bool hsync_options[2] = { true, false };
  const bool pclk_options[2] = { true, false };
  for(int vidx=0; vidx<2; vidx++){
    for(int hidx=0; hidx<2; hidx++){
      for(int pidx=0; pidx<2; pidx++){
        bool vsync_active_high = vsync_options[vidx];
        bool hsync_active_high = hsync_options[hidx];
        bool pclk_rising = pclk_options[pidx];
        // reset idx and start time for each attempt
        idx = 0;
        start = millis();
        ESP_LOGI("hm01b0_esp32", "Attempt capture with VSYNC_active_high=%d HSYNC_active_high=%d PCLK_rising=%d", (int)vsync_active_high, (int)hsync_active_high, (int)pclk_rising);
        // wait for VSYNC to reach active level (frame start)
        if(vsync_active_high){
          while(digitalRead(HM01B0_VSYNC_PIN) == LOW){
            if(timeout_ms && (millis() - start) > timeout_ms){
              ESP_LOGW("hm01b0_esp32", "VSYNC didn't go HIGH before timeout (start=%u now=%u). VSYNC=%d PCLK=%d HSYNC=%d", start, millis(), digitalRead(HM01B0_VSYNC_PIN), digitalRead(HM01B0_PCLK_PIN), digitalRead(HM01B0_HSYNC_PIN));
              goto try_next_combo;
            }
            delay(1);
            esp_task_wdt_reset();
          }
        } else {
          while(digitalRead(HM01B0_VSYNC_PIN) == HIGH){
            if(timeout_ms && (millis() - start) > timeout_ms){
              ESP_LOGW("hm01b0_esp32", "VSYNC didn't go LOW before timeout (start=%u now=%u). VSYNC=%d PCLK=%d HSYNC=%d", start, millis(), digitalRead(HM01B0_VSYNC_PIN), digitalRead(HM01B0_PCLK_PIN), digitalRead(HM01B0_HSYNC_PIN));
              goto try_next_combo;
            }
            delay(1);
            esp_task_wdt_reset();
          }
        }

        // Now within frame: read HM01B0_PIXEL_Y_NUM rows
        for(uint32_t row = 0; row < HM01B0_PIXEL_Y_NUM; row++){
          hm01b0_status_e r = hm01b0_esp32_read_row(&buf[idx], HM01B0_PIXEL_X_NUM, start, timeout_ms, hsync_active_high, pclk_rising);
          if(r != HM01B0_ERR_OK) goto try_next_combo;
          idx += HM01B0_PIXEL_X_NUM;
          ets_delay_us(10);
          esp_task_wdt_reset();
        }

        // wait for VSYNC to clear (frame end)
        if(vsync_active_high){
          while(digitalRead(HM01B0_VSYNC_PIN) == HIGH){
            if(timeout_ms && (millis() - start) > timeout_ms){
              ESP_LOGW("hm01b0_esp32", "VSYNC remained HIGH beyond timeout (start=%u now=%u). VSYNC=%d PCLK=%d HSYNC=%d", start, millis(), digitalRead(HM01B0_VSYNC_PIN), digitalRead(HM01B0_PCLK_PIN), digitalRead(HM01B0_HSYNC_PIN));
              break;
            }
            delay(1);
            esp_task_wdt_reset();
          }
        } else {
          while(digitalRead(HM01B0_VSYNC_PIN) == LOW){
            if(timeout_ms && (millis() - start) > timeout_ms){
              ESP_LOGW("hm01b0_esp32", "VSYNC remained LOW beyond timeout (start=%u now=%u). VSYNC=%d PCLK=%d HSYNC=%d", start, millis(), digitalRead(HM01B0_VSYNC_PIN), digitalRead(HM01B0_PCLK_PIN), digitalRead(HM01B0_HSYNC_PIN));
              break;
            }
            delay(1);
            esp_task_wdt_reset();
          }
        }

        if(idx >= expected) return HM01B0_ERR_OK;
      try_next_combo:;
        ESP_LOGI("hm01b0_esp32", "Attempt failed for VSYNC_active_high=%d HSYNC_active_high=%d PCLK_rising=%d", (int)vsync_active_high, (int)hsync_active_high, (int)pclk_rising);
        // small delay before next attempt
        ets_delay_us(20000);
      }
    }
  }

  // wait for VSYNC low (frame end)
  while(digitalRead(HM01B0_VSYNC_PIN) == HIGH){
    if(timeout_ms && (millis() - start) > timeout_ms){
      ESP_LOGW("hm01b0_esp32", "VSYNC remained HIGH beyond timeout (start=%u now=%u). VSYNC=%d PCLK=%d HSYNC=%d", start, millis(), digitalRead(HM01B0_VSYNC_PIN), digitalRead(HM01B0_PCLK_PIN), digitalRead(HM01B0_HSYNC_PIN));
      break;
    }
    delay(1);
    esp_task_wdt_reset();
  }

  if(idx >= expected) return HM01B0_ERR_OK;
  return HM01B0_ERR;
}
