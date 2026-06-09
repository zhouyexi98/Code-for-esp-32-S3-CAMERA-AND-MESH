#include "Arduino.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "hm01b0.h"
#include "hm01b0_esp32.h"
#include "hm01b0_script.h"
#include "hm01b0_camera_esp32.h"
#include "SPIFFS.h"
#include <driver/ledc.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "mbedtls/base64.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "esp_pm.h"

#define GPIO_PIN_1 GPIO_NUM_1
#define GPIO_PIN_2 GPIO_NUM_2
#define GPIO_PIN_5 GPIO_NUM_5
#define GPIO_PIN_6 GPIO_NUM_6
#define GPIO_PIN_7 GPIO_NUM_7
#define GPIO_PIN_9 GPIO_NUM_9
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_PIN_1) | (1ULL<<GPIO_PIN_2) | (1ULL<<GPIO_PIN_5) | (1ULL<<GPIO_PIN_6) | (1ULL<<GPIO_PIN_7) | (1ULL<<GPIO_PIN_9))

QueueHandle_t gpio_cmd_queue = NULL;

void gpio_control_task(void *arg) {
    uint8_t cmd;
    while(1) {
        if(xQueueReceive(gpio_cmd_queue, &cmd, portMAX_DELAY)) {
            ESP_LOGI("GPIO", "GPIO Control Task: Received cmd 0x%02X", cmd);
            if (cmd == 0x00) {
                // 0x00: All Low
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 0);
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 0);
            } else if (cmd == 0x01) {
                // 0x01: IO1 High, others Low -> Wait -> IO2 High, others Low
                gpio_set_level(GPIO_PIN_1, 1);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 0);
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 1);
                // Ensure others stay low
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 0);
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 0);
            } else if (cmd == 0x03) {
                // 0x03: IO5 High, others Low -> Wait -> IO6 High, others Low
                gpio_set_level(GPIO_PIN_5, 1);
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_6, 0);
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 1);
                // Ensure others stay low
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 0);
            } else if (cmd == 0x05) {
                // 0x05: IO7 High, others Low -> Wait -> IO9 High, others Low
                gpio_set_level(GPIO_PIN_7, 1);
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 0);
                gpio_set_level(GPIO_PIN_9, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
                gpio_set_level(GPIO_PIN_7, 0);
                gpio_set_level(GPIO_PIN_9, 1);
                // Ensure others stay low
                gpio_set_level(GPIO_PIN_1, 0);
                gpio_set_level(GPIO_PIN_2, 0);
                gpio_set_level(GPIO_PIN_5, 0);
                gpio_set_level(GPIO_PIN_6, 0);
            }
        }
    }
}

// Native NimBLE Headers
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_att.h"

static const char *TAG = "main";

// Enable serial Base64 frame dump: set to 1 to print frames to serial
#ifndef DUMP_FRAME_TO_SERIAL
#define DUMP_FRAME_TO_SERIAL 0
#endif

// BLE Service and Characteristic UUIDs
// 4fafc201-1fb5-459e-8fcc-c5c9c331914b
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

// beb5483e-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

uint16_t gatt_svr_chr_val_handle;
uint16_t conn_handle;
bool deviceConnected = false;
volatile bool captureRequested = false;

// Control Service UUID: 5fafc201-1fb5-459e-8fcc-c5c9c331914b
static const ble_uuid128_t gatt_svr_svc_control_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x5f);

// Control Characteristic UUID: ceb5483e-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t gatt_svr_chr_control_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xce);

uint16_t gatt_svr_chr_control_val_handle;

// SHTC3 Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914c
static const ble_uuid128_t gatt_svr_svc_shtc3_uuid =
    BLE_UUID128_INIT(0x4c, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

// SHTC3 Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a9
static const ble_uuid128_t gatt_svr_chr_shtc3_uuid =
    BLE_UUID128_INIT(0xa9, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

uint16_t gatt_svr_chr_shtc3_val_handle;
bool shtc3_notify_enabled = false;

// SHTC3 I2C Address
#define SHTC3_ADDR 0x70
#define SHTC3_SDA_PIN 45
#define SHTC3_SCL_PIN 46
// Use I2C controller 0 for SHTC3 to avoid interrupt conflict with camera (which uses I2C_EXT1)
#define SHTC3_I2C_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

void init_shtc3_i2c() {
    ESP_LOGI(TAG, "Initializing SHTC3 I2C on Port %d (SDA=%d, SCL=%d)", SHTC3_I2C_NUM, SHTC3_SDA_PIN, SHTC3_SCL_PIN);
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SHTC3_SDA_PIN;
    conf.scl_io_num = SHTC3_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    
    i2c_param_config(SHTC3_I2C_NUM, &conf);
    esp_err_t err = i2c_driver_install(SHTC3_I2C_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHTC3 I2C Driver Install Failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SHTC3 I2C Driver Installed Successfully");
    }
}

void read_shtc3(float *temp, float *hum) {
    uint8_t cmd_wakeup[] = {0x35, 0x17};
    uint8_t cmd_measure[] = {0x78, 0x66};
    uint8_t cmd_sleep[] = {0xB0, 0x98};
    uint8_t data[6];

    // Wakeup
    i2c_master_write_to_device(SHTC3_I2C_NUM, SHTC3_ADDR, cmd_wakeup, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    ets_delay_us(250);

    // Measure
    i2c_master_write_to_device(SHTC3_I2C_NUM, SHTC3_ADDR, cmd_measure, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Read
    esp_err_t ret = i2c_master_read_from_device(SHTC3_I2C_NUM, SHTC3_ADDR, data, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        uint16_t t_raw = (data[0] << 8) | data[1];
        uint16_t h_raw = (data[3] << 8) | data[4];

        *temp = -45.0 + 175.0 * ((float)t_raw / 65535.0);
        *hum = 100.0 * ((float)h_raw / 65535.0);
    } else {
        *temp = 0.0;
        *hum = 0.0;
        ESP_LOGW(TAG, "SHTC3 Read Failed: %s", esp_err_to_name(ret));
    }

    // Sleep
    i2c_master_write_to_device(SHTC3_I2C_NUM, SHTC3_ADDR, cmd_sleep, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void shtc3_task(void *arg) {
    float temp, hum;
    while (1) {
        // Only measure if connected AND notifications are enabled
        if (deviceConnected && shtc3_notify_enabled) {
            read_shtc3(&temp, &hum);
            ESP_LOGI(TAG, "SHTC3: T=%.2f, H=%.2f", temp, hum);
            
            // Format: 2 bytes Temp (int16, x100), 2 bytes Hum (int16, x100)
            int16_t t_int = (int16_t)(temp * 100);
            int16_t h_int = (int16_t)(hum * 100);
            uint8_t data[4];
            data[0] = t_int & 0xFF;
            data[1] = (t_int >> 8) & 0xFF;
            data[2] = h_int & 0xFF;
            data[3] = (h_int >> 8) & 0xFF;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(data, sizeof(data));
            ble_gatts_notify_custom(conn_handle, gatt_svr_chr_shtc3_val_handle, om);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static int gatt_svr_chr_shtc3_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // Read support
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        float temp, hum;
        read_shtc3(&temp, &hum);
        int16_t t_int = (int16_t)(temp * 100);
        int16_t h_int = (int16_t)(hum * 100);
        uint8_t data[4];
        data[0] = t_int & 0xFF;
        data[1] = (t_int >> 8) & 0xFF;
        data[2] = h_int & 0xFF;
        data[3] = (h_int >> 8) & 0xFF;
        int rc = os_mbuf_append(ctxt->om, data, sizeof(data));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static int gatt_svr_chr_control_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (ctxt->om->om_len > 0) {
            uint8_t cmd = ctxt->om->om_data[0];
            ESP_LOGI(TAG, "Control Command Received: 0x%02X", cmd);
            if (gpio_cmd_queue) {
                xQueueSend(gpio_cmd_queue, &cmd, 0);
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// BLE Host Task
void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// GATT Access Callback
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Optional: Return current status or empty
        rc = os_mbuf_append(ctxt->om, "Ready", 5);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "BLE Command Received");
        captureRequested = true;
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// GATT Service Definition
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_chr_val_handle,
            },
            {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_control_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_control_uuid.u,
                .access_cb = gatt_svr_chr_control_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &gatt_svr_chr_control_val_handle,
            },
            {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_shtc3_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_shtc3_uuid.u,
                .access_cb = gatt_svr_chr_shtc3_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_chr_shtc3_val_handle,
            },
            {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        0, /* No more services */
    },
};

// Global cfg instance
hm01b0_cfg_t hm01b0_cfg_global = {0};

// GAP Event Handler
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
    {
        ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK" : "FAILED");
        if (event->connect.status == 0) {
            deviceConnected = true;
            conn_handle = event->connect.conn_handle;
            
            // --- WAKE UP CAMERA & MCLK ---
            ESP_LOGI(TAG, "BLE Connected: Waking up Camera & MCLK...");
            
            // 1. Enable MCLK
            hm01b0_esp32_mclk(&hm01b0_cfg_global, true, NULL);
            delay(10); // Wait for clock to stabilize

            // 2. Enable Camera Streaming (Wakeup)
            sensor_t *s_cam = esp_camera_sensor_get();
            if (s_cam && s_cam->set_reg) {
                s_cam->set_reg(s_cam, 0x0100, 0xFF, 0x01); // MODE_SELECT = Streaming
                delay(100); // Wait for sensor to settle
            }
            // -----------------------------

            // Request connection parameter update for higher speed
            struct ble_gap_upd_params params;
            params.itvl_min = 6; // 7.5ms
            params.itvl_max = 12; // 15ms
            params.latency = 0;
            params.supervision_timeout = 100; // 1s
            params.min_ce_len = 0;
            params.max_ce_len = 0;
            
            int rc = ble_gap_update_params(conn_handle, &params);
            if (rc != 0) {
                ESP_LOGW(TAG, "Failed to request params update: %d", rc);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "BLE GAP EVENT MTU update to %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
    {
        ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT");
        deviceConnected = false;
        
        // --- SLEEP CAMERA & MCLK ---
        ESP_LOGI(TAG, "BLE Disconnected: Sleeping Camera & MCLK...");
        
        // 1. Disable Camera Streaming (Standby)
        // Note: Must be done BEFORE disabling MCLK, as I2C needs MCLK
        sensor_t *s_cam = esp_camera_sensor_get();
        if (s_cam && s_cam->set_reg) {
            s_cam->set_reg(s_cam, 0x0100, 0xFF, 0x00); // MODE_SELECT = Standby
            delay(10);
        }

        // 2. Disable MCLK
        hm01b0_esp32_mclk(&hm01b0_cfg_global, false, NULL);
        // ---------------------------

        // Restart advertising
        {
            struct ble_gap_adv_params adv_params;
            memset(&adv_params, 0, sizeof(adv_params));
            adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
            adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
            // Increase advertising interval to save power
            adv_params.itvl_min = 320; // 200ms
            adv_params.itvl_max = 480; // 300ms
            int rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to restart advertising: %d", rc);
            } else {
                ESP_LOGI(TAG, "Advertising restarted");
            }
        }
        break;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
        // Restart advertising
        {
            struct ble_gap_adv_params adv_params;
            memset(&adv_params, 0, sizeof(adv_params));
            adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
            adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
            // Increase advertising interval to save power
            adv_params.itvl_min = 320; // 200ms
            adv_params.itvl_max = 480; // 300ms
            ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "BLE GAP EVENT SUBSCRIBE: handle=%d, cur_notify=%d", 
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        if (event->subscribe.attr_handle == gatt_svr_chr_shtc3_val_handle) {
            shtc3_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "SHTC3 Notify: %s", shtc3_notify_enabled ? "ENABLED" : "DISABLED");
        }
        break;
    }
    return 0;
}

// Sync Callback
static void ble_on_sync(void) {
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    // Set device name
    rc = ble_svc_gap_device_name_set("ESP32-CAM-BLE-NATIVE");
    assert(rc == 0);

    // Configure Advertising Data
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);

    // Flags: General Discoverable + BR/EDR Not Supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Include TX Power Level
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Include Device Name
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    // Start advertising
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // Increase advertising interval to save power (units of 0.625ms)
    // Min: 320 * 0.625 = 200ms
    // Max: 480 * 0.625 = 300ms
    adv_params.itvl_min = 320; 
    adv_params.itvl_max = 480;
    
    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Advertising started");
    } else {
        ESP_LOGE(TAG, "Error enabling advertising; rc=%d", rc);
    }
}

// global cfg instance used by Arduino wrapper in original library
// Moved to top of file
// hm01b0_cfg_t hm01b0_cfg_global = {0};

// Globals for Capture and BLE
uint8_t *frame_buf = NULL;
size_t frame_size = 0;
bool used_camera_driver = false;

static void sample_signals(const char* label, uint32_t duration_ms){
  const uint32_t step = 5;
  uint32_t samples = duration_ms / step;
  uint32_t cnt_mclk = 0, cnt_vsync = 0, cnt_hsync = 0, cnt_pclk = 0;
  for(uint32_t i=0;i<samples;i++){
    if(digitalRead(HM01B0_MCLK_PIN)) cnt_mclk++;
    if(digitalRead(HM01B0_VSYNC_PIN)) cnt_vsync++;
    if(digitalRead(HM01B0_HSYNC_PIN)) cnt_hsync++;
    if(digitalRead(HM01B0_PCLK_PIN)) cnt_pclk++;
    delay(step);
  }
  ESP_LOGI(TAG, "%s sample %ums: MCLK=%u/%u VSYNC=%u/%u HSYNC=%u/%u PCLK=%u/%u",
           label, duration_ms,
           (unsigned)cnt_mclk, (unsigned)samples,
           (unsigned)cnt_vsync, (unsigned)samples,
           (unsigned)cnt_hsync, (unsigned)samples,
           (unsigned)cnt_pclk, (unsigned)samples);
}

// Try to program LEDC for given frequency. Returns true on success.
// Use platform API to set MCLK frequency.
static bool set_mclk_freq(uint32_t freq_hz){
  hm01b0_cfg_t cfg = hm01b0_cfg_global;
  return (hm01b0_esp32_set_mclk_freq(&cfg, freq_hz, NULL) == HM01B0_ERR_OK);
}

static void dump_regs(hm01b0_cfg_t* pcfg, uint16_t start, uint16_t count){
  uint8_t val = 0;
  char buf[256];
  int pos = 0;
  pos += snprintf(buf+pos, sizeof(buf)-pos, "reg dump 0x%04X..0x%04X:\n", start, (int)(start+count-1));
  for(uint16_t a = start; a < start+count; a++){
    if(hm01b0_read_reg(pcfg, a, &val, 1) == HM01B0_ERR_OK){
      pos += snprintf(buf+pos, sizeof(buf)-pos, "%04X=%02X ", a, (unsigned)val);
    } else {
      pos += snprintf(buf+pos, sizeof(buf)-pos, "%04X=-- ", a);
    }
    if(pos > (int)sizeof(buf)-40) break;
  }
  ESP_LOGI(TAG, "%s", buf);
}

static void run_mclk_trig_auto_tests(hm01b0_cfg_t* cfg){
  const uint32_t freqs[] = {4000000, 3000000, 1200000, 300000};
  for(size_t i=0;i<sizeof(freqs)/sizeof(freqs[0]);i++){
    set_mclk_freq(freqs[i]);
    sample_signals("auto-mclk", 100);
    // Try short TRIG
    ESP_LOGI(TAG, "Auto-test: TRIG short (10ms)");
    hm01b0_esp32_trig(cfg, true, NULL);
    delay(10);
    hm01b0_esp32_trig(cfg, false, NULL);
    delay(50);
    sample_signals("post-auto-short", 100);
    // Try long TRIG
    ESP_LOGI(TAG, "Auto-test: TRIG long (200ms)");
    hm01b0_esp32_trig(cfg, true, NULL);
    delay(200);
    hm01b0_esp32_trig(cfg, false, NULL);
    delay(100);
    sample_signals("post-auto-long", 100);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  ESP_LOGI(TAG, "HM01B0 init example (ESP32-S3)");

  // --- Power Management Init ---
#if CONFIG_PM_ENABLE
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 80, // Lower to 80MHz or even lower (e.g. 40) if possible, but 80 is safe for WiFi/BLE
      .light_sleep_enable = true
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
  ESP_LOGI(TAG, "Power Management Enabled (Light Sleep Active)");
#endif
  
  // --- GPIO Init ---
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Create Queue and Task
  gpio_cmd_queue = xQueueCreate(10, sizeof(uint8_t));
  xTaskCreate(gpio_control_task, "gpio_control_task", 2048, NULL, 5, NULL);

  // Reduce noise from deprecated-driver warnings
  esp_log_level_set("mcpwm", ESP_LOG_ERROR);
  esp_log_level_set("i2c", ESP_LOG_ERROR);

  // --- Native NimBLE Initialization ---
  ESP_LOGI(TAG, "Initializing Native NimBLE...");
  
  // 1. Initialize NVS (Required by NimBLE)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 2. Initialize NimBLE Port
  // Note: esp_nimble_hci_init() is called internally by nimble_port_init() in some versions,
  // but let's follow the standard pattern.
  // In ESP-IDF v5.x, nimble_port_init() usually handles the controller init if not already done,
  // OR we might need to do it manually if we want custom config.
  // Let's try the standard nimble_port_init() first.
  nimble_port_init();

  // 3. Initialize GATT Server
  ble_svc_gap_init();
  ble_svc_gatt_init();

  // 4. Configure Services
  int rc = ble_gatts_count_cfg(gatt_svr_svcs);
  assert(rc == 0);

  rc = ble_gatts_add_svcs(gatt_svr_svcs);
  assert(rc == 0);

  // 5. Set Sync Callback
  ble_hs_cfg.sync_cb = ble_on_sync;

  // 6. Start Host Task
  nimble_port_freertos_init(ble_host_task);
  
  ESP_LOGI(TAG, "Native NimBLE Initialized");
  // ------------------------------------

  // populate platform interface
  hm01b0_populate_cfg(&hm01b0_cfg_global);

  // set config pointer in a local cfg var
  hm01b0_cfg_t cfg = hm01b0_cfg_global;

  // enable MCLK
  if(hm01b0_esp32_mclk(&cfg, true, NULL) != HM01B0_ERR_OK){
    ESP_LOGE(TAG, "Failed to enable MCLK");
  }

  // init interface (I2C, trigger pin)
  if(hm01b0_esp32_init(&cfg, NULL) != HM01B0_ERR_OK){
    ESP_LOGE(TAG, "Failed to init HM01B0 interface");
  }

  // read model id
  uint16_t model = 0;
  if(hm01b0_get_modelid(&cfg, &model) == HM01B0_ERR_OK){
    ESP_LOGI(TAG, "HM01B0 model id: 0x%04X", (unsigned)model);
  }else{
    ESP_LOGE(TAG, "Failed to read model id");
  }

  // load script
  hm01b0_status_e init_st = hm01b0_init_system(&cfg, (hm_script_t*)sHM01B0InitScript, sHM01B0InitScriptSize);
  if(init_st != HM01B0_ERR_OK){
    ESP_LOGW(TAG, "hm01b0_init_system returned %d", (int)init_st);
  } else {
    ESP_LOGI(TAG, "hm01b0_init_system completed OK");
  }

  // --- REORDERED INIT SEQUENCE FOR SYNC ---
  // 1. De-init manual I2C (handover to driver)
  hm01b0_esp32_deinit(&hm01b0_cfg_global, NULL);

  // 2. Init Camera Driver (Arms DMA, waits for VSYNC)
  used_camera_driver = false;
  ESP_LOGI(TAG, "Initializing camera driver (expecting real esp32-camera driver)...");
  
  if(hm01b0_camera_init(&hm01b0_cfg_global, NULL) == HM01B0_ERR_OK){
    ESP_LOGI(TAG, "esp_camera initialized - using camera driver capture");
    used_camera_driver = true;
    
    // 3. Pulse TRIG to start streaming (Generates VSYNC -> Triggers DMA)
    ESP_LOGI(TAG, "Pulsing TRIG to start streaming (Syncing DMA)...");
    hm01b0_esp32_trig(&cfg, true, NULL);
    delay(10);
    hm01b0_esp32_trig(&cfg, false, NULL);
    delay(50);

    // 4. Configure Registers via Driver (Test Pattern, etc.)
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_reg) {
        // Ensure Streaming is Enabled (0x0100 = 0x01)
        s->set_reg(s, 0x0100, 0xFF, 0x01);
        
        // Disable Test Pattern (Real Image)
        ESP_LOGI(TAG, "Configuring for Real Image (Test Pattern Disabled)");
        s->set_reg(s, 0x0601, 0xFF, 0x00); 
        
        delay(100); 
    }
  } else {
    ESP_LOGW(TAG, "esp_camera init failed!");
  }
  
  // Allocate buffer for BLE
  frame_size = (size_t)HM01B0_PIXEL_X_NUM * (size_t)HM01B0_PIXEL_Y_NUM;
  frame_buf = (uint8_t*)malloc(frame_size);
  if(frame_buf == NULL){
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    return;
  }

  // Init SHTC3 I2C
  init_shtc3_i2c();

  // Start SHTC3 Task
  xTaskCreate(shtc3_task, "shtc3_task", 4096, NULL, 5, NULL);
}

// Global exposure index for cycling
int exposure_idx = 0;
const uint8_t exposure_values[] = {0x02, 0x10, 0x40, 0x80};

void loop() {
  if (deviceConnected && captureRequested) {
      captureRequested = false;
      hm01b0_cfg_t cfg = hm01b0_cfg_global;
      hm01b0_status_e ret;
      uint32_t actual_len = frame_size;
      
      // --- Cycle Exposure Settings ---
      // DISABLED: Cycling caused "bad data" (noise) at low exposure.
      // Fixed Exposure: 0x80 (Medium-Long) for better indoor quality.
      sensor_t *s = esp_camera_sensor_get();
      if (s && s->set_reg) {
          // uint8_t new_exp = exposure_values[exposure_idx];
          // uint8_t new_exp = 0x80; // Fixed value
          // ESP_LOGI(TAG, "Fixed Exposure: Setting Integration Time to 0x%02X", new_exp);
          
          // Stop Streaming (Standby)
          // s->set_reg(s, 0x0100, 0xFF, 0x00); 
          // delay(10);

          // Update Exposure
          // s->set_reg(s, 0x0202, 0xFF, 0x00);
          // s->set_reg(s, 0x0203, 0xFF, new_exp);
          
          // Ensure Windowing is set (Fix for Diagonal Lines)
          // s->set_reg(s, 0x3010, 0xFF, 0x01); // Force QVGA Window

          // Restart Streaming
          // s->set_reg(s, 0x0100, 0xFF, 0x01);
          // delay(100); // Wait for settle

          // exposure_idx = (exposure_idx + 1) % 4;
      }
      // -------------------------------

      ESP_LOGI(TAG, "Capturing frame for BLE...");
      
      // Clear buffer to ensure no "bad data" / artifacts from previous runs
      memset(frame_buf, 0, frame_size);

      // Retry loop to ensure we get a valid frame of sufficient size
      // This filters out partial/garbage frames that sometimes occur on startup
      int retry_count = 0;
      const int max_retries = 5;
      const uint32_t min_expected_len = 10000; // Adjusted for QQVGA (19200 bytes)

      while (retry_count < max_retries) {
          if(used_camera_driver){
            ret = hm01b0_camera_capture_frame(&cfg, frame_buf, frame_size, &actual_len, 5000);
          } else {
            ret = hm01b0_esp32_read_frame(&cfg, frame_buf, frame_size, 5000);
            actual_len = frame_size;
          }

          if (ret == HM01B0_ERR_OK && actual_len > min_expected_len) {
              ESP_LOGI(TAG, "Valid frame captured: %u bytes", (unsigned)actual_len);
              break; // Success
          }

          ESP_LOGW(TAG, "Invalid frame (len=%u), retrying (%d/%d)...", (unsigned)actual_len, retry_count + 1, max_retries);
          retry_count++;
          delay(100); // Give the sensor/DMA time to recover
      }

      if(ret == HM01B0_ERR_OK){
        ESP_LOGI(TAG, "Frame captured. Sending %u bytes via BLE...", (unsigned)actual_len);
        
        // Dump first 16 bytes to check for saturation/noise
        if(actual_len >= 16) {
            ESP_LOGI(TAG, "First 16 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                frame_buf[0], frame_buf[1], frame_buf[2], frame_buf[3],
                frame_buf[4], frame_buf[5], frame_buf[6], frame_buf[7],
                frame_buf[8], frame_buf[9], frame_buf[10], frame_buf[11],
                frame_buf[12], frame_buf[13], frame_buf[14], frame_buf[15]);
        }

        // Send in chunks with Offset Header (Robust against packet loss)
        // Packet Format: [Offset_Low, Offset_High, Data...]
        // This allows the client to place data at the correct pixel location even if packets are dropped.
        
        // Add a small delay before starting transmission to ensure connection is stable
        delay(20);

        // Dynamic Chunk Size based on MTU
        // MTU = Attribute Opcode (1) + Attribute Handle (2) + Data
        // So Data Space = MTU - 3
        // We use 2 bytes for our Offset Header -> Real Data = MTU - 5
        uint16_t mtu = ble_att_mtu(conn_handle);
        size_t maxDataSize = (mtu > 20) ? (mtu - 5) : 20;
        
        // Ensure we don't exceed our buffer or reasonable limits
        if (maxDataSize > 490) maxDataSize = 490; 
        
        ESP_LOGI(TAG, "MTU: %d, Chunk Size: %u", mtu, (unsigned)maxDataSize);

        size_t i = 0;
        bool abort_frame = false;

        while (i < actual_len && !abort_frame) {
            size_t len = actual_len - i;
            if (len > maxDataSize) len = maxDataSize;
            
            // Prepare packet with header
            uint8_t packet_buf[512]; // Increased buffer size
            packet_buf[0] = (uint8_t)(i & 0xFF);        // Offset Low
            packet_buf[1] = (uint8_t)((i >> 8) & 0xFF); // Offset High
            memcpy(&packet_buf[2], &frame_buf[i], len);

            bool sent = false;
            // Retry loop for each chunk
            const int max_chunk_retries = 20;
            for(int retry=0; retry < max_chunk_retries; retry++){
                // Create mbuf from our packet_buf (header + data)
                struct os_mbuf *om = ble_hs_mbuf_from_flat(packet_buf, len + 2);
                if (om) {
                    int rc = ble_gatts_notify_custom(conn_handle, gatt_svr_chr_val_handle, om);
                    if (rc == 0) {
                        sent = true;
                        break; // Success
                    } else {
                        if (rc == BLE_HS_ENOTCONN) {
                            ESP_LOGW(TAG, "Device disconnected during transfer. Aborting.");
                            abort_frame = true;
                            os_mbuf_free_chain(om);
                            break;
                        }
                        // Log warning only on later retries to reduce noise
                        if (retry > 0) {
                            ESP_LOGW(TAG, "Notify failed rc=%d, retrying (%d/%d)...", rc, retry+1, max_chunk_retries);
                        }
                        os_mbuf_free_chain(om); // Free failed mbuf
                        delay(10); // Wait before retry (reduced from 30)
                    }
                } else {
                    ESP_LOGW(TAG, "mbuf alloc failed, retrying...");
                    delay(10);
                }
            }
            
            if(sent){
                i += len; // Advance by data length (not packet length)
                delay(15); // Inter-packet delay (Increased to 15ms to prevent WDT starvation)
            } else {
                if (!abort_frame) {
                    ESP_LOGE(TAG, "Failed to send chunk at offset %u after %d retries. Aborting frame.", (unsigned)i, max_chunk_retries);
                    abort_frame = true;
                }
            }
        }
        
        if (abort_frame) {
             ESP_LOGW(TAG, "Frame transmission aborted.");
        } else {
             ESP_LOGI(TAG, "Frame sent successfully.");
        }

      } else {
        ESP_LOGW(TAG, "Frame capture failed");
      }
      delay(1000); // Wait before next capture
  } else {
      delay(500);
  }
}
