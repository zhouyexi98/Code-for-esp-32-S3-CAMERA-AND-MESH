// Simple one-shot capture test to verify HM01B0 parallel capture path.
#include "hm01b0.h"
#include "hm01b0_esp32.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "hm01b0_test";

extern "C" void app_main(void){
    ESP_LOGI(TAG, "Starting HM01B0 capture test");
    hm01b0_cfg_t cfg = {0};
    hm01b0_populate_cfg(&cfg);
    if(hm01b0_esp32_init(&cfg, NULL) != HM01B0_ERR_OK){
        ESP_LOGE(TAG, "Platform init failed");
        return;
    }
    if(hm01b0_esp32_mclk(&cfg, true, NULL) != HM01B0_ERR_OK){
        ESP_LOGW(TAG, "Could not enable MCLK");
    }

    size_t expected = (size_t)HM01B0_PIXEL_X_NUM * (size_t)HM01B0_PIXEL_Y_NUM;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(expected, MALLOC_CAP_DEFAULT);
    if(!buf){ ESP_LOGE(TAG, "malloc failed"); return; }

    ESP_LOGI(TAG, "Attempting single frame capture (timeout 5000ms)");
    hm01b0_status_e st = hm01b0_esp32_read_frame(&cfg, buf, expected, 5000);
    if(st == HM01B0_ERR_OK){
        ESP_LOGI(TAG, "Capture OK - first 64 bytes:");
        char line[256];
        int off = 0;
        int toprint = (expected < 64) ? expected : 64;
        for(int i=0;i<toprint;i++){
            off += snprintf(line + off, sizeof(line) - off, "%02X ", buf[i]);
            if((i & 0x0F) == 0x0F){ ESP_LOGI(TAG, "%s", line); off = 0; }
        }
        if(off) ESP_LOGI(TAG, "%s", line);
    } else {
        ESP_LOGW(TAG, "Capture failed (status=%d)", (int)st);
    }

    if(buf) heap_caps_free(buf);
    hm01b0_esp32_deinit(&cfg, NULL);
    ESP_LOGI(TAG, "Capture test complete");
    // stop here - do not run the rest of the app to avoid conflicts
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}
