/*
 * SPDX-FileCopyrightText: 2017-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_mac.h"
#include "driver/gpio.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEVICE_ID 3 /* TODO: Change this to 1, 2, or 3 for each device */

#define PIN_IO1 1
#define PIN_IO2 2
#define PIN_IO5 5
#define PIN_IO6 6
#define PIN_IO7 7
#define PIN_IO9 9

// Current Target Range: 0=Off, 1=Level1, 2=Level2, 3=Level3, 4=AllLow
static volatile int g_target_range = 0; 

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_0, &level_server_0),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

/* Disable OOB security for this example */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 0,
    .output_actions = 0,
#endif
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
}

static void set_all_low(void)
{
    gpio_set_level(PIN_IO1, 0);
    gpio_set_level(PIN_IO2, 0);
    gpio_set_level(PIN_IO5, 0);
    gpio_set_level(PIN_IO6, 0);
    gpio_set_level(PIN_IO7, 0);
    gpio_set_level(PIN_IO9, 0);
}

// Helper to execute the LED sequence: First High -> Wait -> Second High -> Wait
// This runs in the task context, so vTaskDelay is safe.
static void run_sequence_task_context(int pin_first, int pin_second)
{
    // Step 1: All Low
    set_all_low();
    
    // Step 2: First Pin High
    gpio_set_level(pin_first, 1);
    
    // Step 3: Wait 0.4s
    vTaskDelay(pdMS_TO_TICKS(400));
    
    // Step 4: First Pin Low, Second Pin High
    set_all_low(); // turn off pin_first
    gpio_set_level(pin_second, 1);
}

static void led_control_task(void *arg)
{
    int last_range = -1; // Force update on start

    while (1) {
        int current_range = g_target_range;

        // On state change, perform the action once
        if (current_range != last_range) {
            ESP_LOGI(TAG, "LED Task: Range Changed %d -> %d", last_range, current_range);
            
            // "Debounce" handling:
            // When user slides, we might get many changes quickly. 
            // We could add a small delay here to wait for value to settle, if needed.
            // vTaskDelay(pdMS_TO_TICKS(50));
            // current_range = g_target_range; 

            switch (current_range) {
            case 0: // ALL Off
                set_all_low();
                break;
            
            case 1: // Level 1 (1-8000)
                // All devices: IO7 -> IO9
               run_sequence_task_context(PIN_IO7, PIN_IO9);
             /*if (DEVICE_ID == 1) {
                    run_sequence_task_context(PIN_IO7, PIN_IO9);
                   // run_sequence_task_context(PIN_IO1, PIN_IO2);
                } else if (DEVICE_ID == 2) {
                    run_sequence_task_context(PIN_IO1, PIN_IO2);
                } else if (DEVICE_ID == 3) {
                    run_sequence_task_context(PIN_IO5, PIN_IO6);
                }*/
                break;

            case 2: // Level 2 (8001-16000)
                if (DEVICE_ID == 1) {
                   // set_all_low();
                    //run_sequence_task_context(PIN_IO7, PIN_IO9);
                   run_sequence_task_context(PIN_IO1, PIN_IO2); 
                } else if (DEVICE_ID == 2) {
                    run_sequence_task_context(PIN_IO7, PIN_IO9);
                } else if (DEVICE_ID == 3) {
                    run_sequence_task_context(PIN_IO5, PIN_IO6);
                    //run_sequence_task_context(PIN_IO7, PIN_IO9);
                }
                break;

            case 3: // Level 3 (16001-24000)
                if (DEVICE_ID == 1) {
                    run_sequence_task_context(PIN_IO5, PIN_IO6);
                     //set_all_low(); 
                } else if (DEVICE_ID == 2) {
                    run_sequence_task_context(PIN_IO1, PIN_IO2);
                   // set_all_low();
                } else if (DEVICE_ID == 3) {
                    run_sequence_task_context(PIN_IO7, PIN_IO9);
                }
                break;

            case 4: // Level 4 (>24000) - Explicit All Low
                set_all_low();
                break;
                
            default:
                set_all_low();
                break;
            }

            last_range = current_range;
        }

        // Loop delay to yield processor
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void update_led_state(int16_t level)
{
    ESP_LOGI(TAG, "Received Level: %d, Device ID: %d", level, DEVICE_ID);

    int new_range = 0;

    // Generic Level is int16_t (-32768 to 32767).
    // The App Slider likely maps 0% -> -32768 and 100% -> 32767.
    // We need to map this full range to our Logic Levels.
    
    // Map -32768 (Min) to -20000 -> Level 1 (Low ~25%)
    // Map -20000 to -5000        -> Level 2 (Mid-Low ~50%)
    // Map -5000  to 10000        -> Level 3 (Mid-High ~70%)
    // Map > 10000                -> Level 4 (High ~100%, Reset)
    // Values close to -32768 (e.g., < -32000) treated as OFF.

    if (level <= -32000) {
        new_range = 0; // OFF
    }
    else if (level <= -10000) {
        new_range = 1; // Approx 0% - 33% on visualized slider (excluding OFF)
    }
    else if (level <= 10000) {
        new_range = 2; // Approx 33% - 66%
    }
    else if (level <= 25000) {
        new_range = 3; // Approx 66% - 90%
    }
    else {
        new_range = 4; // > 90% -> All Low
    }
    
    // Debug logic mapping
    ESP_LOGI(TAG, "Mapped Level %d to Range %d", level, new_range);

    // Just update the global variable. The Task will handle the heavy lifting.
    g_target_range = new_range;
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
            ESP_LOGI(TAG, "level %d", param->value.state_change.level_set.level);
            update_led_state(param->value.state_change.level_set.level);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
            esp_ble_mesh_gen_level_srv_t *srv = param->model->user_data;
            ESP_LOGI(TAG, "level %d", srv->state.level);
            esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(srv->state.level), (uint8_t *)&srv->state.level);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
            ESP_LOGI(TAG, "level %d", param->value.set.level.level);
            if (param->value.set.level.op_en == false) {
                esp_ble_mesh_gen_level_srv_t *srv = param->model->user_data;
                srv->state.level = param->value.set.level.level;
                update_led_state(srv->state.level);
            }
            /* If op_en is true, the state change event will be triggered */
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(dev_uuid, mac, 6);

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* esp_bt_controller_mem_release is not needed for ESP32-S3 (BLE only) */
    /*
    err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err) {
        ESP_LOGE(TAG, "esp_bt_controller_mem_release failed (err %d)", err);
        return;
    }
    */

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err) {
        ESP_LOGE(TAG, "Bluetooth controller init failed (err %d)", err);
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed (err %d)", err);
        return;
    }
    
    err = esp_bluedroid_init();
    if (err) {
        ESP_LOGE(TAG, "Bluedroid init failed (err %d)", err);
        return;
    }

    err = esp_bluedroid_enable();
    if (err) {
        ESP_LOGE(TAG, "Bluedroid enable failed (err %d)", err);
        return;
    }

    // Initialize all IOs as Output for ALL devices
    gpio_reset_pin(PIN_IO1);
    gpio_set_direction(PIN_IO1, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO1, 0);

    gpio_reset_pin(PIN_IO2);
    gpio_set_direction(PIN_IO2, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO2, 0);

    gpio_reset_pin(PIN_IO5);
    gpio_set_direction(PIN_IO5, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO5, 0);

    gpio_reset_pin(PIN_IO6);
    gpio_set_direction(PIN_IO6, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO6, 0);

    gpio_reset_pin(PIN_IO7);
    gpio_set_direction(PIN_IO7, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO7, 0);

    gpio_reset_pin(PIN_IO9);
    gpio_set_direction(PIN_IO9, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IO9, 0);

    ESP_LOGI(TAG, "Initialized IO1, IO2, IO5, IO6, IO7, IO9 as Outputs");

    // Create the LED control task
    xTaskCreate(led_control_task, "led_task", 2048, NULL, 5, NULL);

    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "BLE Mesh init failed (err %d)", err);
        return;
    }
}
