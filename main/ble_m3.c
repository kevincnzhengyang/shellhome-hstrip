/*
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2024-11-20 21:10:06
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2024-11-23 19:21:45
 * @FilePath    : /shellhome-hstrip/main/ble_m3.c
 * @Description : ble remote panel
 * Copyright (c) 2024 by Zheng, Yang, All Rights Reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_bt.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
// 2a:07:98:04:24:05
// 2a:07:98:02:c1:13
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include "ble_m3.h"
#include "node_status.h"

#define BLE_M3_KEY_BUTT     0
#define BLE_M3_KEY_LEFT     (1<<0)
#define BLE_M3_KEY_UP       (1<<1)
#define BLE_M3_KEY_RIGHT    (1<<2)
#define BLE_M3_KEY_DOWN     (1<<3)
#define BLE_M3_KEY_LIKE     (1<<4)
#define BLE_M3_KEY_TAKE     (1<<5)

extern EventGroupHandle_t g_event_group;

typedef uint8_t ble_m3_pdu_t[3];

typedef enum {
    BLE_M3_INIT,
    BLE_M3_SIGNAL,
    BLE_M3_CONFIRM,
    BLE_M3_BUTT
} BLE_M3_STATUS;

typedef struct {
    BLE_M3_STATUS       status;
    uint8_t            key_set;
    uint8_t             loaded;
    uint8_t             forgot;
    nvs_handle_t    nvs_handle;
    esp_bd_addr_t    saved_bda;
    esp_hid_scan_result_t   m3;
    esp_hidh_dev_t        *dev;
} ble_m3_cb;

static ble_m3_pdu_t m3_sep_3     = {0x01, 0x18, 0x80};
static ble_m3_pdu_t m3_sep_4     = {0xff, 0x17, 0x80};
static ble_m3_pdu_t m3_sep_56    = {0x01, 0xf8, 0x7f};

static ble_m3_pdu_t m3_signal_1  = {0x28, 0x80, 0x11};
static ble_m3_pdu_t m3_signal_2  = {0x3c, 0x80, 0x0c};
static ble_m3_pdu_t m3_signal_4  = {0xae, 0x8f, 0x11};
static ble_m3_pdu_t m3_signal_8  = {0x3c, 0x40, 0xec};
static ble_m3_pdu_t m3_signal_16 = {0xa0, 0x30, 0xe8};
static ble_m3_pdu_t m3_signal_32 = {0xaa, 0x70, 0xf8};

static ble_m3_pdu_t m3_confirm   = {0x01, 0x00, 0x00};     // mouse


static ble_m3_cb g_cb;

#define M3_KEY_SET_INIT     63

#define M3_IS_PDU(pdu1, pdu2) \
    ((pdu1[0] == pdu2[0]) && (pdu1[1] == pdu2[1]) && (pdu1[2] == pdu2[2]))

#define M3_IS_SEP_3(pdu) (((g_cb.key_set & 3) > 0) && M3_IS_PDU(pdu, m3_sep_3))
#define M3_IS_SEP_4(pdu) (((g_cb.key_set & 4) > 0) && M3_IS_PDU(pdu, m3_sep_4))
#define M3_IS_SEP_56(pdu) (((g_cb.key_set & 56) > 0) && M3_IS_PDU(pdu, m3_sep_56))
#define M3_IS_SEP(pdu) (M3_IS_SEP_3(pdu) || M3_IS_SEP_4(pdu) || M3_IS_SEP_56(pdu))

#define M3_IS_SIGNAL_1(pdu)  (((g_cb.key_set & 1) > 0) && M3_IS_PDU(pdu, m3_signal_1))
#define M3_IS_SIGNAL_2(pdu)  (((g_cb.key_set & 2) > 0) && M3_IS_PDU(pdu, m3_signal_2))
#define M3_IS_SIGNAL_4(pdu)  (((g_cb.key_set & 4) > 0) && M3_IS_PDU(pdu, m3_signal_4))
#define M3_IS_SIGNAL_8(pdu)  (((g_cb.key_set & 8) > 0) && M3_IS_PDU(pdu, m3_signal_8))
#define M3_IS_SIGNAL_16(pdu) (((g_cb.key_set & 16) > 0) && M3_IS_PDU(pdu, m3_signal_16))
#define M3_IS_SIGNAL_32(pdu) (((g_cb.key_set & 32) > 0) && M3_IS_PDU(pdu, m3_signal_32))

#define M3_IS_CONFIRM(pdu) M3_IS_PDU(pdu, m3_confirm)

static const char *TAG = "BLE_M3";

static esp_err_t load_m3_from_nvs(void) {
    esp_err_t err;
    size_t required_size = sizeof(esp_bd_addr_t);

    if (pdTRUE == g_cb.loaded) return ESP_OK;

    err = nvs_get_blob(g_cb.nvs_handle, TAG, &(g_cb.saved_bda), &required_size);
    if (ESP_OK == err) {
        ESP_LOGI(TAG, "load from NVS");
        ESP_LOGI(TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(g_cb.saved_bda));
        g_cb.loaded = pdTRUE;
        g_cb.forgot = pdFALSE;
    } else if (ESP_ERR_NVS_NOT_FOUND == err) {
        ESP_LOGI(TAG, "not found in NVS");
        g_cb.loaded = pdFALSE;
        g_cb.forgot = pdTRUE;
    }
    return err;
}

static esp_err_t init_input_generic(void) {
    memset(&g_cb, 0, sizeof (ble_m3_cb));

    g_cb.status = BLE_M3_BUTT;
    g_cb.key_set = M3_KEY_SET_INIT;

    esp_err_t err;

    // Open NVS
    err = nvs_open("ShellHome", NVS_READWRITE, &g_cb.nvs_handle);
    ESP_ERROR_CHECK(err);

    err = load_m3_from_nvs();

    if (ESP_ERR_NVS_NOT_FOUND != err && ESP_OK != err) return err;
    return ESP_OK;
}

static esp_err_t check_input_generic(uint8_t *data, uint8_t* key) {
    if (NULL == data || NULL == key) return ESP_FAIL;

    if (M3_IS_SEP(data)) {
        /* reset and init */
        g_cb.status = BLE_M3_SIGNAL;
        g_cb.key_set = (m3_sep_4[0] == data[0]) ? 4 : \
                    ((m3_sep_3[1] == data[1] ? 3 : 56));
        ESP_LOGI(TAG, "reset status");
        goto Yield;
    } else {
        if (BLE_M3_SIGNAL == g_cb.status) {
            if (M3_IS_SIGNAL_1(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_LEFT;
                goto Yield;
            } else if (M3_IS_SIGNAL_2(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_UP;
                goto Yield;
            } else if (M3_IS_SIGNAL_4(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_RIGHT;
                goto Yield;
            } else if (M3_IS_SIGNAL_8(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_DOWN;
                goto Yield;
            } else if (M3_IS_SIGNAL_16(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_LIKE;
                goto Yield;
            } else if (M3_IS_SIGNAL_32(data)) {
                g_cb.status = BLE_M3_CONFIRM;
                g_cb.key_set = BLE_M3_KEY_TAKE;
                goto Yield;
            } else {
                goto Wrong;
            }
        } else if (BLE_M3_CONFIRM == g_cb.status) {
            if (M3_IS_CONFIRM(data)) {
                g_cb.status = BLE_M3_INIT;
                ESP_LOGI(TAG, "got signal, set bits to %d", (int)g_cb.key_set);
                goto Detected;
            } else {
                goto Wrong;
            }
        }
    }

Wrong:
    g_cb.key_set = M3_KEY_SET_INIT;
    g_cb.status = BLE_M3_BUTT;
    ESP_LOGD(TAG, "wrong pdu, set bits to %d", (int)g_cb.key_set);
    *key = BLE_M3_KEY_BUTT;
    return ESP_FAIL;

Yield:
    *key = BLE_M3_KEY_BUTT;
    return ESP_FAIL;

Detected:
    *key = g_cb.key_set;
    ESP_LOGI(TAG, "find the key %d", (int)g_cb.key_set);
    g_cb.key_set = M3_KEY_SET_INIT;
    return ESP_OK;
}


static char *bda2str(esp_bd_addr_t bda, char *str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void hidh_callback(void *handler_args, esp_event_base_t base,
                          int32_t id, void *event_data) {
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
    uint8_t input_key = BLE_M3_KEY_BUTT;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
            shn_state_status(NODE_RUNNING);
            xEventGroupSetBits(g_event_group, BLE_M3_CONN_BIT);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        ESP_LOGD(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        if (5 == param->input.report_id || 4 == param->input.report_id) {
            if (ESP_OK == check_input_generic(param->input.data, &input_key)) {
                ESP_LOGI(TAG, " host get key %d", (int)input_key);
                switch (input_key) {
                    case BLE_M3_KEY_LEFT:
                        xEventGroupSetBits(g_event_group, BLE_M3_PREV_BIT);
                        break;
                    case BLE_M3_KEY_UP:
                        xEventGroupSetBits(g_event_group, BLE_M3_MORE_BIT);
                        break;
                    case BLE_M3_KEY_RIGHT:
                        xEventGroupSetBits(g_event_group, BLE_M3_NEXT_BIT);
                        break;
                    case BLE_M3_KEY_DOWN:
                        xEventGroupSetBits(g_event_group, BLE_M3_LESS_BIT);
                        break;
                    case BLE_M3_KEY_LIKE:
                        xEventGroupSetBits(g_event_group, BLE_M3_SAVE_BIT);
                        break;
                    case BLE_M3_KEY_TAKE:
                        xEventGroupSetBits(g_event_group, BLE_M3_SWITCH_BIT);
                        break;
                    default:
                        ESP_LOGE(TAG, "Unknown key %d", (int)input_key);
                        break;
                }
            }
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        esp_hidh_dev_close(g_cb.dev);
        xEventGroupSetBits(g_event_group, BLE_M3_LOST_BIT);
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

static esp_err_t scan_ble_m3(void) {
    esp_err_t ret = ESP_FAIL;
    esp_err_t err = ESP_FAIL;

    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    size_t required_size = sizeof(esp_bd_addr_t);

    ESP_LOGI(TAG, "SCAN...");
    shn_state_status(NODE_SCANNING);

    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            // save the result into nvs
            if (pdTRUE == g_cb.forgot) {
                ESP_LOGI(TAG, "save the last result");
                err = nvs_set_blob(g_cb.nvs_handle, TAG, &cr->bda, required_size);
                if (ESP_OK != err) {
                    ESP_LOGI(TAG, "failed to save BLE M3 to nvs %d", err);
                    ret = err;
                } else {
                    nvs_commit(g_cb.nvs_handle);
                    g_cb.forgot = pdFALSE;
                    g_cb.loaded = pdTRUE;
                    ESP_LOGI(TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(cr->bda));
                    memcpy(&g_cb.m3, cr, sizeof(esp_hid_scan_result_t));
                    memcpy(&g_cb.saved_bda, &cr->bda, sizeof(esp_bd_addr_t));
                    ret = ESP_OK;
                }
            } else {
                // check if it's the saved one
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " <-> " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(g_cb.saved_bda), ESP_BD_ADDR_HEX(cr->bda));
                if (0 == memcmp(&cr->bda, &g_cb.saved_bda, sizeof(esp_bd_addr_t))) {
                    memcpy(&g_cb.m3, cr, sizeof(esp_hid_scan_result_t));
                    ESP_LOGI(TAG, "find the correct BLE-M3");
                    ret = ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Wrong BLE-M3");
                    ret = ESP_FAIL;
                }
            }
        }
        ESP_LOGI(TAG, "free results");
        //free the results
        esp_hid_scan_results_free(results);
    }
    return ret;
}

static void hid_m3_task(void *pvParameters)
{
    while (1) {
        if (pdFALSE == g_cb.loaded) load_m3_from_nvs();

        // scan again
        if (ESP_OK != scan_ble_m3()) {
            ESP_LOGW(TAG, "can't find BLE M3");
            vTaskDelay(10000/portTICK_PERIOD_MS);
            continue;
        } else {
            // open device
            g_cb.dev = esp_hidh_dev_open(g_cb.m3.bda, g_cb.m3.transport, g_cb.m3.ble.addr_type);

            while (1) {
                EventBits_t bits = xEventGroupWaitBits(g_event_group,
                                        BLE_M3_FORGET_BIT|BLE_M3_LOST_BIT,
                                        pdTRUE, pdFAIL, portMAX_DELAY);
                if (bits & BLE_M3_FORGET_BIT) {
                    // clear NVS
                    g_cb.loaded = pdFALSE;
                    g_cb.forgot = pdTRUE;
                    nvs_erase_key(g_cb.nvs_handle, TAG);
                    nvs_commit(g_cb.nvs_handle);
                    break;  // connect & scan if in need
                } else if (bits & BLE_M3_LOST_BIT) {
                    // lost M3, try to scan again
                    break;
                } else {
                    continue;   // wait event
                }
            }
        }
    }
    ESP_LOGI(TAG, "task closing");
    nvs_close(g_cb.nvs_handle);
    vTaskDelete(NULL);
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif

/* start BT HID Host */
esp_err_t ble_m3_host_start(void) {
    char bda_str[18] = {0};

    init_input_generic();
    shn_state_status(NODE_POWER_ON);    // Power On


#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#if CONFIG_BT_NIMBLE_ENABLED
    esp_err_t ret;

    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	/* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }
#endif
    xTaskCreate(&hid_m3_task, "hid_task", 6 * 1024, NULL, 2, NULL);
    return ESP_OK;
}
