#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_nimble_gap.h"
#include "esp_nimble_hci.h"   // HCI interface
#include "esp_gatts_api.h"
#include "esp_hidd_api.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define TAG "HID_LED"

// GPIO and LED Configuration
#define LED_STRIP_GPIO 5          // GPIO pin for RGB LED strip
#define NUM_LEDS 16               // Number of LEDs on the strip

led_strip_handle_t led_strip;     // LED strip handle

// HID Descriptor
static const uint8_t hid_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        // Usage Page (Key Codes)
    0x19, 0xE0,        // Usage Minimum (224)
    0x29, 0xE7,        // Usage Maximum (231)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data, Var, Abs)
    0x95, 0x01,        // Report Count (1)
    0x75, 0x08,        // Report Size (8)
    0x81, 0x01,        // Input (Const)
    0x19, 0x00,        // Usage Minimum (0)
    0x29, 0x65,        // Usage Maximum (101)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x65,        // Logical Maximum (101)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x06,        // Report Count (6)
    0x81, 0x00,        // Input (Data, Array)
    0xC0               // End Collection
};

// HID Event Handler
static void hid_event_handler(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    switch (event) {
        case ESP_HIDD_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "HID device connected");
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "HID device disconnected");
            break;
        case ESP_HIDD_EVENT_BLE_REPORT_WRITE_EVT:
            ESP_LOGI(TAG, "Received input report");
            if (param->report_write.len > 0) {
                uint8_t key = param->report_write.data[0];
                ESP_LOGI(TAG, "Key pressed: 0x%02X", key);

                // Control LED colors based on key press
                if (key == 0x04) {  // Key 'A'
                    led_strip_set_pixel(led_strip, 0, 255, 0, 0);  // Red
                } else if (key == 0x05) {  // Key 'B'
                    led_strip_set_pixel(led_strip, 0, 0, 255, 0);  // Green
                } else if (key == 0x06) {  // Key 'C'
                    led_strip_set_pixel(led_strip, 0, 0, 0, 255);  // Blue
                } else {
                    led_strip_clear(led_strip);  // Turn off LEDs
                }
                led_strip_refresh(led_strip);
            }
            break;
        default:
            break;
    }
}

// GAP Event Handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Failed to start advertising");
            }
            break;
        default:
            break;
    }
}

// Main Application
void app_main() {
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize the Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize and enable Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register GAP and HID handlers
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hid_event_handler));

    // Start HID service
    ESP_ERROR_CHECK(esp_hidd_dev_init(&hid_descriptor, sizeof(hid_descriptor)));

    // Initialize LED Strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = NUM_LEDS,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));

    // Start BLE Advertising
    uint8_t adv_data[] = { 0x02, 0x01, 0x06, 0x03, 0x03, 0x12, 0x18 };
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data)));

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
}
