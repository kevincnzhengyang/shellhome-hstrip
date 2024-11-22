/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "node_status.h"
#include "ble_m3.h"
#include "sh_led_strip.h"

EventGroupHandle_t g_event_group;

void app_main(void)
{
    esp_err_t ret;

    g_event_group = xEventGroupCreate();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // init Status
    shn_init_status();

    // start LED Strip
    ESP_ERROR_CHECK(led_strip_start());

    // start HID Host for BLE M3
    ESP_ERROR_CHECK(ble_m3_host_start());

    // flush status LED
    while (1) {
        shn_flush_status();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

}
