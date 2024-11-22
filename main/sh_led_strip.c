/*
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2023-09-24 23:05:20
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2024-11-22 19:52:00
 * @FilePath    : /shellhome-hstrip/main/sh_led_strip.c
 * @Description :
 * Copyright (c) 2023 by Zheng, Yang, All Rights Reserved.
 */

#include "led_strip.h"

#include "sh_led_strip.h"
#include "ble_m3.h"

/**
 * @brief sin() function from 0 to 2π, in total 255 values rounded up，maximum 255，minimum 0
 *
 */
static uint8_t  const SinValue[256]={	128,   131,   134,   137,   140,   143,   147,   150,   153,   156,
                                159,   162,   165,   168,   171,   174,   177,   180,   182,   185,
                                188,   191,   194,   196,   199,   201,   204,   206,   209,   211,
                                214,   216,   218,   220,   223,   225,   227,   229,   230,   232,
                                234,   236,   237,   239,   240,   242,   243,   245,   246,   247,
                                248,   249,   250,   251,   252,   252,   253,   253,   254,   254,
                                255,   255,   255,   255,   255,   255,   255,   255,   255,   254,
                                254,   253,   253,   252,   251,   250,   249,   249,   247,   246,
                                245,   244,   243,   241,   240,   238,   237,   235,   233,   231,
                                229,   228,   226,   224,   221,   219,   217,   215,   212,   210,
                                208,   205,   203,   200,   198,   195,   192,   189,   187,   184,
                                181,   178,   175,   172,   169,   166,   163,   160,   157,   154,
                                151,   148,   145,   142,   139,   136,   132,   129,   126,   123,
                                120,   117,   114,   111,   107,   104,   101,    98,    95,    92,
                                89,    86,    83,    80,    77,    74,    72,    69,    66,    63,
                                61,    58,    55,    53,    50,    48,    45,    43,    41,    38,
                                36,    34,    32,    30,    28,    26,    24,    22,    21,    19,
                                17,    16,    14,    13,    12,    10,     9,     8,     7,     6,
                                5,     4,     4,     3,     2,     2,     1,     1,     1,     0,
                                0,     0,     0,     0,     1,     1,     1,     2,     2,     3,
                                3,     4,     5,     6,     6,     7,     9,    10,    11,    12,
                                14,    15,    17,    18,    20,    21,    23,    25,    27,    29,
                                31,    33,    35,    37,    40,    42,    44,    47,    49,    52,
                                54,    57,    59,    62,    65,    67,    70,    73,    76,    79,
                                82,    85,    88,    91,    94,    97,   100,   103,   106,   109,
                                112,   115,   118,   121,   125,   128
};

// LED Strip
typedef struct {
    led_strip_handle_t    led_strip1;       // LED Strip1
    led_strip_handle_t    led_strip2;       // LED Strip2
    esp_timer_handle_t  timer_handle;       // timer handler for strip2
    nvs_handle_t          nvs_handle;
    bool                     running;       // marquee or breath running flag
    uint8_t                    count;       // count for marquee or breath
    uint32_t                  rgb[3];       // RGB of LED Strip1
} LED_Strip_Stru;

static const char *LS_TAG = "LED_STRIP";

#define LS_MAX_STRIP_NUM        2

static LED_Strip_Stru g_led_strips;

extern EventGroupHandle_t g_event_group;

/**
 * @brief Set RGB for all specific pixel
 *
 * @return
 *      - ESP_OK: Set RGB for a specific pixel successfully
 *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
 *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
 */
static esp_err_t all_set_pixel(void) {
    for (int i = 0; i < CONFIG_STRIP1_LED_NUM; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strips.led_strip1, i,
                                g_led_strips.rgb[0],
                                g_led_strips.rgb[1],
                                g_led_strips.rgb[2]));
    }
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(g_led_strips.led_strip1));

    ESP_LOGI(LS_TAG, "LED strip set RGB %ld,%ld,%ld",
                                g_led_strips.rgb[0],
                                g_led_strips.rgb[1],
                                g_led_strips.rgb[2]);
    return ESP_OK;
}

/**
 * @description : Clear LED Strip
 */
static esp_err_t all_clear(void) {
    led_strip_clear(g_led_strips.led_strip1);
    led_strip_clear(g_led_strips.led_strip2);
    return ESP_OK;
}

/**
 * @description : marquee mode
 * @param        {void} *args
 * @return       {*}
 */
static void marquee_cb(void *args) {
    LED_Strip_Stru *strip = (LED_Strip_Stru *)args;
	uint8_t ir, ib;
    ir = (uint8_t)(strip->count + 85);
    ib = (uint8_t)(strip->count + 170);
    for (int j = 0; j < CONFIG_STRIP2_LED_NUM; j ++) {
        // set RGB
        ESP_ERROR_CHECK(led_strip_set_pixel(strip->led_strip2, j,
                            (uint32_t)SinValue[(ir + 10 * j) % 256],      // red
                            (uint32_t)SinValue[(strip->count + 10 * j) % 256],  // green
                            (uint32_t)SinValue[(ib + 10 * j) % 256]));    // blue
    }
    strip->count++;

    // refresh
    ESP_ERROR_CHECK(led_strip_refresh(strip->led_strip2));
}

/**
 * @description : breath mod
 * @param        {void} *args
 * @return       {*}
 */
static void breath_cb(void *args) {
    LED_Strip_Stru *strip = (LED_Strip_Stru *)args;
	uint8_t ir, ib;
    ir = (uint8_t)(strip->count + 85);
    ib = (uint8_t)(strip->count + 170);
    for (int j = 0; j < CONFIG_STRIP2_LED_NUM; j ++) {
        // set RGB
        ESP_ERROR_CHECK(led_strip_set_pixel(strip->led_strip2, j,
                            (uint32_t)SinValue[ir],         // red
                            (uint32_t)SinValue[strip->count], // green
                            (uint32_t)SinValue[ib]));       // blue
    }
    strip->count++;

    // refresh
    ESP_ERROR_CHECK(led_strip_refresh(strip->led_strip2));
}

/**
 * @description : stop running
 */
static esp_err_t stop_running(void) {
    esp_timer_stop(g_led_strips.timer_handle);
    esp_timer_delete(g_led_strips.timer_handle);
    g_led_strips.running = pdFALSE;
    all_clear();

    ESP_LOGI(LS_TAG, "LED strip running stop");
    return ESP_OK;
}

/**
 * @description : start running
 */
static esp_err_t start_running(void) {
    // start Strips
    all_set_pixel();
    ESP_LOGI(LS_TAG, "LED strip1 set");

    // start Strip2
    esp_timer_create_args_t strip_timer = {
        .arg = (void *)&g_led_strips,
        .callback = marquee_cb,
        .dispatch_method = ESP_TIMER_TASK
    };

    esp_timer_create(&strip_timer, &g_led_strips.timer_handle);
    esp_timer_start_periodic(g_led_strips.timer_handle, CONFIG_STRIP2_INTV * 1000U);
    g_led_strips.running = pdTRUE;

    ESP_LOGI(LS_TAG, "LED strip running start");
    return ESP_OK;
}

static esp_err_t save_rgb_to_nvs(void) {
    size_t required_size = sizeof(uint32_t) * 3;

    required_size = sizeof(uint32_t) * 3;
    esp_err_t err = nvs_set_blob(g_led_strips.nvs_handle, LS_TAG, g_led_strips.rgb, required_size);
    nvs_commit(g_led_strips.nvs_handle);
    ESP_LOGI(LS_TAG, "Save RGB");
    return err;
}


/***
 * @description : init LED Strip
 * @return       {*}
 */
static esp_err_t led_strip_init(void) {
    // init CB
    memset(&g_led_strips, 0, sizeof(g_led_strips));
    g_led_strips.running = pdFALSE;

    ESP_LOGI(LS_TAG, "init ...");

    // Open NVS
    esp_err_t err = nvs_open("ShellHome", NVS_READWRITE, &g_led_strips.nvs_handle);
    ESP_ERROR_CHECK(err);

    // load RGB
    ESP_LOGI(LS_TAG, "load ...");
    size_t required_size = sizeof(uint32_t) * 3;
    err = nvs_get_blob(g_led_strips.nvs_handle, LS_TAG, g_led_strips.rgb, &required_size);
    if (ESP_ERR_NVS_NOT_FOUND == err) {
        // set default and save
        g_led_strips.rgb[0] = 255u;
        g_led_strips.rgb[1] = 153u;
        g_led_strips.rgb[2] = 10u;
        err = save_rgb_to_nvs();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(LS_TAG, "config ...");
    // LED strip general initialization
    led_strip_config_t strip_config1 = {
        .strip_gpio_num = CONFIG_STRIP1_GPIO_NUM,       // The GPIO connected to the LED strip's data line
        .max_leds = CONFIG_STRIP1_LED_NUM,              // The number of LEDs in the strip,
        // .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,       // Pixel format of your LED strip
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,       // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,                  // LED strip model
        .flags.invert_out = false,                      // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,                 // different clock source can lead to different power consumption
        .resolution_hz = CONFIG_LED_STRIP_RESOLUTION_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config1, &rmt_config1,
                                             &(g_led_strips.led_strip1)));
    ESP_LOGI(LS_TAG, "Created LED strip 1 object with RMT backend");

    // LED strip general initialization
    led_strip_config_t strip_config2 = {
        .strip_gpio_num = CONFIG_STRIP2_GPIO_NUM,       // The GPIO connected to the LED strip's data line
        .max_leds = CONFIG_STRIP2_LED_NUM,              // The number of LEDs in the strip,
        // .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,       // Pixel format of your LED strip
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,       // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,                  // LED strip model
        .flags.invert_out = false,                      // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,                 // different clock source can lead to different power consumption
        .resolution_hz = CONFIG_LED_STRIP_RESOLUTION_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config2, &rmt_config2,
                                             &(g_led_strips.led_strip2)));
    ESP_LOGI(LS_TAG, "Created LED strip 2 object with RMT backend");

    stop_running();

    return ESP_OK;
}


static void led_strip_task(void *pvParameters)
{
    ESP_LOGI(LS_TAG, "svc ...");
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(g_event_group,
                                BLE_M3_PREV_BIT|BLE_M3_NEXT_BIT|BLE_M3_MORE_BIT|BLE_M3_LESS_BIT|BLE_M3_SAVE_BIT|BLE_M3_SWITCH_BIT,
                                pdTRUE, pdFAIL, portMAX_DELAY);
        if (bits & BLE_M3_PREV_BIT) {
            // previous, increase R
            ESP_LOGI(LS_TAG, "increase R");
            g_led_strips.rgb[0] += CONFIG_CHANGE_RGB_STEP;
            g_led_strips.rgb[0] &= 0xFF;
            all_set_pixel();
        } else if (bits & BLE_M3_NEXT_BIT) {
            // next, increase B
            ESP_LOGI(LS_TAG, "increase B");
            g_led_strips.rgb[2] += CONFIG_CHANGE_RGB_STEP;
            g_led_strips.rgb[2] &= 0xFF;
            all_set_pixel();
        } else if (bits & BLE_M3_MORE_BIT) {
            // more, increase G
            ESP_LOGI(LS_TAG, "increase G");
            g_led_strips.rgb[1] += CONFIG_CHANGE_RGB_STEP;
            g_led_strips.rgb[1] &= 0xFF;
            all_set_pixel();
        } else if (bits & BLE_M3_LESS_BIT) {
            // TODO

        } else if (bits & BLE_M3_SAVE_BIT) {
            // save
            ESP_LOGI(LS_TAG, "Save to NVS");
            save_rgb_to_nvs();
        } else if (bits & BLE_M3_SWITCH_BIT) {
            // on/off
            if (pdTRUE == g_led_strips.running) {
                ESP_LOGI(LS_TAG, "Turn Off");
                stop_running();
            } else {
                ESP_LOGI(LS_TAG, "Turn On");
                start_running();
            }
        } else {
            ESP_LOGE(LS_TAG, "Unknown Bit set %d", (int)bits);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    ESP_LOGI(LS_TAG, "task closing");
    nvs_close(g_led_strips.nvs_handle);
    vTaskDelete(NULL);
}

/* start LED strip */
esp_err_t led_strip_start(void) {
    ESP_ERROR_CHECK(led_strip_init());

    xTaskCreate(&led_strip_task, "led_task", 4 * 1024, NULL, 2, NULL);
    return ESP_OK;
}