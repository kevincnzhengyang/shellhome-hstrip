/*
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2023-09-27 21:17:26
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2024-11-22 15:51:05
 * @FilePath    : /shellhome-hstrip/main/node_status.c
 * @Description :
 * Copyright (c) 2023 by Zheng, Yang, All Rights Reserved.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "node_status.h"


typedef struct {
    esp_timer_handle_t  timer_handle;       // timer handler
    uint8_t                 on_count;       // count for light on
    uint8_t                   on_num;
    uint8_t                off_count;       // count for light off
    uint8_t                  off_num;
    SHN_STATUS_ENUM           status;       // status of node
    bool                      stated;
} Node_Status_Stru;

Node_Status_Stru g_node_status;

static void status_cb(void *args)
{
    if (!g_node_status.stated) return;

    if (0 < g_node_status.on_num) {
        g_node_status.on_num--;
        gpio_set_level(CONFIG_STATUS_LED_GPIO_NUM, 1);
    } else if (0 < g_node_status.off_num) {
        g_node_status.off_num--;
        gpio_set_level(CONFIG_STATUS_LED_GPIO_NUM, 0);
    }

    // reset if all done
    if (0 == g_node_status.on_num && 0 == g_node_status.off_num) {
        g_node_status.on_num = g_node_status.on_count;
        g_node_status.off_num = g_node_status.off_count;
    }
}

/***
 * @description : init status light
 * @return       {*}
 */
void shn_init_status(void)
{
    memset(&g_node_status, 0, sizeof(Node_Status_Stru));
    g_node_status.status = NODE_STATUS_BUTT;
    g_node_status.stated = false;
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,     //disable interrupt
        .mode = GPIO_MODE_OUTPUT,     //set as output mode
        .pin_bit_mask = (1ULL << CONFIG_STATUS_LED_GPIO_NUM),  //bit mask of the pins
        .pull_down_en = 0,                  //disable pull-down mode
        .pull_up_en = 0                     //disable pull-up mode
    };
    gpio_config(&io_conf);                  //configure GPIO

    esp_timer_create_args_t status_timer = {
        .arg = NULL,
        .callback = status_cb,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "status_timer"
    };

    esp_timer_create(&status_timer, &g_node_status.timer_handle);
}

/***
 * @description : state the status of the node
 * @param        {SHN_STATUS_ENUM} status
 * @return       {*}
 */
void shn_state_status(SHN_STATUS_ENUM status)
{
    if (NODE_STATUS_BUTT == status) return;

    if (g_node_status.stated) {
        // stop timer if stated
        esp_timer_stop(g_node_status.timer_handle);
        gpio_set_level(CONFIG_STATUS_LED_GPIO_NUM, 0);

    }

    switch (status)
    {
    case NODE_POWER_ON:
        g_node_status.on_num = g_node_status.on_count = 10;
        g_node_status.off_num = g_node_status.off_count = 0;
        break;
    case NODE_SCANNING:
        g_node_status.on_num = g_node_status.on_count = 20;
        g_node_status.off_num = g_node_status.off_count = 50;
        break;
    case NODE_RUNNING:
        g_node_status.on_num = g_node_status.on_count = 2;
        g_node_status.off_num = g_node_status.off_count = 2;

    default:
        break;
    }

    g_node_status.stated = true;
    g_node_status.status = status;

    // start timer
    esp_timer_start_periodic(g_node_status.timer_handle, 100 * 1000U);
}

/***
 * @description : flush status
 * @return       {*}
 */
void shn_flush_status(void)
{
    // TODO watch dog
    printf("flush node status\n");
}

