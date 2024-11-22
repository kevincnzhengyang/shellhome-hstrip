/***
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2023-09-24 23:05:28
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2023-09-24 23:05:32
 * @FilePath    : /shellhomenode/components/ledstrip/include/led_strip.h
 * @Description :
 * @Copyright (c) 2023 by Zheng, Yang, All Rights Reserved.
 */
#ifndef SHELLHOME_LED_STRIP
#define SHELLHOME_LED_STRIP

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "led_strip.h"


/* start LED strip */
esp_err_t led_strip_start(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SHELLHOME_LED_STRIP */
