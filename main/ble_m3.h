/***
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2024-11-20 21:10:52
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2024-11-20 21:10:53
 * @FilePath    : /shellhome-hstrip/main/ble_m3.h
 * @Description : ble remote panel
 * @Copyright (c) 2024 by Zheng, Yang, All Rights Reserved.
 */
#ifndef BLE_M3_H
#define BLE_M3_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"

#define BLE_M3_FORGET_BIT   BIT0
#define BLE_M3_LOST_BIT     BIT1
#define BLE_M3_CONN_BIT     BIT2

#define BLE_M3_PREV_BIT     BIT3
#define BLE_M3_NEXT_BIT     BIT4
#define BLE_M3_MORE_BIT     BIT5
#define BLE_M3_LESS_BIT     BIT6
#define BLE_M3_SAVE_BIT     BIT7
#define BLE_M3_SWITCH_BIT   BIT8

/* start BT HID Host */
esp_err_t ble_m3_host_start(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BLE_M3_H */
