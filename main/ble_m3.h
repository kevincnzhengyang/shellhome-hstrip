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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"

#define BLE_M3_KEY_BUTT     0
#define BLE_M3_KEY_LEFT     (1<<0)
#define BLE_M3_KEY_UP       (1<<1)
#define BLE_M3_KEY_RIGHT    (1<<2)
#define BLE_M3_KEY_DOWN     (1<<3)
#define BLE_M3_KEY_LIKE     (1<<4)
#define BLE_M3_KEY_TAKE     (1<<5)

/* start BT HID Host */
esp_err_t ble_m3_host_start(void);

#endif /* BLE_M3_H */
