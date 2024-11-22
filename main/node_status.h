/***
 * @Author      : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @Date        : 2023-09-27 21:17:16
 * @LastEditors : kevin.z.y <kevin.cn.zhengyang@gmail.com>
 * @LastEditTime: 2023-09-27 21:17:20
 * @FilePath    : /shellhomenode/components/shnode/include/node_status.h
 * @Description :
 * @Copyright (c) 2023 by Zheng, Yang, All Rights Reserved.
 */
#ifndef SHELLHOME_NODE_STATUS_H
#define SHELLHOME_NODE_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * to state the status of node by a paire of (on, off) number of frame.
 * each frame is fixed to 100ms
*/
typedef enum
{
    NODE_POWER_ON,              // (10, 0)
    NODE_SCANNING,              // (20, 50)
    NODE_RUNNING,               // (2, 2)
    NODE_STATUS_BUTT
} SHN_STATUS_ENUM;

/***
 * @description : init status light
 * @return       {*}
 */
void shn_init_status(void);

/***
 * @description : state the status of the node
 * @param        {SHN_STATUS_ENUM} status
 * @return       {*}
 */
void shn_state_status(SHN_STATUS_ENUM status);

/***
 * @description : flush status
 * @return       {*}
 */
void shn_flush_status(void);

#ifdef __cplusplus
}
#endif

#endif /* SHELLHOME_NODE_STATUS_H */
