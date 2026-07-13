/**
 * @file    jy61p.h
 * @brief   JY61P 陀螺仪 — UART_61 (UART3, PB3=RX, PB2=TX), 115200
 */

#ifndef __JY61P_H__
#define __JY61P_H__

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 数据结构
 *===========================================================================*/
typedef struct {
    float roll;             /* 横滚角  (°)    */
    float pitch;            /* 俯仰角  (°)    */
    float yaw;              /* 偏航角  (°)    */
    float gyro_z;           /* Z轴角速度 (°/s) */
    bool  updated;          /* 新数据标志      */
} jy61p_data_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/
void jy61p_init(void);
void jy61p_feed_byte(uint8_t b);    /* UART ISR 调用 */
void jy61p_get_data(jy61p_data_t *d);
bool jy61p_is_new(void);
void jy61p_clear_new(void);

#endif
