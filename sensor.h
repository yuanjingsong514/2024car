/**
 * @file    sensor.h
 * @brief   12路灰度传感器 — I2C 通信 (PB2=SCL, PB3=SDA, 0x48)
 *
 * 传感器交替发送:
 *   '#' + 6字节 = 前半 (S0~S5)
 *   '!' + 6字节 = 后半 (S6~S11)
 * 每字节 0-255, 值越小越黑
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "system.h"
#include <stdint.h>
#include <stdbool.h>

#define SENSOR_COUNT            12

/* 位置计算 */
#define SENSOR_POSITION_MAX     550
#define SENSOR_POSITION_MIN     -550
#define SENSOR_POSITION_LOST    10000
#define SENSOR_POSITION_CROSS   -10000

/* 黑白阈值: 低于此值视为黑线 */
#define SENSOR_BLACK_THRESHOLD  50    /* 更敏感: 深黑<50, 浅白>50 */

void    sensor_init(void);
int16_t sensor_calc_position(void);
uint8_t sensor_get_black_count(void);
uint8_t sensor_get_raw(uint8_t index);
bool    sensor_is_cross(void);
bool    sensor_is_lost(void);
int16_t sensor_get_last_position(void);

#endif
