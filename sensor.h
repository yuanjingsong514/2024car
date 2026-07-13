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

/*===========================================================================
 * 路口检测器
 *===========================================================================*/
typedef struct {
    uint8_t  confirm_count;         /* 连续检测到路口的次数          */
    uint8_t  confirm_threshold;     /* 确认阈值 (建议 3~8)          */
    uint8_t  black_threshold;       /* 黑点个数阈值 (建议 6)        */
    uint16_t cooldown_counter;      /* 冷却计数器                   */
    uint16_t cooldown_period;       /* 冷却周期 (5ms单位, 建议100)  */
    bool     active;                /* 路口激活标志                  */
} junction_detector_t;

void    sensor_init(void);
void    sensor_feed_byte(uint8_t byte);   /* UART ISR调用 */
int16_t sensor_calc_position(void);
uint8_t sensor_get_black_count(void);
uint8_t sensor_get_raw(uint8_t index);
bool    sensor_is_cross(void);
bool    sensor_is_lost(void);
int16_t sensor_get_last_position(void);

/*=== 路口检测 ===*/
void    junction_detector_init(junction_detector_t *jd,
                               uint8_t black_thresh,
                               uint8_t confirm_thresh,
                               uint16_t cooldown);
bool    junction_detector_update(junction_detector_t *jd);
void    junction_detector_reset(junction_detector_t *jd);
uint8_t sensor_get_black_left(void);    /* S0~S5 黑点数 */
uint8_t sensor_get_black_right(void);   /* S6~S11 黑点数 */
int8_t  sensor_junction_side(void);     /* -1=偏左 0=居中 +1=偏右 */

#endif
