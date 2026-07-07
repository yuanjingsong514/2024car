/**
 * @file    system.h
 * @brief   系统定义 — 所有引脚、定时器、全局变量
 *
 * GPIO 全部由 SysConfig 管理 (ti_msp_dl_config.h)
 * PWM 由 SysConfig 管理 (MotorPWM_Left=TIMG6 PA29, MotorPWM_Right=TIMG8 PA7)
 * 控制定时器 TIMG12 手动配置
 */

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * PWM 参数 (SysConfig: Period=1600, 20kHz, Edge-aligned, INIT_VAL_LOW)
 *
 * INIT_VAL_LOW 下占空比计算:
 *   CC=1600 → 0% duty,  CC=0 → 100% duty
 *   因此实际 CC = PERIOD - (speed * PERIOD / MAX)
 *===========================================================================*/
#define MOTOR_PWM_PERIOD        1600
#define MOTOR_PWM_MAX           1000
#define MOTOR_DEADBAND          50

/*===========================================================================
 * 控制定时器 — TIMG12 (手动配置, 5ms=200Hz)
 *===========================================================================*/
#define CONTROL_TIMER           TIMG12
#define CONTROL_TIMER_IRQn      TIMG12_INT_IRQn
#define CONTROL_TIMER_IIDX      DL_TIMER_IIDX_ZERO

/*===========================================================================
 * 全局变量
 *===========================================================================*/
extern volatile bool     g_control_flag;
extern volatile int32_t  g_enc_left_count;
extern volatile int32_t  g_enc_right_count;

/*===========================================================================
 * 函数声明
 *===========================================================================*/
void encoder_interrupt_init(void);
void control_timer_init(void);

#endif /* __SYSTEM_H__ */
