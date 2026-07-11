/**
 * @file    motor.h
 * @brief   电机控制 — TB6612, PWM 由 SysConfig 管理
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "system.h"

void motor_start(void);
void motor_set_both(int16_t left_speed, int16_t right_speed);
void motor_stop(void);
void motor_test_chA(int16_t speed);   /* TIMG6/PA29 */
void motor_test_chB(int16_t speed);   /* TIMG8/PA7 */

#endif /* __MOTOR_H__ */
