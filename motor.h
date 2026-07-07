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

#endif /* __MOTOR_H__ */
