/**
 * @file    pid.c
 * @brief   PID 控制器实现
 */

#include "pid.h"
#include <math.h>

/*===========================================================================
 * PID 初始化
 *===========================================================================*/
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float setpoint, float out_limit, float int_limit)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = setpoint;
    pid->output_limit   = out_limit;
    pid->integral_limit = int_limit;
    pid->integral_sep_threshold = out_limit * 0.5f;  /* 默认: 偏差大时分离开 */

    pid->integral            = 0.0f;
    pid->prev_error          = 0.0f;
    pid->prev_measurement    = 0.0f;
    pid->output              = 0.0f;

    pid->derivative_on_measurement = false;  /* 默认: 对误差微分 */
}

/*===========================================================================
 * 设置 PID 参数
 *===========================================================================*/
void pid_set_params(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*===========================================================================
 * 设置目标值
 *===========================================================================*/
void pid_set_setpoint(pid_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/*===========================================================================
 * PID 计算
 *
 * 位置式 PID: output = Kp*e(k) + Ki*Σe(k)*dt + Kd*(e(k)-e(k-1))/dt
 *
 * 特性:
 *   1. 积分分离: 当 |error| > integral_sep_threshold 时, Ki=0
 *      避免大偏差时积分饱和
 *   2. 积分限幅: 限制积分项的最大值，防止 windup
 *   3. 微分先行: derivative_on_measurement=true 时,
 *      微分项 = -Kd * (meas(k) - meas(k-1)) / dt
 *      而非 Kd * (e(k) - e(k-1)) / dt
 *      减少目标值突变时的微分冲击
 *   4. 输出限幅: 输出值被限制在 [-output_limit, +output_limit]
 *===========================================================================*/
float pid_calculate(pid_t *pid, float measurement, float dt)
{
    float error;
    float p_term, i_term, d_term;
    float output;

    /* 防止除零 */
    if (dt < 1e-6f) {
        return pid->output;
    }

    /* 计算误差 */
    error = pid->setpoint - measurement;

    /* --- 比例项 --- */
    p_term = pid->kp * error;

    /* --- 积分项 (带积分分离) --- */
    if (fabsf(error) <= pid->integral_sep_threshold) {
        pid->integral += error * dt;
    }
    /* 积分限幅 */
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    i_term = pid->ki * pid->integral;

    /* --- 微分项 --- */
    if (pid->derivative_on_measurement) {
        /* 微分先行: 对测量值微分，减少设定值突变冲击 */
        d_term = -pid->kd * (measurement - pid->prev_measurement) / dt;
        pid->prev_measurement = measurement;
    } else {
        /* 标准: 对误差微分 */
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    /* --- 合成输出 --- */
    output = p_term + i_term + d_term;

    /* --- 输出限幅 --- */
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    pid->output = output;
    return output;
}

/*===========================================================================
 * 重置 PID
 *===========================================================================*/
void pid_reset(pid_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output           = 0.0f;
}
