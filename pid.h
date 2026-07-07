/**
 * @file    pid.h
 * @brief   PID 控制器模块 - 位置式 PID 算法
 *
 * 算法:
 *   output = Kp * error + Ki * ∫error·dt + Kd * d(error)/dt
 *
 * 特性:
 *   - 积分分离 (减少大偏差时的积分饱和)
 *   - 积分限幅 (Anti-windup)
 *   - 输出限幅
 *   - 微分先行 (可选，对测量值微分而非误差)
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <stdbool.h>
#include <float.h>

/*===========================================================================
 * PID 结构体
 *===========================================================================*/
typedef struct {
    /* 参数 */
    float   kp;                 /* 比例系数                         */
    float   ki;                 /* 积分系数                         */
    float   kd;                 /* 微分系数                         */

    /* 限幅参数 */
    float   integral_limit;     /* 积分饱和上限 (绝对值)            */
    float   output_limit;       /* 输出限幅上限 (绝对值)            */
    float   integral_sep_threshold; /* 积分分离阈值                 */

    /* 状态变量 */
    float   setpoint;           /* 目标值                           */
    float   integral;           /* 积分累加值                       */
    float   prev_error;         /* 上一次误差                       */
    float   prev_measurement;   /* 上一次测量值 (用于微分先行)      */
    float   output;             /* 本次输出                         */

    /* 配置标志 */
    bool    derivative_on_measurement;  /* true=微分先行            */
} pid_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 PID 控制器
 * @param pid      PID 结构体指针
 * @param kp       比例系数
 * @param ki       积分系数
 * @param kd       微分系数
 * @param setpoint 目标值
 * @param out_limit 输出限幅 (绝对值)
 * @param int_limit 积分限幅 (绝对值)
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float setpoint, float out_limit, float int_limit);

/**
 * @brief 设置 PID 参数
 */
void pid_set_params(pid_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置目标值
 */
void pid_set_setpoint(pid_t *pid, float setpoint);

/**
 * @brief PID 计算 (位置式, 增量输出)
 *
 * @param pid         PID 结构体指针
 * @param measurement 当前测量值
 * @param dt          时间间隔 (秒)
 * @return            控制输出
 */
float pid_calculate(pid_t *pid, float measurement, float dt);

/**
 * @brief 重置 PID 状态 (清除积分和误差)
 */
void pid_reset(pid_t *pid);

/**
 * @brief 获取上次输出
 */
static inline float pid_get_output(const pid_t *pid)
{
    return pid->output;
}

#endif /* __PID_H__ */
