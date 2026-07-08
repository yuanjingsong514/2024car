/**
 * @file    line_track.h
 * @brief   循迹控制模块 - 位置PID + 速度PID 双闭环控制
 *
 * 控制架构:
 *
 *   传感器位置 ──→ 位置PID ──→ 差速量 ──┬──→ 左速度PID ──→ 左PWM
 *                                       │        ↑
 *                    基础速度 ──────────┤    左编码器反馈
 *                                       │
 *                                       └──→ 右速度PID ──→ 右PWM
 *                                                ↑
 *                                            右编码器反馈
 *
 * 调速参数通过宏定义配置，方便调试和调节
 */

#ifndef __LINE_TRACK_H__
#define __LINE_TRACK_H__

#include "system.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 循迹基础速度配置
 *===========================================================================*/
#define LINE_BASE_SPEED         200         /* 基础速度 (0~1000)          */
#define LINE_MAX_SPEED          800         /* 最大速度限制                */
#define LINE_MIN_SPEED          80          /* 最小速度限制                */

/*===========================================================================
 * 位置 PID 默认参数 (可根据实际调试修改)
 *
 * 调节指南:
 *   Kp: 先单独调节 → 小车能循迹但有稳态误差
 *   Kd: 加入微分 → 减少震荡，提高稳定性
 *   Ki: 最后加入 → 消除稳态误差
 *
 * 典型范围: Kp=0.5~2.0, Ki=0.01~0.1, Kd=1.0~5.0
 *===========================================================================*/
#define POS_KP_DEFAULT          1.2f        /* 位置环比例系数             */
#define POS_KI_DEFAULT          0.02f       /* 位置环积分系数             */
#define POS_KD_DEFAULT          3.0f        /* 位置环微分系数             */
#define POS_OUT_LIMIT           500.0f      /* 位置环输出限幅 (±差速量)   */
#define POS_INT_LIMIT           200.0f      /* 位置环积分限幅             */

/*===========================================================================
 * 速度 PID 默认参数 (左右电机共用)
 *
 * 调节指南:
 *   Kp: 先调节 → 电机能快速响应速度变化
 *   Ki: 加入 → 消除速度稳态误差
 *   Kd: 可选 → 一般速度环不需要微分
 *
 * 典型范围: Kp=0.3~1.0, Ki=0.05~0.3, Kd=0
 *===========================================================================*/
#define SPD_KP_DEFAULT          0.5f        /* 速度环比例系数             */
#define SPD_KI_DEFAULT          0.1f        /* 速度环积分系数             */
#define SPD_KD_DEFAULT          0.0f        /* 速度环微分系数 (通常为0)   */
#define SPD_OUT_LIMIT           1000.0f     /* 速度环输出限幅 (±PWM值)    */
#define SPD_INT_LIMIT           500.0f      /* 速度环积分限幅             */

/*===========================================================================
 * 循迹状态枚举
 *===========================================================================*/
typedef enum {
    TRACK_STATE_NORMAL  = 0,    /* 正常循迹                        */
    TRACK_STATE_LOST    = 1,    /* 线丢失, 正在找回                 */
    TRACK_STATE_CROSS   = 2,    /* 十字路口                         */
    TRACK_STATE_STOP    = 3,    /* 停止                             */
} track_state_t;

/*===========================================================================
 * 循迹控制器结构体
 *===========================================================================*/
typedef struct {
    /* PID 控制器 */
    pid_t   pos_pid;            /* 位置环 PID                      */
    pid_t   spd_left_pid;       /* 左轮速度环 PID                  */
    pid_t   spd_right_pid;      /* 右轮速度环 PID                  */

    /* 速度参数 */
    int16_t base_speed;         /* 基础速度 (0~1000)               */

    /* 状态 */
    track_state_t state;        /* 当前循迹状态                    */
    int16_t line_position;      /* 当前黑线位置                    */
    int16_t last_position;      /* 上一次黑线位置 (用于丢失找回)   */
    int16_t left_speed;         /* 左轮当前目标速度                */
    int16_t right_speed;        /* 右轮当前目标速度                */

    /* 丢失线计数器 */
    uint32_t lost_count;        /* 连续丢失次数                    */
    uint32_t lost_max;          /* 丢失容忍上限                    */
} line_track_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化循迹控制器
 */
void line_track_init(void);

/**
 * @brief 设置基础速度
 * @param speed 基础速度 (0~1000)
 */
void line_track_set_base_speed(int16_t speed);

/**
 * @brief 设置位置 PID 参数
 */
void line_track_set_pos_pid(float kp, float ki, float kd);

/**
 * @brief 设置速度 PID 参数
 */
void line_track_set_spd_pid(float kp, float ki, float kd);

/**
 * @brief 执行一次循迹控制循环 (每 5ms 调用一次)
 *
 * 流程:
 *   1. 读取传感器位置
 *   2. 处理异常状态 (丢失/十字)
 *   3. 位置PID计算差速量
 *   4. 读取编码器 → 速度PID计算PWM
 *   5. 更新电机输出
 */
void line_track_run(void);

/**
 * @brief 获取循迹控制器实例 (供调试用)
 */
line_track_t* line_track_get_instance(void);

/**
 * @brief 紧急停止
 */
void line_track_stop(void);

#endif /* __LINE_TRACK_H__ */
