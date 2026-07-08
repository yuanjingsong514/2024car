/**
 * @file    line_track.c
 * @brief   循迹控制 — 纯整数比例控制 (M0+无FPU, 浮点太慢)
 *
 * 算法 (参照 STM32 成功项目 tracker.c 的加权差速思想):
 *
 *   sensor_calc_position() → position [-550, +550]
 *           │
 *           ▼
 *   diff = (position * KP_NUM) >> KP_SHIFT     ← 纯整数, <1μs
 *           │
 *           ▼
 *   left  = base_speed + diff                  ← 正值=线在左→右轮加速→左转
 *   right = base_speed - diff
 *           │
 *           ▼
 *   motor_set_both(left, right)
 */

#include "line_track.h"
#include "sensor.h"
#include "motor.h"

/*===========================================================================
 * 比例控制参数 (整数运算, 避免浮点)
 *
 * diff = position * KP_NUM / 2^KP_SHIFT
 * 当前: KP=0.75 → diff_max = 550*3/4 ≈ 412
 *
 * 调大 KP_NUM → 转弯更猛; 调大 KP_SHIFT → 转弯更柔
 *===========================================================================*/
#define KP_NUM       3        /* 分子: 3 → Kp=0.75 */
#define KP_SHIFT     2        /* 右移2位 = 除以4 */

/*===========================================================================
 * 传感器状态追踪
 *===========================================================================*/
static int16_t  s_last_position;
static uint16_t s_lost_count;

/*===========================================================================
 * 初始化
 *===========================================================================*/
void line_track_init(void)
{
    s_last_position = 0;
    s_lost_count    = 0;
}

/*===========================================================================
 * 参数设置接口 (保留兼容性)
 *===========================================================================*/
void line_track_set_base_speed(int16_t speed)
{
    /* 当前版本忽略, 使用 LINE_BASE_SPEED 宏 */
    (void)speed;
}

void line_track_set_pos_pid(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
    /* 浮点PID已移除, 使用整数KP_NUM/KP_SHIFT替代 */
}

void line_track_set_spd_pid(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
    /* 速度PID已移除, M0+性能不足 */
}

line_track_t* line_track_get_instance(void)
{
    return NULL;  /* 内部状态已简化, 不再暴露 */
}

/*===========================================================================
 * 急停
 *===========================================================================*/
void line_track_stop(void)
{
    motor_set_both(0, 0);
}

/*===========================================================================
 * 循迹主循环 — 每5ms调用
 *
 * 纯整数运算, M0+在32MHz下<10μs完成
 *===========================================================================*/
void line_track_run(void)
{
    /* !!! 终极诊断: 硬编码极端差速, 完全绕过所有逻辑 !!! */
    motor_set_both(700, 100);   /* 左快右慢 → 应明显右转 */
}
