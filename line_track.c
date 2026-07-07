/**
 * @file    line_track.c
 * @brief   循迹控制实现 - 双闭环 PID 控制
 *
 * 核心控制逻辑:
 *   1. 读取传感器 → 计算黑线位置
 *   2. 位置 PID → 得出差速量 (turn)
 *   3. 目标左速 = base_speed + turn, 目标右速 = base_speed - turn
 *   4. 读取编码器 → 得到实际速度
 *   5. 速度 PID → 得出 PWM 值
 *   6. 更新电机
 *
 * 线丢失恢复策略:
 *   - 短时丢失: 保持上次方向以低速前进
 *   - 长时丢失: 原地旋转搜索线
 */

#include "line_track.h"
#include "sensor.h"
#include "motor.h"
#include <stdlib.h>   /* abs() */

/*===========================================================================
 * 全局变量
 *===========================================================================*/
static line_track_t g_tracker;          /* 循迹控制器实例          */
static float dt = 0.005f;               /* 控制周期 5ms (秒)       */

/*===========================================================================
 * 编码器速度读取
 *
 * 读取并清零编码器脉冲计数
 * 返回本周期内的脉冲增量 (带符号)
 *===========================================================================*/
static int32_t encoder_read_left(void)
{
    int32_t count;
    __disable_irq();                     /* 临界区: 防止ISR同时修改 */
    count = g_enc_left_count;
    g_enc_left_count = 0;
    __enable_irq();
    return count;
}

static int32_t encoder_read_right(void)
{
    int32_t count;
    __disable_irq();
    count = g_enc_right_count;
    g_enc_right_count = 0;
    __enable_irq();
    return count;
}

/*===========================================================================
 * 初始化循迹控制器
 *===========================================================================*/
void line_track_init(void)
{
    line_track_t *t = &g_tracker;

    /* 初始化位置 PID (目标=0, 即线在正中间) */
    pid_init(&t->pos_pid,
             POS_KP_DEFAULT, POS_KI_DEFAULT, POS_KD_DEFAULT,
             0.0f,                                    /* setpoint = 0     */
             POS_OUT_LIMIT, POS_INT_LIMIT);

    /* 位置 PID 使用微分先行，减少线位置突变时的冲击 */
    t->pos_pid.derivative_on_measurement = true;
    t->pos_pid.integral_sep_threshold = 200.0f;      /* 偏差>200时分离开*/

    /* 初始化左轮速度 PID (setpoint 在每次控制循环中动态设置) */
    pid_init(&t->spd_left_pid,
             SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f,                                    /* 动态设置        */
             SPD_OUT_LIMIT, SPD_INT_LIMIT);

    /* 初始化右轮速度 PID */
    pid_init(&t->spd_right_pid,
             SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f,
             SPD_OUT_LIMIT, SPD_INT_LIMIT);

    /* 默认参数 */
    t->base_speed    = LINE_BASE_SPEED;
    t->state         = TRACK_STATE_NORMAL;
    t->line_position = 0;
    t->last_position = 0;
    t->left_speed    = 0;
    t->right_speed   = 0;
    t->lost_count    = 0;
    t->lost_max      = 40;                /* 200ms容忍 (40 * 5ms)    */
}

/*===========================================================================
 * 设置基础速度
 *===========================================================================*/
void line_track_set_base_speed(int16_t speed)
{
    if (speed > LINE_MAX_SPEED)  speed = LINE_MAX_SPEED;
    if (speed < LINE_MIN_SPEED)  speed = LINE_MIN_SPEED;
    g_tracker.base_speed = speed;
}

/*===========================================================================
 * 设置 PID 参数
 *===========================================================================*/
void line_track_set_pos_pid(float kp, float ki, float kd)
{
    pid_set_params(&g_tracker.pos_pid, kp, ki, kd);
}

void line_track_set_spd_pid(float kp, float ki, float kd)
{
    pid_set_params(&g_tracker.spd_left_pid,  kp, ki, kd);
    pid_set_params(&g_tracker.spd_right_pid, kp, ki, kd);
}

/*===========================================================================
 * 处理线丢失情况
 *
 * 策略:
 *   1. 短时丢失 (< lost_max 次): 保持上次方向，低速前进
 *   2. 长时丢失 (>= lost_max 次): 原地旋转搜索 (先右转找线)
 *===========================================================================*/
static void handle_line_lost(line_track_t *t)
{
    t->lost_count++;

    if (t->lost_count < t->lost_max) {
        /* 短时丢失: 保持上次方向低速前进 */
        int16_t base = LINE_MIN_SPEED + 50;
        if (t->last_position < 0) {
            /* 上次线在左侧 → 左转寻找 */
            t->left_speed  = base - 100;
            t->right_speed = base + 100;
        } else {
            /* 上次线在右侧 → 右转寻找 */
            t->left_speed  = base + 100;
            t->right_speed = base - 100;
        }
        t->state = TRACK_STATE_LOST;
    } else {
        /* 长时丢失: 原地旋转搜索 */
        /* 第一阶段: 原地左转, 第二阶段: 原地右转 */
        if (t->lost_count < t->lost_max * 2) {
            /* 原地左转 */
            t->left_speed  = -200;
            t->right_speed =  200;
        } else {
            /* 原地右转 (扩大搜索范围) */
            t->left_speed  =  200;
            t->right_speed = -200;
        }
        t->state = TRACK_STATE_LOST;
    }
}

/*===========================================================================
 * 处理十字路口
 *
 * 策略: 直行通过 (保持最后有效方向)，不改变差速
 *===========================================================================*/
static void handle_cross(line_track_t *t)
{
    t->state = TRACK_STATE_CROSS;

    /* 直行: 保持基础速度，差速为零 */
    t->left_speed  = t->base_speed;
    t->right_speed = t->base_speed;
}

/*===========================================================================
 * 执行一次循迹控制循环
 *
 * 每 5ms 由主循环调用一次
 *===========================================================================*/
void line_track_run(void)
{
    line_track_t *t = &g_tracker;
    int16_t position;
    int16_t left_spd, right_spd;

    position = sensor_calc_position();
    t->line_position = position;

    if (position == SENSOR_POSITION_LOST) {
        /* 丢线: 原地左转搜索 ← 和直行完全不同! */
        left_spd  = -200;
        right_spd =  200;
        t->state = TRACK_STATE_LOST;
    } else if (position == SENSOR_POSITION_CROSS) {
        /* 十字: 直行 */
        left_spd  = 300;
        right_spd = 300;
        t->state = TRACK_STATE_CROSS;
    } else if (position > 100) {
        /* 线在右边 → 右转 → 左快右慢 */
        left_spd  = 500;
        right_spd = 100;
        t->state = TRACK_STATE_NORMAL;
    } else if (position < -100) {
        /* 线在左边 → 左转 → 右快左慢 */
        left_spd  = 100;
        right_spd = 500;
        t->state = TRACK_STATE_NORMAL;
    } else {
        /* 居中 → 直行 */
        left_spd  = 300;
        right_spd = 300;
        t->state = TRACK_STATE_NORMAL;
    }

    t->left_speed  = left_spd;
    t->right_speed = right_spd;
    motor_set_both(left_spd, right_spd);
}

/*===========================================================================
 * 获取循迹控制器实例
 *===========================================================================*/
line_track_t* line_track_get_instance(void)
{
    return &g_tracker;
}

/*===========================================================================
 * 紧急停止
 *===========================================================================*/
void line_track_stop(void)
{
    g_tracker.state = TRACK_STATE_STOP;
    g_tracker.left_speed  = 0;
    g_tracker.right_speed = 0;
    pid_reset(&g_tracker.pos_pid);
    pid_reset(&g_tracker.spd_left_pid);
    pid_reset(&g_tracker.spd_right_pid);
    motor_stop();
}
