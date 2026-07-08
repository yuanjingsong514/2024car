/**
 * @file    line_track.c
 * @brief   循迹控制 — 纯整数比例控制
 *
 *   sensor → position → diff = position * KP_NUM >> KP_SHIFT
 *                    → left = base + diff, right = base - diff
 */

#include "line_track.h"
#include "sensor.h"
#include "motor.h"

/* 遥测变量 (main.c 定义, 此处更新) */
extern volatile int16_t g_telem_pos;
extern volatile int16_t g_telem_left;
extern volatile int16_t g_telem_right;

/*===========================================================================
 * 比例控制参数
 *   diff = position * KP_NUM / 2^KP_SHIFT
 *   Kp = 3/4 = 0.75, diff_max = 550*3/4 ≈ 412
 *===========================================================================*/
#define KP_NUM       3
#define KP_SHIFT     2

/*===========================================================================
 * 状态
 *===========================================================================*/
static int16_t  s_last_position;
static uint16_t s_lost_count;

void line_track_init(void)
{
    s_last_position = 0;
    s_lost_count    = 0;
}

/* 兼容接口 (保留但空实现) */
void line_track_set_base_speed(int16_t s)  { (void)s; }
void line_track_set_pos_pid(float a,float b,float c) { (void)a;(void)b;(void)c; }
void line_track_set_spd_pid(float a,float b,float c) { (void)a;(void)b;(void)c; }
line_track_t* line_track_get_instance(void) { return NULL; }

void line_track_stop(void)
{
    motor_set_both(0, 0);
}

/*===========================================================================
 * 循迹主循环
 *===========================================================================*/
void line_track_run(void)
{
    int16_t position;
    int16_t diff;
    int16_t left_pwm, right_pwm;
    int16_t base = LINE_BASE_SPEED;

    /*--- 读传感器 ---*/
    position = sensor_calc_position();

    /*--- 丢线处理 ---*/
    if (position == SENSOR_POSITION_LOST) {
        s_lost_count++;
        if (s_lost_count > 200) {
            /* 丢线超1秒 → 半速直行 */
            motor_set_both(base / 2, base / 2);
            return;
        }
        position = s_last_position;
    } else {
        s_lost_count = 0;
    }
    s_last_position = position;

    /*--- 整数比例差速 ---*/
    diff = (int16_t)(((int32_t)position * KP_NUM) >> KP_SHIFT);

    /*--- 左右PWM ---*/
    left_pwm  = base + diff;
    right_pwm = base - diff;

    /*--- 限幅 ---*/
    if (left_pwm  > LINE_MAX_SPEED)  left_pwm  = LINE_MAX_SPEED;
    if (left_pwm  < -LINE_MAX_SPEED) left_pwm  = -LINE_MAX_SPEED;
    if (right_pwm > LINE_MAX_SPEED)  right_pwm = LINE_MAX_SPEED;
    if (right_pwm < -LINE_MAX_SPEED) right_pwm = -LINE_MAX_SPEED;

    if (left_pwm  > -LINE_MIN_SPEED && left_pwm  < LINE_MIN_SPEED) left_pwm  = 0;
    if (right_pwm > -LINE_MIN_SPEED && right_pwm < LINE_MIN_SPEED) right_pwm = 0;

    /*--- 更新遥测 ---*/
    g_telem_pos   = position;
    g_telem_left  = left_pwm;
    g_telem_right = right_pwm;

    /*--- 输出 ---*/
    motor_set_both(left_pwm, right_pwm);
}
