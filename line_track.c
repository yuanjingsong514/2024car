/**
 * @file    line_track.c
 * @brief   循迹控制 — 当前: 简单双轮前进测试
 */

#include "line_track.h"
#include "sensor.h"
#include "motor.h"
#include <stdlib.h>

static line_track_t g_tracker;
static float dt = 0.005f;

void line_track_init(void)
{
    line_track_t *t = &g_tracker;
    pid_init(&t->pos_pid, POS_KP_DEFAULT, POS_KI_DEFAULT, POS_KD_DEFAULT,
             0.0f, POS_OUT_LIMIT, POS_INT_LIMIT);
    pid_init(&t->spd_left_pid, SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f, SPD_OUT_LIMIT, SPD_INT_LIMIT);
    pid_init(&t->spd_right_pid, SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f, SPD_OUT_LIMIT, SPD_INT_LIMIT);
    t->base_speed = LINE_BASE_SPEED;
    t->state = TRACK_STATE_NORMAL;
}

void line_track_set_base_speed(int16_t speed) { g_tracker.base_speed = speed; }
void line_track_set_pos_pid(float kp, float ki, float kd) { pid_set_params(&g_tracker.pos_pid, kp, ki, kd); }
void line_track_set_spd_pid(float kp, float ki, float kd) {
    pid_set_params(&g_tracker.spd_left_pid, kp, ki, kd);
    pid_set_params(&g_tracker.spd_right_pid, kp, ki, kd);
}
line_track_t* line_track_get_instance(void) { return &g_tracker; }

/*===========================================================================
 * 简单测试: 两轮同时前进
 *===========================================================================*/
void line_track_run(void)
{
    motor_set_both(400, 400);
}
