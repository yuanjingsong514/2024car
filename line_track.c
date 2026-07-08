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
    int16_t position;
    int16_t diff;
    int16_t left_pwm, right_pwm;
    int16_t base = LINE_BASE_SPEED;   /* 400 */

    /*--- 第1步: 读传感器 (诊断: 跳过, 用虚拟位置) ---*/
    /* !!! 诊断模式 — 不调用任何 I2C 函数 !!! */
    {
        static int16_t diag_pos = 0;
        static int8_t  diag_dir = 1;
        static uint16_t diag_cnt = 0;

        diag_cnt++;
        if (diag_cnt > 50) {        /* 每250ms切换方向 */
            diag_cnt = 0;
            diag_dir = -diag_dir;
        }
        diag_pos = diag_dir * 400;  /* 直接在 ±400 之间跳变 */

        position = diag_pos;
        s_lost_count = 0;           /* 永不丢线 */
    }

    /*--- 第2步: 丢线处理 ---*/
    if (position == SENSOR_POSITION_LOST) {
        s_lost_count++;
        if (s_lost_count > 200) {
            /* 持续丢线1秒以上: 尝试半速直行 */
            motor_set_both(base / 2, base / 2);
            return;
        }
        /* 短暂丢线: 保持上次位置 */
        position = s_last_position;
    } else {
        s_lost_count = 0;
    }
    s_last_position = position;

    /*--- 第3步: 整数比例差速 ---*/
    diff = (int16_t)(((int32_t)position * KP_NUM) >> KP_SHIFT);

    /*--- 第4步: 左右PWM = base ± diff ---*/
    left_pwm  = base + diff;
    right_pwm = base - diff;

    /*--- 第5步: 限幅 ---*/
    if (left_pwm  > LINE_MAX_SPEED)  left_pwm  = LINE_MAX_SPEED;
    if (left_pwm  < -LINE_MAX_SPEED) left_pwm  = -LINE_MAX_SPEED;
    if (right_pwm > LINE_MAX_SPEED)  right_pwm = LINE_MAX_SPEED;
    if (right_pwm < -LINE_MAX_SPEED) right_pwm = -LINE_MAX_SPEED;

    /* 死区 */
    if (left_pwm  > -LINE_MIN_SPEED && left_pwm  < LINE_MIN_SPEED)
        left_pwm  = 0;
    if (right_pwm > -LINE_MIN_SPEED && right_pwm < LINE_MIN_SPEED)
        right_pwm = 0;

    /*--- 第6步: 输出 ---*/
    motor_set_both(left_pwm, right_pwm);
}
