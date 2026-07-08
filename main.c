/**
 * @file    main.c
 * @brief   循迹小车主程序 — MSPM0G3507 + TB6612 + 12路灰度
 *
 * 引脚由 SysConfig 统一管理, 代码不再手动配 GPIO
 * 参考成功项目 D:\qianrushi\code32\car 结构
 */

#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "line_track.h"
#include "uart_pid.h"

/*===========================================================================
 * 开机自检: 前进3秒 → 右轮转3秒 → 左轮转3秒 → 停止
 *
 * 使用 delay_cycles() 精确延时: CPUCLK_FREQ=32MHz, 每秒3200万周期
 * 3秒 = 32,000,000 * 3 = 96,000,000 cycles
 *
 * 主动制动: 先短暂反转(100ms)强制刹停, 再设为0
 *===========================================================================*/
#define DELAY_3S     (CPUCLK_FREQ * 3)        /* 3 秒 */
#define DELAY_BRAKE  (CPUCLK_FREQ / 10)       /* 100ms 制动 */

static void brake_both(void)
{
    /* 主动反转 → 强制刹停 */
    motor_set_both(-200, -200);
    delay_cycles(DELAY_BRAKE);
    motor_set_both(0, 0);
}

static void brake_left(void)
{
    motor_set_both(-200, 0);
    delay_cycles(DELAY_BRAKE);
    motor_set_both(0, 0);
}

static void brake_right(void)
{
    motor_set_both(0, -200);
    delay_cycles(DELAY_BRAKE);
    motor_set_both(0, 0);
}

static void selftest_motors(void)
{
    /* 第1步: 两轮同时前进 3 秒 */
    motor_set_both(300, 300);
    delay_cycles(DELAY_3S);

    /* 刹停 → 0.5s 停顿 */
    brake_both();
    delay_cycles(CPUCLK_FREQ / 2);

    /* 第2步: 右轮转 3 秒 (左轮刹停) */
    motor_set_both(0, 300);
    delay_cycles(DELAY_3S);

    brake_both();
    delay_cycles(CPUCLK_FREQ / 2);

    /* 第3步: 左轮转 3 秒 (右轮刹停) */
    motor_set_both(300, 0);
    delay_cycles(DELAY_3S);

    /* 第4步: 停止 */
    brake_both();
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
    /*--- 第1步: SysConfig 统一初始化 (时钟+GPIO+PWM, STBY=LOW, PWM停止) ---*/
    SYSCFG_DL_init();

    /*--- 第2步: 启动电机 (coast → 启动PWM → 拉高STBY) ---*/
    motor_start();

    /*--- 第3步: 传感器初始化 ---*/
    sensor_init();

    /*--- 第4步: 开机自检 (前进3s → 右轮3s → 左轮3s → 停止) ---*/
    /* 此时中断未开启, 延时不受干扰 */
    selftest_motors();

    /*--- 第5步: 初始化循迹系统 ---*/
    encoder_interrupt_init();
    line_track_init();
    control_timer_init();
    uart_pid_init();

    /*--- 第6步: 启动控制定时器 + 开全局中断 ---*/
    DL_TimerG_startCounter(CONTROL_TIMER);
    __enable_irq();

    /*--- 第7步: 主循环 — 循迹 ---*/
    while (1) {
        if (g_control_flag) {
            g_control_flag = false;
            line_track_run();
        }
        uart_pid_send_telemetry();
    }
}
