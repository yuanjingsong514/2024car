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
 * 开机自检: 两轮前进 3 秒 → 刹车 1 秒 (验证电机硬件是否正常)
 *===========================================================================*/
static void selftest_motors(void)
{
    volatile uint32_t d;

    /* 两轮同时前进 2 秒 */
    motor_set_both(400, 400);
    for (d = 0; d < 16000000; d++) { __NOP(); }

    /* 刹车 */
    motor_set_both(0, 0);
    for (d = 0; d < 8000000; d++)  { __NOP(); }
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
    /*--- 第1步: SysConfig 统一初始化 (时钟+GPIO+PWM, STBY=LOW, PWM停止) ---*/
    SYSCFG_DL_init();

    /*--- 第2步: 启动电机 (刹车 → 启动PWM → 拉高STBY) ---*/
    motor_start();

    /*--- 第3步: 编码器中断使能 ---*/
    encoder_interrupt_init();

    /*--- 第4步: 传感器 + PID 初始化 ---*/
    sensor_init();
    line_track_init();

    /*--- 第5步: 控制定时器 (TIMG12 5ms) ---*/
    control_timer_init();
    DL_TimerG_startCounter(CONTROL_TIMER);
    __enable_irq();

    /*--- 第6步: UART 初始化 (AI 调参串口) ---*/
    uart_pid_init();

    /*--- 第7步: 开机自检 ---*/
    selftest_motors();

    /*--- 第8步: 主循环 ---*/
    while (1) {
        if (g_control_flag) {
            g_control_flag = false;
            line_track_run();
        }
        uart_pid_send_telemetry();  /* 不受控制循环影响 */
    }
}
