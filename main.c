/**
 * @file    main.c
 * @brief   循迹小车主程序 — MSPM0G3507 + TB6612 + 12路灰度
 *
 * 引脚由 SysConfig 统一管理
 */

#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "line_track.h"

/*===========================================================================
 * 主函数 — 直接进入循迹, 无自检, 无定时器依赖
 *===========================================================================*/
int main(void)
{
    /*--- 第1步: 硬件初始化 ---*/
    SYSCFG_DL_init();
    motor_start();
    sensor_init();

    /*--- 第2步: 编码器中断 (用于测速, 不影响主循环) ---*/
    encoder_interrupt_init();
    __enable_irq();

    /*--- 第3步: 循迹初始化 ---*/
    line_track_init();

    /*--- 第4步: 主循环 — 传感器循迹 ---*/
    while (1) {
        line_track_run();
        delay_cycles(CPUCLK_FREQ / 200);   /* 5ms */
    }
}
