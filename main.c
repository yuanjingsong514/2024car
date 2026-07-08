/**
 * @file    main.c
 * @brief   循迹小车 — MSPM0G3507 + 传感器 + UART遥测
 */

#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "line_track.h"
#include "uart_pid.h"
#include <stdio.h>

/*===========================================================================
 * 遥测用的全局变量 (line_track_run 更新, main 循环发送)
 *===========================================================================*/
volatile int16_t g_telem_pos   = 0;
volatile int16_t g_telem_left  = 0;
volatile int16_t g_telem_right = 0;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
    /*--- 硬件初始化 ---*/
    SYSCFG_DL_init();
    motor_start();
    sensor_init();
    encoder_interrupt_init();
    __enable_irq();

    /*--- UART 初始化 (115200, PB6=TX) ---*/
    uart_pid_init();

    /*--- 循迹初始化 ---*/
    line_track_init();

    /*--- 主循环 ---*/
    uint16_t tick = 0;

    while (1) {
        line_track_run();

        /* 每 100ms (20次×5ms) 发送一次遥测 */
        tick++;
        if (tick >= 20) {
            tick = 0;
            char buf[64];
            int len = snprintf(buf, sizeof(buf),
                "P=%d L=%d R=%d\r\n",
                (int)g_telem_pos,
                (int)g_telem_left,
                (int)g_telem_right);
            for (int i = 0; i < len; i++) {
                DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
            }
        }

        delay_cycles(CPUCLK_FREQ / 200);   /* 5ms */
    }
}
