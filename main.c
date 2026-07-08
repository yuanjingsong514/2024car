/**
 * @file    main.c
 * @brief   小车运动测试 — 无UART, 纯运动
 */

#include "system.h"
#include "motor.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    while (1) {
        /* 前进 2s */
        motor_set_both(200, 200);
        delay_cycles(CPUCLK_FREQ * 2);

        /* 右转 2s */
        motor_set_both(400, 0);
        delay_cycles(CPUCLK_FREQ * 2);

        /* 左转 2s */
        motor_set_both(0, 400);
        delay_cycles(CPUCLK_FREQ * 2);
    }
}
