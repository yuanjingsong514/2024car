/**
 * @file    main.c
 * @brief   小车基础运动测试 — 无传感器, 无UART
 */

#include "system.h"
#include "motor.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    while (1) {
        /* 前进 2 秒 */
        motor_set_both(200, 200);
        delay_cycles(CPUCLK_FREQ * 2);

        /* 右转 2 秒 (左快右慢) */
        motor_set_both(400, 0);
        delay_cycles(CPUCLK_FREQ * 2);

        /* 左转 2 秒 (左慢右快) */
        motor_set_both(0, 400);
        delay_cycles(CPUCLK_FREQ * 2);
    }
}
