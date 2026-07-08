/**
 * @file    main.c
 * @brief   小车运动测试 + UART遥测 (115200, PB6)
 */

#include "system.h"
#include "motor.h"
#include <stdio.h>

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    int16_t  left = 200, right = 200;
    uint32_t phase_timer  = 0;
    uint32_t uart_timer   = 0;
    uint8_t  phase        = 0;  /* 0=前进 1=右转 2=左转 */
    uint32_t i;

    motor_set_both(left, right);

    while (1) {
        /*=== 运动阶段切换 (每2秒) ===*/
        phase_timer++;
        if (phase_timer >= CPUCLK_FREQ * 2) {
            phase_timer = 0;
            phase = (phase + 1) % 3;
            switch (phase) {
                case 0: left = 200; right = 200; break;  /* 前进 */
                case 1: left = 400; right =   0; break;  /* 右转 */
                case 2: left =   0; right = 400; break;  /* 左转 */
            }
            motor_set_both(left, right);
        }

        /*=== UART 遥测 (每500ms) ===*/
        uart_timer++;
        if (uart_timer >= CPUCLK_FREQ / 2) {
            uart_timer = 0;
            char buf[40];
            int len = snprintf(buf, sizeof(buf),
                "%d %d %d\r\n", (int)phase, (int)left, (int)right);
            for (i = 0; i < (uint32_t)len && buf[i]; i++) {
                DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
            }
        }
    }
}
