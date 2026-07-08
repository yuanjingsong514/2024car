/**
 * @file    main.c
 * @brief   小车运动 + UART遥测
 */
#include "system.h"
#include "motor.h"
#include <stdio.h>

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    /* 启动消息 — 用 DL_UART_Main_transmitDataBlocking (对齐参考项目) */
    const char *start = "START\r\n";
    for (int i = 0; start[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)start[i]);

    uint8_t phase = 0;

    while (1) {
        int16_t left, right;
        const char *name;

        switch (phase) {
            case 0: left = 200; right = 200; name = "FWD";   break;
            case 1: left = 400; right =   0; name = "RIGHT"; break;
            case 2: left =   0; right = 400; name = "LEFT";  break;
        }

        motor_set_both(left, right);

        /* 串口输出 */
        char buf[40];
        int len = snprintf(buf, sizeof(buf),
            "%s L=%d R=%d\r\n", name, (int)left, (int)right);
        for (int i = 0; i < len && buf[i]; i++)
            DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);

        /* 保持状态 2 秒 (每200ms输出一次) */
        for (int j = 0; j < 10; j++) {
            delay_cycles(CPUCLK_FREQ / 5);
            if (j < 9) {
                for (int i = 0; i < len && buf[i]; i++)
                    DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
            }
        }

        phase = (phase + 1) % 3;
    }
}
