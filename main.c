/**
 * @file    main.c
 * @brief   小车运动 + UART遥测
 */
#include "system.h"
#include "motor.h"
#include "uart_pid.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    /* UART 初始化 (uart_pid.c已验证能输出READY) */
    uart_pid_init();

    uint8_t phase = 0;
    uint16_t tick = 0;

    while (1) {
        int16_t left, right;

        switch (phase) {
            case 0: left = 200; right = 200; break;
            case 1: left = 400; right =   0; break;
            case 2: left =   0; right = 400; break;
        }

        motor_set_both(left, right);

        /* 每 500ms 发遥测 (uart_pid_init已验证DL_UART_transmitDataBlocking可用) */
        tick++;
        if (tick >= 100) {
            tick = 0;
            const char *name;
            switch (phase) {
                case 0: name = "FWD";   break;
                case 1: name = "RIGHT"; break;
                case 2: name = "LEFT";  break;
            }
            char c;
            for (const char *p = name; (c = *p); p++)
                DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)c);
            DL_UART_transmitDataBlocking(PID_UART_INST, '\r');
            DL_UART_transmitDataBlocking(PID_UART_INST, '\n');
        }

        delay_cycles(CPUCLK_FREQ / 200);  /* 5ms */
    }
}
