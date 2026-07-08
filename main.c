/**
 * @file    main.c
 * @brief   小车运动 + UART遥测 + 传感器位置
 */
#include "system.h"
#include "motor.h"
#include "uart_pid.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_start();
    sensor_init();
    uart_pid_init();

    uint16_t tick = 0;

    while (1) {
        /* 固定前进 — 先验证运动+遥测, 传感器稍后接入 */
        int16_t left  = 200;
        int16_t right = 200;

        motor_set_both(left, right);

        /* 每 200ms 发遥测: 位置 左PWM 右PWM */
        tick++;
        if (tick >= 200) {   /* 1秒一次 */
            tick = 0;

            /* 手动拼字符串, 不用 snprintf */
            char buf[32];
            uint8_t idx = 0;

            /* 位置 (可能有负号) */
            if (pos < 0) { buf[idx++] = '-'; pos = -pos; }
            if (pos >= 100) buf[idx++] = '0' + (pos / 100) % 10;
            if (pos >= 10)  buf[idx++] = '0' + (pos / 10) % 10;
            buf[idx++] = '0' + (pos % 10);
            buf[idx++] = ' ';

            /* 左PWM */
            int16_t v = left;
            if (v < 0) { buf[idx++] = '-'; v = -v; }
            if (v >= 100) buf[idx++] = '0' + (v / 100) % 10;
            if (v >= 10)  buf[idx++] = '0' + (v / 10) % 10;
            buf[idx++] = '0' + (v % 10);
            buf[idx++] = ' ';

            /* 右PWM */
            v = right;
            if (v < 0) { buf[idx++] = '-'; v = -v; }
            if (v >= 100) buf[idx++] = '0' + (v / 100) % 10;
            if (v >= 10)  buf[idx++] = '0' + (v / 10) % 10;
            buf[idx++] = '0' + (v % 10);
            /* 左PWM 右PWM */
            buf[idx++] = ' ';
            buf[idx++] = 'O';
            buf[idx++] = 'K';
            buf[idx++] = '\r';
            buf[idx++] = '\n';

            for (uint8_t i = 0; i < idx; i++)
                DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
        }

        delay_cycles(CPUCLK_FREQ / 200);  /* 5ms */
    }
}
