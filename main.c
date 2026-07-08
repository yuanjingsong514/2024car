/**
 * @file    main.c
 * @brief   小车运动 + UART遥测 + 传感器位置
 */
#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "uart_pid.h"

int main(void)
{
    SYSCFG_DL_init();
    motor_start();
    sensor_init();
    uart_pid_init();

    uint16_t tick = 0;

    while (1) {
        /* 读传感器 */
        int16_t pos = sensor_calc_position();   /* 第一个传感器的原始值 */
        if (pos == 10000 || pos == -10000) pos = 0;

        /* 简单比例控制 */
        int16_t diff = (pos * 3) >> 2;   /* pos * 3/4 */
        int16_t base = 200;
        int16_t left  = base + diff;
        int16_t right = base - diff;

        /* 限幅 */
        if (left  > 800) left  = 800;
        if (left  < -800) left  = -800;
        if (right > 800) right = 800;
        if (right < -800) right = -800;

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
            /* 全部12路传感器 S=000000000000 (0=白 1=黑) */
            buf[idx++] = ' ';
            buf[idx++] = 'S';
            buf[idx++] = '=';
            for (uint8_t s = 0; s < 12; s++)
                buf[idx++] = '0' + (sensor_get_raw(s) ? 1 : 0);
            buf[idx++] = '\r';
            buf[idx++] = '\n';

            for (uint8_t i = 0; i < idx; i++)
                DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
        }

        delay_cycles(CPUCLK_FREQ / 200);  /* 5ms */
    }
}
