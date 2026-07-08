/**
 * @file    main.c
 * @brief   小车运动测试 + UART遥测
 */

#include "system.h"
#include "motor.h"
#include <stdio.h>

/* 发送字符串到 UART */
static void uart_send(const char *s)
{
    while (*s) {
        DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)*s++);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    motor_start();

    /* 启动消息 — 验证 UART 工作 */
    uart_send("START\r\n");

    uint8_t phase = 0;  /* 0=前进 1=右转 2=左转 */

    while (1) {
        int16_t left, right;
        const char *name;

        switch (phase) {
            case 0: left = 200; right = 200; name = "FWD";   break;
            case 1: left = 400; right =   0; name = "RIGHT"; break;
            case 2: left =   0; right = 400; name = "LEFT";  break;
        }

        motor_set_both(left, right);

        /* UART 输出 */
        char buf[40];
        int len = snprintf(buf, sizeof(buf),
            "%s L=%d R=%d\r\n", name, (int)left, (int)right);
        uart_send(buf);

        /* 保持当前状态 2 秒, 期间每 200ms 再发一次 */
        for (int i = 0; i < 10; i++) {
            delay_cycles(CPUCLK_FREQ / 5);   /* 200ms */
            if (i < 9) uart_send(buf);       /* 再发9次, 共10次/2秒 */
        }

        phase = (phase + 1) % 3;
    }
}
