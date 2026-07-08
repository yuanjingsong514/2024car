/**
 * @file    main.c
 * @brief   传感器循迹 — UART接收 + 加权P控制 + 遥测
 */
#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "uart_pid.h"

/* 传感器数据由 UART ISR 喂入, 本文件只读取 */

/* UART1 ISR: 收字节 → 喂给传感器解析器 */
void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(PID_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        sensor_feed_byte(DL_UART_Main_receiveData(PID_UART_INST));
    }
}

int main(void)
{
    SYSCFG_DL_init();
    motor_start();
    sensor_init();

    /* 使能 UART RX */
    DL_UART_Main_enableInterrupt(PID_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(PID_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_UART_INST_INT_IRQN);

    /* 启动消息 */
    const char *b = "TRACK\r\n";
    for (int i = 0; b[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)b[i]);

    uint16_t tick = 0;
    while (1) {
        /*=== 读传感器位置 ===*/
        int16_t pos = sensor_calc_position();

        /*=== P控制 ===*/
        int16_t diff = (pos * 3) >> 2;       /* pos × 0.75 */
        int16_t base = 200;
        int16_t left  = base + diff;
        int16_t right = base - diff;

        /* 限幅 */
        if (left  > 800) left  = 800;
        if (left  < -800) left = -800;
        if (right > 800) right = 800;
        if (right < -800) right = -800;
        if (left > -80 && left < 80) left = 0;
        if (right > -80 && right < 80) right = 0;

        motor_set_both(left, right);

        /*=== 每200ms遥测 ===*/
        tick++;
        if (tick >= 40) {
            tick = 0;
            char buf[48]; uint8_t n = 0;
            int16_t v;

            /* pos */
            v = pos;
            if (v < 0) { buf[n++] = '-'; v = -v; }
            if (v >= 100) buf[n++] = '0' + (v/100)%10;
            if (v >= 10)  buf[n++] = '0' + (v/10)%10;
            buf[n++] = '0' + (v % 10);
            buf[n++] = ' ';

            /* left */
            v = left;
            if (v < 0) { buf[n++] = '-'; v = -v; }
            if (v >= 100) buf[n++] = '0' + (v/100)%10;
            if (v >= 10)  buf[n++] = '0' + (v/10)%10;
            buf[n++] = '0' + (v % 10);
            buf[n++] = ' ';

            /* right */
            v = right;
            if (v < 0) { buf[n++] = '-'; v = -v; }
            if (v >= 100) buf[n++] = '0' + (v/100)%10;
            if (v >= 10)  buf[n++] = '0' + (v/10)%10;
            buf[n++] = '0' + (v % 10);
            buf[n++] = ' ';

            /* 传感器 12bit */
            buf[n++] = '[';
            for (uint8_t s = 0; s < 12; s++)
                buf[n++] = sensor_get_raw(s) ? '1' : '0';
            buf[n++] = ']';
            buf[n++] = '\r'; buf[n++] = '\n';

            for (uint8_t i = 0; i < n; i++)
                DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
        }

        delay_cycles(CPUCLK_FREQ / 200);  /* 5ms */
    }
}
