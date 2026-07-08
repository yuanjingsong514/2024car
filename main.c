/**
 * @file    main.c
 * @brief   传感器检测 — 收数据但不回显, 显示计数和最后一帧
 */
#include "system.h"
#include "uart_pid.h"

volatile uint16_t g_rx_count = 0;
static char   g_last_frame[16];   /* 保存最后一帧 */
static uint8_t g_frame_idx = 0;
static uint8_t g_in_frame = 0;    /* 是否在帧内 */
static uint8_t g_frame_ready = 0; /* 完整帧就绪 */

void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(PID_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t ch = DL_UART_Main_receiveData(PID_UART_INST);
        g_rx_count++;

        if (ch == '#') {
            g_in_frame = 1;
            g_frame_idx = 0;
        }
        if (g_in_frame && g_frame_idx < 15) {
            g_last_frame[g_frame_idx++] = ch;
        }
        if (ch == '!' && g_in_frame) {
            g_last_frame[g_frame_idx] = '\0';
            g_frame_ready = 1;
            g_in_frame = 0;
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();

    DL_UART_Main_enableInterrupt(PID_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(PID_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_UART_INST_INT_IRQN);

    const char *b = "SENSOR_OK\r\n";
    for (int i = 0; b[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)b[i]);

    uint16_t last_rx = 0;
    while (1) {
        delay_cycles(CPUCLK_FREQ * 2);  /* 2秒一次 */

        char buf[48]; uint8_t n = 0;
        uint16_t r = g_rx_count;
        uint16_t d = r - last_rx; last_rx = r;

        buf[n++]='R'; buf[n++]='X'; buf[n++]='=';
        buf[n++]='0'+(r/10000)%10; buf[n++]='0'+(r/1000)%10;
        buf[n++]='0'+(r/100)%10;  buf[n++]='0'+(r/10)%10;
        buf[n++]='0'+(r%10);
        buf[n++]=' '; buf[n++]='+'; buf[n++]='=';
        buf[n++]='0'+(d/1000)%10; buf[n++]='0'+(d/100)%10;
        buf[n++]='0'+(d/10)%10; buf[n++]='0'+(d%10);
        buf[n++]='/'; buf[n++]='2'; buf[n++]='s';
        buf[n++]=' '; buf[n++]='[';

        if (g_frame_ready) {
            g_frame_ready = 0;
            for (uint8_t i = 0; g_last_frame[i]; i++)
                buf[n++] = g_last_frame[i];
        } else {
            buf[n++]='?';
        }
        buf[n++]=']'; buf[n++]='\r'; buf[n++]='\n';

        int j;
        for (j = 0; j < n; j++)
            DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[j]);
    }
}
