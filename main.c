/**
 * @file    main.c
 * @brief   传感器回显 — 收到什么发什么, 验证传感器UART通信
 *
 * 接线: 传感器 TX → PB7(MSPM0 RX),  USB-TTL RX → PB6(MSPM0 TX)
 */
#include "system.h"
#include "uart_pid.h"

volatile uint16_t g_rx_count = 0;

/* UART1 接收中断 — 收一个字节就原样发回去 */
void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(PID_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t ch = DL_UART_Main_receiveData(PID_UART_INST);
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, ch);  /* 回显 */
        g_rx_count++;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    /* 使能 UART RX 中断 */
    DL_UART_Main_enableInterrupt(PID_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(PID_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_UART_INST_INT_IRQN);

    /* 启动消息 */
    const char *b = "SENSOR_ECHO\r\n";
    for (int i = 0; b[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)b[i]);

    uint16_t last_rx = 0;
    while (1) {
        delay_cycles(CPUCLK_FREQ * 3);  /* 3秒发一次 */
        uint16_t r = g_rx_count;
        char buf[30];
        buf[0] = 'R'; buf[1] = 'X'; buf[2] = '=';
        buf[3] = '0' + (r / 10000) % 10; buf[4] = '0' + (r / 1000) % 10;
        buf[5] = '0' + (r / 100) % 10; buf[6] = '0' + (r / 10) % 10;
        buf[7] = '0' + (r % 10);
        buf[8] = ' '; buf[9] = '+'; buf[10] = '=';
        uint16_t d = r - last_rx; last_rx = r;
        buf[11] = '0' + (d / 100) % 10;
        buf[12] = '0' + (d / 10) % 10; buf[13] = '0' + (d % 10);
        buf[14] = '/'; buf[15] = '3'; buf[16] = 's';
        buf[17] = '\r'; buf[18] = '\n';
        int j;
        for (j = 0; j < 19; j++)
            DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[j]);
    }
}
