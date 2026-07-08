/**
 * @file    uart_pid.c
 * @brief   UART1: 传感器RX + 遥测TX + 指令解析
 *
 * UART1: PB6=TX, PB7=RX, 115200 8N1
 * 对齐参考 D:\BaiduNetdiskDownload\Vscode\demo\MSPM0G3507\UART\uart_echo
 */

#include "system.h"
#include "sensor.h"
#include "uart_pid.h"

/* 指令接收缓冲区 */
static char    g_rx_buf[64];
static uint8_t g_rx_idx = 0;

/*===========================================================================
 * 初始化
 *===========================================================================*/
void uart_pid_init(void)
{
    /* 暂不启用 RX 中断 — 先用遥测确认小车运动正常 */
    g_rx_idx = 0;
    g_rx_idx = 0;

    /* 启动消息 */
    const char *banner = "READY\r\n";
    for (int i = 0; banner[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)banner[i]);
}

/*===========================================================================
 * UART1 接收中断 — 对齐参考 UART_1_INST_IRQHandler
 *===========================================================================*/
void UART1_IRQHandler(void)
{
    static uint8_t in_sensor = 0;

    if (DL_UART_Main_getPendingInterrupt(PID_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        uint8_t ch = DL_UART_Main_receiveData(PID_UART_INST);

        /* 传感器帧: '#'开头 → 路由到 sensor */
        if (ch == '#') {
            in_sensor = 1;
            g_rx_idx = 0;
        }

        if (in_sensor) {
            sensor_feed_byte(ch);
            if (ch == '!') in_sensor = 0;
            return;
        }

        /* 指令: 换行 → 暂存 (当前版本不解析) */
        if (ch != '\n' && ch != '\r' && g_rx_idx < sizeof(g_rx_buf) - 1)
            g_rx_buf[g_rx_idx++] = (char)ch;
        else
            g_rx_idx = 0;
    }
}
