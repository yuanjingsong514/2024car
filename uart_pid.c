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

/* UART1_IRQHandler 在 main.c 中定义 */
