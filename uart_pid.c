/**
 * @file    uart_pid.c
 * @brief   AI 调参串口 — 硬件由 SysConfig 管理, 本文件只管收发逻辑
 *
 * UART1: PB6=TX, PB7=RX, 115200 8N1 (SysConfig: SYSCFG_DL_PID_UART_init)
 */

#include "system.h"
#include "motor.h"
#include "line_track.h"
#include "uart_pid.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 接收缓冲区 */
static char  g_rx_buf[64];
static uint8_t g_rx_idx = 0;

/* 遥测计数器 (每20次控制周期≈100ms 发送一次) */
static uint16_t g_telemetry_cnt = 0;

/*===========================================================================
 * 初始化 — 硬件已由 SysConfig 配好, 只使能 RX 中断
 *===========================================================================*/
void uart_pid_init(void)
{
    DL_UART_enableInterrupt(PID_UART_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(PID_UART_INST_INT_IRQN);
    g_rx_idx = 0;
    g_telemetry_cnt = 0;

    /* 启动消息 — 确认串口存活 */
    const char *banner = "READY\r\n";
    for (int i = 0; banner[i]; i++)
        DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)banner[i]);
}

/*===========================================================================
 * 发送单字节
 *===========================================================================*/
static void uart_putc(char c)
{
    DL_UART_transmitDataBlocking(PID_UART_INST, (uint8_t)c);
}

/*===========================================================================
 * 遥测上传 — 每 100ms 发送一行
 *   D,位置,左目标速度,右目标速度,左编码器,右编码器,差速量,状态\r\n
 *===========================================================================*/
void uart_pid_send_telemetry(void)
{
    g_telemetry_cnt++;
    if (g_telemetry_cnt < 20) return;
    g_telemetry_cnt = 0;

    line_track_t *t = line_track_get_instance();
    char buf[128];

    int len = snprintf(buf, sizeof(buf),
        "D,%d,%d,%d,%ld,%ld,%d,%d\r\n",
        (int)t->line_position,
        (int)t->left_speed,
        (int)t->right_speed,
        (long)g_enc_left_count,
        (long)g_enc_right_count,
        (int)t->pos_pid.output,
        (int)t->state);

    for (int i = 0; i < len && buf[i]; i++) uart_putc(buf[i]);
}

/*===========================================================================
 * 解析并执行下发指令
 *===========================================================================*/
static void parse_command(const char *cmd)
{
    char type[8];
    float v1, v2, v3;
    char rsp[64];
    int len;

    if (strncmp(cmd, "STOP", 4) == 0) {
        motor_stop();
        len = snprintf(rsp, sizeof(rsp), "OK,STOP\r\n");
        for (int i = 0; i < len && rsp[i]; i++) uart_putc(rsp[i]);
        return;
    }

    /* 4参数: TYPE,v1,v2,v3 */
    if (sscanf(cmd, "%7[^,],%f,%f,%f", type, &v1, &v2, &v3) == 4) {

        if (strcmp(type, "POS") == 0) {
            if (v1 < 0.1f) v1 = 0.1f; if (v1 > 5.0f) v1 = 5.0f;
            if (v2 < 0.0f) v2 = 0.0f; if (v2 > 0.5f) v2 = 0.5f;
            if (v3 < 0.0f) v3 = 0.0f; if (v3 > 10.0f) v3 = 10.0f;
            line_track_set_pos_pid(v1, v2, v3);
            len = snprintf(rsp, sizeof(rsp),
                "OK,POS,%.2f,%.3f,%.1f\r\n", v1, v2, v3);
            for (int i = 0; i < len && rsp[i]; i++) uart_putc(rsp[i]);
            return;
        }

        if (strcmp(type, "SPD") == 0) {
            if (v1 < 0.1f) v1 = 0.1f; if (v1 > 3.0f) v1 = 3.0f;
            if (v2 < 0.0f) v2 = 0.0f; if (v2 > 1.0f) v2 = 1.0f;
            if (v3 < 0.0f) v3 = 0.0f; if (v3 > 5.0f) v3 = 5.0f;
            line_track_set_spd_pid(v1, v2, v3);
            len = snprintf(rsp, sizeof(rsp),
                "OK,SPD,%.2f,%.3f,%.1f\r\n", v1, v2, v3);
            for (int i = 0; i < len && rsp[i]; i++) uart_putc(rsp[i]);
            return;
        }
    }

    /* 2参数: TYPE,v1 */
    if (sscanf(cmd, "%7[^,],%f", type, &v1) == 2) {
        if (strcmp(type, "BASE") == 0) {
            if (v1 < 50) v1 = 50; if (v1 > 800) v1 = 800;
            line_track_set_base_speed((int16_t)v1);
            len = snprintf(rsp, sizeof(rsp), "OK,BASE,%d\r\n", (int)v1);
            for (int i = 0; i < len && rsp[i]; i++) uart_putc(rsp[i]);
            return;
        }
    }

    len = snprintf(rsp, sizeof(rsp), "ERR,Unknown\r\n");
    for (int i = 0; i < len && rsp[i]; i++) uart_putc(rsp[i]);
}

/*===========================================================================
 * UART1 接收中断 — 逐字符接收, 遇 \n 解析
 *===========================================================================*/
void UART1_IRQHandler(void)
{
    uint8_t ch;

    if (DL_UART_getEnabledInterruptStatus(PID_UART_INST,
            DL_UART_INTERRUPT_RX)) {

        ch = DL_UART_receiveData(PID_UART_INST);

        if (ch == '\n' || ch == '\r') {
            if (g_rx_idx > 0) {
                g_rx_buf[g_rx_idx] = '\0';
                parse_command(g_rx_buf);
                g_rx_idx = 0;
            }
        } else if (g_rx_idx < sizeof(g_rx_buf) - 1) {
            g_rx_buf[g_rx_idx++] = (char)ch;
        }
    }
}
