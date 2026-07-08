/**
 * @file    sensor.c
 * @brief   12路灰度传感器 — UART协议 (对齐参考 UART\uart_echo)
 *
 * UART1, PB7=RX (传感器), PB6=TX (遥测)
 * 协议: '#' + 12字节('0'=白/'1'=黑) + '!'
 * 传感器主动发送, 每帧14字节
 */

#include "sensor.h"

static volatile uint8_t g_binary[12];   /* 0=白 1=黑 */
static volatile uint8_t g_ready = 0;    /* 新数据就绪 */

static uint8_t s_buf[12];
static uint8_t s_idx = 0;
static uint8_t s_rx   = 0;  /* 0=等'#' 1=收数据 */

/*===========================================================================
 * 喂一个字节给传感器解析器 — 由 UART ISR 调用
 *===========================================================================*/
void sensor_feed_byte(uint8_t b)
{
    if (!s_rx) {
        if (b == '#') { s_rx = 1; s_idx = 0; }
        return;
    }

    if (b == '!') {
        /* 帧尾 — 转二进制 */
        for (uint8_t i = 0; i < 12; i++)
            g_binary[i] = (s_buf[i] == '1') ? 1 : 0;
        g_ready = 1;
        s_rx = 0;
        return;
    }

    if ((b == '0' || b == '1') && s_idx < 12)
        s_buf[s_idx++] = b;
    else
        s_rx = 0;  /* 非法字符→丢弃 */
}

/*===========================================================================
 * 初始化
 *===========================================================================*/
void sensor_init(void)
{
    uint8_t i;
    for (i = 0; i < 12; i++) g_binary[i] = 0;
    g_ready = 0;
    s_rx    = 0;
    s_idx   = 0;
}

/*===========================================================================
 * 计算黑线位置 — 加权求和 (对齐 STM32 tracker.c)
 *
 * 权重: S0=-6 ... S5=-1 | S6=+1 ... S11=+6
 * 仅 g_binary[i]==1 (黑) 的传感器计入
 *===========================================================================*/
int16_t sensor_calc_position(void)
{
    static const int8_t w[12] = {-6,-5,-4,-3,-2,-1, 1,2,3,4,5,6};
    int16_t sum = 0;
    int16_t pos;
    uint8_t i;

    if (!g_ready) return sensor_get_last_position();
    g_ready = 0;

    for (i = 0; i < 12; i++) {
        if (g_binary[i]) sum += w[i];
    }

    pos = sum * 50;

    if (pos > 550)  pos = 550;
    if (pos < -550) pos = -550;

    g_last_position = pos;
    return pos;
}

/*===========================================================================
 * 辅助函数
 *===========================================================================*/
static int16_t g_last_position;

uint8_t sensor_get_black_count(void)
{
    uint8_t i, n = 0;
    for (i = 0; i < 12; i++) if (g_binary[i]) n++;
    return n;
}

uint8_t sensor_get_raw(uint8_t idx)
{
    return (idx < 12) ? g_binary[idx] : 0;
}

int16_t sensor_get_last_position(void) { return g_last_position; }
bool sensor_is_cross(void) { return false; }
bool sensor_is_lost(void)  { return (sensor_get_black_count() == 0); }
