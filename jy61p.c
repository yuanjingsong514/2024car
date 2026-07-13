/**
 * @file    jy61p.c
 * @brief   JY61P 解析 — 0x55协议
 */

#include "jy61p.h"
#include "ti_msp_dl_config.h"

static uint8_t  s_buf[11];
static uint8_t  s_idx;
static uint8_t  s_state;

static volatile float g_roll, g_pitch, g_yaw;
static volatile float g_wz;
static volatile bool  g_new_data;
volatile uint32_t g_jy61_pkt = 0;

void jy61p_feed_byte(uint8_t b)
{
    uint8_t i, sum;

    if (s_state == 0) {
        if (b == 0x55) { s_buf[0] = b; s_state = 1; s_idx = 1; }
        return;
    }
    if (s_state == 1) {
        if (b == 0x53 || b == 0x52) { s_buf[1] = b; s_state = 2; s_idx = 2; }
        else { s_state = 0; }
        return;
    }
    s_buf[s_idx++] = b;
    if (s_idx >= 11) {
        s_state = 0;
        sum = 0;
        for (i = 0; i < 10; i++) sum += s_buf[i];
        if (sum != s_buf[10]) return;
        if (s_buf[1] == 0x53) {
            g_roll  = (float)((int16_t)((uint16_t)s_buf[3] << 8 | s_buf[2])) / 32768.0f * 180.0f;
            g_pitch = (float)((int16_t)((uint16_t)s_buf[5] << 8 | s_buf[4])) / 32768.0f * 180.0f;
            g_yaw   = (float)((int16_t)((uint16_t)s_buf[7] << 8 | s_buf[6])) / 32768.0f * 180.0f;
            g_new_data = true;
            g_jy61_pkt++;
        } else if (s_buf[1] == 0x52) {
            g_wz = (float)((int16_t)((uint16_t)s_buf[7] << 8 | s_buf[6])) / 32768.0f * 2000.0f;
            g_new_data = true;
            g_jy61_pkt++;
        }
    }
}

void jy61p_init(void) { s_state = 0; s_idx = 0; g_new_data = false; }
void jy61p_get_data(jy61p_data_t *d) { d->roll = g_roll; d->pitch = g_pitch; d->yaw = g_yaw; d->gyro_z = g_wz; d->updated = g_new_data; }
bool jy61p_is_new(void) { return g_new_data; }
void jy61p_clear_new(void) { g_new_data = false; }
