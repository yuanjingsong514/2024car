/**
 * @file    sensor.c
 * @brief   12路灰度传感器 I2C — 完全参考 D:\BaiduNetdiskDownload\Vscode\demo\MSPM0G3507\IIC
 *
 * I2C1, PB2=SCL, PB3=SDA, 400kHz, 0x48
 * 每帧7字节: 头('#','!') + 6字节灰度(0=黑,255=白)
 */

#include "sensor.h"

#define I2C_ADDR          0x48
#define I2C_PACKET_SIZE   7

static uint8_t  g_sensor_raw[12];
static uint8_t  g_sensor_updated;
static int16_t  g_last_position;
static uint8_t  g_black_count;

/*===========================================================================
 * 初始化 — 关闭模拟毛刺滤波器 (参考已验证项目)
 *===========================================================================*/
void sensor_init(void)
{
    uint8_t i;
    for (i = 0; i < SENSOR_COUNT; i++) g_sensor_raw[i] = 255;
    g_sensor_updated = 0;
    g_last_position  = SENSOR_POSITION_LOST;
    g_black_count    = 0;

    /* 关键: 关闭模拟毛刺滤波器, 参考已验证项目 */
    DL_I2C_disableAnalogGlitchFilter(I2C_Sensor_INST);
}

/*===========================================================================
 * 读一帧 — 完全参考已验证项目 (无超时, 阻塞式)
 *===========================================================================*/
static bool sensor_i2c_read(uint8_t *buf)
{
    uint8_t i;

    DL_I2C_startControllerTransfer(I2C_Sensor_INST, I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, I2C_PACKET_SIZE);

    for (i = 0; i < I2C_PACKET_SIZE; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Sensor_INST))
            ;
        buf[i] = DL_I2C_receiveControllerData(I2C_Sensor_INST);
    }

    if (buf[0] != '#' && buf[0] != '!') return false;
    return true;
}

/*===========================================================================
 * 更新一帧
 *===========================================================================*/
static void sensor_update(void)
{
    uint8_t buf[I2C_PACKET_SIZE];
    uint8_t i;

    if (!sensor_i2c_read(buf)) return;

    if (buf[0] == '#') {
        for (i = 0; i < 6; i++) g_sensor_raw[i + 6] = buf[i + 1];  /* '#'=后半 */
        g_sensor_updated |= 0x02;
    } else {
        for (i = 0; i < 6; i++) g_sensor_raw[i] = buf[i + 1];      /* '!'=前半 */
        g_sensor_updated |= 0x01;
    }
}

/*===========================================================================
 * 计算位置 — 加权质心法
 *===========================================================================*/
int16_t sensor_calc_position(void)
{
    int32_t weighted_sum = 0;
    uint8_t count = 0;
    uint8_t i;

    /* 每次读一帧, 两帧拼成完整12路 */
    sensor_update();

    if (g_sensor_updated != 0x03) {
        return g_last_position;
    }
    g_sensor_updated = 0;

    for (i = 0; i < SENSOR_COUNT; i++) {
        if (g_sensor_raw[i] < SENSOR_BLACK_THRESHOLD) {
            weighted_sum += (int32_t)i * 100;
            count++;
        }
    }

    g_black_count = count;

    if (count == 0) {
        g_last_position = SENSOR_POSITION_LOST;
        return SENSOR_POSITION_LOST;
    }
    if (count == SENSOR_COUNT) {
        g_last_position = SENSOR_POSITION_CROSS;
        return SENSOR_POSITION_CROSS;
    }

    int16_t pos = (int16_t)(weighted_sum / count - 550);

    if (pos > SENSOR_POSITION_MAX)  pos = SENSOR_POSITION_MAX;
    if (pos < SENSOR_POSITION_MIN)  pos = SENSOR_POSITION_MIN;

    g_last_position = pos;
    return pos;
}

uint8_t sensor_get_black_count(void)   { return g_black_count; }
bool    sensor_is_cross(void)          { return g_last_position == SENSOR_POSITION_CROSS; }
bool    sensor_is_lost(void)           { return g_last_position == SENSOR_POSITION_LOST; }
int16_t sensor_get_last_position(void) { return g_last_position; }
