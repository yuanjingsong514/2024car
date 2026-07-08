/**
 * @file    sensor.c
 * @brief   12路灰度传感器 — 完全对照 D:\BaiduNetdiskDownload\Vscode\demo\MSPM0G3507\IIC
 *
 * I2C1, PB2=SCL, PB3=SDA, 400kHz, 0x48
 * 协议: 7字节/帧, '#',/!'+6字节灰度(0=最黑,255=最白)
 */

#include "sensor.h"

#define I2C_ADDR          0x48
#define I2C_PACKET_SIZE   7

static uint8_t  g_rx_packet[7];       /* 对应参考 gRxPacket */
static uint8_t  g_sensor_data[12];    /* 对应参考 iic_data[0..11] */
static int16_t  g_last_position;
static uint8_t  g_data_ready;         /* 两帧都收到过 */

/*===========================================================================
 * 读取 I2C — 完全复制参考 Read_iic()
 *===========================================================================*/
static void read_iic(void)
{
    uint8_t i;

    DL_I2C_startControllerTransfer(I2C_Sensor_INST, I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, I2C_PACKET_SIZE);

    for (i = 0; i < I2C_PACKET_SIZE; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Sensor_INST))
            ;
        g_rx_packet[i] = DL_I2C_receiveControllerData(I2C_Sensor_INST);
    }

    /* 完全复制参考的处理逻辑 */
    if (g_rx_packet[0] == '#') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i] = g_rx_packet[i + 1];
        g_data_ready |= 0x01;
    } else if (g_rx_packet[0] == '!') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i + 6] = g_rx_packet[i + 1];   /* 参考: i-5 即 i+1 */
        g_data_ready |= 0x02;
    }
    /* else: 无效帧, 忽略 */
}

/*===========================================================================
 * 初始化
 *===========================================================================*/
void sensor_init(void)
{
    uint8_t i;
    for (i = 0; i < 12; i++) g_sensor_data[i] = 255;
    g_last_position = SENSOR_POSITION_LOST;
    g_data_ready = 0;

    /* 参考项目关了毛刺滤波器 */
    DL_I2C_disableAnalogGlitchFilter(I2C_Sensor_INST);
}

/*===========================================================================
 * 计算黑线位置 — 每5ms调用一次, 每次读一帧
 *===========================================================================*/
int16_t sensor_calc_position(void)
{
    int32_t sum_left = 0, sum_right = 0;
    int16_t pos;
    uint8_t i;

    /* 每次调用读一帧 (参考 Read_iic) */
    read_iic();

    if (g_data_ready != 0x03) {
        return g_last_position;   /* 数据不完整 */
    }
    g_data_ready = 0;

    /* 左半 vs 右半比大小 */
    for (i = 0; i < 6; i++)
        sum_left  += g_sensor_data[i];       /* S0~S5 */
    for (i = 6; i < 12; i++)
        sum_right += g_sensor_data[i];       /* S6~S11 */

    pos = (int16_t)((sum_left - sum_right) / 4);
    /* 正值=左半更黑=线在左, 车应左转 */

    if (pos > SENSOR_POSITION_MAX)  pos = SENSOR_POSITION_MAX;
    if (pos < SENSOR_POSITION_MIN)  pos = SENSOR_POSITION_MIN;

    g_last_position = pos;
    return pos;
}

int16_t sensor_get_last_position(void) { return g_last_position; }
