/**
 * @file    sensor.c
 * @brief   12路灰度传感器 — 严格对齐参考项目 iic_echo
 *
 * I2C1, PB2=SCL, PB3=SDA, 400kHz, 0x48
 * 协议: 7字节/帧, '#'/'!'+6字节灰度 (0=最黑, 255=最白)
 * 传感器交替发送 '#' 帧(S0~S5) 和 '!' 帧(S6~S11)
 */

#include "sensor.h"

#define I2C_ADDR          0x48
#define I2C_PACKET_SIZE   7

static uint8_t  g_rx_packet[7];
static uint8_t  g_sensor_data[12];
static int16_t  g_last_position;
static uint8_t  g_data_ready;

/*===========================================================================
 * 读取 I2C — 完全对齐参考项目 Read_iic(), 加超时保护
 *===========================================================================*/
#define I2C_TIMEOUT  10000   /* ~300μs/byte */

static bool read_iic(void)
{
    uint8_t i;
    uint32_t timeout;

    DL_I2C_startControllerTransfer(I2C_Sensor_INST, I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, I2C_PACKET_SIZE);

    for (i = 0; i < I2C_PACKET_SIZE; i++) {
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Sensor_INST)) {
            if (--timeout == 0) return false;
        }
        g_rx_packet[i] = DL_I2C_receiveControllerData(I2C_Sensor_INST);
    }

    /* 对齐参考: '#'→S0~S5, '!'→S6~S11 */
    if (g_rx_packet[0] == '#') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i] = g_rx_packet[i + 1];
        g_data_ready |= 0x01;
    } else if (g_rx_packet[0] == '!') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i + 6] = g_rx_packet[i + 1];
        g_data_ready |= 0x02;
    }
    /* 无效帧: 忽略 */

    return true;
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

    DL_I2C_disableAnalogGlitchFilter(I2C_Sensor_INST);

    /* 启用PB2(SCL)和PB3(SDA)内部上拉 — I2C必须有上拉才能通信 */
    DL_GPIO_enablePullUp(GPIOB, DL_GPIO_PIN_2);
    DL_GPIO_enablePullUp(GPIOB, DL_GPIO_PIN_3);
}

/*===========================================================================
 * 计算黑线位置 — 每5ms读一帧, 两帧凑齐后计算
 *
 * 加权求和 (对齐 STM32 参考 tracker.c):
 *   权重: S0=-6 ... S5=-1 | S6=+1 ... S11=+6
 *   仅低于阈值(压黑线)的传感器计入
 *===========================================================================*/
int16_t sensor_calc_position(void)
{
    int16_t pos;
    uint8_t i;

    /* 读一帧 */
    if (!read_iic()) {
        return g_last_position;
    }

    /* 两帧未凑齐 → 返回上次有效位置 */
    if (g_data_ready != 0x03) {
        return g_last_position;
    }
    g_data_ready = 0;   /* 凑齐了, 重置等待下一轮 */

    /*--- 加权求和 ---*/
    {
        static const int8_t w[12] = {-6,-5,-4,-3,-2,-1, 1,2,3,4,5,6};
        int16_t sum = 0;
        for (i = 0; i < 12; i++) {
            if (g_sensor_data[i] < SENSOR_BLACK_THRESHOLD) {
                sum += w[i];
            }
        }
        pos = sum * 50;
    }

    if (pos > SENSOR_POSITION_MAX)  pos = SENSOR_POSITION_MAX;
    if (pos < SENSOR_POSITION_MIN)  pos = SENSOR_POSITION_MIN;

    g_last_position = pos;
    return pos;
}

int16_t sensor_get_last_position(void) { return g_last_position; }

uint8_t sensor_get_raw(uint8_t index)
{
    if (index < 12) return g_sensor_data[index];
    return 0;
}

uint8_t sensor_get_black_count(void)
{
    uint8_t i, count = 0;
    for (i = 0; i < 12; i++) {
        if (g_sensor_data[i] < SENSOR_BLACK_THRESHOLD) count++;
    }
    return count;
}

bool sensor_is_cross(void) { return false; }
bool sensor_is_lost(void)  { return g_last_position == SENSOR_POSITION_LOST; }
