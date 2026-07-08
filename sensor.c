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
static uint8_t  g_i2c_error_count;    /* I2C连续超时计数 */

/*===========================================================================
 * 读取 I2C — 带超时保护 (防止传感器掉线时死循环)
 *
 * I2C超时值推导:
 *   400kHz I2C, 7字节 ≈ 7×9bits / 400kHz ≈ 160μs (无stretch)
 *   留100倍余量: 16000μs ≈ 500,000 cycles @32MHz
 *===========================================================================*/
#define I2C_TIMEOUT_CYCLES  10000    /* ~300μs/byte, 足够400kHz I2C+时钟拉伸 */

static bool read_iic(void)
{
    uint8_t i;
    uint32_t timeout;

    DL_I2C_startControllerTransfer(I2C_Sensor_INST, I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, I2C_PACKET_SIZE);

    for (i = 0; i < I2C_PACKET_SIZE; i++) {
        timeout = I2C_TIMEOUT_CYCLES;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_Sensor_INST)) {
            if (--timeout == 0) {
                return false;   /* 超时放弃, 下个周期重试 */
            }
        }
        g_rx_packet[i] = DL_I2C_receiveControllerData(I2C_Sensor_INST);
    }

    if (g_rx_packet[0] == '#') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i] = g_rx_packet[i + 1];
        g_data_ready |= 0x01;
    } else if (g_rx_packet[0] == '!') {
        for (i = 0; i < 6; i++)
            g_sensor_data[i + 6] = g_rx_packet[i + 1];
        g_data_ready |= 0x02;
    }
    /* else: 无效帧, 忽略但不算失败 */

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
    g_i2c_error_count = 0;

    /* 参考项目关了毛刺滤波器 */
    DL_I2C_disableAnalogGlitchFilter(I2C_Sensor_INST);
}

/*===========================================================================
 * 计算黑线位置 — 参照 STM32 成功项目加权求和算法
 *
 * 改进:
 *   1. 一次调用连续读多帧直到凑齐 '#' + '!'
 *   2. 二值化 + 加权求和 (对齐参考项目 tracker.c)
 *   3. 12路权重对称分布
 *===========================================================================*/
int16_t sensor_calc_position(void)
{
    int16_t  sum = 0;
    int16_t  pos;
    uint8_t  i;
    uint8_t  attempt;

    /*--- 连续读帧, 直到凑齐两帧或超时 ---*/
    g_data_ready = 0;
    for (attempt = 0; attempt < 4; attempt++) {
        if (!read_iic()) {
            g_i2c_error_count++;
            if (g_i2c_error_count > 100) {
                g_last_position = SENSOR_POSITION_LOST;
            }
            return g_last_position;
        }
        if (g_data_ready == 0x03) break;  /* 两帧都收到了 */
    }

    if (g_data_ready != 0x03) {
        /* 4次都没凑齐 → 数据异常 */
        return g_last_position;
    }
    g_i2c_error_count = 0;

    /*--- 加权求和 (对齐参考项目): 黑线=低值, 阈值以下视为压线 ---*/
    /* 权重: S0(左端)=-6 ... S5=-1,  S6=+1 ... S11(右端)=+6 */
    static const int8_t weight[12] = {-6, -5, -4, -3, -2, -1,
                                        1,  2,  3,  4,  5,  6};

    for (i = 0; i < 12; i++) {
        if (g_sensor_data[i] < SENSOR_BLACK_THRESHOLD) {
            sum += weight[i];
        }
    }

    /* sum>0 → 线在右 → 右转 (左轮加速)
     * sum<0 → 线在左 → 左转 (右轮加速)
     *
     * 放大到 [-550, +550] 范围: sum×50 (sum范围约±21, ×50≈±1050, 限幅到±550)
     */
    pos = sum * 50;

    if (pos > SENSOR_POSITION_MAX)  pos = SENSOR_POSITION_MAX;
    if (pos < SENSOR_POSITION_MIN)  pos = SENSOR_POSITION_MIN;

    /* !!! 诊断: 强制注入偏移, 验证 sensor→line_track 数据流 !!! */
    {
        static uint16_t tick = 0;
        tick++;
        if (tick < 100)       pos =  300;   /* 前0.5s: 强制右偏 */
        else if (tick < 200)  pos = -300;   /* 后0.5s: 强制左偏 */
        else                  tick = 0;     /* 循环       */
    }

    g_last_position = pos;
    return pos;
}

int16_t sensor_get_last_position(void) { return g_last_position; }
