/**
 * @file    mpu6050.c
 * @brief   MPU6050 陀螺仪加速度计 — I2C 驱动实现 (MSPM0G3507)
 *
 * I2C: PB2=SCL, PB3=SDA, 400kHz Fast Mode
 * 地址: 0x68 (AD0 接 GND)
 *
 * 使用说明:
 *   1. SysConfig 中配置 I2C 为 Controller (Master), Fast Mode (400kHz)
 *   2. 将生成的 I2C 实例名填入下方的 MPU6050_I2C
 *   3. 调用 mpu6050_init() → mpu6050_calibrate() → 循环 mpu6050_update_angle()
 */

#include "mpu6050.h"
#include <math.h>

/*===========================================================================
 * I2C 实例 — 需根据 SysConfig 生成的名称修改
 *===========================================================================*/
/* 若 SysConfig 中 I2C 模块名为 "I2C_Sensor" → 使用 I2C_Sensor_INST */
#ifndef MPU6050_I2C
#define MPU6050_I2C             I2C_Sensor_INST
#endif

/*===========================================================================
 * 内部状态
 *===========================================================================*/
static mpu6050_gyro_range_t  g_gyro_range  = MPU6050_GYRO_250;
static mpu6050_accel_range_t g_accel_range = MPU6050_ACCEL_2G;
static float g_gyro_lsb  = 131.0f;          /* 陀螺仪分辨率 (LSB/°/s) */
static float g_accel_lsb = 16384.0f;        /* 加速度分辨率 (LSB/g)  */

/* 零偏 */
static float g_gyro_bias_x  = 0.0f;
static float g_gyro_bias_y  = 0.0f;
static float g_gyro_bias_z  = 0.0f;
static float g_accel_bias_x = 0.0f;
static float g_accel_bias_y = 0.0f;
static float g_accel_bias_z = 0.0f;

/* 互补滤波姿态角 */
static float g_roll  = 0.0f;
static float g_pitch = 0.0f;
static float g_yaw   = 0.0f;

/* 互补滤波系数 (0~1, 越大越信任陀螺仪) */
#define ALPHA  0.98f

/*===========================================================================
 * I2C 底层读写 (适配 MSPM0 DriverLib)
 *===========================================================================*/

/**
 * @brief 写 MPU6050 寄存器
 * @param reg  寄存器地址
 * @param data 数据字节
 * @return true=成功
 */
static bool mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };

    DL_I2C_ControllerTransfer transfer = {
        .targetAddr   = MPU6050_ADDR,
        .data         = buf,
        .dataSize     = 2,
        .direction    = DL_I2C_CONTROLLER_DIRECTION_TX,
        .timeout      = MPU6050_I2C_TIMEOUT,
    };

    DL_I2C_startControllerTransfer(MPU6050_I2C, &transfer);

    /* 等待传输完成 */
    uint32_t timeout = MPU6050_I2C_TIMEOUT * 1000;
    while (DL_I2C_isControllerBusy(MPU6050_I2C) && --timeout);
    if (timeout == 0) return false;

    /* 检查 NACK */
    if (DL_I2C_getControllerStatus(MPU6050_I2C) & DL_I2C_CONTROLLER_STATUS_NACK) {
        return false;
    }
    return true;
}

/**
 * @brief 读 MPU6050 多个寄存器
 * @param reg   起始寄存器地址
 * @param buf   输出缓冲区
 * @param len   读取字节数
 * @return true=成功
 */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 先写寄存器地址 */
    DL_I2C_ControllerTransfer tx = {
        .targetAddr   = MPU6050_ADDR,
        .data         = &reg,
        .dataSize     = 1,
        .direction    = DL_I2C_CONTROLLER_DIRECTION_TX,
        .timeout      = MPU6050_I2C_TIMEOUT,
    };
    DL_I2C_startControllerTransfer(MPU6050_I2C, &tx);

    uint32_t timeout = MPU6050_I2C_TIMEOUT * 1000;
    while (DL_I2C_isControllerBusy(MPU6050_I2C) && --timeout);
    if (timeout == 0) return false;

    /* 再读取数据 */
    DL_I2C_ControllerTransfer rx = {
        .targetAddr   = MPU6050_ADDR,
        .data         = buf,
        .dataSize     = len,
        .direction    = DL_I2C_CONTROLLER_DIRECTION_RX,
        .timeout      = MPU6050_I2C_TIMEOUT,
    };
    DL_I2C_startControllerTransfer(MPU6050_I2C, &rx);

    timeout = MPU6050_I2C_TIMEOUT * 1000;
    while (DL_I2C_isControllerBusy(MPU6050_I2C) && --timeout);
    if (timeout == 0) return false;

    return true;
}

/**
 * @brief 读 MPU6050 单个寄存器
 */
static bool mpu6050_read_reg(uint8_t reg, uint8_t *val)
{
    return mpu6050_read_regs(reg, val, 1);
}

/*===========================================================================
 * 公开接口
 *===========================================================================*/

/**
 * @brief 读取 WHO_AM_I，验证 I2C 通信
 */
uint8_t mpu6050_who_am_i(void)
{
    uint8_t id = 0;
    if (!mpu6050_read_reg(MPU6050_WHO_AM_I, &id))
        return 0;
    return id;
}

/**
 * @brief 初始化 MPU6050
 *
 * 流程:  唤醒 → 复位 → 设采样率 → 设滤波器 → 设陀螺仪量程 → 设加速度量程
 */
bool mpu6050_init(mpu6050_gyro_range_t gyro_range,
                  mpu6050_accel_range_t accel_range,
                  mpu6050_dlpf_t dlpf)
{
    uint8_t id;

    /* 1. 验证 I2C 通信 */
    id = mpu6050_who_am_i();
    if (id != 0x68) return false;

    /* 2. 唤醒 MPU6050 (清除 SLEEP 位) + 复位 */
    if (!mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80))
        return false;

    /* 等待复位完成 (约 100ms) */
    for (volatile uint32_t i = 0; i < 400000; i++);

    /* 唤醒: 时钟源选 PLL X 轴陀螺仪 */
    if (!mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01))
        return false;

    /* 等待稳定 */
    for (volatile uint32_t i = 0; i < 400000; i++);

    /* 3. 采样率分频 — 1kHz / (1+4) = 200Hz */
    if (!mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x04))
        return false;

    /* 4. 低通滤波器 */
    if (!mpu6050_write_reg(MPU6050_CONFIG, (uint8_t)dlpf))
        return false;

    /* 5. 陀螺仪量程 */
    if (!mpu6050_write_reg(MPU6050_GYRO_CONFIG, (uint8_t)gyro_range))
        return false;

    /* 6. 加速度计量程 */
    if (!mpu6050_write_reg(MPU6050_ACCEL_CONFIG, (uint8_t)accel_range))
        return false;

    /* 保存配置 */
    g_gyro_range  = gyro_range;
    g_accel_range = accel_range;

    /* 7. 分辨率换算 */
    switch (gyro_range) {
        case MPU6050_GYRO_250:   g_gyro_lsb = 131.0f;   break;
        case MPU6050_GYRO_500:   g_gyro_lsb = 65.5f;    break;
        case MPU6050_GYRO_1000:  g_gyro_lsb = 32.8f;    break;
        case MPU6050_GYRO_2000:  g_gyro_lsb = 16.4f;    break;
    }

    switch (accel_range) {
        case MPU6050_ACCEL_2G:   g_accel_lsb = 16384.0f;  break;
        case MPU6050_ACCEL_4G:   g_accel_lsb = 8192.0f;   break;
        case MPU6050_ACCEL_8G:   g_accel_lsb = 4096.0f;   break;
        case MPU6050_ACCEL_16G:  g_accel_lsb = 2048.0f;   break;
    }

    return true;
}

/**
 * @brief 读取原始数据 — 14字节突发读 (ACCEL_X ~ GYRO_Z)
 */
bool mpu6050_read_raw(mpu6050_raw_t *raw)
{
    uint8_t buf[14];

    if (!mpu6050_read_regs(MPU6050_ACCEL_XOUT_H, buf, 14))
        return false;

    raw->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

/*===========================================================================
 * 工程单位转换
 *===========================================================================*/

/**
 * @brief 读取加速度计 (g)
 */
void mpu6050_read_accel(float *ax, float *ay, float *az)
{
    mpu6050_raw_t raw;
    if (!mpu6050_read_raw(&raw)) return;

    *ax = (float)raw.accel_x / g_accel_lsb - g_accel_bias_x;
    *ay = (float)raw.accel_y / g_accel_lsb - g_accel_bias_y;
    *az = (float)raw.accel_z / g_accel_lsb - g_accel_bias_z;
}

/**
 * @brief 读取陀螺仪 (°/s)
 */
void mpu6050_read_gyro(float *gx, float *gy, float *gz)
{
    mpu6050_raw_t raw;
    if (!mpu6050_read_raw(&raw)) return;

    *gx = (float)raw.gyro_x / g_gyro_lsb - g_gyro_bias_x;
    *gy = (float)raw.gyro_y / g_gyro_lsb - g_gyro_bias_y;
    *gz = (float)raw.gyro_z / g_gyro_lsb - g_gyro_bias_z;
}

/**
 * @brief 读取温度
 */
float mpu6050_read_temp(void)
{
    mpu6050_raw_t raw;
    if (!mpu6050_read_raw(&raw)) return 0.0f;
    return (float)raw.temp / 340.0f + 36.53f;
}

/*===========================================================================
 * 零偏校准
 *===========================================================================*/

/**
 * @brief 零偏校准 — 小车静止平放，采样 200 次求均值
 */
void mpu6050_calibrate(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    const int N = 200;

    for (int i = 0; i < N; i++) {
        mpu6050_raw_t raw;
        if (!mpu6050_read_raw(&raw)) continue;

        sum_gx += (float)raw.gyro_x  / g_gyro_lsb;
        sum_gy += (float)raw.gyro_y  / g_gyro_lsb;
        sum_gz += (float)raw.gyro_z  / g_gyro_lsb;
        sum_ax += (float)raw.accel_x / g_accel_lsb;
        sum_ay += (float)raw.accel_y / g_accel_lsb;
        sum_az += (float)raw.accel_z / g_accel_lsb;

        /* 延时 ~5ms */
        for (volatile uint32_t d = 0; d < 40000; d++);
    }

    /* 陀螺仪零偏 */
    g_gyro_bias_x = sum_gx / (float)N;
    g_gyro_bias_y = sum_gy / (float)N;
    g_gyro_bias_z = sum_gz / (float)N;

    /* 加速度计零偏 (Z轴扣除重力) */
    g_accel_bias_x = sum_ax / (float)N;
    g_accel_bias_y = sum_ay / (float)N;
    g_accel_bias_z = (sum_az / (float)N) - 1.0f;

    /* 重置姿态角 */
    g_roll  = 0.0f;
    g_pitch = 0.0f;
    g_yaw   = 0.0f;
}

/*===========================================================================
 * 互补滤波姿态解算
 *===========================================================================*/

/**
 * @brief 互补滤波 — 每 5ms 调用一次
 *
 *   angle = ALPHA * (angle + gyro*dt) + (1-ALPHA) * accel_angle
 *   ALPHA=0.98: 98%信任陀螺仪 (短期准), 2%信任加速度计 (长期不漂)
 */
void mpu6050_update_angle(float dt)
{
    float ax, ay, az, gx, gy, gz;
    mpu6050_raw_t raw;

    if (!mpu6050_read_raw(&raw)) return;

    /* 工程单位 */
    ax = (float)raw.accel_x / g_accel_lsb - g_accel_bias_x;
    ay = (float)raw.accel_y / g_accel_lsb - g_accel_bias_y;
    az = (float)raw.accel_z / g_accel_lsb - g_accel_bias_z;
    gx = (float)raw.gyro_x  / g_gyro_lsb  - g_gyro_bias_x;
    gy = (float)raw.gyro_y  / g_gyro_lsb  - g_gyro_bias_y;
    gz = (float)raw.gyro_z  / g_gyro_lsb  - g_gyro_bias_z;

    /* 加速度计推算角度 */
    float accel_roll  = atan2f(ay, az) * 180.0f / 3.14159265f;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.14159265f;

    /* 互补滤波 */
    g_roll  = ALPHA * (g_roll  + gx * dt) + (1.0f - ALPHA) * accel_roll;
    g_pitch = ALPHA * (g_pitch + gy * dt) + (1.0f - ALPHA) * accel_pitch;

    /* 偏航角纯积分 (会漂移, 仅作参考) */
    g_yaw += gz * dt;
}

/**
 * @brief 获取姿态角
 */
mpu6050_angle_t mpu6050_get_angle(void)
{
    mpu6050_angle_t angle = { g_roll, g_pitch, g_yaw };
    return angle;
}

/**
 * @brief 获取 Z 轴角速度 — 用于循迹转向控制
 */
float mpu6050_get_gyro_z(void)
{
    mpu6050_raw_t raw;
    if (!mpu6050_read_raw(&raw)) return 0.0f;
    return (float)raw.gyro_z / g_gyro_lsb - g_gyro_bias_z;
}
