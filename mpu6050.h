/**
 * @file    mpu6050.h
 * @brief   MPU6050 陀螺仪加速度计 — I2C 驱动 (MSPM0G3507)
 *
 * I2C 引脚: PB2=SCL, PB3=SDA, 地址 0x68 (AD0=GND)
 * 最高支持 400kHz Fast Mode
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * I2C 地址
 *===========================================================================*/
#define MPU6050_ADDR            0x68        /* AD0=GND */
#define MPU6050_I2C_TIMEOUT     1000        /* I2C 超时 (ms) */

/*===========================================================================
 * 寄存器映射
 *===========================================================================*/
#define MPU6050_WHO_AM_I        0x75        /* 器件ID, 应返回 0x68 */
#define MPU6050_SMPLRT_DIV      0x19        /* 采样率分频 */
#define MPU6050_CONFIG          0x1A        /* 低通滤波器配置 */
#define MPU6050_GYRO_CONFIG     0x1B        /* 陀螺仪量程 */
#define MPU6050_ACCEL_CONFIG    0x1C        /* 加速度计量程 */
#define MPU6050_PWR_MGMT_1      0x6B        /* 电源管理 */
#define MPU6050_ACCEL_XOUT_H    0x3B        /* 加速度计 X 高字节 */
#define MPU6050_GYRO_XOUT_H     0x43        /* 陀螺仪 X 高字节 */
#define MPU6050_TEMP_OUT_H      0x41        /* 温度传感器 */

/*===========================================================================
 * 陀螺仪量程 (±°/s)  —  GYRO_CONFIG[4:3]
 *===========================================================================*/
typedef enum {
    MPU6050_GYRO_250   = 0x00,             /* ±250°/s,  分辨率 131 LSB/°/s  */
    MPU6050_GYRO_500   = 0x08,             /* ±500°/s,  分辨率 65.5 LSB/°/s */
    MPU6050_GYRO_1000  = 0x10,             /* ±1000°/s, 分辨率 32.8 LSB/°/s */
    MPU6050_GYRO_2000  = 0x18,             /* ±2000°/s, 分辨率 16.4 LSB/°/s */
} mpu6050_gyro_range_t;

/*===========================================================================
 * 加速度计量程 (±g)  —  ACCEL_CONFIG[4:3]
 *===========================================================================*/
typedef enum {
    MPU6050_ACCEL_2G   = 0x00,             /* ±2g,  分辨率 16384 LSB/g  */
    MPU6050_ACCEL_4G   = 0x08,             /* ±4g,  分辨率 8192 LSB/g   */
    MPU6050_ACCEL_8G   = 0x10,             /* ±8g,  分辨率 4096 LSB/g   */
    MPU6050_ACCEL_16G  = 0x18,             /* ±16g, 分辨率 2048 LSB/g   */
} mpu6050_accel_range_t;

/*===========================================================================
 * 低通滤波器带宽  —  CONFIG[2:0]
 *===========================================================================*/
typedef enum {
    MPU6050_DLPF_260HZ = 0x00,             /* 加速度 260Hz,  陀螺仪 256Hz  */
    MPU6050_DLPF_184HZ = 0x01,             /* 加速度 184Hz,  陀螺仪 188Hz  */
    MPU6050_DLPF_94HZ  = 0x02,             /* 加速度 94Hz,   陀螺仪 98Hz   */
    MPU6050_DLPF_44HZ  = 0x03,             /* 加速度 44Hz,   陀螺仪 42Hz   */
    MPU6050_DLPF_21HZ  = 0x04,             /* 加速度 21Hz,   陀螺仪 20Hz   */
    MPU6050_DLPF_10HZ  = 0x05,             /* 加速度 10Hz,   陀螺仪 10Hz   */
    MPU6050_DLPF_5HZ   = 0x06,             /* 加速度 5Hz,    陀螺仪 5Hz    */
} mpu6050_dlpf_t;

/*===========================================================================
 * 原始数据结构
 *===========================================================================*/
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_t;

/*===========================================================================
 * 姿态角数据结构 (互补滤波结果)
 *===========================================================================*/
typedef struct {
    float roll;                             /* 横滚角  (°)              */
    float pitch;                            /* 俯仰角  (°)              */
    float yaw;                              /* 偏航角  (°) — 会漂移     */
} mpu6050_angle_t;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化 MPU6050
 * @param gyro_range  陀螺仪量程
 * @param accel_range 加速度计量程
 * @param dlpf        低通滤波器带宽
 * @return true=成功, false=I2C通信失败
 */
bool mpu6050_init(mpu6050_gyro_range_t gyro_range,
                  mpu6050_accel_range_t accel_range,
                  mpu6050_dlpf_t dlpf);

/**
 * @brief 读取 WHO_AM_I 寄存器
 * @return 器件ID (应为 0x68), 失败返回 0
 */
uint8_t mpu6050_who_am_i(void);

/**
 * @brief 读取原始传感器数据 (加速度计 + 陀螺仪 + 温度)
 * @param raw 输出结构体指针
 * @return true=成功
 */
bool mpu6050_read_raw(mpu6050_raw_t *raw);

/**
 * @brief 读取加速度计 (g)
 * @param ax X轴加速度输出
 * @param ay Y轴加速度输出
 * @param az Z轴加速度输出
 */
void mpu6050_read_accel(float *ax, float *ay, float *az);

/**
 * @brief 读取陀螺仪 (°/s)
 * @param gx X轴角速度输出
 * @param gy Y轴角速度输出
 * @param gz Z轴角速度输出
 */
void mpu6050_read_gyro(float *gx, float *gy, float *gz);

/**
 * @brief 温度 (°C)
 * @return 芯片温度
 */
float mpu6050_read_temp(void);

/**
 * @brief 零偏校准 (静止平放, 采样100次取均值)
 */
void mpu6050_calibrate(void);

/**
 * @brief 互补滤波姿态更新 (需每 5ms 调用一次)
 * @param dt 采样周期 (秒), 如 0.005
 */
void mpu6050_update_angle(float dt);

/**
 * @brief 获取当前姿态角
 * @return 角度结构体
 */
mpu6050_angle_t mpu6050_get_angle(void);

/**
 * @brief 获取 z 轴角速度 (°/s) — 循迹偏航角速度
 */
float mpu6050_get_gyro_z(void);

#endif /* __MPU6050_H__ */
