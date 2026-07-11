/**
 * @file    motor.c
 * @brief   电机控制 — TB6612, PWM 由 SysConfig 管理
 *
 * SysConfig PWM: Edge-aligned, INIT_VAL_LOW
 *   CC=1600 → 0% duty, CC=0 → 100% duty
 *   所以实际 CC = 1600 - speed*1600/1000
 *
 * TB6612 控制逻辑 (参考成功项目 D:\qianrushi\code32\car):
 *   前进: AIN1=HIGH AIN2=LOW  (左) / BIN1=LOW BIN2=HIGH (右, 对称安装)
 *   后退: AIN1=LOW AIN2=HIGH  (左) / BIN1=HIGH BIN2=LOW (右)
 *   刹车: AIN1=HIGH AIN2=HIGH (两轮)
 *   滑行: AIN1=LOW AIN2=LOW  (两轮)
 *   STBY=HIGH 使能
 */

#include "motor.h"

/*===========================================================================
 * 设置左电机方向和 PWM
 *===========================================================================*/
static void motor_left_set(int16_t speed)
{
    /* 死区 → 刹车 */
    if (speed > -MOTOR_DEADBAND && speed < MOTOR_DEADBAND) {
        /* 滑行 (coast): AIN1=LOW, AIN2=LOW */
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN_0_PIN);
        DL_TimerG_setCaptureCompareValue(MotorPWM_Left_INST,
            MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
        return;
    }

    /* 限幅 */
    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        /* 左轮前进 (实测验证): AIN1=HIGH, AIN2=LOW */
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN_0_PIN);
    } else {
        /* 左轮后退: AIN1=LOW, AIN2=HIGH */
        speed = -speed;
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN_1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN_0_PIN);
    }

    /* 反转占空比: CC=1600 → 0%, CC=0 → 100% */
    uint32_t cc = MOTOR_PWM_PERIOD -
        (uint32_t)((uint32_t)speed * MOTOR_PWM_PERIOD / MOTOR_PWM_MAX);

    DL_TimerG_setCaptureCompareValue(MotorPWM_Left_INST,
        cc, DL_TIMER_CC_0_INDEX);
}

/*===========================================================================
 * 设置右电机方向和 PWM (对称安装, 方向与左电机相反)
 *===========================================================================*/
static void motor_right_set(int16_t speed)
{
    if (speed > -MOTOR_DEADBAND && speed < MOTOR_DEADBAND) {
        /* 滑行 (coast): BIN1=LOW, BIN2=LOW */
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN_2_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN_3_PIN);
        DL_TimerG_setCaptureCompareValue(MotorPWM_Right_INST,
            MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
        return;
    }

    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        /* 右轮前进 (对称安装, 方向与左轮相反): BIN1=HIGH, BIN2=LOW */
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN_2_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN_3_PIN);
    } else {
        /* 后退: BIN1=LOW, BIN2=HIGH */
        speed = -speed;
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN_2_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN_3_PIN);
    }

    uint32_t cc = MOTOR_PWM_PERIOD -
        (uint32_t)((uint32_t)speed * MOTOR_PWM_PERIOD / MOTOR_PWM_MAX);

    DL_TimerG_setCaptureCompareValue(MotorPWM_Right_INST,
        cc, DL_TIMER_CC_0_INDEX);
}

/*===========================================================================
 * 启动: 先刹车 + 再拉高 STBY (SysConfig 初始化后 STBY=LOW)
 *===========================================================================*/
void motor_start(void)
{
    /* 先设置滑行状态 (coast, 避免 brake 模式异常) */
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN_1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN_0_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN_2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN_3_PIN);

    /* PWM 占空比 = 0% */
    DL_TimerG_setCaptureCompareValue(MotorPWM_Left_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MotorPWM_Right_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);

    /* 启动 PWM 计数器 */
    DL_TimerG_startCounter(MotorPWM_Left_INST);
    DL_TimerG_startCounter(MotorPWM_Right_INST);

    /* PWM 就绪 → 拉高 STBY 使能 TB6612 */
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN_4_PIN);
}

/*===========================================================================
 * 同时设置两轮速度
 *===========================================================================*/
void motor_set_both(int16_t left_speed, int16_t right_speed)
{
    motor_left_set(left_speed);    /* 左轮=ChA=TIMG6/PA29 */
    motor_right_set(right_speed);  /* 右轮=ChB=TIMG7/PA7 */
}

/*===========================================================================
 * 紧急停止 → 刹车 + STBY=LOW
 *===========================================================================*/
void motor_stop(void)
{
    motor_left_set(0);
    motor_right_set(0);
    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN_4_PIN);
}

void motor_test_chA(int16_t speed) { motor_left_set(speed); }
void motor_test_chB(int16_t speed) { motor_right_set(speed); }
