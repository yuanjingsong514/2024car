/**
 * @file    system.c
 * @brief   编码器中断 + 控制定时器 + ISR
 *
 * GPIO 初始化由 SysConfig (SYSCFG_DL_GPIO_init) 完成
 * PWM 初始化由 SysConfig (SYSCFG_DL_MotorPWM_*) 完成
 * 本文件只负责: 编码器中断使能、控制定时器、中断服务
 */

#include "system.h"

volatile bool     g_control_flag    = false;
volatile int32_t  g_enc_left_count  = 0;
volatile int32_t  g_enc_right_count = 0;

/*===========================================================================
 * 编码器中断使能
 *
 * SysConfig 已配好 GPIO Input+Pull-up, 这里只需:
 *   1. 设置中断触发边沿 (上升沿)
 *   2. 使能 GPIO 中断
 *   3. 使能 NVIC
 *===========================================================================*/
void encoder_interrupt_init(void)
{
    /* 左编码器 A 相 — PB18, 上升沿中断 (PIN>=16 用 Upper) */
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_18_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_18);
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_18);

    /* 右编码器 A 相 — PB21, 上升沿中断 */
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_21_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_21);
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_21);

    /* GPIOB 中断由 GROUP2_IRQHandler 处理, 使能 NVIC */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/*===========================================================================
 * 控制定时器 — TIMG12, 32MHz / (8*9) ≈ 444kHz, period=2222 → ~5ms (200Hz)
 *===========================================================================*/
void control_timer_init(void)
{
    DL_TimerG_enablePower(TIMG12);

    DL_TimerG_setClockConfig(TIMG12,
        &(DL_TimerG_ClockConfig) {
            .clockSel    = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
            .prescale    = 8,
        });

    /* period = 32MHz / 72 * 0.005s ≈ 2222 */
    DL_TimerG_initTimerMode(TIMG12,
        &(DL_TimerG_TimerConfig) {
            .period     = 2222,
            .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
            .startTimer = DL_TIMER_STOP,
        });

    DL_TimerG_enableInterrupt(TIMG12, CONTROL_TIMER_IIDX);
    NVIC_EnableIRQ(CONTROL_TIMER_IRQn);
}

/*===========================================================================
 * ISR: TIMG12 控制定时器 (每 5ms)
 *===========================================================================*/
void TIMG12_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMG12, CONTROL_TIMER_IIDX);
    g_control_flag = true;
}

/*===========================================================================
 * ISR: GPIOB 编码器中断 (GROUP2)
 *
 * PB18=左编码器A相, PB21=右编码器A相
 * 上升沿触发后, 读 B 相电平判断方向:
 *   A↑且B=HIGH → 正转 → count++
 *   A↑且B=LOW  → 反转 → count--
 *===========================================================================*/
void GROUP2_IRQHandler(void)
{
    /* 读 A 相电平 — 上升沿中断后引脚应为 HIGH */
    uint32_t left_a  = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_18);
    uint32_t right_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_21);

    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_18 | DL_GPIO_PIN_21);

    /* 左编码器 (PB18=A, PB17=B) */
    if (left_a) {
        if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_17))
            g_enc_left_count++;
        else
            g_enc_left_count--;
    }

    /* 右编码器 (PB21=A, PB22=B) */
    if (right_a) {
        if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_22))
            g_enc_right_count++;
        else
            g_enc_right_count--;
    }
}
