/**
 * @file    main.c
 * @brief   循迹小车 — 赛道状态机 + 航向锁定 + JY61P陀螺仪
 *
 * 赛道: A→弧线(左转)→B→直线→C→弧线(右转)→D→直线→A停车
 * 传感器: 两个弧形 + 中间空白, 空白处陀螺仪航向锁定直行
 */

#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "uart_pid.h"
#include "jy61p.h"

/*===========================================================================
 * 控制参数
 *===========================================================================*/
#define BASE_SPEED          200
#define MAX_SPEED           800
#define MIN_SPEED           30    /* 死区, 低于此值电机不动 */
#define JUNC_BLACK_THRESH   6
#define JUNC_CONFIRM        5
#define JUNC_COOLDOWN       120
#define LOST_TIMEOUT        9999  /* 基本不触发超时, 依赖航向锁定 */
#define HEADING_KP          12    /* 航向锁定: 每度偏差修正量 */
#define HEADING_MAX_ERR     30    /* 航向修正最大偏差 (°) */
#define HEADING_LOST_SLOW   60    /* 丢线超时后低速 */

/*===========================================================================
 * UART1 ISR — 传感器
 *===========================================================================*/
void UART1_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(PID_UART_INST) == DL_UART_MAIN_IIDX_RX)
        sensor_feed_byte(DL_UART_Main_receiveData(PID_UART_INST));
}

/*===========================================================================
 * UART3 ISR — JY61P
 *===========================================================================*/
volatile uint32_t g_jy61_rx_cnt = 0;

#define RAW_BUF_SIZE 32
static volatile uint8_t g_raw_buf[RAW_BUF_SIZE];
static volatile uint8_t g_raw_idx = 0;

void UART3_IRQHandler(void)
{
    uint8_t b = DL_UART_Main_receiveData(UART_61_INST);
    g_raw_buf[g_raw_idx % RAW_BUF_SIZE] = b;
    g_raw_idx++;
    jy61p_feed_byte(b);
    g_jy61_rx_cnt++;
}

/*===========================================================================
 * 遥测
 *===========================================================================*/
static uint8_t itoa_small(int16_t v, char *buf)
{
    uint8_t n = 0;
    if (v < 0) { buf[n++] = '-'; v = -v; }
    if (v >= 100) buf[n++] = '0' + (v / 100) % 10;
    if (v >= 10)  buf[n++] = '0' + (v / 10) % 10;
    buf[n++] = '0' + (v % 10);
    return n;
}

static void uart_send(const char *s)
{
    for (int i = 0; s[i]; i++)
        DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)s[i]);
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
    SYSCFG_DL_init();
    motor_start();
    sensor_init();
    jy61p_init();

    junction_detector_t jd;
    junction_detector_init(&jd, JUNC_BLACK_THRESH, JUNC_CONFIRM, JUNC_COOLDOWN);

    /* UART1 RX 中断: 传感器 */
    DL_UART_Main_enableInterrupt(PID_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(PID_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(PID_UART_INST_INT_IRQN);

    /* UART3 RX 中断: JY61P */
    DL_UART_Main_enableInterrupt(UART_61_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_61_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_61_INST_INT_IRQN);

    uart_send("TRACK\r\n");

    /* 控制变量 */
    uint16_t tick        = 0;
    uint8_t  junc_cnt    = 0;
    uint16_t lost_cnt    = 0;
    int16_t  pos         = 0;
    int16_t  left        = BASE_SPEED;
    int16_t  right       = BASE_SPEED;
    float    last_yaw    = 0.0f;
    float    gyro_z      = 0.0f;
    float    gyro_z_cal  = 0.0f;
    float    gyro_bias   = 0.0f;
    int      bias_cnt    = 0;

    /* 航向锁定 */
    float    heading_target = 0.0f;
    bool     heading_valid  = false;
    float    heading_error  = 0.0f;

    while (1) {
        /* 变量声明必须在 block 开头 (C89) */
        jy61p_data_t gyro_d;

        pos = sensor_calc_position();

        /* 始终读取最新陀螺仪数据 */
        jy61p_get_data(&gyro_d);

        if (jy61p_is_new()) {
            gyro_z = (gyro_d.yaw - last_yaw) * 200.0f;
            last_yaw = gyro_d.yaw;

            if (bias_cnt < 200) {
                gyro_bias += gyro_z;
                bias_cnt++;
                if (bias_cnt == 200) gyro_bias /= 200.0f;
            }
            gyro_z_cal = gyro_z - gyro_bias;
            jy61p_clear_new();
        }

        /* ═══════════════════════════════════════════════════════
         * 控制: 传感器循迹 + 陀螺仪航向锁定穿越空白
         *
         *  |gyro_z| 小 → 直线间隙 → 航向锁定直行
         *  |gyro_z| 大 → 弯道丢线 → 不锁航向，自然过弯
         * ═══════════════════════════════════════════════════════ */
        {
            uint8_t black_cnt = sensor_get_black_count();

            /* 始终计算航向偏差 */
            heading_error = gyro_d.yaw - heading_target;
            if (heading_error > 180.0f)  heading_error -= 360.0f;
            if (heading_error < -180.0f) heading_error += 360.0f;

            if (black_cnt == 0) {
                lost_cnt++;

                /* 空白间隙 → 航向锁定直行 */
                if (heading_valid) {
                    /* 限幅修正: 防止角度差太大导致猛转 */
                    float err = heading_error;
                    if (err > HEADING_MAX_ERR)  err = HEADING_MAX_ERR;
                    if (err < -HEADING_MAX_ERR) err = -HEADING_MAX_ERR;
                    int16_t corr = (int16_t)(err * HEADING_KP);
                    left  = BASE_SPEED + corr;
                    right = BASE_SPEED - corr;
                } else {
                    left  = BASE_SPEED;
                    right = BASE_SPEED;
                }
            } else {
                /* 有黑线 → 传感器P控制 + 更新航向 */
                lost_cnt = 0;
                if (black_cnt < JUNC_BLACK_THRESH) {
                    heading_target = gyro_d.yaw;
                    heading_valid  = true;
                    heading_error  = 0.0f;
                }
                int16_t diff  = (pos * 3) >> 2;
                left  = BASE_SPEED + diff;
                right = BASE_SPEED - diff;
            }
        }

        /* 速度限幅 */
        if (left  > MAX_SPEED)  left  = MAX_SPEED;
        if (left  < -MAX_SPEED) left  = -MAX_SPEED;
        if (right > MAX_SPEED)  right = MAX_SPEED;
        if (right < -MAX_SPEED) right = -MAX_SPEED;
        if (left  > -MIN_SPEED && left  < MIN_SPEED) left  = 0;
        if (right > -MIN_SPEED && right < MIN_SPEED) right = 0;

        motor_set_both(left, right);

        if (junction_detector_update(&jd)) junc_cnt++;

        /* ═══════════════════════════════════════════════════════
         * 遥测 (每 ~200ms)
         * ═══════════════════════════════════════════════════════ */
        tick++;
        if (tick >= 40) {
            tick = 0;

            /* ── GYRO行: 陀螺仪角度 + 角速度 + 航向偏差 + 电机速度 ── */
            {
                extern volatile uint32_t g_jy61_pkt;
                char jbuf[80]; uint8_t jn = 0;
                /* "G: Yaw Pitch GyroZ HdgErr L_motor R_motor PKT" */
                jbuf[jn++] = 'G'; jbuf[jn++] = ':';
                jn += itoa_small((int16_t)(gyro_d.yaw * 10), jbuf + jn);   jbuf[jn++] = ' ';
                jn += itoa_small((int16_t)(gyro_d.pitch * 10), jbuf + jn); jbuf[jn++] = ' ';
                jn += itoa_small((int16_t)(gyro_z_cal * 10), jbuf + jn);   jbuf[jn++] = ' ';
                jn += itoa_small((int16_t)(heading_error * 10), jbuf + jn); jbuf[jn++] = ' ';
                jn += itoa_small(left, jbuf + jn);                         jbuf[jn++] = ' ';
                jn += itoa_small(right, jbuf + jn);                        jbuf[jn++] = ' ';
                jn += itoa_small((int16_t)g_jy61_pkt, jbuf + jn);
                jbuf[jn++] = '\r'; jbuf[jn++] = '\n'; jbuf[jn] = '\0';
                uart_send(jbuf);
            }

            /* ── 主遥测行: 位置 速度 传感器 ── */
            {
                char buf[80]; uint8_t n = 0;
                n += itoa_small(pos, buf + n);  buf[n++] = ' ';
                n += itoa_small(left, buf + n); buf[n++] = ' ';
                n += itoa_small(right, buf + n); buf[n++] = ' ';
                n += itoa_small((int16_t)lost_cnt, buf + n); buf[n++] = ' ';
                n += itoa_small((int16_t)(gyro_z_cal * 10), buf + n); buf[n++] = ' ';
                n += itoa_small((int16_t)sensor_get_black_count(), buf + n); buf[n++] = ' ';
                buf[n++] = '[';
                for (uint8_t s = 0; s < 12; s++)
                    buf[n++] = sensor_get_raw(s) ? '1' : '0';
                buf[n++] = ']';
                buf[n++] = '\r'; buf[n++] = '\n';
                for (uint8_t i = 0; i < n; i++)
                    DL_UART_Main_transmitDataBlocking(PID_UART_INST, (uint8_t)buf[i]);
            }
        }

        delay_cycles(CPUCLK_FREQ / 200);
    }
}
