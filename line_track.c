/**
 * @file    line_track.c
 * @brief   循迹控制 — 位置PID + 速度PID 双闭环
 *
 * 控制架构 (对齐 STM32 成功项目 D:\qianrushi\code32\car):
 *
 *   传感器位置 ──→ 位置PID (setpoint=0) ──→ 差速量
 *                                                  ↓
 *                      base_speed ± differential ──→ 左右目标速度
 *                                                  ↓
 *   编码器脉冲 ──→ 估算PWM ──→ 速度PID ──→ PWM修正
 *                                                  ↓
 *                                          motor_set_both(left, right)
 *
 * 参考文献:
 *   - STM32成功项目 tracker.c (加权求和+TrackCmd)
 *   - STM32成功项目 motor.c   (位置式PID+PWM映射)
 */

#include "line_track.h"
#include "sensor.h"
#include "motor.h"

/*===========================================================================
 * 传感器方向配置
 *
 * sensor_calc_position 约定:
 *   pos = (sum_left - sum_right) / 4
 *   正值 = 左半更黑 = 线在左 → 车应左转 (右轮加速)
 *   负值 = 右半更黑 = 线在右 → 车应右转 (左轮加速)
 *
 * 如果实测方向相反, 将 SENSOR_INVERT 改为 -1
 *===========================================================================*/
#define SENSOR_INVERT       1       /* 1=正常, -1=翻转传感器方向 */

/*===========================================================================
 * 编码器速度估算参数
 *
 * 编码器: MG513XP28, 13PPR×30减速比=390脉冲/圈 (单边沿)
 * 速度估算: PWM ≈ (counts/5ms) × SPEED_SCALE
 *
 * 推导:
 *   满载 PWM=1000 时 ≈ 300RPM = 5rps
 *   5 rps × 390 counts/rev = 1950 counts/s
 *   每5ms: 1950 × 0.005 ≈ 10 counts/5ms
 *   因此: PWM_est = counts/5ms × 100
 *===========================================================================*/
#define SPEED_SCALE         100     /* counts/5ms → 近似PWM */
#define ENCODER_PPR         390     /* 电机单圈脉冲数(单边沿) */

/*===========================================================================
 * 全局实例
 *===========================================================================*/
static line_track_t g_tracker;

/*===========================================================================
 * 编码器速度测量状态
 *===========================================================================*/
static int32_t s_enc_left_prev  = 0;
static int32_t s_enc_right_prev = 0;

/*===========================================================================
 * 获取编码器速度 (counts / 5ms)
 *===========================================================================*/
static int16_t encoder_get_speed_left(void)
{
    int32_t curr = g_enc_left_count;
    int16_t speed = (int16_t)(curr - s_enc_left_prev);
    s_enc_left_prev = curr;
    return speed;
}

static int16_t encoder_get_speed_right(void)
{
    int32_t curr = g_enc_right_count;
    int16_t speed = (int16_t)(curr - s_enc_right_prev);
    s_enc_right_prev = curr;
    return speed;
}

/*===========================================================================
 * 初始化
 *===========================================================================*/
void line_track_init(void)
{
    line_track_t *t = &g_tracker;

    /* 位置PID: 传感器位置→差速量, 微分先行避免震荡 */
    pid_init(&t->pos_pid, POS_KP_DEFAULT, POS_KI_DEFAULT, POS_KD_DEFAULT,
             0.0f, POS_OUT_LIMIT, POS_INT_LIMIT);
    t->pos_pid.derivative_on_measurement = true;  /* 微分先行, 转弯更丝滑 */

    /* 速度PID 左 */
    pid_init(&t->spd_left_pid, SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f, SPD_OUT_LIMIT, SPD_INT_LIMIT);

    /* 速度PID 右 */
    pid_init(&t->spd_right_pid, SPD_KP_DEFAULT, SPD_KI_DEFAULT, SPD_KD_DEFAULT,
             0.0f, SPD_OUT_LIMIT, SPD_INT_LIMIT);

    t->base_speed     = LINE_BASE_SPEED;
    t->state          = TRACK_STATE_NORMAL;
    t->line_position  = 0;
    t->last_position  = 0;
    t->left_speed     = 0;
    t->right_speed    = 0;
    t->lost_count     = 0;
    t->lost_max       = 40;  /* 连续丢线40次(200ms)判定为丢失 */

    /* 初始化编码器基准 */
    s_enc_left_prev  = g_enc_left_count;
    s_enc_right_prev = g_enc_right_count;
}

/*===========================================================================
 * 参数设置接口
 *===========================================================================*/
void line_track_set_base_speed(int16_t speed)
{
    if (speed > LINE_MAX_SPEED)  speed = LINE_MAX_SPEED;
    if (speed < LINE_MIN_SPEED)  speed = LINE_MIN_SPEED;
    g_tracker.base_speed = speed;
}

void line_track_set_pos_pid(float kp, float ki, float kd)
{
    pid_set_params(&g_tracker.pos_pid, kp, ki, kd);
}

void line_track_set_spd_pid(float kp, float ki, float kd)
{
    pid_set_params(&g_tracker.spd_left_pid,  kp, ki, kd);
    pid_set_params(&g_tracker.spd_right_pid, kp, ki, kd);
}

line_track_t* line_track_get_instance(void)
{
    return &g_tracker;
}

/*===========================================================================
 * 急停
 *===========================================================================*/
void line_track_stop(void)
{
    pid_reset(&g_tracker.pos_pid);
    pid_reset(&g_tracker.spd_left_pid);
    pid_reset(&g_tracker.spd_right_pid);
    g_tracker.state = TRACK_STATE_STOP;
    motor_set_both(0, 0);
}

/*===========================================================================
 * 循迹主循环 — 每5ms由 TIMG12 ISR 触发
 *
 * 流程:
 *   1. 读传感器 → 位置
 *   2. 位置PID → 差速量
 *   3. base_speed ± diff → 左右目标速度
 *   4. 读编码器 → 左右实测速度 (换算为近似PWM)
 *   5. 速度PID → PWM修正
 *   6. 输出 motor_set_both()
 *
 * 方向逻辑 (验证正确):
 *   line在左 → pos>0 → PID error<0 → output<0
 *   → left=base+(-out)=慢, right=base-(-out)=快 → 右轮加速 → 左转追线 ✓
 *===========================================================================*/
void line_track_run(void)
{
    line_track_t *t = &g_tracker;
    const float dt = 0.005f;  /* 5ms 控制周期 */
    int16_t position;
    float pos_output;
    float left_target, right_target;
    int16_t left_enc, right_enc;
    float left_measured, right_measured;
    float left_pwm, right_pwm;

    /* 急停状态不响应 */
    if (t->state == TRACK_STATE_STOP) {
        return;
    }

    /*=================================================================
     * 第1步: 读取传感器位置
     *=================================================================*/
    position = sensor_calc_position();

    /* 丢线检测 */
    if (position == SENSOR_POSITION_LOST) {
        position = t->last_position;
        t->lost_count++;
        if (t->lost_count > t->lost_max) {
            t->state = TRACK_STATE_LOST;
        }
    } else {
        t->lost_count = 0;
        t->state      = TRACK_STATE_NORMAL;
    }

    t->line_position = position;
    t->last_position = position;

    /*=================================================================
     * 第2步: 位置PID → 差速量
     *
     * setpoint=0 表示黑线居中
     * 应用 SENSOR_INVERT 允许一键翻转方向
     *=================================================================*/
    pos_output = pid_calculate(&t->pos_pid,
                   (float)(position * SENSOR_INVERT), dt);

    /*=================================================================
     * 第3步: 计算左右目标速度 (PWM单位)
     *=================================================================*/
    if (t->state == TRACK_STATE_LOST) {
        /* 丢线: 减速直行 (不转圈, 靠惯性找回线) */
        left_target  = (float)t->base_speed * 0.5f;  /* 半速直行 */
        right_target = (float)t->base_speed * 0.5f;
    } else {
        left_target  = (float)t->base_speed + pos_output;
        right_target = (float)t->base_speed - pos_output;
    }

    /* 限幅 */
    if (left_target  > LINE_MAX_SPEED)  left_target  = LINE_MAX_SPEED;
    if (left_target  < -LINE_MAX_SPEED) left_target  = -LINE_MAX_SPEED;
    if (right_target > LINE_MAX_SPEED)  right_target = LINE_MAX_SPEED;
    if (right_target < -LINE_MAX_SPEED) right_target = -LINE_MAX_SPEED;

    /* 死区: 避免电机低速啸叫 */
    if (left_target  > -LINE_MIN_SPEED && left_target  < LINE_MIN_SPEED)
        left_target  = 0.0f;
    if (right_target > -LINE_MIN_SPEED && right_target < LINE_MIN_SPEED)
        right_target = 0.0f;

    /*=================================================================
     * 第4步: 读取编码器速度 → 换算为近似PWM值
     *=================================================================*/
    left_enc  = encoder_get_speed_left();
    right_enc = encoder_get_speed_right();

    left_measured  = (float)left_enc  * (float)SPEED_SCALE;
    right_measured = (float)right_enc * (float)SPEED_SCALE;

    /* 限幅 */
    if (left_measured  > MOTOR_PWM_MAX)  left_measured  = MOTOR_PWM_MAX;
    if (left_measured  < -MOTOR_PWM_MAX) left_measured  = -MOTOR_PWM_MAX;
    if (right_measured > MOTOR_PWM_MAX)  right_measured = MOTOR_PWM_MAX;
    if (right_measured < -MOTOR_PWM_MAX) right_measured = -MOTOR_PWM_MAX;

    /*=================================================================
     * 第5步: 速度PID → PWM修正量
     *
     * 修正逻辑: 实测速度 < 目标 → PID输出正补偿 → 加速
     *=================================================================*/
    pid_set_setpoint(&t->spd_left_pid,  left_target);
    pid_set_setpoint(&t->spd_right_pid, right_target);

    float left_correction  = pid_calculate(&t->spd_left_pid,
                                   left_measured, dt);
    float right_correction = pid_calculate(&t->spd_right_pid,
                                   right_measured, dt);

    /*=================================================================
     * 第6步: 最终 PWM = 目标 + 速度修正
     *=================================================================*/
    left_pwm  = left_target  + left_correction;
    right_pwm = right_target + right_correction;

    /* 最终限幅 */
    if (left_pwm  > MOTOR_PWM_MAX)  left_pwm  = MOTOR_PWM_MAX;
    if (left_pwm  < -MOTOR_PWM_MAX) left_pwm  = -MOTOR_PWM_MAX;
    if (right_pwm > MOTOR_PWM_MAX)  right_pwm = MOTOR_PWM_MAX;
    if (right_pwm < -MOTOR_PWM_MAX) right_pwm = -MOTOR_PWM_MAX;

    /* 最终死区过滤 */
    if (left_pwm  > -MOTOR_DEADBAND && left_pwm  < MOTOR_DEADBAND)
        left_pwm  = 0.0f;
    if (right_pwm > -MOTOR_DEADBAND && right_pwm < MOTOR_DEADBAND)
        right_pwm = 0.0f;

    /*=================================================================
     * 第7步: 输出到电机
     *=================================================================*/
    t->left_speed  = (int16_t)left_pwm;
    t->right_speed = (int16_t)right_pwm;
    motor_set_both((int16_t)left_pwm, (int16_t)right_pwm);
}
