# 循迹小车项目文档

## 一、项目概述

基于 TI MSPM0G3507 的智能循迹小车，采用双弧形传感器布局 + JY61P 陀螺仪航向锁定，实现椭圆形赛道自动行驶。

### 赛道规格

```
        A ══════ 弧线(左转180°) ══════ B
        ‖                              ‖
      直线                            直线
     (100cm)                         (100cm)
        ‖                              ‖
        D ══════ 弧线(右转180°) ══════ C

总长 220cm × 总宽 120cm   |   弧线半径 R=40cm   |   总路程 ≈351cm
```

### 行驶路径

```
A → 弧线(左转) → B → 直线 → C → 弧线(右转) → D → 直线 → A
```

---

## 二、硬件清单

| 模块 | 型号 | 数量 | 用途 |
|------|------|------|------|
| MCU | MSPM0G3507 LaunchPad | 1 | 主控 |
| 电机驱动 | TB6612 | 1 | 双路电机驱动 |
| 电机 | MG513 (1:28 减速) | 2 | 左右轮驱动 |
| 陀螺仪 | JY61P | 1 | 航向角度检测 |
| 灰度传感器 | 12 路 (UART) | 1 | 黑线检测 |
| 电池 | 7.2~12V | 1 | 电源 |

---

## 三、接线总表

### 3.1 电机驱动 (TB6612)

| TB6612 | MSPM0 引脚 | LaunchPad | 功能 |
|--------|-----------|-----------|------|
| PWMA | PA29 | J2-8 | 左电机 PWM |
| AIN1 | PA12 | J2-13 | 左电机方向1 |
| AIN2 | PA13 | J2-14 | 左电机方向2 |
| PWMB | PA7 | J2-13 | 右电机 PWM |
| BIN1 | PB0 | J1-12 | 右电机方向1 |
| BIN2 | PB1 | J4-37 | 右电机方向2 |
| STBY | PB12 | J4-27 | 待机使能 (HIGH=使能) |
| VM | 电池+ | — | 电机电源 |
| VCC | 3.3V | J1-1 | 逻辑电源 |
| GND | GND | J1-10 | 公共地 |

### 3.2 灰度传感器 (UART1)

| 传感器 | MSPM0 引脚 | 说明 |
|--------|-----------|------|
| TX → MCU | PB7 (UART1 RX) | 传感器数据 115200bps |
| RX ← MCU | PB6 (UART1 TX) | 遥测输出 / 预留 |
| S0~S11 | PA14,PA15,PA16,PA17,PA24,PA25,PA26,PA27,PA0,PA28,PA1,PA31 | 12路数字输入 |

### 3.3 陀螺仪 (JY61P, UART3)

| JY61P | MSPM0 引脚 | 说明 |
|-------|-----------|------|
| TX | PB3 (UART3 RX) | 姿态数据 115200bps |
| RX | PB2 (UART3 TX) | 配置命令 (可选) |
| VCC | 3.3V/5V | 电源 |
| GND | GND | 公共地 |

### 3.4 串口遥测

| USB-TTL | MSPM0 引脚 | 说明 |
|---------|-----------|------|
| RX | PB6 (UART1 TX) | 接收遥测数据 115200bps |
| GND | GND | 公共地 |

---

## 四、JY61P 陀螺仪配置

使用 JY61P 上位机软件配置（Windows）：

| 参数 | 设置值 |
|------|--------|
| 通讯速率 | **115200** |
| 回传速率 | **200Hz** |
| 输出内容 | 加速度 + 角速度 + 欧拉角 + 磁场 |
| 算法 | 六轴 |
| 安装方向 | 水平 |
| 带宽 | 20Hz |

上位机软件中勾选「欧拉角」输出（0x53 角度包），协议格式：

```
帧头    类型    数据(8字节)           校验和
0x55    0x53   RollL RollH PitchL PitchH YawL YawH TmpL TmpH  SUM
```

---

## 五、软件架构

### 5.1 文件结构

```
empty/
├── main.c          # 主程序: 控制逻辑 + 状态机
├── jy61p.c/h       # JY61P 陀螺仪 0x55 协议解析
├── sensor.c/h      # 12 路灰度传感器 + 路口检测
├── motor.c/h       # TB6612 电机 PWM 驱动
├── pid.c/h         # PID 控制器
├── system.c/h      # 系统初始化
├── uart_pid.c/h    # UART 通信
├── empty.syscfg    # SysConfig 引脚配置
└── ti_msp_dl_config.h  # 自动生成的驱动配置
```

### 5.2 控制架构

```
┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────┐
│ 12路灰度  │───→│ sensor_calc  │───→│   P 控制器    │───→│ TB6612   │───→ 左右电机
│ 传感器   │    │ _position()  │    │              │    │ 电机驱动  │
│ UART协议  │    │ 加权求位置   │    │ left =base+diff│    │          │
│          │    │ pos∈[-550,550]│   │ right=base-diff│   │          │
└──────────┘    └──────────────┘    └──────┬───────┘    └──────────┘
                                           │
                                    ┌──────┴───────┐
                                    │   JY61P      │
                                    │   航向锁定    │
                                    │  (空白间隙)   │
                                    └──────────────┘
```

### 5.3 控制策略

```
主循环 (200Hz, 每 5ms)
  │
  ├─ 读取传感器位置 pos
  ├─ 读取陀螺仪 Yaw 角度
  │
  ├─ black_cnt > 0 ?
  │   ├─ YES → P控制循迹 + 更新 heading_target
  │   └─ NO  → 航向锁定直行
  │              heading_error = Yaw - heading_target
  │              限幅 ±30° → corr = err × KP
  │              left=BASE+corr, right=BASE-corr
  │
  └─ 速度限幅 → motor_set_both() → delay 5ms
```

---

## 六、核心算法

### 6.1 传感器位置计算 (P 控制)

```c
// 12 路传感器权重 (双弧形布局)
w[12] = {-6,-5,-4,-3,-2,-1, 1,2,3,4,5,6}

// 加权求和
pos = sum(black_sensors × w) × 50    // 范围: [-550, +550]

// P 控制
diff = pos × 0.75
left  = BASE_SPEED + diff
right = BASE_SPEED - diff
```

### 6.2 陀螺仪航向锁定

```c
// 有黑线时持续记录航向
heading_target = Yaw    // JY61P 欧拉角, ±180°

// 空白间隙时计算偏差
heading_error = Yaw - heading_target    // 归一化到 [-180°, +180°]

// 限幅修正 (防止猛转)
err = clamp(heading_error, -30°, +30°)
corr = err × HEADING_KP    // KP=12, 每度偏差修正12单位速度

left  = BASE_SPEED + corr
right = BASE_SPEED - corr
```

### 6.3 路口检测

```c
// 黑点数 ≥ JUNC_BLACK_THRESH(6) → 持续确认 → 路口触发
// 冷却时间 JUNC_COOLDOWN(120) 防止重复触发
uint8_t black_cnt = sensor_get_black_count();
if (black_cnt >= 6) confirm_count++;
if (confirm_count >= 5) → 路口触发, 进入冷却
```

---

## 七、关键参数

| 参数 | 默认值 | 说明 | 调大效果 | 调小效果 |
|------|--------|------|---------|---------|
| `BASE_SPEED` | 200 | 基础速度 (PWM 0~1000) | 更快 | 更慢更稳 |
| `HEADING_KP` | 12 | 航向修正强度 | 更猛回正 | 更温和 |
| `HEADING_MAX_ERR` | 30° | 航向修正上限 | 允许更大修正 | 更保守 |
| `MIN_SPEED` | 30 | 死区阈值 | 更灵敏 | 过滤抖动 |
| `LOST_TIMEOUT` | 9999 | 丢线超时 | 永不超时 | 早点减速 |
| `JUNC_BLACK_THRESH` | 6 | 路口黑点阈值 | 更难触发 | 更易触发 |
| `JUNC_COOLDOWN` | 120 | 路口冷却周期 | 更长间隔 | 更短间隔 |

---

## 八、串口遥测格式

### GYRO 行 (每 ~200ms)

```
GYRO: Yaw×10 Pitch×10 GyroZ×10 HdgErr×10 L_speed R_speed PKT
```

| 字段 | 含义 | 单位 |
|------|------|------|
| Yaw | 偏航角 | 0.1° |
| Pitch | 俯仰角 | 0.1° |
| GyroZ | Z轴角速度 | 0.1°/s |
| HdgErr | 航向偏差 | 0.1° |
| L_speed | 左轮速度 | PWM |
| R_speed | 右轮速度 | PWM |
| PKT | 有效包数 | 个 |

### 主遥测行 (每 ~200ms)

```
pos left right lost_cnt gyroZ black_cnt [sensor_bits]
```

- `pos`: 黑线位置 [-550, +550]
- `sensor_bits`: 12 位二进制, 1=检测到黑线

---

## 九、编译与烧录

### 环境要求

- Code Composer Studio (CCS) Theia
- MSPM0 SDK ≥ 2.10
- SysConfig ≥ 1.26

### 编译步骤

1. 用 CCS 打开 `empty/` 目录
2. 确保 `empty.syscfg` 引脚配置正确
3. Build → Debug → 烧录到 LaunchPad
4. 串口助手连接 PB6 (UART1 TX), **115200 8N1**

---

## 十、调试建议

1. **Pitch 异常大 (如 60°)**: JY61P 安装方向不对, 在上位机改「安装方向」为「垂直」
2. **陀螺仪无数据**: 检查 JY61P TX→PB3 接线, 波特率是否都设 115200
3. **间隙处跑偏**: 增大 `HEADING_KP` (15~20), 或减小 `HEADING_MAX_ERR` (20°)
4. **弯道转不过来**: 减小 `BASE_SPEED` (150~180), 让车慢点过弯
5. **死区不转**: `MIN_SPEED=30` 以下电机不动, 太低可调至 20
