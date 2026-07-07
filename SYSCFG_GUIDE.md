# SysConfig GPIO 完整配置清单

在 CCS Theia 中打开 `empty.syscfg`，点击 **ADD → GPIO**，依次添加以下全部引脚。

---

## 一、电机方向控制 (5个 GPIO Output)

| 添加顺序 | Name | Port | Pin | Direction | Init Value |
|---------|------|------|-----|-----------|------------|
| 1 | `Motor_AIN1` | GPIOA | PA12 | Output | Low |
| 2 | `Motor_AIN2` | GPIOA | PA13 | Output | Low |
| 3 | `Motor_BIN1` | GPIOA | PA14 | Output | Low |
| 4 | `Motor_BIN2` | GPIOA | PA15 | Output | Low |
| 5 | `Motor_STBY` | GPIOB | PB1 | Output | **High** |

---

## 二、编码器 (4个 GPIO Input)

| 添加顺序 | Name | Port | Pin | Direction | Resistor | Interrupt |
|---------|------|------|-----|-----------|----------|-----------|
| 6 | `Enc_LeftA` | GPIOA | PA10 | Input | Pull-up | **Rising edge** |
| 7 | `Enc_LeftB` | GPIOA | PA11 | Input | Pull-up | (不勾选) |
| 8 | `Enc_RightA` | GPIOB | PB6 | Input | Pull-up | **Rising edge** |
| 9 | `Enc_RightB` | GPIOB | PB7 | Input | Pull-up | (不勾选) |

---

## 三、12路灰度传感器 (12个 GPIO Input)

| 添加顺序 | Name | Port | Pin | Direction |
|---------|------|------|-----|-----------|
| 10 | `Sensor_S0` | GPIOA | PA16 | Input |
| 11 | `Sensor_S1` | GPIOA | PA17 | Input |
| 12 | `Sensor_S2` | GPIOA | PA18 | Input |
| 13 | `Sensor_S3` | GPIOA | PA21 | Input |
| 14 | `Sensor_S4` | GPIOA | PA22 | Input |
| 15 | `Sensor_S5` | GPIOA | PA23 | Input |
| 16 | `Sensor_S6` | GPIOA | PA26 | Input |
| 17 | `Sensor_S7` | GPIOA | PA27 | Input |
| 18 | `Sensor_S8` | GPIOA | PA28 | Input |
| 19 | `Sensor_S9` | GPIOA | PA29 | Input |
| 20 | `Sensor_S10` | GPIOB | PB2 | Input |
| 21 | `Sensor_S11` | GPIOB | PB3 | Input |

---

## 四、PWM 模块

ADD → PWM:

| 属性 | 值 |
|------|-----|
| Name | `MotorPWM` |
| Peripheral | TIMG0 |
| CCP0 Pin | PA5 |
| CCP1 Pin | PA6 |
| Timer Count | **1600** |
| Clock Prescale | 1 |

---

## 五、TIMER 模块

ADD → TIMER:

| 属性 | 值 |
|------|-----|
| Name | `CtrlTimer` |
| Peripheral | TIMG12 |
| Timer Mode | Periodic |
| Timer Count | 20000 |
| Clock Prescale | 8 |
| Start Timer | 不勾选 |
| Interrupts | 勾选 ZERO |

---

## 完成后的模块清单

```
SYSCTL        (系统时钟)
Board         (板级支持)
Motor_AIN1    GPIOA.PA12  Output Low
Motor_AIN2    GPIOA.PA13  Output Low
Motor_BIN1    GPIOA.PA14  Output Low
Motor_BIN2    GPIOA.PA15  Output Low
Motor_STBY    GPIOB.PB1   Output High
Enc_LeftA     GPIOA.PA10  Input Pull-up IntRise
Enc_LeftB     GPIOA.PA11  Input Pull-up
Enc_RightA    GPIOB.PB6   Input Pull-up IntRise
Enc_RightB    GPIOB.PB7   Input Pull-up
Sensor_S0~S11 (PA16~PA29, PB2~PB3)  12路 Input
MotorPWM      TIMG0  PA5/PA6  20kHz
CtrlTimer     TIMG12  200Hz 中断
```

## 六、保存编译

Ctrl+S → Build (🔨) → Debug (🐞)