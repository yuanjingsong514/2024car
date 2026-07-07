#!/usr/bin/env python3
"""
AI PID 调参中间件 v2
=====================
支持 DeepSeek / 通义千问 / OpenAI 等兼容 API

用法:
  pip install pyserial openai

  # DeepSeek (推荐, 免费)
  python ai_pid_tuner.py --port COM7 --api-key sk-xxx --provider deepseek

  # 通义千问
  python ai_pid_tuner.py --port COM7 --api-key sk-xxx --provider qwen

  # 试运行 (只看不调)
  python ai_pid_tuner.py --port COM7 --api-key sk-xxx --dry-run
"""

import serial
import time
import argparse
import re
from collections import deque
from typing import Optional
from openai import OpenAI

# ─── 多模型配置 ──────────────────────────────────────
PROVIDERS = {
    "deepseek": {
        "base_url": "https://api.deepseek.com",
        "model": "deepseek-chat",
        "name": "DeepSeek",
    },
    "qwen": {
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "model": "qwen-plus",
        "name": "通义千问",
    },
    "openai": {
        "base_url": "https://api.openai.com/v1",
        "model": "gpt-4o-mini",
        "name": "OpenAI",
    },
}

# ─── 配置 ───────────────────────────────────────────
WINDOW_SECONDS  = 3
TELEMETRY_RATE  = 0.1
WINDOW_SIZE     = int(WINDOW_SECONDS / TELEMETRY_RATE)

SAFETY = {
    "POS_Kp": (0.1, 5.0),  "POS_Ki": (0.0, 0.5), "POS_Kd": (0.0, 10.0),
    "SPD_Kp": (0.1, 3.0),  "SPD_Ki": (0.0, 1.0), "SPD_Kd": (0.0, 5.0),
    "BASE":   (50, 800),
}

current_params = {
    "POS_Kp": 1.2,  "POS_Ki": 0.02, "POS_Kd": 3.0,
    "SPD_Kp": 0.5,  "SPD_Ki": 0.10, "SPD_Kd": 0.0,
    "BASE":   400,
}

SYSTEM_PROMPT = """你是一个嵌入式PID调参专家。你正在通过串口实时调试一辆循迹小车。

## 小车硬件
- MCU: MSPM0G3507 (Cortex-M0+, 32MHz)
- 12路灰度传感器 (I2C), 检测黑线位置 (范围-550~+550, 0=居中)
- TB6612驱动两个MG513电机 (带编码器, 330PPR)
- 控制周期: 5ms (200Hz)

## PID架构 (双闭环)
位置环(外环): 输入=黑线位置, 输出=差速量 → 当前 Kp={POS_Kp}, Ki={POS_Ki}, Kd={POS_Kd}
速度环(内环): 输入=目标速度 vs 编码器脉冲, 输出=PWM → 当前 Kp={SPD_Kp}, Ki={SPD_Ki}, Kd={SPD_Kd}
基础速度: {BASE}

## 数据格式 (每行100ms)
D,位置,左目标速度,右目标速度,左编码器,右编码器,差速量,状态
状态: 0=正常, 1=丢线, 2=十字

## 诊断
| 症状 | 位置环 | 速度环 |
|------|--------|--------|
| 摇摆震荡 | Kp↓ 或 Kd↑ | — |
| 反应迟钝 | Kp↑ | — |
| 偏一边 | Ki↑ | — |
| 过冲 | Kd↑ | — |
| 电机响应慢 | — | Kp↑ |
| 速度波动 | — | Kp↓ 或 Ki↑ |
| 小幅度抖动 | Kd↓ | — |

## 规则
1. 每次只调一个环, 先位置后速度
2. 调整幅度 ≤ ±30%
3. 如果状态=1超过30%，先降基础速度再调Kp

## 输出 (严格一行)
POS,kp,ki,kd   ← 调位置环
SPD,kp,ki,kd   ← 调速度环
BASE,speed     ← 调基础速度
OK             ← 不用调
"""

def clamp(name: str, value: float) -> float:
    lo, hi = SAFETY.get(name, (value, value))
    return max(lo, min(hi, value))

def call_ai(client, model: str, data_lines: list[str]) -> Optional[str]:
    """调用大模型分析数据"""
    params_text = "\n".join(f"  {k}: {v}" for k, v in current_params.items())
    system = SYSTEM_PROMPT.replace("{POS_Kp}", str(current_params["POS_Kp"])) \
                          .replace("{POS_Ki}", str(current_params["POS_Ki"])) \
                          .replace("{POS_Kd}", str(current_params["POS_Kd"])) \
                          .replace("{SPD_Kp}", str(current_params["SPD_Kp"])) \
                          .replace("{SPD_Ki}", str(current_params["SPD_Ki"])) \
                          .replace("{SPD_Kd}", str(current_params["SPD_Kd"])) \
                          .replace("{BASE}", str(current_params["BASE"]))

    data_text = "\n".join(data_lines)
    user_prompt = f"最近{WINDOW_SECONDS}秒数据({len(data_lines)}行):\n{data_text}\n\n请分析并给出调整指令。"

    try:
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.3,
            max_tokens=100,
            timeout=60,  # DeepSeek 有时慢, 60秒超时
        )
        return response.choices[0].message.content.strip()
    except Exception as e:
        print(f"[AI Error] {e}")
        return None

def process_ai_response(response: str, ser: serial.Serial) -> bool:
    """解析并下发给小车"""
    response = response.strip().upper()

    if response == "OK":
        print("[AI] 参数良好, 无需调整")
        return False

    m = re.match(r"POS,([\d.]+),([\d.]+),([\d.]+)", response)
    if m:
        kp, ki, kd = float(m[1]), float(m[2]), float(m[3])
        kp = clamp("POS_Kp", kp); ki = clamp("POS_Ki", ki); kd = clamp("POS_Kd", kd)
        cmd = f"POS,{kp:.2f},{ki:.3f},{kd:.1f}\r\n"
        ser.write(cmd.encode())
        current_params.update({"POS_Kp": kp, "POS_Ki": ki, "POS_Kd": kd})
        print(f"[PID] 位置环: Kp={kp:.2f}, Ki={ki:.3f}, Kd={kd:.1f}")
        return True

    m = re.match(r"SPD,([\d.]+),([\d.]+),([\d.]+)", response)
    if m:
        kp, ki, kd = float(m[1]), float(m[2]), float(m[3])
        kp = clamp("SPD_Kp", kp); ki = clamp("SPD_Ki", ki); kd = clamp("SPD_Kd", kd)
        cmd = f"SPD,{kp:.2f},{ki:.3f},{kd:.1f}\r\n"
        ser.write(cmd.encode())
        current_params.update({"SPD_Kp": kp, "SPD_Ki": ki, "SPD_Kd": kd})
        print(f"[PID] 速度环: Kp={kp:.2f}, Ki={ki:.3f}, Kd={kd:.1f}")
        return True

    m = re.match(r"BASE,([\d.]+)", response)
    if m:
        speed = float(m[1]); speed = clamp("BASE", speed)
        cmd = f"BASE,{int(speed)}\r\n"
        ser.write(cmd.encode())
        current_params["BASE"] = int(speed)
        print(f"[BASE] 基础速度: {int(speed)}")
        return True

    print(f"[AI] 无法解析: {response}")
    return False

def main():
    parser = argparse.ArgumentParser(description="AI PID Tuner v2")
    parser.add_argument("--port", default=None, help="串口号, 如 COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--api-key", default=None, help="API Key")
    parser.add_argument("--provider", default="deepseek",
                        choices=["deepseek", "qwen", "openai"])
    parser.add_argument("--dry-run", action="store_true", help="只看不调")
    parser.add_argument("--window", type=int, default=WINDOW_SECONDS)
    args = parser.parse_args()

    # 缺少参数时交互输入
    if not args.port:
        args.port = input("串口号 (如 COM7): ").strip()
    if not args.api_key:
        args.api_key = input("API Key: ").strip()

    cfg = PROVIDERS[args.provider]
    window_size = int(args.window / TELEMETRY_RATE)

    client = OpenAI(api_key=args.api_key, base_url=cfg["base_url"])

    print(f"=== AI PID Tuner v2 ===")
    print(f"模型: {cfg['name']} ({cfg['model']})")
    print(f"串口: {args.port} | 窗口: {args.window}s ({window_size}行)")
    print(f"Dry-run: {args.dry_run}")
    print(f"等待数据...\n")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] 串口: {e}")
        return

    buf = deque(maxlen=window_size)
    last_call = time.time()

    try:
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                time.sleep(0.1)
                continue

            if not line:
                continue

            if line.startswith("D,"):
                buf.append(line)
                parts = line.split(",")
                if len(parts) >= 8:
                    print(f"  pos={parts[1]:>5s} L={parts[2]:>4s} R={parts[3]:>4s} "
                          f"enc={parts[4]:>5s},{parts[5]:>5s} st={parts[7]}", end="\r")

            elif line.startswith("OK,"):
                print(f"\n[小车] {line}")

            now = time.time()
            if now - last_call >= args.window and len(buf) >= window_size // 2:
                print(f"\n\n=== 分析 {len(buf)} 行 ===")
                last_call = now

                valid = [l for l in buf if l.split(",")[-1] == "0"]
                if len(valid) < window_size // 3:
                    print("[跳过] 正常循迹数据不足")
                    continue

                lines_to_send = list(valid)[-window_size:]

                if args.dry_run:
                    print(f"[Dry-run] 将发送 {len(lines_to_send)} 行")
                    for l in lines_to_send[:2]: print(f"  {l}")
                    continue

                resp = call_ai(client, cfg["model"], lines_to_send)
                if resp:
                    print(f"[AI] {resp}")
                    process_ai_response(resp, ser)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
