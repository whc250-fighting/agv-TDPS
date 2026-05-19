#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGV 上位机控制程序 - Orange Pi 4 Pro
串口: /dev/ttyS7, 波特率: 115200
"""

import serial
import threading
import time
import sys

# ==================== 配置 ====================
SERIAL_PORT  = "/dev/ttyS7"
BAUD_RATE    = 115200
HEARTBEAT_MS = 300   # 心跳间隔，必须小于 STM32 的 500ms 超时

# ==================== 帧工具 ====================

def calc_xor(data: str) -> str:
    """计算 XOR 校验值，范围是 $ 到 # 之间所有字节"""
    xor = ord('$')
    for ch in data:
        xor ^= ord(ch)
    return f"{xor:02X}"

def build_frame(cmd: str, param: str = "") -> bytes:
    """构造发送帧: $<CMD>[PARAM]#<XOR>\n"""
    body = f"{cmd}{param}#"
    xor  = calc_xor(body)
    frame = f"${body}{xor}\n"
    return frame.encode('ascii')

def parse_status(line: str):
    """
    解析 STM32 上报的状态帧
    格式: $S<mode>,<lspd>,<rspd>,<sensor_hex>,<error>#<XOR>\r\n
    返回 dict 或 None
    """
    line = line.strip()
    if not line.startswith('$S') or '#' not in line:
        return None
    try:
        body = line[1:line.index('#')]       # S1,450,-200,18,0.50
        xor_str = line[line.index('#')+1:]   # XOR 部分
        # 验证校验
        expected = calc_xor(body + '#')
        if expected != xor_str[:2].upper():
            return None
        parts = body[1:].split(',')          # 去掉开头的 'S'
        return {
            'mode':   int(parts[0]),
            'lspd':   int(parts[1]),
            'rspd':   int(parts[2]),
            'sensor': int(parts[3], 16),
            'error':  float(parts[4]),
        }
    except Exception:
        return None

# ==================== AGV 控制器 ====================

class AGVController:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.running   = True
        self.status    = {}
        self.lock      = threading.Lock()

        # 启动心跳线程和接收线程
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop,      daemon=True).start()
        print(f"[AGV] 已连接 {port} @ {baud}")

    def send(self, cmd: str, param: str = ""):
        frame = build_frame(cmd, param)
        self.ser.write(frame)

    def _heartbeat_loop(self):
        while self.running:
            self.send('H')
            time.sleep(HEARTBEAT_MS / 1000)

    def _recv_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if not line:
                    continue
                if line.startswith('$S'):
                    s = parse_status(line)
                    if s:
                        with self.lock:
                            self.status = s
                        self._on_status(s)
                elif line.startswith('!'):
                    print(f"[STM32] {line.strip()}")
            except Exception as e:
                print(f"[RECV ERR] {e}")

    def _on_status(self, s):
        """收到状态帧时的处理（可扩展圆圈检测）"""
        # 暂时只打印，后续在这里加圆圈陷阱检测
        print(f"[STATUS] mode={s['mode']} L={s['lspd']:+4d} R={s['rspd']:+4d} "
              f"sensor=0x{s['sensor']:02X} err={s['error']:+.2f}")

    def stop(self):
        self.running = False
        self.send('S')
        self.ser.close()

# ==================== 主程序（交互控制台）====================

def main():
    agv = AGVController(SERIAL_PORT, BAUD_RATE)
    time.sleep(0.5)  # 等待连接稳定

    print("\n指令: T=循线  S=停止  F=前进  B=后退  L=左转  R=右转  V<速度>=设速  Q=退出")
    print("示例: V500 设置速度为500\n")

    try:
        while True:
            cmd_input = input("> ").strip().upper()
            if not cmd_input:
                continue
            if cmd_input == 'Q':
                break
            elif cmd_input.startswith('V'):
                param = cmd_input[1:]
                if param.isdigit() and 0 < int(param) <= 999:
                    agv.send('V', param)
                else:
                    print("速度范围 1-999")
            elif cmd_input in ('T', 'S', 'F', 'B', 'L', 'R', 'H'):
                agv.send(cmd_input)
            else:
                print("未知指令")
    except KeyboardInterrupt:
        pass
    finally:
        agv.stop()
        print("[AGV] 已停止")

if __name__ == "__main__":
    main()
