#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
激光雷达底层通信与电机启动诊断脚本
测试多种常见波特率 (150000, 115200, 230400) 并拉高 DTR/RTS 信号
发送启动指令并监听是否有数据流回
"""

import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

# 测试的常见波特率列表
BAUDRATES = [150000, 115200, 230400]

print(f"==================================================")
print(f"正在诊断激光雷达串口: {PORT}")
print(f"==================================================")

for baud in BAUDRATES:
    print(f"\n[测试波特率: {baud}] 尝试打开串口...")
    try:
        ser = serial.Serial(PORT, baud, timeout=1.0)
    except Exception as e:
        print(f"  打开串口失败: {e}")
        continue

    # 1. 显式拉高 DTR 和 RTS 控制线 (许多 STC/CDC 驱动板以此作为电机启停信号)
    ser.setDTR(True)
    ser.setRTS(True)
    time.sleep(0.2)

    # 2. 发送启动指令 A5 60 (重复发送 3 次确保 STC 接收)
    print("  发送启动指令 (0xA5 0x60)...")
    for _ in range(3):
        ser.write(bytes([0xA5, 0x60]))
        time.sleep(0.05)

    # 3. 监听接收缓冲区
    time.sleep(0.5)
    n = ser.in_waiting
    print(f"  当前接收缓冲区待读字节数: {n}")
    
    if n > 0:
        data = ser.read(min(n, 64))
        hex_str = ' '.join(f'{b:02X}' for b in data)
        print(f"  >>> 成功接收到底层数据 ({len(data)} 字节):")
        print(f"      HEX: {hex_str}")
        if data[0:2] == b'\xAA\x55' or b'\xAA\x55' in data:
            print("  >>> 匹配到雷达数据帧头 0xAA 0x55! 此波特率完全正确!")
            print(f"  >>> 恭喜! 雷达物理电机应已启动运转!")
            ser.close()
            sys.exit(0)
    else:
        print("  未接收到返回数据，尝试下一波特率...")

    ser.close()
    time.sleep(0.3)

print("\n==================================================")
print("诊断完成。如果三个波特率下电机均未旋转，请检查雷达小板供电或接线是否松动。")
print("==================================================")
