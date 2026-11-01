# -*- coding: utf-8 -*-
"""
USB-TTL（CH340）串口诊断工具 —— 用于直接和 HC-05 模块对话，绕开蓝牙和手机。

用法：
    python uart_diag.py list                 列出所有串口
    python uart_diag.py loop   COM3          环回测试（需要把 CH340 的 TX 和 RX 短接）
    python uart_diag.py at     COM3 [baud]   发 AT 指令（模块 EN 脚接 3.3V 后上电才有效，默认 38400）
    python uart_diag.py sniff  COM3 [baud]   只监听，不发送（看线上有没有数据）
    python uart_diag.py send   COM3 9600 46  按 9600 发一个十六进制字节，看有没有回应
"""
import sys
import time

import serial
from serial.tools import list_ports


def show(port_name):
    ports = list(list_ports.comports())
    if not ports:
        print("!! 没有找到任何串口。检查：USB-TTL 插好了吗？CH340 驱动装了吗？")
        return
    for p in ports:
        print(f"{p.device:<8} {p.description}   [{p.hwid}]")


def dump(tag, data):
    if not data:
        print(f"  {tag}: (无数据)")
        return
    hexs = " ".join(f"{b:02X}" for b in data)
    try:
        txt = data.decode("ascii", errors="replace")
    except Exception:
        txt = ""
    print(f"  {tag}: {len(data)} 字节")
    print(f"        HEX  {hexs}")
    print(f"        ASCII {txt!r}")


def loopback(port, baud=9600):
    """CH340 自身环回：必须把 TX 和 RX 用一根线短接"""
    print(f"== 环回测试 {port} @ {baud} ==")
    with serial.Serial(port, baud, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        payload = b"HELLO123"
        ser.write(payload)
        time.sleep(0.3)
        got = ser.read(256)
        dump("收到", got)
        if got == payload:
            print(">> 结果：环回成功，USB-TTL 和驱动完全正常")
        elif got:
            print(">> 结果：收到了内容但不一致 —— 线可能有接触问题")
        else:
            print(">> 结果：什么都没收到 —— TX/RX 没短接，或者 CH340 坏了")


def at_commands(port, baud=38400):
    """AT 模式：模块 EN/KEY 接 3.3V 后再上电，波特率固定 38400-8-N-1"""
    cmds = [
        ("AT", "握手"),
        ("AT+VERSION?", "固件版本"),
        ("AT+NAME?", "蓝牙名"),
        ("AT+ADDR?", "模块MAC地址"),
        ("AT+UART?", "当前串口参数（这一条最关键）"),
        ("AT+ROLE?", "主从角色"),
        ("AT+PSWD?", "配对密码"),
        ("AT+STATE?", "当前状态"),
    ]
    print(f"== AT 指令测试 {port} @ {baud} ==")
    print("   （模块 EN 脚必须接 3.3V 后再上电，否则不会回话）\n")
    with serial.Serial(port, baud, timeout=0.6) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        for cmd, meaning in cmds:
            ser.reset_input_buffer()
            ser.write((cmd + "\r\n").encode())
            time.sleep(0.35)
            got = ser.read(256)
            print(f"[{cmd}]  ({meaning})")
            dump("回应", got)
            print()
            if not got:
                print("  → 没回应。可能：EN 没接高、波特率不对、TXD 这条线没接通\n")


def sniff(port, baud=9600, seconds=6):
    print(f"== 监听 {port} @ {baud}，共 {seconds} 秒 ==")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        end = time.time() + seconds
        total = 0
        while time.time() < end:
            data = ser.read(256)
            if data:
                total += len(data)
                dump("收到", data)
        print(f">> 共收到 {total} 字节")


def send(port, baud, hexbyte):
    print(f"== 发送 {port} @ {baud}，字节 0x{hexbyte} ==")
    with serial.Serial(port, baud, timeout=0.5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(bytes([hexbyte]))
        time.sleep(0.4)
        dump("回应", ser.read(256))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    mode = sys.argv[1].lower()

    if mode == "list":
        show(None)
    elif mode == "loop":
        loopback(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 9600)
    elif mode == "at":
        at_commands(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 38400)
    elif mode == "sniff":
        sniff(sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 9600,
              int(sys.argv[4]) if len(sys.argv) > 4 else 6)
    elif mode == "send":
        send(sys.argv[2], int(sys.argv[3]), int(sys.argv[4], 16))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
