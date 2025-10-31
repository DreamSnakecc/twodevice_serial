import serial
import csv
import time
import threading
from datetime import datetime

# ------------------- 串口配置 -------------------
USB1_PORT = '/dev/ttyUSB0'  # 扭矩传感器
USB2_PORT = '/dev/ttyUSB1'  # STM32
USB1_BAUD = 19200
USB2_BAUD = 19200   # ✅ 可单独配置波特率
CSV_FILE = 'dual_usb_data.csv'

usb1_data = {'timestamp': None, 'torque': None, 'speed': None}
usb2_data = {'timestamp': None, 'speed': None, 'current': None}
lock = threading.Lock()

# ------------------- CRC16 校验 -------------------
def crc16(data: bytes):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# ------------------- USB1 解码 -------------------
# 格式: D1 D2 D3 D4 D5 D6
def decode_usb1(frame: bytes):
    if len(frame) != 6:
        return None, None
    d1, d2, d3, d4, d5, d6 = frame
    recv_crc = (d6 << 8) | d5
    calc_crc = crc16(frame[:4])
    if recv_crc != calc_crc:
        print(f"[USB1] CRC Error! recv={recv_crc:04X}, calc={calc_crc:04X}")
        return None, None

    torque = (d1 << 8) | d2
    if torque & 0x8000:
        torque -= 0x10000

    speed = (d3 << 8) | d4
    if speed & 0x8000:
        speed -= 0x10000

    return torque / 100.0, speed / 100.0

# ------------------- USB2 解码 -------------------
# 格式: 0x51 0x51 + speed(int32) + current(int32) 共10字节
def decode_usb2(buffer: bytearray):
    while len(buffer) >= 10:
        # 查找帧头
        if buffer[0] == 0x51 and buffer[1] == 0x51:
            if len(buffer) >= 10:
                # 提取速度、电流（小端、有符号）
                speed_raw = int.from_bytes(buffer[2:6], byteorder='little', signed=True)
                current_raw = int.from_bytes(buffer[6:10], byteorder='little', signed=True)
                del buffer[:10]  # 移除整个帧
                return speed_raw / 1000.0, current_raw / 1000.0
            else:
                break
        else:
            del buffer[0]  # 去掉无效字节
    return None

# ------------------- USB1 读取线程 -------------------
def read_usb1():
    ser = serial.Serial(USB1_PORT, USB1_BAUD, timeout=0.1)
    print(f"[INFO] Reading USB1 at {USB1_BAUD} baud from {USB1_PORT}")
    buffer = bytearray()

    while True:
        data = ser.read()
        if not data:
            continue
        buffer += data
        while len(buffer) >= 6:
            frame = buffer[:6]
            buffer = buffer[6:]
            torque, speed = decode_usb1(frame)
            if torque is not None:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                with lock:
                    usb1_data.update({'timestamp': ts, 'torque': torque, 'speed': speed})
                print(f"[USB1] {ts}  torque={torque:.3f}  speed={speed:.3f}")

# ------------------- USB2 读取线程 -------------------
def read_usb2():
    ser = serial.Serial(USB2_PORT, USB2_BAUD, timeout=0.1)
    print(f"[INFO] Reading USB2 at {USB2_BAUD} baud from {USB2_PORT}")
    buffer = bytearray()

    while True:
        data = ser.read()
        if not data:
            continue
        buffer += data
        result = decode_usb2(buffer)
        if result:
            speed, current = result
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with lock:
                usb2_data.update({'timestamp': ts, 'speed': speed, 'current': current})
            print(f"[USB2] {ts}  speed={speed:.3f}  current={current:.3f}")

# ------------------- 数据对齐 + 存储线程 -------------------
def save_to_csv():
    with open(CSV_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'usb1_torque', 'usb1_speed', 'usb2_speed', 'usb2_current'])
        last_t1, last_t2 = None, None

        while True:
            time.sleep(0.01)
            with lock:
                t1, t2 = usb1_data['timestamp'], usb2_data['timestamp']
                if t1 and t2 and (t1 != last_t1 or t2 != last_t2):
                    ts = t1 if t1 <= t2 else t2
                    writer.writerow([
                        ts,
                        usb1_data['torque'],
                        usb1_data['speed'],
                        usb2_data['speed'],
                        usb2_data['current']
                    ])
                    f.flush()
                    last_t1, last_t2 = t1, t2

# ------------------- 主程序入口 -------------------
if __name__ == '__main__':
    t1 = threading.Thread(target=read_usb1, daemon=True)
    t2 = threading.Thread(target=read_usb2, daemon=True)
    t3 = threading.Thread(target=save_to_csv, daemon=True)
    t1.start()
    t2.start()
    t3.start()

    print("[INFO] Dual USB logging started. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Logging stopped.")
