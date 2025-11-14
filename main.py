import argparse
import struct
import time
from threading import Thread

import serial.tools.list_ports
import vgamepad as vg

# 解析命令行参数
parser = argparse.ArgumentParser(description='DJI Mavic 3 RC231, RC-N1 Emulator')
parser.add_argument('-p', '--port', help='RC Serial Port', default="COM9")
args = parser.parse_args()

# 初始化虚拟手柄
gamepad = vg.VX360Gamepad()
gamepad.reset()
time.sleep(1)

# 全局变量（统一管理）
global camera, sequence_number
camera = 0
sequence_number = 0x34eb  # 包序列号
st = {"rh": 0, "rv": 0, "lh": 0, "lv": 0, "b1": 0, "b2": 0, "b3": 0, "b4": 0, "t1": 0}

# CRC校验表
CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x92b9, 0x8330,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

# 头部校验表
HDR_CHECKSUM_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35
]

def calc_checksum(packet):
    """计算校验和"""
    v = 0x3692
    for byte in packet:
        vv = v >> 8
        v = vv ^ CRC_TABLE[((byte ^ v) & 0xFF)]
    return v

def calc_pkt55_hdr_checksum(packet):
    """计算头部校验和"""
    chksum = 0x77
    for byte in packet[:3]:
        chksum = HDR_CHECKSUM_TABLE[((byte ^ chksum) & 0xFF)]
    return chksum

def send_duml(s, source, target, cmd_type, cmd_set, cmd_id, payload=None):
    """发送DUML数据包"""
    global sequence_number
    packet = bytearray([0x55])  # 起始符
    length = 13  # 基础长度
    if payload:
        length += len(payload)
    
    if length > 0x3ff:
        print("Packet too large")
        return
    
    # 长度字段
    packet += struct.pack('B', length & 0xff)
    packet += struct.pack('B', (length >> 8) | 0x4)
    # 头部校验
    hdr_crc = calc_pkt55_hdr_checksum(packet)
    packet += struct.pack('B', hdr_crc)
    # 地址、序列号、命令类型
    packet += struct.pack('BB', source, target)
    packet += struct.pack('<H', sequence_number)
    packet += struct.pack('BBB', cmd_type, cmd_set, cmd_id)
    # 负载
    if payload:
        packet += payload
    # 整体校验
    crc = calc_checksum(packet)
    packet += struct.pack('<H', crc)
    
    try:
        s.write(packet)
        sequence_number += 1
    except Exception as e:
        print(f"Send failed: {e}")

def parse_input(input_data):
    """解析通道值"""
    raw_val = int.from_bytes(input_data, byteorder='little')
    output = (raw_val - 1024) * 50.97  # 预计算系数
    return max(-32767, min(32767, int(output)))

def read_duml_packet(s):
    """修复：简化读取逻辑，兼容原数据格式"""
    buffer = bytearray()
    # 等待起始符 0x55
    while True:
        b = s.read(1)
        if not b:
            return None
        if b == b'\x55':
            buffer.append(0x55)
            break
    
    # 读取长度字段（2字节）
    length_bytes = s.read(2)
    if len(length_bytes) != 2:
        return None
    buffer.extend(length_bytes)
    length = struct.unpack('<H', length_bytes)[0]
    pl = length & 0x3ff  # 实际包长度
    
    # 读取剩余数据
    remaining = s.read(pl - 3)  # 已读3字节（0x55 + 2字节长度）
    if len(remaining) != pl - 3:
        return None
    buffer.extend(remaining)
    
    return buffer

def threaded_function():
    """手柄控制线程（修复频率，避免资源占用）"""
    while True:
        # 摇杆控制
        gamepad.left_joystick(int(st["lh"]), int(st["lv"]))
        gamepad.right_joystick(int(st["rh"]), int(st["rv"]))
        # 按钮处理
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_A) if st["b1"] else gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        gamepad.right_trigger(255) if st["b2"] else gamepad.right_trigger(0)
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_X) if st["b3"] else gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
        gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER) if st["b4"] else gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
        # 相机控制
        if camera > 32000:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
        elif camera < -32000:
            gamepad.press_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
        else:
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
            gamepad.release_button(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
        gamepad.update()
        time.sleep(0.005)  # 合理频率，避免卡顿

# -------------------------- 串口初始化（还原兼容逻辑）--------------------------
s = None
try:
    ports = serial.tools.list_ports.comports(True)
    print("Available serial ports:")
    for port in ports:
        try:
            print(f"  - {port.name}: {port.description}")
            if args.port == port.name or "For Protocol" in port.description:
                # 修复：使用宽松的串口参数，兼容更多设备
                s = serial.Serial(
                    port=port.name,
                    baudrate=115200,
                    timeout=0.1,  # 延长超时，避免读取失败
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    xonxoff=False,
                    rtscts=False
                )
                print(f"\nSuccessfully opened serial port: {s.name}")
                break
        except (OSError, serial.SerialException) as e:
            print(f"  ! Failed to open {port.name}: {e}")
except serial.SerialException as e:
    print(f'Error listing serial ports: {e}')
    exit(1)

# 检查串口状态
if s is None or not s.is_open:
    print(f"\nError: Could not open serial port (tried {args.port} and auto-detect)")
    print("Please check:")
    print("1. DJI remote is connected to PC")
    print("2. Correct port is specified with -p/--port")
    print("3. No other program is using the port")
    exit(1)
# ------------------------------------------------------------------------

print('app version: 3.0.0\n')
print('Dji RC231 emulation started...\n')
print('Close terminal to stop\n')
print('*******************************************************\n')
print('* ERAY CAN *\n')
print('* DJI RC N3  *\n')
print('*******************************************************\n')

# 启动手柄控制线程
thread = Thread(target=threaded_function, daemon=True)
thread.start()

try:
    # 启用模拟器模式
    send_duml(s, 0x0a, 0x06, 0x40, 0x06, 0x24, bytearray.fromhex('01'))
    print("Enabled simulator mode for RC")

    while True:
        # 发送读取指令（降低频率，避免串口拥堵）
        send_duml(s, 0x0a, 0x06, 0x40, 0x06, 0x01, bytearray.fromhex(''))
        send_duml(s, 0x0a, 0x06, 0x40, 0x06, 0x27, bytearray.fromhex(''))
        
        # 读取数据包
        buffer = read_duml_packet(s)
        if not buffer:
            time.sleep(0.005)
            continue
        
        # 解析数据（保持原逻辑，确保兼容性）
        if len(buffer) == 38:
            st["rh"] = parse_input(buffer[13:15])
            st["rv"] = parse_input(buffer[16:18])
            st["lv"] = parse_input(buffer[19:21])
            st["lh"] = parse_input(buffer[22:24])
            camera = parse_input(buffer[25:27])
        elif len(buffer) == 58:
            # 解析按键状态
            bytes_btn = buffer[28:30]
            ival = int.from_bytes(bytes_btn, byteorder="big")
            st['b1'] = 1 if (ival & 0x1060) == 0x1060 else 0
            st['b2'] = 1 if (ival & 0x1080) == 0x1080 else 0
            st['b3'] = 1 if (ival & 0x1004) == 0x1004 else 0
            st['b4'] = 1 if (ival & 0x1002) == 0x1002 else 0

            bytes_trigger = buffer[27:29]
            ival2 = int.from_bytes(bytes_trigger, byteorder="big")
            st['t1'] = 32767 if ival2 == 0x0 else -32767 if (ival2 & 0x20) == 0x20 else 0

        time.sleep(0.005)

except serial.SerialException as e:
    print(f'\nError reading/writing serial port: {e}')
except KeyboardInterrupt:
    print('\nDetected keyboard interrupt. Exiting...')
finally:
    # 清理资源
    if s and s.is_open:
        s.close()
        print("Serial port closed")
    gamepad.reset()
    print("Virtual gamepad reset")

print('Stopped.')