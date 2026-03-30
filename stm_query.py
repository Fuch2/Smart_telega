#!/usr/bin/env python3
import serial
import struct
import time

PORT     = '/dev/ttyAMA0'
BAUDRATE = 115200

SOF0 = 0xAA
SOF1 = 0x55
VERSION = 0x01
FRAME_TYPE_CMD  = 0x01
FRAME_TYPE_RESP = 0x02

CMD_GET_SWITCH_STATE = 0x04


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_frame(cmd_id: int, seq: int = 0, payload: bytes = b'') -> bytes:
    header = struct.pack('<BBBBH',
        VERSION,
        FRAME_TYPE_CMD,
        seq,
        cmd_id,
        len(payload)
    )
    body = header + payload
    crc = crc16_ccitt_false(body)
    return bytes([SOF0, SOF1]) + body + struct.pack('<H', crc)


def read_frame(ser, timeout: float = 1.0) -> bytes:
    """Читает до тех пор пока данные перестают поступать или вышел таймаут."""
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if len(buf) >= 10:
                break
        else:
            time.sleep(0.005)
    return buf


def parse_response(data: bytes):
    idx = -1
    for i in range(len(data) - 1):
        if data[i] == SOF0 and data[i+1] == SOF1:
            idx = i + 2
            break
    if idx < 0:
        print("SOF не найден")
        return None

    if len(data) - idx < 8:
        print("Фрейм слишком короткий")
        return None

    ver, ftype, seq, cmd, plen = struct.unpack_from('<BBBBH', data, idx)
    payload_start = idx + 6
    payload_end   = payload_start + plen
    crc_end       = payload_end + 2

    if len(data) < crc_end:
        print("Данных не хватает для полного фрейма")
        return None

    body     = data[idx : payload_end]
    rx_crc   = struct.unpack_from('<H', data, payload_end)[0]
    calc_crc = crc16_ccitt_false(body)

    if rx_crc != calc_crc:
        print(f"CRC ошибка: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
        return None

    payload = data[payload_start:payload_end]
    return {'ver': ver, 'type': ftype, 'seq': seq, 'cmd': cmd, 'payload': payload}


def get_switch_state(ser, prev_bits=[None]):
    frame = build_frame(CMD_GET_SWITCH_STATE, seq=1)
    ser.write(frame)
    resp = read_frame(ser)

    parsed = parse_response(resp)
    if parsed is None:
        return

    if parsed['type'] != FRAME_TYPE_RESP:
        return

    p = parsed['payload']
    if len(p) >= 3:
        bits = p[0] | (p[1] << 8) | (p[2] << 16)
        if bits != prev_bits[0]:  # печатаем только при изменении
            active = [i for i in range(24) if bits & (1 << i)]
            print(f"[{time.strftime('%H:%M:%S')}] 0x{bits:06X} → каналы: {active if active else 'нет'}")
            prev_bits[0] = bits


if __name__ == '__main__':
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        while True:
            get_switch_state(ser)
            time.sleep(0.05)  # 20 Hz опросw
