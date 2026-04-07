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
FRAME_TYPE_EVT  = 0x05

CMD_GET_SWITCH_SNAPSHOT = 0x10
CMD_GET_SWITCH_STATE_LEGACY = 0x04
EVT_READY = 0xE0
EVT_SWITCH_CHANGED = 0xE2
MAX_PAYLOAD = 128


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


def read_exact(ser, size: int, timeout: float) -> bytes:
    deadline = time.time() + timeout
    buf = b''
    while len(buf) < size and time.time() < deadline:
        chunk = ser.read(size - len(buf))
        if chunk:
            buf += chunk
        else:
            time.sleep(0.005)
    return buf


def read_frame(ser, timeout: float = 1.0):
    """Читает один полный protocol frame, пропуская мусор до SOF."""
    deadline = time.time() + timeout

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != SOF0:
            continue

        b = ser.read(1)
        if not b:
            return None
        if b[0] != SOF1:
            continue

        header = read_exact(ser, 6, max(0.0, deadline - time.time()))
        if len(header) != 6:
            print("Фрейм слишком короткий")
            return None

        ver, ftype, seq, cmd, plen = struct.unpack('<BBBBH', header)
        if plen > MAX_PAYLOAD:
            print(f"Некорректная длина payload: {plen}")
            return None

        tail = read_exact(ser, plen + 2, max(0.0, deadline - time.time()))
        if len(tail) != plen + 2:
            print("Данных не хватает для полного фрейма")
            return None

        payload = tail[:plen]
        rx_crc = struct.unpack_from('<H', tail, plen)[0]
        calc_crc = crc16_ccitt_false(header + payload)
        if rx_crc != calc_crc:
            print(f"CRC ошибка: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")
            return None

        return {'ver': ver, 'type': ftype, 'seq': seq, 'cmd': cmd, 'payload': payload}

    print("SOF не найден")
    return None


def print_event(frame):
    if frame['cmd'] == EVT_READY:
        print(f"[{time.strftime('%H:%M:%S')}] EVT_READY")
        return

    if frame['cmd'] == EVT_SWITCH_CHANGED and len(frame['payload']) >= 2:
        channel = frame['payload'][0]
        state = bool(frame['payload'][1])
        print(f"[{time.strftime('%H:%M:%S')}] EVT_SWITCH_CHANGED: channel={channel} occupied={state}")
        return

    print(f"[{time.strftime('%H:%M:%S')}] EVT cmd=0x{frame['cmd']:02X} payload={frame['payload'].hex()}")


def wait_response(ser, cmd_id: int, seq: int, timeout: float = 1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        frame = read_frame(ser, max(0.0, deadline - time.time()))
        if frame is None:
            continue

        if frame['type'] == FRAME_TYPE_EVT:
            print_event(frame)
            continue

        if frame['type'] == FRAME_TYPE_RESP and frame['seq'] == seq and frame['cmd'] == cmd_id:
            return frame

    return None


def get_switch_state(ser, prev_bits=[None], seq_box=[0]):
    seq_box[0] = (seq_box[0] + 1) & 0xFF
    seq = seq_box[0]

    frame = build_frame(CMD_GET_SWITCH_SNAPSHOT, seq=seq)
    ser.write(frame)
    resp = wait_response(ser, CMD_GET_SWITCH_SNAPSHOT, seq, timeout=0.5)

    if resp is None or len(resp['payload']) < 3:
        # Совместимость со старой прошивкой, где snapshot был на 0x04.
        seq_box[0] = (seq_box[0] + 1) & 0xFF
        seq = seq_box[0]
        ser.write(build_frame(CMD_GET_SWITCH_STATE_LEGACY, seq=seq))
        resp = wait_response(ser, CMD_GET_SWITCH_STATE_LEGACY, seq, timeout=0.5)

    if resp is None:
        return None

    if resp['type'] != FRAME_TYPE_RESP:
        return

    p = resp['payload']
    if len(p) >= 3:
        bits = p[0] | (p[1] << 8) | (p[2] << 16)
        if bits != prev_bits[0]:  # печатаем только при изменении
            active = [i for i in range(24) if bits & (1 << i)]
            print(f"[{time.strftime('%H:%M:%S')}] 0x{bits:06X} → каналы: {active if active else 'нет'}")
            prev_bits[0] = bits


if __name__ == '__main__':
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            while True:
                get_switch_state(ser)
                time.sleep(0.05)  # 20 Hz опрос
    except serial.SerialException as ex:
        print(f"UART ошибка: {ex}")
        print("Проверь, что smart_cart_ui закрыт и порт /dev/ttyAMA0 не занят другим процессом.")
