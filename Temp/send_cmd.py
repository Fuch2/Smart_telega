#!/usr/bin/env python3
import serial
import time
import sys

PORT = "/dev/tty.wchusbserial130"  # ← сюда подставь свой порт
BAUD = 115200

def send_cmd(cmd, data=None):
    """
    Посылает кадр протокола:
    STX ' ' cmd ' ' data ' ' LF
    cmd и data - ASCII
    """
    if data is None:
        frame_str = f" {cmd}  "      # пробел + cmd + два пробела
    else:
        frame_str = f" {cmd} {data} "  # пробел + cmd + пробел + data + пробел

    frame = b"\x02" + frame_str.encode("ascii") + b"\x0A"

    with serial.Serial(PORT, BAUD, timeout=0.5) as ser:
        # небольшая пауза, чтобы порт успел открыться
        time.sleep(0.1)
        ser.write(frame)

if __name__ == "__main__":
    # Примеры:
    #   python3 send_cmd.py start
    #   python3 send_cmd.py T_set 500
    #   python3 send_cmd.py N_set 5
    if len(sys.argv) < 2:
        print("Usage: send_cmd.py CMD [DATA]")
        sys.exit(1)

    cmd = sys.argv[1]
    data = sys.argv[2] if len(sys.argv) >= 3 else None
    send_cmd(cmd, data)
