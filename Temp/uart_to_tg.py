#!/usr/bin/env python3
import os
import time
import requests
import serial

PORT = "/dev/ttyAMA0"
BAUD = 115200

BOT_TOKEN = os.environ["BOT_TOKEN"]
CHAT_ID = os.environ["CHAT_ID"]

SEND_EVERY_SEC = 20.0
MAX_LINES_PER_MSG = 2
MAX_CHARS = 3500

DEBUG = True  # <-- DEBUG: можно выключить

def tg_send(text: str) -> None:
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    r = requests.post(url, data={
        "chat_id": CHAT_ID,
        "text": text,
        "disable_web_page_preview": "true",
    }, timeout=10)

    if DEBUG:  # <-- DEBUG: увидишь ответ Telegram при отправке
        print("TG send:", r.status_code, r.text[:300], flush=True)

    r.raise_for_status()

def clean_ascii(line: bytes) -> str:
    filtered = bytes(b for b in line if b in (9, 10, 13) or 32 <= b <= 126)
    return filtered.decode("ascii", errors="ignore").strip()

def make_frame(cmd: str, data: str | None = None) -> bytes:
    if data is None:
        frame_str = f" {cmd}  "          # пробел + cmd + два пробела
    else:
        frame_str = f" {cmd} {data} "    # пробел + cmd + пробел + data + пробел
    return b"\x02" + frame_str.encode("ascii") + b"\x0A"


def main():
    buf = []
    last_send = time.time()

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        if DEBUG:  # <-- DEBUG: подтвердим, что порт открылся
            print(f"UART opened: {PORT} @ {BAUD}", flush=True)
            print(f"CHAT_ID={CHAT_ID}", flush=True)

        # (необязательно) тестовое сообщение "started"
        #tg_send("uart_to_tg: started")

        ser.reset_input_buffer()
        ser.write(make_frame("start"))
        ser.flush()
        if DEBUG:
            print("UART TX: start", flush=True)

        while True:
            raw = ser.readline()

            if DEBUG:  # <-- DEBUG: видим, вообще есть ли байты с UART
                if raw:
                    print("UART raw:", raw, flush=True)

            if raw:
                s = clean_ascii(raw)

                if DEBUG:  # <-- DEBUG: что получилось после чистки
                    print("UART text:", repr(s), flush=True)

                if s:
                    buf.append(s)

            now = time.time()
            if buf and (now - last_send >= SEND_EVERY_SEC or len(buf) >= MAX_LINES_PER_MSG):
                out_lines = []
                total = 0
                while buf and len(out_lines) < MAX_LINES_PER_MSG:
                    line = buf.pop(0)
                    if total + len(line) + 1 > MAX_CHARS:
                        break
                    out_lines.append(line)
                    total += len(line) + 1

                text = "\n".join(out_lines)

                if DEBUG:  # <-- DEBUG: что именно собираемся отправить
                    print(f"Sending {len(out_lines)} lines, {len(text)} chars", flush=True)

                try:
                    tg_send(text)
                    if DEBUG:
                        print("TG send OK", flush=True)
                except Exception as e:
                    if DEBUG:  # <-- DEBUG: покажем причину, а не молча проглотим
                        print("TG send ERROR:", repr(e), flush=True)
                    buf = out_lines + buf
                    time.sleep(2)

                last_send = now

if __name__ == "__main__":
    main()
