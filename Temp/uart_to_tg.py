#!/usr/bin/env python3
import os
import time
import requests
import serial

PORT = "/dev/ttyAMA0"
BAUD = 115200

BOT_TOKEN = os.environ["BOT_TOKEN"]
CHAT_ID = os.environ["CHAT_ID"]

SEND_EVERY_SEC = 2.0       # как часто отправлять пачку
MAX_LINES_PER_MSG = 20     # сколько строк в одном сообщении
MAX_CHARS = 3500           # запас до лимита Telegram (4096)

r = requests.get(
    f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage",
    params={"chat_id": CHAT_ID, "text": "uart_to_tg: started"}
)
print("sendMessage:", r.status_code, r.text)

def tg_send(text: str) -> None:
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    r = requests.post(url, data={
        "chat_id": CHAT_ID,
        "text": text,
        "disable_web_page_preview": "true",
    }, timeout=10)
    r.raise_for_status()

def clean_ascii(line: bytes) -> str:
    # оставляем печатные ASCII + CR/LF/Tab, остальное выкидываем
    filtered = bytes(b for b in line if b in (9, 10, 13) or 32 <= b <= 126)
    # часто мешает STX 0x02 — он и так выкинется
    return filtered.decode("ascii", errors="ignore").strip()

def main():
    buf = []
    last_send = time.time()

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        # читаем построчно; если устройство шлёт \r\n — будет отлично
        while True:
            raw = ser.readline()  # bytes до '\n' (или по timeout)
            if raw:
                s = clean_ascii(raw)
                if s:
                    buf.append(s)

            now = time.time()
            if buf and (now - last_send >= SEND_EVERY_SEC or len(buf) >= MAX_LINES_PER_MSG):
                # собираем сообщение в пределах лимита
                out_lines = []
                total = 0
                while buf and len(out_lines) < MAX_LINES_PER_MSG:
                    line = buf.pop(0)
                    if total + len(line) + 1 > MAX_CHARS:
                        break
                    out_lines.append(line)
                    total += len(line) + 1

                text = "\n".join(out_lines)
                try:
                    tg_send(text)
                except Exception as e:
                    # если сеть упала — не теряем данные: вернём строки назад
                    buf = out_lines + buf
                    time.sleep(2)

                last_send = now

if __name__ == "__main__":
    main()
