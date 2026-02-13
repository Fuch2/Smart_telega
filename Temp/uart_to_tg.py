#!/usr/bin/env python3
import os
import time
import requests
import serial
import threading

running = True
uart_lock = threading.Lock()


PORT = "/dev/ttyAMA0"
BAUD = 115200

BOT_TOKEN = os.environ["BOT_TOKEN"]
CHAT_ID = os.environ["CHAT_ID"]

SEND_EVERY_SEC = 20.0
MAX_LINES_PER_MSG = 2
MAX_CHARS = 3500

DEBUG = True  # <-- DEBUG: можно выключить

def tg_get_updates(offset: int | None = None, timeout: int = 30):
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/getUpdates"
    params = {"timeout": timeout}
    if offset is not None:
        params["offset"] = offset
    r = requests.get(url, params=params, timeout=timeout + 10)
    r.raise_for_status()
    return r.json()

def uart_send_cmd(ser: serial.Serial, cmd: str, data: str | None = None):
    frame = make_frame(cmd, data)
    if DEBUG:
        print("UART TX bytes:", frame, flush=True)
    with uart_lock:
        ser.write(frame)
        ser.flush()


def tg_command_loop(ser: serial.Serial):
    global running
    offset = None

    while running:
        try:
            js = tg_get_updates(offset=offset, timeout=30)
            if not js.get("ok"):
                time.sleep(1)
                continue

            for upd in js.get("result", []):
                offset = upd["update_id"] + 1

                msg = upd.get("message") or upd.get("edited_message")
                if not msg:
                    continue

                chat_id = msg["chat"]["id"]
                text = (msg.get("text") or "").strip()

                # Разрешаем управление только одному чату
                if str(chat_id) != str(CHAT_ID):
                    continue

                if text in ("/start", "start"):
                    uart_send_cmd(ser, "start")
                    tg_send("OK: start отправлен в UART")
                elif text in ("/stop", "stop"):
                    uart_send_cmd(ser, "stop")
                    tg_send("OK: stop отправлен в UART")
                elif text.startswith("/send "):
                    # /send <cmd> [data...]
                    payload = text[len("/send "):].strip()
                    if not payload:
                        tg_send("Формат: /send <cmd> [data]")
                        continue
                    parts = payload.split(maxsplit=1)
                    cmd = parts[0]
                    data = parts[1] if len(parts) == 2 else None
                    uart_send_cmd(ser, cmd, data)
                    tg_send(f"OK: отправлено: {cmd}" + (f" {data}" if data else ""))
                elif text in ("/help", "help"):
                    tg_send(
                        "Команды:\n"
                        "/start — отправить start\n"
                        "/stop — отправить stop\n"
                        "/send <cmd> [data] — отправить произвольную команду"
                    )

        except Exception as e:
            if DEBUG:
                print("TG poll ERROR:", repr(e), flush=True)
            time.sleep(2)


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

        t = threading.Thread(target=tg_command_loop, args=(ser,), daemon=True)
        t.start()

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
