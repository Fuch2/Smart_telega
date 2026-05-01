# Протокол STM32 ↔ Raspberry Pi (UART)

## Обзор

Протокол обеспечивает двустороннюю связь между Raspberry Pi (host) и STM32 (device) через UART. Используется для управления LED-подсветкой, чтения состояния переключателей (switches) и диагностики.

**Параметры UART:**
- Скорость: 115200 baud (по умолчанию)
- Формат: 8N1 (8 бит данных, без паритета, 1 стоп-бит)
- Устройство: `/dev/ttyAMA0` (Raspberry Pi)

## Формат фрейма

Все сообщения передаются в виде фреймов фиксированной структуры:

```
┌────────┬────────┬─────────┬──────┬─────┬─────┬────────┬─────────┬────────┬────────┐
│  SOF1  │  SOF2  │ Version │ Type │ Seq │ Cmd │ Len_Lo │ Len_Hi  │ Payload│  CRC16 │
│  0xAA  │  0x55  │  0x01   │  1B  │ 1B  │ 1B  │   1B   │   1B    │  0-128B│   2B   │
└────────┴────────┴─────────┴──────┴─────┴─────┴────────┴─────────┴────────┴────────┘
```

**Поля:**
- `SOF1`, `SOF2` — Start of Frame (0xAA, 0x55) — маркеры начала фрейма
- `Version` — версия протокола (0x01)
- `Type` — тип фрейма (Cmd, Resp, Ack, Nack, Evt)
- `Seq` — порядковый номер (для сопоставления запросов и ответов)
- `Cmd` — идентификатор команды
- `Len_Lo`, `Len_Hi` — длина payload (little-endian, max 128 байт)
- `Payload` — данные команды (0-128 байт)
- `CRC16` — контрольная сумма CRC-16-CCITT-FALSE (little-endian)

**CRC16 вычисляется для:** `[Version, Type, Seq, Cmd, Len_Lo, Len_Hi, Payload]`

**Максимальный размер фрейма:** 2 + 6 + 128 + 2 = 138 байт

## Типы фреймов

```cpp
enum class FrameType : uint8_t {
    Cmd  = 0x01,  // Команда от host к device
    Resp = 0x02,  // Ответ от device с данными
    Ack  = 0x03,  // Подтверждение успешного выполнения
    Nack = 0x04,  // Отказ (ошибка выполнения)
    Evt  = 0x05   // Unsolicited event от device
};
```

## Команды

### Общие команды

#### 0x01 — Ping
**Описание:** Проверка связи с устройством.

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Ack):**
- Payload: пустой

---

#### 0x02 — GetReadyState
**Описание:** Проверка готовности устройства к работе.

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Resp):**
- Payload: `[ready_flag]`
  - `ready_flag` (1 байт): 0x01 = готов, 0x00 = не готов

---

#### 0x03 — GetFwVersion
**Описание:** Получить версию прошивки STM32.

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Resp):**
- Payload: `[major, minor, patch]`
  - `major` (1 байт): мажорная версия
  - `minor` (1 байт): минорная версия
  - `patch` (1 байт): патч-версия

**Пример:** `[0x01, 0x02, 0x03]` = версия 1.2.3

---

### Команды переключателей (Switches)

#### 0x10 — GetSwitchSnapshot
**Описание:** Получить текущее состояние всех переключателей (snapshot).

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Resp):**
- Payload: `[bitmap...]`
  - Bitmap состояния переключателей (1 бит = 1 переключатель)
  - Для 24 слотов: 3 байта (24 бита)
  - Бит = 1: переключатель замкнут (материал есть)
  - Бит = 0: переключатель разомкнут (материала нет)

**Формат bitmap:**
```
Byte 0: [ch7 ch6 ch5 ch4 ch3 ch2 ch1 ch0]
Byte 1: [ch15 ch14 ch13 ch12 ch11 ch10 ch9 ch8]
Byte 2: [ch23 ch22 ch21 ch20 ch19 ch18 ch17 ch16]
```

**Пример:**
```
Payload: [0xFF, 0x00, 0x0F]
→ ch0-7: все замкнуты
→ ch8-15: все разомкнуты
→ ch16-19: замкнуты, ch20-23: разомкнуты
```

---

### Команды LED

#### 0x20 — LedSetSlot
**Описание:** Установить цвет LED для одного слота.

**Запрос (Cmd):**
- Payload: `[led_index, r, g, b]`
  - `led_index` (1 байт): индекс LED (0-47 для 24 слотов, по 2 LED на слот)
  - `r` (1 байт): красный (0-255)
  - `g` (1 байт): зелёный (0-255)
  - `b` (1 байт): синий (0-255)

**Ответ (Ack):**
- Payload: пустой

**Примечание:** Изменения применяются только после команды `LedApply`.

---

#### 0x22 — LedClearAll
**Описание:** Выключить все LED (установить в чёрный цвет).

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Ack):**
- Payload: пустой

---

#### 0x23 — LedApply
**Описание:** Применить изменения LED (обновить физические LED).

**Запрос (Cmd):**
- Payload: пустой

**Ответ (Ack):**
- Payload: пустой

**Примечание:** Команды `LedSetSlot`, `LedClearAll` изменяют только буфер. Физические LED обновляются только после `LedApply`.

---

## События (Unsolicited Events)

События отправляются STM32 без запроса от host.

#### 0xE2 — EvtSwitchChanged
**Описание:** Изменилось состояние переключателей.

**Фрейм (Evt):**
- Payload: `[changed_bitmap..., current_bitmap...]`
  - `changed_bitmap`: биты изменившихся переключателей (1 = изменился)
  - `current_bitmap`: текущее состояние изменившихся переключателей

**Примечание:** Это событие отправляется автоматически при изменении состояния переключателей (debounced).

---

## Коды ошибок (StatusCode)

При ошибке STM32 отправляет фрейм типа `Nack` с кодом ошибки в первом байте payload:

```cpp
enum class StatusCode : uint8_t {
    Ok                    = 0x00,
    ErrUnknownCmd         = 0x01,
    ErrBadPayload         = 0x02,
    ErrBadLength          = 0x03,
    ErrUnsupportedVersion = 0x05,
    ErrBusy               = 0x06,
    ErrInvalidSlot        = 0x07,
    ErrInvalidParam       = 0x08,
    ErrHwFailure          = 0x09,
    ErrNotReady           = 0x0A,
    ErrRateLimit          = 0x0B,
    ErrInternal           = 0x0C
};
```

---

## Примеры использования

### Пример 1: Подсветить слот зелёным

```cpp
// 1. Установить цвет LED
Frame cmd;
cmd.type = FrameType::Cmd;
cmd.cmdId = CommandId::LedSetSlot;
cmd.payload = {10, 0, 255, 0};  // LED 10, RGB=(0,255,0)
link->sendCommand(cmd);

// 2. Применить изменения
Frame apply;
apply.type = FrameType::Cmd;
apply.cmdId = CommandId::LedApply;
link->sendCommand(apply);
```

---

### Пример 2: Получить snapshot переключателей

```cpp
Frame cmd;
cmd.type = FrameType::Cmd;
cmd.cmdId = CommandId::GetSwitchSnapshot;

std::optional<Frame> resp = link->sendCommand(cmd);
if (resp && resp->type == FrameType::Resp) {
    const auto& payload = resp->payload;
    // Интерпретировать bitmap
    for (int ch = 0; ch < 24; ++ch) {
        int byteIdx = ch / 8;
        int bitIdx = ch % 8;
        bool closed = (payload[byteIdx] >> bitIdx) & 0x01;
        // ...
    }
}
```

---

## Реализация на Raspberry Pi

### Отправка команды

```cpp
auto link = std::make_unique<UartStm32Link>("/dev/ttyAMA0", 115200, 1000);
link->open();

Frame cmd;
cmd.type = FrameType::Cmd;
cmd.cmdId = CommandId::Ping;

std::optional<Frame> resp = link->sendCommand(cmd);
if (resp && resp->type == FrameType::Ack) {
    // Успех
}
```

### Обработка событий

```cpp
link->setEventCallback([](const Frame& event) {
    if (event.cmdId == CommandId::EvtSwitchChanged) {
        // Обработать изменение переключателей
    }
});
```

---

**Автор:** SmartCart Team  
**Дата:** 2026-05-01  
**Версия протокола:** 0x01
