# Code Review: smart-cart

Дата: 2026-05-08  
Ветка: `claude/xenodochial-borg-30fd6b`

---

## Критические баги

### BUG-1 — Use-after-free в `waitReady()`: dangling reference в callback

**Файл:** `src/application/services/StartupService.cpp:126`  
**Серьёзность:** Критический

`waitReady()` создаёт `mtx`, `cv`, `readyEvent` на стеке и захватывает их по ссылке в лямбде callback:

```cpp
std::mutex mtx;
std::condition_variable cv;
bool readyEvent = false;

link_.setEventCallback([&mtx, &cv, &readyEvent](const stm32::Frame& evt) {
    std::lock_guard lock(mtx);   // dangling после возврата функции
    readyEvent = true;
    cv.notify_one();
});
// ...
cv.wait_for(lock, timeout, ...);
link_.setEventCallback(nullptr); // ← здесь ещё нет nullptr
return true;                     // ← стек уничтожен, но rxThread ещё жив
```

Между разблокировкой `wait_for` и вызовом `setEventCallback(nullptr)` есть гонка: `rxThread_` в `UartStm32Link` может вызвать callback и обратиться к уничтоженным переменным.

**Исправление:** использовать `shared_ptr` для `mtx`, `cv`, `readyEvent` и захватывать по значению — аналогично тому, как уже сделано в `AddReelService.cpp:105-107`.

---

### BUG-2 — Use-after-free: detached thread захватывает `this`

**Файлы:** `src/application/services/AddReelService.cpp:109`, `src/application/services/ReplaceReelService.cpp:127`  
**Серьёзность:** Критический

```cpp
std::thread([this, opId, slotIndex, barcode, ...]() {
    // обращается к: link_, reelRepo_, opRepo_, config_, cancelled_, onComplete_
    reelRepo_.setSlotState(...);
    if (onComplete_) onComplete_(...);
}).detach();
```

Если владелец сервиса уничтожит его пока поток ещё работает — все обращения через `this` дают use-after-free. Отсоединённый поток не имеет механизма остановки при разрушении объекта.

**Исправление:** хранить `std::thread` как член класса, вызывать `join()` в деструкторе (аналогично `RfidModuleMonitorService` и `Stm32PollingService`).

---

### BUG-3 — Целочисленное переполнение в exponential backoff

**Файл:** `src/infrastructure/hw/stm32/UartStm32Link.cpp:102`  
**Серьёзность:** Критический

```cpp
const int backoffMs = std::min(5000 * (1 << reconnectAttempts_), 60000);
```

При `reconnectAttempts_ >= 19`: `5000 * (1 << 19) = 2,621,440,000` — переполняет 32-bit `int`. Результат становится отрицательным. `std::min(отрицательное, 60000)` возвращает отрицательное число. Условие `elapsed.count() < backoffMs` никогда не выполняется, и reconnect происходит при каждом вызове `healthCheck()` — каждые 500ms, без ограничений.

**Исправление:**
```cpp
const int clampedAttempts = std::min(reconnectAttempts_, 13); // 5000 * 2^13 = 40960000, safe
const int backoffMs = std::min(5000 * (1 << clampedAttempts), 60000);
```

---

## Высокий приоритет

### BUG-4 — Гонка данных: `fd_` в `UartStm32Link` при concurrent close/write

**Файл:** `src/infrastructure/hw/stm32/UartStm32Link.cpp:87, 141`  
**Серьёзность:** Высокий

`sendCommand()` берёт `commandMtx_`, проверяет `running_` (true), затем делает `::write(fd_, ...)`. Но `close()` не берёт `commandMtx_`:

```
sendCommand: running_.load() → true → ...готовится к write...
close():     running_.store(false) → join rxThread_ → ::close(fd_); fd_=-1
sendCommand: ::write(fd_, ...)  ← пишет в закрытый fd
```

Запись в закрытый (или переиспользованный ОС) файловый дескриптор — UB.

**Исправление:** `close()` должен брать `commandMtx_` перед закрытием `fd_`, либо `fd_` должен быть защищён отдельным мьютексом.

---

### BUG-5 — Гонки данных в `RfidModuleMonitorService`

**Файл:** `src/application/services/RfidModuleMonitorService.hpp:54-60`  
**Серьёзность:** Высокий

Поля доступны из двух потоков без синхронизации:

| Поле | Читается | Пишется | Защита |
|---|---|---|---|
| `lastSeen_` | `lastSeen()` — main thread | `monitorLoop()` — monitor thread | нет |
| `moduleOnline_` | `isOnline()` — main thread | `setModuleOnline()` — monitor thread | нет |
| `switchCb_` | `monitorLoop()` — monitor thread | `setModuleSwitchCallback()` — main thread | нет |

**Исправление:** `lastSeen_` и `moduleOnline_` — защитить `statusMtx_` (он уже есть в классе). `switchCb_` — отдельный мьютекс или `std::atomic`.

---

### BUG-6 — Busy-wait 100% CPU в `HidScannerProvider`

**Файл:** `src/infrastructure/hw/scanner/HidScannerProvider.cpp:57, 79-81`  
**Серьёзность:** Высокий

```cpp
fd_ = ::open(devicePath_.c_str(), O_RDONLY | O_NONBLOCK);  // O_NONBLOCK
// ...
while (active_.load()) {
    const ssize_t n = ::read(fd_, &ev, sizeof(ev));
    if (n != sizeof(ev)) continue;  // EAGAIN → крутится без sleep
```

С `O_NONBLOCK`, когда данных нет, `read()` возвращает `-1` с `errno=EAGAIN`. Цикл продолжается немедленно, потребляя 100% одного ядра CPU.

**Исправление:** убрать `O_NONBLOCK` и использовать блокирующий read с `select()`/`poll()` и таймаутом, либо добавить `sleep_for` при `EAGAIN`.

---

## Средний приоритет

### BUG-7 — Игнорирование ошибок `tcgetattr`/`tcsetattr`

**Файл:** `src/infrastructure/hw/scanner/SerialScannerProvider.cpp:40-51`  
**Серьёзность:** Средний

```cpp
tcgetattr(fd_, &tty);        // возврат не проверяется
// ... настройка tty ...
tcsetattr(fd_, TCSANOW, &tty);  // возврат не проверяется
```

При ошибке `tcgetattr` структура `tty` остаётся нулевой/мусорной. Порт конфигурируется с неверными параметрами — устройство "работает", но данные приходят побитыми. В `UartStm32Link::open()` те же вызовы проверяются корректно.

**Исправление:** аналогично `UartStm32Link::open()` — проверить возврат обоих вызовов и вернуть ошибку.

---

### BUG-8 — Гонка в `Rc522RfidProvider::stop()` при двойном вызове

**Файл:** `src/infrastructure/hw/rfid/Rc522RfidProvider.cpp:110-120`  
**Серьёзность:** Средний

```cpp
void Rc522RfidProvider::stop() {
    if (!active_.exchange(false)) {
        closeDevice();  // вызывается без ioMtx_ и без join потока
        return;
    }
    if (thread_.joinable()) thread_.join();
    closeDevice();
}
```

При двойном вызове `stop()` из разных потоков: второй вызов попадает в ветку `closeDevice()` без ожидания завершения потока. `pollLoop()` в это время может держать `ioMtx_` и работать с `fd_`.

---

### BUG-9 — `initializeChip()` вызывается при каждом чтении RFID

**Файл:** `src/infrastructure/hw/rfid/Rc522RfidProvider.cpp:314`  
**Серьёзность:** Средний

```cpp
std::optional<std::string> Rc522RfidProvider::tryReadUid() {
    if (!initializeChip() || ...)  // soft reset + sleep(50ms) на каждый вызов
```

`initializeChip()` содержит `sleep_for(50ms)` для soft reset чипа. При `pollMs=500ms` и `readTimeoutMs=350ms` это значит 50ms оверхед на каждую итерацию мониторинга — более 14% времени уходит на лишний сброс.

---

### BUG-10 — Дублирование `slotIndexForChannel` в трёх местах

**Файлы:** `src/application/services/StartupService.cpp:258`, `src/application/services/Stm32PollingService.cpp:592`, `src/application/services/SwitchEventHandler.cpp`  
**Серьёзность:** Средний

Три независимые реализации одной логики. Граничные проверки различаются: `StartupService` проверяет `channel >= config_.slotCount`, `Stm32PollingService` возвращает `std::nullopt` для out-of-range. При изменении логики маппинга нужно менять в трёх местах.

---

## Оптимизации

### OPT-1 — Линейный поиск O(N) в hot path опроса

**Файл:** `src/application/services/Stm32PollingService.cpp:581-589`

`isIgnoredChannel()` и `isTrackedChannel()` используют `std::find` по `std::vector`. Вызываются для каждого из N каналов при каждом `applySnapshot()` (каждые 500ms) — итого O(N²) сравнений. Замена на `std::unordered_set` в конфиге даёт O(1) lookup.

---

### OPT-2 — Дублирование `parseSnapshotPayload`

**Файлы:** `src/application/services/StartupService.cpp:26`, `src/application/services/Stm32PollingService.cpp:25`

Идентичная функция скопирована дважды. Должна быть вынесена в общий заголовок или утилитарный модуль.

---

### OPT-3 — `std::ostringstream` аллокации в hot path

**Файл:** `src/application/services/Stm32PollingService.cpp:220, 461, 504`

`ostringstream` создаётся и уничтожается при каждом событии переключателя и каждом применении snapshot. При 24 каналах × 2 Hz это ~48 аллокаций в секунду только для строк логов. Альтернатива: `std::format` (C++20) или `fmt::format` без промежуточного объекта потока.

---

### OPT-4 — Лишняя копия данных в `FrameCodec::StreamParser`

**Файл:** `src/infrastructure/hw/stm32/FrameCodec.cpp` (класс `StreamParser`)

`crc_data_` накапливает все байты хедера и payload для последующего подсчёта CRC. Но эти байты уже хранятся в `header_[]` и `payload_`. CRC можно считать инкрементально при добавлении каждого байта через `crc16CcittFalse` с аккумулятором — вектор `crc_data_` и его аллокации станут ненужными.

---

### OPT-5 — Неполная таблица keycodes в `HidScannerProvider`

**Файл:** `src/infrastructure/hw/scanner/HidScannerProvider.cpp:19-41`

Таблица конвертации keycodes покрывает только цифры и буквы en-US QWERTY. Пропущены символы `-`, `.`, `/`, `_` и другие, которые часто встречаются в производственных штрихкодах (например артикулы с дефисом). При сканировании такого кода символы молча теряются без какого-либо предупреждения.

---

## Сводная таблица

| ID | Файл | Описание | Серьёзность |
|---|---|---|---|
| BUG-1 | `StartupService.cpp:126` | Dangling ref в waitReady callback | **Критический** |
| BUG-2 | `AddReelService.cpp:109`, `ReplaceReelService.cpp:127` | Use-after-free: detached thread + this | **Критический** |
| BUG-3 | `UartStm32Link.cpp:102` | Integer overflow → бесконечный reconnect | **Критический** |
| BUG-4 | `UartStm32Link.cpp:87,141` | Data race: fd_ при close vs write | **Высокий** |
| BUG-5 | `RfidModuleMonitorService.hpp:54-60` | Data race: lastSeen_, moduleOnline_, switchCb_ | **Высокий** |
| BUG-6 | `HidScannerProvider.cpp:57,80` | Busy-wait 100% CPU (O_NONBLOCK без sleep) | **Высокий** |
| BUG-7 | `SerialScannerProvider.cpp:40-51` | Ошибки tcgetattr/tcsetattr игнорируются | **Средний** |
| BUG-8 | `Rc522RfidProvider.cpp:110` | Гонка при двойном stop() | **Средний** |
| BUG-9 | `Rc522RfidProvider.cpp:314` | initializeChip() + 50ms sleep на каждое чтение | **Средний** |
| BUG-10 | 3 файла | Дублирование slotIndexForChannel | **Средний** |
| OPT-1 | `Stm32PollingService.cpp:581` | O(N) поиск в hot path → unordered_set | Оптимизация |
| OPT-2 | `StartupService.cpp:26`, `Stm32PollingService.cpp:25` | Дублирование parseSnapshotPayload | Оптимизация |
| OPT-3 | `Stm32PollingService.cpp:220+` | ostringstream аллокации в hot path | Оптимизация |
| OPT-4 | `FrameCodec.cpp` | Лишний вектор crc_data_ в StreamParser | Оптимизация |
| OPT-5 | `HidScannerProvider.cpp:19` | Неполная keycode-таблица (нет - . / _) | Оптимизация |
