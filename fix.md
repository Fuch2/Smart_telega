# Инженерный анализ проекта Smart_Telega

**Дата:** 2026-05-01  
**Версия:** commit 2535820  
**Статус:** MVP → Production-ready

---

## Краткая оценка

Проект находится на стадии **инженерного прототипа/MVP**. Архитектура правильная (Clean Architecture, порты/адаптеры), бизнес-логика работает, железо общается, тесты есть. 

**Основная проблема:** не хватает эксплуатационной надёжности для production. Риски не в бизнес-логике, а в SQLite, hardware recovery, observability и разрастании UI-классов.

**Оценка готовности к production:** 6.5/10

---

## Метрики проекта

| Метрика | Значение |
|---------|----------|
| Строк кода (C++) | ~12,669 |
| Тестовых файлов | 12 (unit + integration) |
| Покрытие тестами | Domain ~60%, Services ~40%, Infrastructure ~30% |
| Миграций БД | 6 (без rollback) |
| Документации | Почти нет (architecture.md, protocol_stm32_uart.md пустые) |
| Deployment | Есть скрипты, но без health-check |

---

## Критичные проблемы (блокеры для production)

### 🔴 1. Репозиторий загрязнён runtime-файлами

**Проблема:**
```
smartcart.db (65KB)
smartcart.db-shm
smartcart.db-wal
smartcart.log
.DS_Store
CMakeLists_fixed.txt
```

Всё это tracked в git. БД с реальными данными в репозитории — это риск утечки и конфликты при merge.

**Решение:**
```bash
git rm --cached smart-cart/smartcart.db*
git rm --cached smart-cart/smartcart.log
git rm --cached smart-cart/.DS_Store
git rm --cached smart-cart/CMakeLists_fixed.txt

# Обновить .gitignore
echo "*.db" >> smart-cart/.gitignore
echo "*.db-*" >> smart-cart/.gitignore
echo "*.log" >> smart-cart/.gitignore
echo ".DS_Store" >> smart-cart/.gitignore
```

---

### 🔴 2. SQLite не настроен для concurrent access

**Проблема:**
- Нет `WAL` mode (Write-Ahead Logging)
- Нет `busy_timeout` для retry при блокировках
- Qt UI, web dashboard и polling-сервисы могут конфликтовать
- Критичные операции (размещение, выдача, импорт заказа) не всегда в транзакциях

**Файлы:** `SqliteConnection.cpp:130-140`

**Решение:**
```cpp
SqliteConnection::SqliteConnection(const std::string& sqlitePath) {
    const int rc = sqlite3_open(sqlitePath.c_str(), &db_);
    if (rc != SQLITE_OK || db_ == nullptr) {
        // ... error handling
    }
    
    // Включить WAL для concurrent reads
    sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr);
    
    // Установить busy timeout (5 секунд)
    sqlite3_busy_timeout(db_, 5000);
    
    // Нормальный fsync (баланс между скоростью и надёжностью)
    sqlite3_exec(db_, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, nullptr);
}
```

**Обернуть критичные операции в транзакции:**
```cpp
// WorkflowService::completeIssuing, OrderImportService::import и т.д.
conn_.execute("BEGIN IMMEDIATE;");
try {
    // ... бизнес-операции
    conn_.execute("COMMIT;");
} catch (...) {
    conn_.execute("ROLLBACK;");
    throw;
}
```

---

### 🔴 3. Документация отсутствует

**Проблема:**
- `docs/architecture.md` — 1 строка (пустой)
- `docs/protocol_stm32_uart.md` — 1 строка (пустой)
- `docs/error_handling.md` — 1 строка (пустой)
- Нет описания workflow states
- Нет описания RFID module roles

**Это блокер для передачи проекта другим разработчикам.**

**Решение:**
Документировать:
1. **Архитектуру:** слои, зависимости, основные сервисы
2. **STM32 протокол:** формат фреймов, команды, payload для каждой команды
3. **Workflow:** диаграмма состояний, переходы, условия
4. **RFID:** как работает MultiRc522, module roles (REEL/FEEDER), offline detection
5. **Deployment:** как собрать, как задеплоить, как откатить

---

## Высокоприоритетные проблемы

### 🟡 4. Большие классы (God Objects)

**Проблема:**
- `WorkerView.cpp` — 1281 строка
- `WorkerViewModel.cpp` — 851 строка
- `Stm32PollingService.cpp` — 624 строки

Это не "сломано", но поддерживать будет всё тяжелее. Каждый из этих классов делает слишком много.

**WorkerView отвечает за:**
- UI layout
- Event filtering (глобальный перехват клавиатуры)
- Barcode input handling
- Modal dialogs (workflow steps)
- Slot grid updates

**Решение:**
Разбить на:
- `BarcodeInputHandler` — перехват клавиатуры, валидация
- `WorkflowDialogManager` — модальные окна этапов
- `WorkerView` — только layout и координация

**Stm32PollingService совмещает:**
- Polling loop
- Snapshot parsing
- Event handling (switch changes)
- Barcode scan recording
- Workflow integration

**Решение:**
Извлечь:
- `SnapshotProcessor` — парсинг и применение snapshot
- `SwitchEventHandler` — обработка EvtSwitchChanged
- Оставить в `Stm32PollingService` только polling loop и dispatch

---

### 🟡 5. Слои протекают

**Проблема:**

**Application зависит от Qt:**
```cpp
// AppStateMachine.cpp
#include <QMetaObject>
#include <QMetaType>
#include <QColor>

emit slotHighlighted(slot.slotIndex, QColor(30, 80, 200));
```

**Application зависит от Infrastructure:**
```cpp
// IStm32Link.hpp (application/ports)
#include "infrastructure/hw/stm32/Protocol.hpp"
```

Для диплома/прототипа терпимо, но для инженерного проекта лучше чистить.

**Решение:**

1. **Убрать Qt из application:**
```cpp
// Вместо QColor использовать domain-тип
struct RgbColor { uint8_t r, g, b; };
emit slotHighlighted(slot.slotIndex, RgbColor{30, 80, 200});
```

2. **Вынести Protocol в domain:**
```
domain/
  protocol/
    Frame.hpp
    CommandId.hpp
```

Или использовать DTO для передачи данных между слоями.

---

### 🟡 6. Hardware recovery недостаточен

**Проблема:**
- При падении RFID система переходит в `OFFLINE`, но не пытается восстановиться
- При сбое UART нет автоматического переподключения
- Нет health status по каждому устройству (RFID online/offline, STM32 online/offline, last error)
- Нет degraded mode (работа без RFID, только по штрихкодам)

**Файлы:** `RfidModuleMonitorService.cpp`, `UartStm32Link.cpp`

**Решение:**

1. **Добавить health status:**
```cpp
struct HardwareStatus {
    bool stm32Online;
    std::chrono::system_clock::time_point stm32LastSeen;
    std::string stm32LastError;
    
    bool rfidOnline;
    std::chrono::system_clock::time_point rfidLastSeen;
    std::string rfidLastError;
};
```

2. **Реализовать reconnect для UART:**
```cpp
void UartStm32Link::healthCheck() {
    if (!isOpen()) {
        if (std::chrono::steady_clock::now() - lastReconnectAttempt_ > 5s) {
            open();
            lastReconnectAttempt_ = std::chrono::steady_clock::now();
        }
    }
}
```

3. **Добавить degraded mode:**
- Если RFID offline → работать только по штрихкодам
- Если STM32 offline → работать в read-only режиме (просмотр заказов, но не размещение)

---

### 🟡 7. Deployment без health-check

**Проблема:**
- `deploy_local.sh` деплоит и перезапускает сервисы, но не проверяет, что они успешно стартовали
- Нет автоматического rollback при падении
- Старые релизы накапливаются в `/opt/smartcart/releases`

**Файлы:** `deploy/rpi/deploy_local.sh`, `deploy/rpi/rollback.sh`

**Решение:**

1. **Добавить health-check:**
```bash
# После restart
sleep 5
if ! systemctl --user is-active smartcart-ui.service; then
    log "ERROR: smartcart-ui.service failed to start"
    log "Rolling back to previous release..."
    "${SMARTCART_SRC}/deploy/rpi/rollback.sh"
    exit 1
fi
```

2. **Cleanup старых релизов:**
```bash
# Оставить последние 5 релизов
ls -t "${SMARTCART_ROOT}/releases" | tail -n +6 | while read old_release; do
    log "Removing old release: ${old_release}"
    rm -rf "${SMARTCART_ROOT}/releases/${old_release}"
done
```

---

## Средний приоритет (улучшения)

### 🟢 8. Web без авторизации

**Проблема:**
`smartcart_web.py` не имеет аутентификации. Любой в локальной сети может управлять тележкой.

**Оценка риска:**
- Если web только в локальной сети / через Tailscale для тебя → нормально
- Если "для предприятия" → нужна хотя бы простая защита

**Решение (если нужно):**
```python
from flask_httpauth import HTTPBasicAuth

auth = HTTPBasicAuth()

users = {
    "operator": "password123",
    "admin": "admin456"
}

@auth.verify_password
def verify_password(username, password):
    if username in users and users[username] == password:
        return username

@app.route('/api/orders')
@auth.login_required
def get_orders():
    # ...
```

---

### 🟢 9. Нет CI/CD

**Проблема:**
- Нет автоматических проверок при коммите
- Нет pre-commit hooks
- Нет автоматического запуска тестов на PR

**Решение:**
Настроить GitHub Actions:

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y cmake g++ libsqlite3-dev libspdlog-dev
          
      - name: Build
        run: |
          cd smart-cart
          cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
          cmake --build build -j4
          
      - name: Run tests
        run: |
          cd smart-cart
          ctest --test-dir build --output-on-failure
```

---

### 🟢 10. Нет мониторинга

**Проблема:**
- Нет метрик производительности
- Нет alerting при сбоях UART/RFID
- Невозможно отследить путь заказа через систему

**Решение (долгосрочно):**
- Добавить Prometheus exporter
- Метрики: RFID read latency, UART errors, workflow transitions, order completion time
- Grafana dashboard для визуализации

---

### 🟢 11. Миграции БД без rollback

**Проблема:**
- 6 миграций есть, но нет down-миграций
- При ошибке миграции делается ROLLBACK, но recovery-механизма нет
- Нет проверки совместимости версий

**Решение:**
- Добавить `schema_version` в БД
- Реализовать up/down миграции (как в Rails/Django)
- Проверять версию при старте приложения

---

## Что в исходной сводке было преувеличено

### ✅ Smart pointers

**Утверждение:** "Всего 6 использований smart pointer'ов"  
**Реальность:** В проекте много `unique_ptr/make_unique`, особенно в `AppBootstrap`. В Qt сырые указатели нормальны, если есть parent ownership.

### ✅ Валидация конфигурации

**Утверждение:** "Нет валидации конфигурации"  
**Реальность:** `ConfigLoader::validate()` есть.

### ✅ Тесты WorkflowService

**Утверждение:** "Нет unit-тестов для WorkflowService"  
**Реальность:** Есть `workflow_service_test.cpp`.

### ✅ Mock аппаратуры

**Утверждение:** "Нет mock-объектов для аппаратных интерфейсов"  
**Реальность:** Есть `MockStm32Link`, `MockRfidProvider`, `MockScannerProvider`.

### ✅ Версионирование БД

**Утверждение:** "Нет версионирования схемы БД"  
**Реальность:** Есть `schema_migrations` и 6 миграций. Но rollback-миграций действительно нет.

### ✅ Rollback deploy

**Утверждение:** "Нет rollback deploy"  
**Реальность:** Есть `deploy/rpi/rollback.sh`. Но нет health-check и автоматического rollback после неудачного запуска.

### ✅ Console output

**Утверждение:** "0 cout/cerr — хорошо"  
**Реальность:** В CLI `smartcart_app` они есть, и это нормально.

### ✅ C-style casts

**Утверждение:** "29 unsafe casts — критично"  
**Реальность:** В основном это `reinterpret_cast` вокруг `sqlite3_column_text`. Для SQLite C API это обычная зона, не главный риск.

---

## Roadmap к production

### Этап 1: Эксплуатационная надёжность (1-2 недели)

**Цель:** Устранить блокеры для production

- [ ] Почистить репозиторий: убрать БД/WAL/log/.DS_Store из git
- [ ] Усилить SQLite: WAL, busy_timeout, транзакции для критичных операций
- [ ] Документировать архитектуру, STM32-протокол, workflow, RFID
- [ ] Добавить health-check в deploy, cleanup старых релизов
- [ ] Реализовать hardware health status (RFID online/offline, STM32 online/offline)

### Этап 2: Рефакторинг (1-2 месяца)

**Цель:** Улучшить поддерживаемость

- [ ] Разбить `WorkerView`, `WorkerViewModel`, `Stm32PollingService`
- [ ] Убрать Qt из application layer
- [ ] Вынести Protocol в domain или использовать DTO
- [ ] Реализовать reconnect для UART/RFID
- [ ] Добавить degraded mode (работа без RFID)

### Этап 3: Инфраструктура (2-3 месяца)

**Цель:** Автоматизация и observability

- [ ] Настроить CI/CD (GitHub Actions)
- [ ] Добавить Prometheus exporter
- [ ] Реализовать structured logging (JSON)
- [ ] Grafana dashboards
- [ ] Alerting на критичные события
- [ ] Rollback-миграции БД

---

## Итоговая оценка

### Текущий статус: **MVP / Инженерный прототип**

| Критерий | Оценка | Комментарий |
|----------|--------|-------------|
| **Архитектура** | 8/10 | Clean Architecture, порты/адаптеры — правильно |
| **Бизнес-логика** | 7/10 | Работает, покрыта тестами |
| **Надёжность** | 5/10 | SQLite слабый, hardware recovery недостаточен |
| **Поддерживаемость** | 6/10 | Большие классы, слои протекают |
| **Документация** | 2/10 | Почти нет |
| **Deployment** | 6/10 | Скрипты есть, но без health-check |
| **Observability** | 3/10 | Нет метрик, нет structured logging |

### Общая оценка готовности к production: **6.5/10**

**Вывод:** Архитектура правильная по направлению, но проект находится на стадии MVP. Основные риски не в бизнес-логике, а в эксплуатационной надёжности: SQLite, hardware recovery, observability, документация и разрастание UI-классов.

**Оценка времени до production-ready:** 1-2 недели на критичные проблемы + 1-2 месяца на рефакторинг.

---

**Автор анализа:** Kiro AI  
**Дата:** 2026-05-01
