# Архитектура SmartCart

## Обзор

SmartCart — система управления умной тележкой для подбора и выдачи материалов на производстве. Проект построен на принципах **Clean Architecture** с разделением на слои и использованием паттерна **Ports & Adapters** (Hexagonal Architecture).

## Слои архитектуры

```
┌─────────────────────────────────────────────────────────┐
│                    Presentation                         │
│  (Qt UI, Web API)                                       │
│  - WorkerView, AdminView                                │
│  - MainWindow, ViewModels                               │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                    Application                          │
│  (Use Cases, Services, State Machine)                   │
│  - WorkflowService, StartupService                      │
│  - AddReelService, ReplaceReelService                   │
│  - Stm32PollingService, RfidModuleMonitorService        │
│  - AppStateMachine                                      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                      Ports                              │
│  (Interfaces)                                           │
│  - IReelRepository, IOrderRepository                    │
│  - IStm32Link, IRfidProvider, IScannerProvider          │
│  - IEventLogger                                         │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                  Infrastructure                         │
│  (Adapters, Hardware, Database)                         │
│  - SqliteConnection, *RepositorySqlite                  │
│  - UartStm32Link, MockStm32Link                         │
│  - Rc522RfidProvider, MultiRc522RfidProvider            │
│  - HidScannerProvider, SerialScannerProvider            │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                      Domain                             │
│  (Entities, Value Objects, Business Rules)              │
│  - ReelRecord, Slot, Operation                          │
│  - CartWorkflow, OrderItem                              │
│  - ErrorCode, SlotState                                 │
└─────────────────────────────────────────────────────────┘
```

## Domain Layer

**Ответственность:** Бизнес-логика, сущности, правила предметной области.

**Ключевые компоненты:**
- `ReelRecord` — запись о катушке материала (barcode, slot, timestamps)
- `Slot` — слот в модуле (index, state: Free/Occupied/Reserved/Error)
- `Operation` — операция (AddReel, ReplaceReel, status, timestamps)
- `CartWorkflow` — состояние workflow тележки (state, currentOrderId)
- `OrderItem` — позиция заказа (barcode, quantity, status)
- `ErrorCode` — коды ошибок (InvalidBarcode, SlotOccupied, NoFreeSlot и т.д.)

**Правила:**
- Domain слой не зависит ни от каких других слоёв
- Только чистый C++, без Qt, без SQLite, без hardware
- Все бизнес-правила инкапсулированы здесь

## Application Layer

**Ответственность:** Оркестрация use cases, координация между domain и infrastructure.

**Основные сервисы:**

### WorkflowService
Управляет жизненным циклом заказа:
- `startFeederLoading()` — начать загрузку питателей
- `completeFeederLoading()` — завершить загрузку питателей
- `notifyMaterialPlaced()` — уведомить о размещении материала
- `startIssuingToLine()` — начать выдачу на линию
- `markItemIssued()` — отметить материал как выданный
- `completeIssuing()` — завершить выдачу
- `markLeftoverReturnedBySlot()` — вернуть остаток на склад

### StartupService
Инициализация системы при запуске:
- Проверка связи с STM32
- Чтение snapshot слотов
- Восстановление состояния из БД

### AddReelService / ReplaceReelService
Размещение и замена катушек:
- Поиск свободного слота
- Резервирование слота
- Подсветка LED
- Запись в БД

### Stm32PollingService
Опрос STM32 в фоновом потоке:
- Периодический запрос snapshot (состояние всех слотов)
- Обработка событий (EvtSwitchChanged)
- Обновление состояния слотов в БД

### RfidModuleMonitorService
Мониторинг RFID-модулей:
- Периодическое чтение RFID-меток
- Определение online/offline статуса модулей
- Запрос на переключение модуля при появлении новой метки

### AppStateMachine
Координация состояния приложения:
- Переходы между состояниями (Idle → Initializing → Ready → Operating → Error)
- Интеграция с Qt signals/slots
- Обработка ошибок

## Ports (Interfaces)

**Ответственность:** Определение контрактов между application и infrastructure.

**Репозитории:**
- `IReelRepository` — работа с катушками (add, getBySlot, markRemoved)
- `IOrderRepository` — работа с заказами (create, getItems, updateStatus)
- `IWorkflowRepository` — работа с workflow (get, setState, clearCurrentOrder)
- `IOperationRepository` — работа с операциями (add, update, getActive)
- `IModuleRepository` — работа с модулями (add, updateStatus, getBySerial)

**Hardware провайдеры:**
- `IStm32Link` — связь с STM32 (sendCommand, setEventCallback)
- `IRfidProvider` — чтение RFID (readOnce, readAllOnce, setRfidCallback)
- `IScannerProvider` — чтение штрихкодов (readBarcode, setCallback)

**Логирование:**
- `IEventLogger` — запись событий (log(level, code, message))

## Infrastructure Layer

**Ответственность:** Реализация портов, работа с внешними системами.

### Database (SQLite)

**SqliteConnection:**
- Управление соединением с БД
- WAL mode для concurrent access
- busy_timeout для retry при блокировках
- Транзакции (beginTransaction, commit, rollback)
- Миграции (runMigrations)

**Репозитории:**
- `ReelRepositorySqlite` — таблица `reels`
- `OrderRepositorySqlite` — таблицы `orders`, `order_items`
- `WorkflowRepositorySqlite` — таблица `cart_workflow`
- `ModuleRepositorySqlite` — таблица `modules`

### Hardware

**STM32 (UART):**
- `UartStm32Link` — реальная связь через `/dev/ttyAMA0`
- `MockStm32Link` — mock для тестов
- `FrameCodec` — кодирование/декодирование фреймов протокола

**RFID (SPI):**
- `Rc522RfidProvider` — один RC522 считыватель
- `MultiRc522RfidProvider` — несколько RC522 (до 6 SPI устройств)
- `MockRfidProvider` — mock для тестов

**Scanner:**
- `HidScannerProvider` — USB HID сканер
- `SerialScannerProvider` — Serial сканер
- `MockScannerProvider` — mock для тестов

### Configuration

**ConfigLoader:**
- Загрузка `config.json`
- Валидация конфигурации
- Маппинг слотов на LED

**ModuleProfileLoader:**
- Загрузка профилей модулей (tray24.json)
- Определение количества слотов

## Presentation Layer

**Ответственность:** UI, взаимодействие с пользователем.

### Qt UI

**MainWindow:**
- Главное окно приложения
- Переключение между WorkerView и AdminView

**WorkerView:**
- Интерфейс оператора
- Ввод штрихкодов
- Отображение слотов (SlotGridWidget)
- Модальные окна этапов workflow

**AdminView:**
- Интерфейс администратора
- Импорт заказов
- Диагностика
- Просмотр логов

**ViewModels:**
- `WorkerViewModel` — бизнес-логика для WorkerView
- `AdminViewModel` — бизнес-логика для AdminView

### Web API

**smartcart_web.py:**
- Flask REST API
- Просмотр заказов, операций, логов
- Управление workflow (для удалённого мониторинга)

## Паттерны и принципы

### Dependency Inversion
Application слой зависит от портов (интерфейсов), а не от конкретных реализаций. Infrastructure реализует порты.

### Repository Pattern
Абстракция доступа к данным. Application не знает, что данные хранятся в SQLite.

### Service Layer
Бизнес-логика инкапсулирована в сервисах. UI только вызывает методы сервисов.

### Event-Driven
- Qt signals/slots для UI
- Callbacks для hardware (RFID, Scanner, STM32 events)

### Polling + Events
- Polling для STM32 snapshot (каждые 500ms)
- Events для switch changes, RFID появления/исчезновения

## Потоки выполнения

### Главный поток (Qt UI)
- Обработка UI событий
- Вызовы сервисов
- Обновление UI

### Фоновые потоки
- `Stm32PollingService::pollLoop()` — опрос STM32
- `RfidModuleMonitorService::pollLoop()` — опрос RFID
- `UartStm32Link::rxThread_` — чтение UART

### Синхронизация
- `std::mutex` для защиты shared state
- `std::atomic` для флагов (running_)
- Qt signals/slots для межпоточной коммуникации

## Конфигурация

**config.json:**
```json
{
  "stm32_device": "/dev/ttyAMA0",
  "stm32_poll_ms": 500,
  "rfid_enabled": true,
  "rfid_spi_devices": ["/dev/spidev0.0", "/dev/spidev0.1", ...],
  "rfid_module_roles": {
    "E201FC03": "REEL",
    "E3906D2D": "FEEDER"
  },
  "rfid_poll_ms": 500,
  "rfid_offline_timeout_ms": 3000,
  "slot_to_led_map": [0, 2, 4, 6, ...],
  "switch_tracked_channels": [0, 1, 2, ...]
}
```

## Deployment

**Структура на Raspberry Pi:**
```
/opt/smartcart/
  current -> releases/20260501-123456-abc123/
  releases/
    20260501-123456-abc123/
      smart_cart_ui
      smartcart_app
      config/
      migrations/
      tools/smartcart_web.py
  shared/
    smartcart.db
    smartcart.log
```

**Systemd services:**
- `smartcart-ui.service` — Qt UI
- `smartcart-web.service` — Flask web API

## Тестирование

**Unit tests:**
- `frame_codec_tests` — тесты протокола STM32
- `domain_tests` — тесты domain entities

**Integration tests:**
- `startup_flow_test` — тест инициализации
- `add_reel_flow_test` — тест размещения катушки
- `workflow_service_test` — тест workflow transitions
- `rfid_module_monitor_test` — тест RFID мониторинга

**Моки:**
- `MockStm32Link` — для тестов без железа
- `MockRfidProvider` — для тестов без RFID
- `MockScannerProvider` — для тестов без сканера

## Известные ограничения

1. **SQLite для concurrent access** — подходит для одной тележки, но не для нескольких процессов на разных машинах
2. **Polling вместо interrupts** — STM32 опрашивается каждые 500ms, а не по прерываниям
3. **Qt в application layer** — `AppStateMachine` зависит от Qt (QObject, signals/slots)
4. **Hardcoded moduleId** — сервисы работают с одним модулем, нет поддержки нескольких тележек

## Roadmap

1. **Убрать Qt из application layer** — использовать domain-типы вместо QColor, QString
2. **Event-driven вместо polling** — STM32 должен отправлять события по UART
3. **Multi-cart support** — поддержка нескольких тележек одновременно
4. **PostgreSQL для production** — для distributed deployment
5. **Microservices** — разделить на независимые сервисы с message broker

---

**Автор:** SmartCart Team  
**Дата:** 2026-05-01
