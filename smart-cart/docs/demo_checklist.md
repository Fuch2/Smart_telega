# Demo Checklist

## 1. Сборка и тесты

```bash
cd ~/Documents/Smart_telega/smart-cart
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target smartcart_app smart_cart_ui frame_codec_tests domain_tests integration_tests -j4
ctest --test-dir build --output-on-failure
```

Строгая проверка перед финальным показом:

```bash
cmake -S . -B build/werror -DCMAKE_BUILD_TYPE=Release -DSMARTCART_WERROR=ON
cmake --build build/werror --target smartcart_app smart_cart_ui frame_codec_tests domain_tests integration_tests -j4
ctest --test-dir build/werror --output-on-failure
```

## 2. Конфиг

- `config/config.json` содержит `"demo_mode": false`.
- `stm32_device` совпадает с реальным UART STM32 на Raspberry Pi.
- `rfid_enabled` включён только если RFID-модули подключены.
- `switch_channel_to_slot_map` и `slot_to_led_map` совпадают с физической тележкой.
- `switch_ignored_channels` содержит только реально неиспользуемые каналы.

## 3. Runtime на Raspberry Pi

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/deploy_local.sh
systemctl --user status smartcart-ui.service
journalctl --user -u smartcart-ui.service -n 100 --no-pager
```

Ожидаемо:
- работает только `smartcart-ui.service`;
- старые web/VNC-сервисы остановлены;
- в свежих логах нет `ERROR`;
- приложение открывается на экране Raspberry Pi.

## 4. Железо

- STM32 отвечает, переключатели меняют состояние слотов.
- Каждый физический слот соответствует правильному номеру в UI.
- Подсветка каждого целевого слота совпадает с номером в UI.
- RFID видит активный модуль и не теряет его без физического удаления.
- Сканер штрихкодов вводит код в активный сценарий без ручного фокуса мышью.

## 5. Основной сценарий

- Холодный старт приложения.
- Выбор или распознавание модуля.
- Импорт заказа.
- Переход в подготовку фидера.
- Скан нужной катушки.
- Подсветка нужного слота.
- Установка катушки в правильный слот.
- Ошибка при установке в неправильный слот.
- Завершение выдачи.
- Возврат остатков.
- Перезапуск приложения после сценария: состояние БД корректно сохраняется.

## 6. Быстрый откат

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/rollback.sh
systemctl --user restart smartcart-ui.service
```
