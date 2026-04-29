# SmartCart: нативный деплой на Raspberry Pi

Эта схема не использует Docker. Raspberry Pi сама подтягивает код, собирает
проект, прогоняет тесты и устанавливает новую версию только после успешной
проверки.

```text
MacBook -> git push -> Raspberry Pi -> git pull -> build -> tests -> release -> restart
```

## Структура на Raspberry Pi

```text
/opt/smartcart/
  current -> /opt/smartcart/releases/<version>
  releases/
    <version>/
      smart_cart_ui
      config/
      migrations/
      VERSION
      COMMIT
  shared/
    smartcart.db
    smartcart.log
```

`current` указывает на активную версию. `shared` хранит данные, которые нельзя
терять при обновлении: базу, логи и локальные рабочие файлы.

## Установка зависимостей

Один раз на Raspberry Pi:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  qt6-base-dev \
  libsqlite3-dev \
  libspdlog-dev \
  nlohmann-json3-dev \
  libgtest-dev \
  sqlite3 \
  git
```

## Первичная установка runtime

Если приложение запускается с физического экрана Raspberry Pi:

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/install_runtime_service.sh
```

Если запуск идёт через xrdp и `echo $DISPLAY` показывает, например `:10.0`:

```bash
cd ~/Documents/Smart_telega/smart-cart
SMARTCART_DISPLAY=:10.0 ./deploy/rpi/install_runtime_service.sh
```

Скрипт создаёт user-level systemd сервисы:

```text
smartcart-ui.service      запускает текущую версию UI
smartcart-deploy.timer    периодически проверяет git
smartcart-deploy.service  собирает и устанавливает новую версию
```

Если текущий commit уже установлен, деплой ничего не пересобирает.

## Сборка и установка новой версии

Обычно это делает timer. Вручную можно запустить так:

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/deploy_local.sh
```

Скрипт делает:

```text
git pull --ff-only
cmake configure
cmake build
ctest
copy release to /opt/smartcart/releases/<version>
switch /opt/smartcart/current
restart smartcart-ui.service
```

Если сборка или тесты упали, текущая рабочая версия не переключается.

## Просмотр логов

```bash
systemctl --user status smartcart-ui.service
systemctl --user status smartcart-deploy.timer
journalctl --user -u smartcart-ui.service -n 100 -f
journalctl --user -u smartcart-deploy.service -n 100 -f
```

## Откат

Откат на предыдущий релиз:

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/rollback.sh
```

Откат на конкретный релиз:

```bash
./deploy/rpi/rollback.sh 20260429-120000-a1b2c3d4e5f6
```

## Полезные проверки

Проверить активную версию:

```bash
readlink -f /opt/smartcart/current
cat /opt/smartcart/current/VERSION
cat /opt/smartcart/current/COMMIT
```

Проверить базу:

```bash
sqlite3 /opt/smartcart/shared/smartcart.db ".tables"
```
