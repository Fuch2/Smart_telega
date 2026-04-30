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
  python3 \
  unzip \
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
smartcart-web.service     запускает web-экран оператора
smartcart-deploy.timer    периодически проверяет git
smartcart-deploy.service  собирает и устанавливает новую версию
```

Если текущий commit уже установлен, деплой ничего не пересобирает.
При этом он всё равно запускает оба runtime-сервиса: Qt UI и web-экран.

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
restart smartcart-web.service
```

Если сборка или тесты упали, текущая рабочая версия не переключается.

## Запуск UI и web-экрана без пересборки

Если релиз уже установлен и нужно просто поднять оба экрана:

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/start_runtime_services.sh
```

Скрипт запускает:

```text
smartcart-ui.service   Qt-приложение на экране Raspberry Pi
smartcart-web.service  web-экран на порту 8080
```

Эти процессы независимы: если web-экран перезапускается, Qt не закрывается, и
если Qt упал, web-экран всё равно может показывать состояние из SQLite.

## Просмотр логов

```bash
systemctl --user status smartcart-ui.service
systemctl --user status smartcart-web.service
systemctl --user status smartcart-deploy.timer
journalctl --user -u smartcart-ui.service -n 100 -f
journalctl --user -u smartcart-web.service -n 100 -f
journalctl --user -u smartcart-deploy.service -n 100 -f
```

## Web-экран оператора

После успешного деплоя web-экран доступен в локальной сети:

```text
http://<ip-raspberry-pi>:8080
```

На самой Raspberry Pi его можно открыть так:

```text
http://127.0.0.1:8080
```

Экран работает в режиме просмотра и читает SQLite:

```text
/opt/smartcart/shared/smartcart.db
```

Он показывает крупный текущий этап, инструкцию для оператора, маршрут тележки,
модули, слоты, материалы заказа, связь и последние события.

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
