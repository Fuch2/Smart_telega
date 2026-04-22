# SmartCart: автодеплой на Raspberry Pi через Docker

Цель схемы простая:

```text
MacBook -> git push -> Raspberry Pi -> git pull -> docker build -> docker up
```

После этого не нужно вручную пересобирать проект на малине после каждого изменения. Малина сама периодически проверяет git, подтягивает новые коммиты, собирает Docker-образ и перезапускает приложение.

## Что где хранится

- Код проекта: `~/Documents/Smart_telega/smart-cart`
- Dockerfile: `docker/Dockerfile.rpi`
- Docker Compose: `docker-compose.rpi.yml`
- Автодеплой: `deploy/rpi/autodeploy.sh`
- systemd timer: `deploy/rpi/systemd/smartcart-autodeploy.timer`
- База и логи на малине: `runtime/smartcart.db`, `runtime/smartcart.log`

Контейнер запускает приложение из `/opt/smartcart`, а рабочая директория контейнера `/data` примонтирована к `./runtime`. Поэтому относительные пути из `config/config.json`:

```json
"sqlite_path": "./smartcart.db",
"log_file": "./smartcart.log"
```

попадают именно в `smart-cart/runtime`.

## Первый запуск на Raspberry Pi

Выполнить один раз:

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y docker.io docker-compose-v2 x11-xserver-utils git
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
sudo reboot
```

После перезагрузки:

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/install_autodeploy.sh
```

Скрипт установит user-level systemd timer и сразу выполнит первый деплой.

Если `docker-compose-v2` не находится, значит на системе не включён репозиторий
`universe` или используется другой образ Ubuntu. Тогда можно поставить Docker из
официального Docker-репозитория, где пакет называется `docker-compose-plugin`.

## Ручной запуск деплоя

```bash
cd ~/Documents/Smart_telega/smart-cart
./deploy/rpi/autodeploy.sh
```

## Проверка статуса

```bash
systemctl --user status smartcart-autodeploy.timer
journalctl --user -u smartcart-autodeploy.service -n 100 -f
docker logs -f smartcart-ui
```

## Остановка

```bash
cd ~/Documents/Smart_telega/smart-cart
docker compose -f docker-compose.rpi.yml down
systemctl --user disable --now smartcart-autodeploy.timer
```

## GUI и железо

Compose-файл пробрасывает в контейнер:

- `/tmp/.X11-unix` для окна Qt;
- `/dev` для UART, SPI, RFID и USB-TTL адаптеров;
- `./runtime` для базы и логов.

Если окно Qt не открывается, выполнить на малине:

```bash
xhost +local:root
```

Если UART-адаптеры подключены через USB-хаб, лучше смотреть стабильные имена так:

```bash
ls -l /dev/serial/by-path/
```

И уже эти пути прописывать в `config/config.json` в `module_channels`.

## Обычный рабочий цикл

На ноутбуке:

```bash
git add .
git commit -m "..."
git push
```

На малине ничего нажимать не нужно. В течение минуты timer выполнит:

```text
git pull --ff-only
docker compose build
docker compose up -d
```
