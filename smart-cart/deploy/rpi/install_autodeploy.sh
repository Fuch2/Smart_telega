#!/usr/bin/env bash
set -euo pipefail

SMARTCART_DIR="${SMARTCART_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker не найден. Для Ubuntu 24 обычно нужно:"
    echo "sudo apt install -y software-properties-common"
    echo "sudo add-apt-repository -y universe"
    echo "sudo apt update"
    echo "sudo apt install -y docker.io docker-compose-v2"
    exit 1
fi

if ! docker compose version >/dev/null 2>&1 && ! command -v docker-compose >/dev/null 2>&1; then
    echo "Docker Compose не найден. Для Ubuntu 24 обычно нужно: sudo apt install -y docker-compose-v2"
    exit 1
fi

mkdir -p "${USER_SYSTEMD_DIR}"
chmod +x "${SMARTCART_DIR}/deploy/rpi/autodeploy.sh"

sed "s#__SMARTCART_DIR__#${SMARTCART_DIR}#g" \
    "${SMARTCART_DIR}/deploy/rpi/systemd/smartcart-autodeploy.service.in" \
    > "${USER_SYSTEMD_DIR}/smartcart-autodeploy.service"

cp "${SMARTCART_DIR}/deploy/rpi/systemd/smartcart-autodeploy.timer" \
   "${USER_SYSTEMD_DIR}/smartcart-autodeploy.timer"

systemctl --user daemon-reload
systemctl --user enable --now smartcart-autodeploy.timer
systemctl --user start smartcart-autodeploy.service

if command -v loginctl >/dev/null 2>&1; then
    loginctl enable-linger "${USER}" >/dev/null 2>&1 || true
fi

echo "SmartCart autodeploy установлен."
echo "Статус: systemctl --user status smartcart-autodeploy.timer"
echo "Логи:   journalctl --user -u smartcart-autodeploy.service -n 100 -f"
