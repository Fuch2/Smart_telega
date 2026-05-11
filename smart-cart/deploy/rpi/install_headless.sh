#!/usr/bin/env bash
# Устанавливает зависимости и unit-файлы для headless web-режима.
# Сам режим переключается через: smartcart-mode.sh [web|monitor]
set -euo pipefail

SMARTCART_SRC="${SMARTCART_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SYSTEMD_SRC="${SMARTCART_SRC}/deploy/rpi/systemd"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"

# ── Системные пакеты ─────────────────────────────────────────────────────────
echo "Устанавливаю системные пакеты..."
sudo apt-get update -qq
sudo apt-get install -y xvfb x11vnc novnc websockify openbox

# ── Systemd unit-файлы ───────────────────────────────────────────────────────
mkdir -p "${USER_SYSTEMD_DIR}"

cp "${SYSTEMD_SRC}/smartcart-xvfb.service"    "${USER_SYSTEMD_DIR}/"
cp "${SYSTEMD_SRC}/smartcart-openbox.service" "${USER_SYSTEMD_DIR}/"
cp "${SYSTEMD_SRC}/smartcart-x11vnc.service"  "${USER_SYSTEMD_DIR}/"
cp "${SYSTEMD_SRC}/smartcart-novnc.service"   "${USER_SYSTEMD_DIR}/"

systemctl --user daemon-reload

# Linger — сервисы работают без активной сессии
loginctl enable-linger "${USER}" 2>/dev/null || true

# ── Готово ───────────────────────────────────────────────────────────────────
echo ""
echo "Установка завершена. Теперь выбирай режим запуска:"
echo ""
echo "  bash ${SMARTCART_SRC}/deploy/rpi/smartcart-mode.sh"
echo ""
echo "  или:"
echo "    smartcart-mode.sh web      — запустить в браузере"
echo "    smartcart-mode.sh monitor  — запустить на физическом экране"
