#!/usr/bin/env bash
# Устанавливает стек headless-доступа: Xvfb + x11vnc + noVNC
# После этого приложение будет доступно в браузере на порту 6080
set -euo pipefail

SMARTCART_SRC="${SMARTCART_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SYSTEMD_SRC="${SMARTCART_SRC}/deploy/rpi/systemd"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"
USER_UID="$(id -u)"

# ── Зависимости ──────────────────────────────────────────────────────────────
echo "Устанавливаю системные пакеты..."
sudo apt-get update -qq
sudo apt-get install -y xvfb x11vnc novnc websockify

# ── Systemd unit-файлы ───────────────────────────────────────────────────────
mkdir -p "${USER_SYSTEMD_DIR}"

cp "${SYSTEMD_SRC}/smartcart-xvfb.service"   "${USER_SYSTEMD_DIR}/"
cp "${SYSTEMD_SRC}/smartcart-x11vnc.service" "${USER_SYSTEMD_DIR}/"
cp "${SYSTEMD_SRC}/smartcart-novnc.service"  "${USER_SYSTEMD_DIR}/"

# Drop-in override для smartcart-ui: переключает на виртуальный дисплей
DROP_IN_DIR="${USER_SYSTEMD_DIR}/smartcart-ui.service.d"
mkdir -p "${DROP_IN_DIR}"
sed \
    -e "s#__UID__#${USER_UID}#g" \
    "${SYSTEMD_SRC}/smartcart-ui-headless.conf.in" \
    > "${DROP_IN_DIR}/headless.conf"

# ── Включить и запустить ─────────────────────────────────────────────────────
systemctl --user daemon-reload
systemctl --user enable --now smartcart-xvfb.service
systemctl --user enable --now smartcart-x11vnc.service
systemctl --user enable --now smartcart-novnc.service

# Linger уже включён install_runtime_service.sh, но на всякий случай
loginctl enable-linger "${USER}" 2>/dev/null || true

# ── Итог ─────────────────────────────────────────────────────────────────────
LOCAL_IP="$(hostname -I | awk '{print $1}')"
echo
echo "Headless-стек установлен и запущен."
echo
echo "  Браузер:   http://${LOCAL_IP}:6080/vnc.html"
echo "  Статус:    systemctl --user status smartcart-xvfb smartcart-x11vnc smartcart-novnc"
echo "  Логи:      journalctl --user -u smartcart-novnc -f"
echo
echo "Перезапусти smartcart-ui, чтобы он подхватил виртуальный дисплей:"
echo "  systemctl --user restart smartcart-ui.service"
