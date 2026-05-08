#!/usr/bin/env bash
set -euo pipefail

SMARTCART_SRC="${SMARTCART_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SMARTCART_ROOT="${SMARTCART_ROOT:-/opt/smartcart}"
DISPLAY_VALUE="${SMARTCART_DISPLAY:-${DISPLAY:-:0}}"
XAUTHORITY_VALUE="${XAUTHORITY:-${HOME}/.Xauthority}"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"

if ! command -v systemctl >/dev/null 2>&1; then
    echo "systemctl не найден" >&2
    exit 1
fi

sudo mkdir -p \
    "${SMARTCART_ROOT}/releases" \
    "${SMARTCART_ROOT}/shared"
sudo chown -R "${USER}:$(id -gn)" "${SMARTCART_ROOT}"

mkdir -p "${USER_SYSTEMD_DIR}"

sed \
    -e "s#__SMARTCART_ROOT__#${SMARTCART_ROOT}#g" \
    -e "s#__DISPLAY__#${DISPLAY_VALUE}#g" \
    -e "s#__XAUTHORITY__#${XAUTHORITY_VALUE}#g" \
    "${SMARTCART_SRC}/deploy/rpi/systemd/smartcart-ui.service.in" \
    > "${USER_SYSTEMD_DIR}/smartcart-ui.service"

sed \
    -e "s#__SMARTCART_SRC__#${SMARTCART_SRC}#g" \
    -e "s#__SMARTCART_ROOT__#${SMARTCART_ROOT}#g" \
    "${SMARTCART_SRC}/deploy/rpi/systemd/smartcart-deploy.service.in" \
    > "${USER_SYSTEMD_DIR}/smartcart-deploy.service"

cp "${SMARTCART_SRC}/deploy/rpi/systemd/smartcart-deploy.timer" \
   "${USER_SYSTEMD_DIR}/smartcart-deploy.timer"

systemctl --user import-environment DISPLAY XAUTHORITY >/dev/null 2>&1 || true
systemctl --user daemon-reload
systemctl --user enable smartcart-ui.service
systemctl --user enable --now smartcart-deploy.timer
systemctl --user start --no-block smartcart-deploy.service

if command -v loginctl >/dev/null 2>&1; then
    loginctl enable-linger "${USER}" >/dev/null 2>&1 || true
fi

echo "Runtime SmartCart подготовлен."
echo "Корень:  ${SMARTCART_ROOT}"
echo "Экран:   ${DISPLAY_VALUE}"
echo "Автодеплой включён: systemctl --user status smartcart-deploy.timer"
