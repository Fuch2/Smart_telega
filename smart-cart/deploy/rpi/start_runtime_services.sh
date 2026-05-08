#!/usr/bin/env bash
set -euo pipefail

SMARTCART_ROOT="${SMARTCART_ROOT:-/opt/smartcart}"

if ! command -v systemctl >/dev/null 2>&1; then
    echo "systemctl не найден" >&2
    exit 1
fi

if [ ! -x "${SMARTCART_ROOT}/current/smart_cart_ui" ]; then
    echo "Активный релиз не найден: ${SMARTCART_ROOT}/current/smart_cart_ui" >&2
    echo "Сначала выполни deploy/rpi/deploy_local.sh" >&2
    exit 1
fi

systemctl --user daemon-reload
systemctl --user start smartcart-ui.service

echo "SmartCart UI запущен."
echo
systemctl --user --no-pager --full status smartcart-ui.service
