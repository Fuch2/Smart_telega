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

disable_obsolete_web_services() {
    local services=(
        smartcart-web
        smartcart-xvfb
        smartcart-openbox
        smartcart-x11vnc
        smartcart-novnc
    )

    for svc in "${services[@]}"; do
        systemctl --user stop "${svc}.service" 2>/dev/null || true
        systemctl --user disable "${svc}.service" 2>/dev/null || true
    done

    rm -f "${HOME}/.config/systemd/user/smartcart-ui.service.d/headless.conf"
    rmdir "${HOME}/.config/systemd/user/smartcart-ui.service.d" 2>/dev/null || true
}

systemctl --user daemon-reload
disable_obsolete_web_services
systemctl --user daemon-reload
systemctl --user start smartcart-ui.service

echo "SmartCart UI запущен."
echo
systemctl --user --no-pager --full status smartcart-ui.service
