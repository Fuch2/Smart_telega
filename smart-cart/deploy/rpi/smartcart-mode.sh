#!/usr/bin/env bash
# Переключатель режима запуска SmartCart.
# Использование: smartcart-mode.sh [web|monitor]
set -euo pipefail

SMARTCART_SRC="${SMARTCART_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SYSTEMD_SRC="${SMARTCART_SRC}/deploy/rpi/systemd"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"
DROP_IN_DIR="${USER_SYSTEMD_DIR}/smartcart-ui.service.d"
DROP_IN="${DROP_IN_DIR}/headless.conf"
USER_UID="$(id -u)"

WEB_SERVICES=(smartcart-xvfb smartcart-x11vnc smartcart-novnc)

# ── Выбор режима ─────────────────────────────────────────────────────────────
if [[ $# -ge 1 ]]; then
    MODE="$1"
else
    echo "Как запустить SmartCart?"
    echo ""
    PS3="Выбор: "
    select opt in "Монитор (физический экран)" "Web (браузер через VNC)"; do
        case "$REPLY" in
            1) MODE="monitor"; break ;;
            2) MODE="web";     break ;;
            *) echo "Введи 1 или 2" ;;
        esac
    done
fi

# ── Режим: монитор ───────────────────────────────────────────────────────────
if [[ "$MODE" == "monitor" ]]; then
    echo "Переключаюсь на режим монитора..."

    # Убрать headless drop-in
    rm -f "${DROP_IN}"
    rmdir "${DROP_IN_DIR}" 2>/dev/null || true

    # Остановить headless-стек
    for svc in "${WEB_SERVICES[@]}"; do
        systemctl --user stop "${svc}.service" 2>/dev/null || true
        systemctl --user disable "${svc}.service" 2>/dev/null || true
    done

    systemctl --user daemon-reload
    systemctl --user restart smartcart-ui.service

    echo ""
    echo "Режим: монитор. SmartCart запущен на физическом дисплее."

# ── Режим: web ───────────────────────────────────────────────────────────────
elif [[ "$MODE" == "web" ]]; then
    echo "Переключаюсь на web-режим..."

    # Создать headless drop-in
    mkdir -p "${DROP_IN_DIR}"
    sed \
        -e "s#__UID__#${USER_UID}#g" \
        "${SYSTEMD_SRC}/smartcart-ui-headless.conf.in" \
        > "${DROP_IN}"

    # Включить и запустить headless-стек
    for svc in "${WEB_SERVICES[@]}"; do
        systemctl --user enable "${svc}.service"
        systemctl --user start  "${svc}.service"
    done

    systemctl --user daemon-reload
    systemctl --user restart smartcart-ui.service

    LOCAL_IP="$(hostname -I | awk '{print $1}')"
    TAILSCALE_IP="$(tailscale ip 2>/dev/null | head -1 || echo '')"

    echo ""
    echo "Режим: web. SmartCart доступен в браузере:"
    echo ""
    echo "  Локальная сеть:  http://${LOCAL_IP}:6080/vnc.html"
    [[ -n "$TAILSCALE_IP" ]] && \
    echo "  Tailscale:       http://${TAILSCALE_IP}:6080/vnc.html"
    echo ""

else
    echo "Неизвестный режим: '${MODE}'. Используй: web | monitor" >&2
    exit 1
fi
