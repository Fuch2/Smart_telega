#!/usr/bin/env bash
set -euo pipefail

SMARTCART_ROOT="${SMARTCART_ROOT:-/opt/smartcart}"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

if [ ! -d "${SMARTCART_ROOT}/releases" ]; then
    echo "Нет каталога релизов: ${SMARTCART_ROOT}/releases" >&2
    exit 1
fi

current_target="$(readlink -f "${SMARTCART_ROOT}/current" 2>/dev/null || true)"

if [ "${1:-}" != "" ]; then
    if [ -d "$1" ]; then
        target="$1"
    else
        target="${SMARTCART_ROOT}/releases/$1"
    fi
else
    target=""
    while IFS= read -r candidate; do
        if [ "${candidate}" != "${current_target}" ]; then
            target="${candidate}"
            break
        fi
    done < <(find "${SMARTCART_ROOT}/releases" -mindepth 1 -maxdepth 1 -type d | sort -r)
fi

if [ "${target}" = "" ] || [ ! -d "${target}" ]; then
    echo "Не найден релиз для отката." >&2
    echo "Доступные релизы:" >&2
    find "${SMARTCART_ROOT}/releases" -mindepth 1 -maxdepth 1 -type d | sort -r >&2
    exit 1
fi

ln -sfnT "${target}" "${SMARTCART_ROOT}/current"
log "Переключено на ${target}"

if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
    systemctl --user restart smartcart-ui.service
    log "smartcart-ui.service перезапущен"
else
    log "smartcart-ui.service не установлен"
fi
