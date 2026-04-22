#!/usr/bin/env bash
set -euo pipefail

SMARTCART_DIR="${SMARTCART_DIR:-$HOME/Documents/Smart_telega/smart-cart}"
COMPOSE_FILE="${SMARTCART_DIR}/docker-compose.rpi.yml"
LOCK_FILE="${SMARTCART_DIR}/runtime/autodeploy.lock"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

compose_cmd() {
    if docker compose version >/dev/null 2>&1; then
        printf 'docker compose'
    elif command -v docker-compose >/dev/null 2>&1; then
        printf 'docker-compose'
    else
        log "Docker Compose не найден. Установи docker-compose-plugin."
        exit 1
    fi
}

mkdir -p "${SMARTCART_DIR}/runtime"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    log "Автодеплой уже выполняется, пропускаю запуск."
    exit 0
fi

cd "${SMARTCART_DIR}"

if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    xhost +local:root >/dev/null 2>&1 || true
fi

compose="$(compose_cmd)"
before="$(git rev-parse HEAD)"

log "Проверяю обновления в git..."
git fetch --prune
git pull --ff-only

after="$(git rev-parse HEAD)"

if [ "${SMARTCART_FORCE_BUILD:-0}" = "1" ] ||
   [ "${before}" != "${after}" ] ||
   ! docker image inspect smartcart-ui:local >/dev/null 2>&1; then
    log "Собираю Docker-образ smartcart-ui..."
    COMPOSE_BAKE=false ${compose} -f "${COMPOSE_FILE}" build smartcart-ui
else
    log "Новых коммитов нет, образ уже существует."
fi

log "Запускаю/обновляю контейнер smartcart-ui..."
${compose} -f "${COMPOSE_FILE}" up -d --remove-orphans smartcart-ui

log "Готово."
