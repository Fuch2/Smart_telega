#!/usr/bin/env bash
set -euo pipefail

SMARTCART_SRC="${SMARTCART_SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SMARTCART_ROOT="${SMARTCART_ROOT:-/opt/smartcart}"
BUILD_DIR="${SMARTCART_BUILD_DIR:-${SMARTCART_SRC}/build/release}"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

jobs_count() {
    getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Не найдена команда: $1" >&2
        exit 1
    fi
}

require_command git
require_command cmake
require_command ctest
require_command systemctl

# Запускает сервис только если он не работает — не прерывает работающее приложение.
ensure_runtime_services() {
    systemctl --user daemon-reload

    if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
        if ! systemctl --user is-active --quiet smartcart-ui.service; then
            log "Запускаю smartcart-ui.service..."
            systemctl --user start smartcart-ui.service
        else
            log "smartcart-ui.service уже работает, не перезапускаем."
        fi
    else
        log "Сервис smartcart-ui.service ещё не установлен."
    fi
}

# Перезапускает сервис — используется только при деплое новой версии.
restart_runtime_services() {
    systemctl --user daemon-reload

    if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
        log "Перезапускаю smartcart-ui.service (новая версия)..."
        systemctl --user restart smartcart-ui.service
    else
        log "Сервис smartcart-ui.service ещё не установлен."
    fi
}

if [ ! -d "${SMARTCART_ROOT}/releases" ] ||
   [ ! -d "${SMARTCART_ROOT}/shared" ]; then
    echo "Runtime-каталоги не готовы. Сначала выполни:" >&2
    echo "${SMARTCART_SRC}/deploy/rpi/install_runtime_service.sh" >&2
    exit 1
fi

cd "${SMARTCART_SRC}"

# Останавливаем таймер чтобы он не перезапустил сервисы во время деплоя.
# Запустим его обратно в конце (или при откате).
if systemctl --user cat smartcart-deploy.timer >/dev/null 2>&1; then
    systemctl --user stop smartcart-deploy.timer 2>/dev/null || true
fi
trap 'systemctl --user start smartcart-deploy.timer 2>/dev/null || true' EXIT

log "Проверяю обновления git..."
git fetch --prune
git pull --ff-only

commit="$(git rev-parse --short=12 HEAD)"
current_commit=""
if [ -f "${SMARTCART_ROOT}/current/COMMIT" ]; then
    current_commit="$(cat "${SMARTCART_ROOT}/current/COMMIT")"
fi

if [ "${SMARTCART_FORCE_DEPLOY:-0}" != "1" ] &&
   [ "${current_commit}" = "${commit}" ]; then
    log "Версия ${commit} уже установлена, пересборка не нужна."
    ensure_runtime_services
    exit 0
fi

release_id="$(date '+%Y%m%d-%H%M%S')-${commit}"
release_dir="${SMARTCART_ROOT}/releases/${release_id}"
jobs="$(jobs_count)"

log "Собираю SmartCart (${jobs} потоков)..."
cmake -S "${SMARTCART_SRC}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --target \
    smartcart_app \
    smart_cart_ui \
    frame_codec_tests \
    domain_tests \
    integration_tests \
    -j"${jobs}"

log "Запускаю тесты..."
ctest --test-dir "${BUILD_DIR}" --output-on-failure

log "Устанавливаю релиз ${release_id}..."
mkdir -p "${release_dir}"
cp "${BUILD_DIR}/smartcart_app" "${release_dir}/smartcart_app"
cp "${BUILD_DIR}/build_qt/smart_cart_ui" "${release_dir}/smart_cart_ui"
cp -a "${BUILD_DIR}/build_qt/config" "${release_dir}/config"
cp -a "${BUILD_DIR}/build_qt/migrations" "${release_dir}/migrations"
printf '%s\n' "${release_id}" > "${release_dir}/VERSION"
printf '%s\n' "${commit}" > "${release_dir}/COMMIT"

ln -sfnT "${release_dir}" "${SMARTCART_ROOT}/current"

restart_runtime_services

log "Проверяю запуск сервисов..."

# Ждём до 15 секунд: Qt-приложение стартует ~3-5 с, даём запас.
ui_ok=0
if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
    for i in 1 2 3; do
        sleep 5
        if systemctl --user is-active --quiet smartcart-ui.service; then
            ui_ok=1
            break
        fi
        log "Ждём smartcart-ui.service (попытка $i/3)..."
    done
    if [ "${ui_ok}" -eq 0 ]; then
        log "ERROR: smartcart-ui.service не запустился"
        log "Откатываю к предыдущему релизу..."
        "${SMARTCART_SRC}/deploy/rpi/rollback.sh"
        exit 1
    fi
    log "✓ smartcart-ui.service работает"
fi

# Cleanup старых релизов (оставить последние 5)
log "Удаляю старые релизы..."
ls -t "${SMARTCART_ROOT}/releases" | tail -n +6 | while read old_release; do
    log "Удаляю старый релиз: ${old_release}"
    rm -rf "${SMARTCART_ROOT}/releases/${old_release}"
done

log "Готово: ${SMARTCART_ROOT}/current -> ${release_dir}"
