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

start_runtime_services() {
    systemctl --user daemon-reload

    if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
        log "Запускаю smartcart-ui.service..."
        systemctl --user restart smartcart-ui.service
    else
        log "Сервис smartcart-ui.service ещё не установлен."
    fi

    if systemctl --user cat smartcart-web.service >/dev/null 2>&1; then
        log "Запускаю smartcart-web.service..."
        systemctl --user restart smartcart-web.service
    else
        log "Сервис smartcart-web.service ещё не установлен."
    fi
}

if [ ! -d "${SMARTCART_ROOT}/releases" ] ||
   [ ! -d "${SMARTCART_ROOT}/shared" ]; then
    echo "Runtime-каталоги не готовы. Сначала выполни:" >&2
    echo "${SMARTCART_SRC}/deploy/rpi/install_runtime_service.sh" >&2
    exit 1
fi

cd "${SMARTCART_SRC}"

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
    start_runtime_services
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
mkdir -p "${release_dir}/tools"
cp "${SMARTCART_SRC}/tools/smartcart_web.py" "${release_dir}/tools/smartcart_web.py"
chmod +x "${release_dir}/tools/smartcart_web.py"
printf '%s\n' "${release_id}" > "${release_dir}/VERSION"
printf '%s\n' "${commit}" > "${release_dir}/COMMIT"

ln -sfnT "${release_dir}" "${SMARTCART_ROOT}/current"

start_runtime_services

log "Проверяю запуск сервисов..."
sleep 5

# Health check для smartcart-ui.service
if systemctl --user cat smartcart-ui.service >/dev/null 2>&1; then
    if ! systemctl --user is-active --quiet smartcart-ui.service; then
        log "ERROR: smartcart-ui.service не запустился"
        log "Откатываю к предыдущему релизу..."
        "${SMARTCART_SRC}/deploy/rpi/rollback.sh"
        exit 1
    fi
    log "✓ smartcart-ui.service работает"
fi

# Health check для smartcart-web.service
if systemctl --user cat smartcart-web.service >/dev/null 2>&1; then
    if ! systemctl --user is-active --quiet smartcart-web.service; then
        log "WARNING: smartcart-web.service не запустился"
    else
        log "✓ smartcart-web.service работает"
    fi
fi

# Cleanup старых релизов (оставить последние 5)
log "Удаляю старые релизы..."
ls -t "${SMARTCART_ROOT}/releases" | tail -n +6 | while read old_release; do
    log "Удаляю старый релиз: ${old_release}"
    rm -rf "${SMARTCART_ROOT}/releases/${old_release}"
done

log "Готово: ${SMARTCART_ROOT}/current -> ${release_dir}"
