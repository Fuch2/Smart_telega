#!/usr/bin/env bash
set -euo pipefail

# Определяем корень репозитория относительно расположения скрипта
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUT_BIN="${ROOT_DIR}/full_stage_check"

echo "[INFO] Building host checks binary..."

cc -std=c99 -Wall -Wextra -Werror \
  -I"${ROOT_DIR}/App/Inc" \
  -I"${ROOT_DIR}/Hw/Inc" \
  "${ROOT_DIR}/App/Src/protocol_crc16.c" \
  "${ROOT_DIR}/App/Src/protocol_frame.c" \
  "${ROOT_DIR}/App/Src/cmd_runtime.c" \
  "${ROOT_DIR}/App/Src/cmd_handlers.c" \
  "${ROOT_DIR}/App/Src/cmd_dispatcher.c" \
  "${ROOT_DIR}/App/Src/switch_scan.c" \
  "${ROOT_DIR}/Hw/Src/hw_gpio_if.c" \
  "${ROOT_DIR}/tools/proto_check/main.c" \
  -o "${OUT_BIN}"

echo "[INFO] Running host checks..."
"${OUT_BIN}"

echo "[INFO] Host checks passed."
