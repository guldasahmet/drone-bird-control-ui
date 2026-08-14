#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAILO_APPS_DIR="${PROJECT_DIR}/../hailo-apps"

if [[ ! -f "${HAILO_APPS_DIR}/setup_env.sh" ]]; then
    echo "Hata: Hailo Apps ortamı bulunamadı: ${HAILO_APPS_DIR}" >&2
    exit 1
fi

cd "${HAILO_APPS_DIR}"
source setup_env.sh >/dev/null
set -u
OVERLAY_SOURCE="${PROJECT_DIR}/native/target_overlay.cpp"
OVERLAY_PLUGIN="${PROJECT_DIR}/native/build/libgstbdtargetoverlay.so"
if [[ ! -f "${OVERLAY_PLUGIN}" || "${OVERLAY_SOURCE}" -nt "${OVERLAY_PLUGIN}" ]]; then
    "${PROJECT_DIR}/native/build.sh"
fi
export GST_PLUGIN_PATH="${PROJECT_DIR}/native/build:${GST_PLUGIN_PATH:-}"
export PYTHONPATH="${PROJECT_DIR}/src:${PYTHONPATH:-}"
exec python "${PROJECT_DIR}/src/app.py" "$@"
