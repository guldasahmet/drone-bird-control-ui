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

export PYTHONPATH="${PROJECT_DIR}/src:${PYTHONPATH:-}"
exec python "${PROJECT_DIR}/src/dual_camera_headless.py" "$@"
