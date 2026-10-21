#!/usr/bin/env bash
set -euo pipefail
model_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
viewer_python="${PYTHON:-python3}"
exec "$viewer_python" -u "$model_dir/view_glove.py"
