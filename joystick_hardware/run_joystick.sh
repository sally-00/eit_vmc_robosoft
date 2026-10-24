#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$script_dir"
exec "$script_dir/../miniconda3/bin/python" -E "$script_dir/plot_joystick.py" "$@"
