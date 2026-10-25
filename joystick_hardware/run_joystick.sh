#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$script_dir"
exec /usr/bin/python3 -E "$script_dir/plot_joystick.py" "$@"
