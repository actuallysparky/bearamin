#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -x _local/venv/bin/python ]]; then
  python3 -m venv _local/venv
  _local/venv/bin/python -m pip install -r requirements.txt
fi
exec _local/venv/bin/python bridge.py "$@"
