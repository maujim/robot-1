#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v uv >/dev/null 2>&1; then
  echo "Error: uv is not installed." >&2
  echo "Install it first: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 1
fi

# If lerobot is tracked as a submodule, initialize/update it.
if git ls-files --stage lerobot 2>/dev/null | grep -q '^160000 '; then
  git submodule update --init --recursive lerobot
elif [ ! -d "lerobot/.git" ]; then
  git clone https://github.com/huggingface/lerobot.git lerobot
fi

if [ -d ".venv" ]; then
  echo "Using existing virtual environment at .venv"
else
  uv venv
fi

uv pip install --python .venv/bin/python -r requirements.txt
