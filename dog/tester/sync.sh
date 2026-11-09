#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 || ( $# -eq 2 && "$2" != "--dry-run" ) ]]; then
    printf 'Usage: %s <host or user@host> [--dry-run]\n' "$0" >&2
    exit 1
fi

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
remote="$1"
if [[ "$remote" != *@* ]]; then
    remote="pi@$remote"
fi

options=(-av --delete --protect-args)
if [[ "${2:-}" == "--dry-run" ]]; then
    options+=(--dry-run)
fi

excludes=(
    --exclude='.venv/'
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='*.egg-info/'
    --exclude='build/'
    --exclude='logs/'
    --exclude='captures/'
    --exclude='.runner_history'
)

rsync "${options[@]}" "${excludes[@]}" \
    --include='/src/***' \
    --include='/pyproject.toml' \
    --filter='H *' \
    "$project_dir/" "$remote:/home/pi/robotrio_src/"

rsync "${options[@]}" "${excludes[@]}" \
    "$project_dir/ws/" "$remote:/home/pi/robotrio_ws/"
