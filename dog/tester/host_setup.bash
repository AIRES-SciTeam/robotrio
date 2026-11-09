#!/usr/bin/env bash
set -euo pipefail

# Создаёт окружение в папке запуска, зависимости берёт из проекта tester.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="$PWD/.venv"

python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 11) else "Python 3.11 or newer is required")'
python3 -m venv "$venv_dir"

"$venv_dir/bin/python" - "$project_dir" <<'PY'
import re
import subprocess
import sys
import tomllib
from pathlib import Path

project_dir = Path(sys.argv[1])
with (project_dir / "pyproject.toml").open("rb") as file:
    config = tomllib.load(file)

dependencies = []
camera_dependencies = []
for requirement in config["project"]["dependencies"]:
    name = re.match(r"[A-Za-z0-9_.-]+", requirement).group().lower()
    if name == "picamera2":
        camera_dependencies.append(requirement)
    else:
        dependencies.append(requirement)

def install(*requirements):
    subprocess.run(
        [sys.executable, "-m", "pip", "install", *requirements],
        check=True,
    )

install("--upgrade", "pip")
if dependencies:
    install(*dependencies)
if camera_dependencies:
    install("--no-deps", *camera_dependencies)

PY

printf '\nEnvironment ready: %s\n' "$venv_dir"
printf 'Activate it with: source %q/bin/activate\n' "$venv_dir"
