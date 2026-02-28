#!/usr/bin/env bash
set -euo pipefail

# One-time environment bootstrap for NeuPAN python deps.
# - Creates/updates ./neupan_env
# - Installs python deps from src/neupan_nav2_controller/requirements.txt
#
# Usage:
#   ./scripts/setup_neupan_env.sh
#   PYTHON_BIN=python3 ./scripts/setup_neupan_env.sh
#   VENV_DIR=./neupan_env ./scripts/setup_neupan_env.sh

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$WS_DIR/neupan_env}"
REQ_FILE="${REQ_FILE:-$WS_DIR/src/neupan_nav2_controller/requirements.txt}"
EXTRA_PKGS="${EXTRA_PKGS:-}"

# Defaults tuned for ROS2 workflows:
# - VENV_SYSTEM_SITE_PACKAGES=1: venv can import system ROS python pkgs (rclpy, launch, etc)
# - NEUPAN_EXTRAS=1: install common non-ROS deps often used by NeuPAN tooling
VENV_SYSTEM_SITE_PACKAGES="${VENV_SYSTEM_SITE_PACKAGES:-1}"
NEUPAN_EXTRAS="${NEUPAN_EXTRAS:-1}"
RECREATE_VENV="${RECREATE_VENV:-0}"

# ECOS handling:
# - If ECOS_WHEEL is set to a local wheel path, we force-reinstall it into the venv.
# - If PATCH_ECOS=1, we ensure ecos/ecos.py contains the sparse-array compatibility patch.
ECOS_WHEEL="${ECOS_WHEEL:-}"
PATCH_ECOS="${PATCH_ECOS:-1}"

if [[ ! -f "$REQ_FILE" ]]; then
  echo "[ERROR] requirements.txt not found: $REQ_FILE" >&2
  exit 1
fi

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "[ERROR] PYTHON_BIN not found: $PYTHON_BIN" >&2
  exit 1
fi

if [[ "$RECREATE_VENV" == "1" && -d "$VENV_DIR" ]]; then
  echo "[WARN] RECREATE_VENV=1 set; removing existing venv: $VENV_DIR"
  rm -rf "$VENV_DIR"
fi

if [[ ! -d "$VENV_DIR" ]]; then
  echo "[INFO] Creating venv: $VENV_DIR"
  if [[ "$VENV_SYSTEM_SITE_PACKAGES" == "1" ]]; then
    echo "[INFO] Venv uses system site-packages (for ROS python pkgs)"
    "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
  else
    "$PYTHON_BIN" -m venv "$VENV_DIR"
  fi
fi

# Ensure colcon ignores the virtualenv directory so builds skip it
COLCON_IGNORE_FILE="$VENV_DIR/COLCON_IGNORE"
if [[ ! -f "$COLCON_IGNORE_FILE" ]]; then
  echo "[INFO] Creating COLCON_IGNORE in $VENV_DIR"
  : > "$COLCON_IGNORE_FILE"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

echo "[INFO] Upgrading pip tooling"
# 注意：ROS/colcon 依赖对 setuptools 版本有上限（例如 colcon-core 要求 < 80）。
# 这里显式限制 setuptools，避免升级到过新版本导致依赖冲突。
python -m pip install -q --upgrade pip wheel "setuptools<80"

echo "[INFO] Installing NeuPAN python requirements"
# torch==...+cpu typically needs the CPU wheel index.
python -m pip install -r "$REQ_FILE" --extra-index-url https://download.pytorch.org/whl/cpu

if [[ -n "$ECOS_WHEEL" ]]; then
  if [[ ! -f "$ECOS_WHEEL" ]]; then
  echo "[ERROR] ECOS_WHEEL not found: $ECOS_WHEEL" >&2
  exit 1
  fi
  echo "[INFO] Forcing ECOS install from wheel: $ECOS_WHEEL"
  python -m pip install -q --force-reinstall --no-deps "$ECOS_WHEEL"
fi

if [[ "$PATCH_ECOS" == "1" ]]; then
  echo "[INFO] Ensuring ECOS sparse-array compatibility patch is applied"
  python - <<'PY'
import importlib
import os

try:
  ecos_mod = importlib.import_module('ecos.ecos')
except Exception as e:
  raise SystemExit(f"[ERROR] cannot import ecos.ecos for patching: {e!r}")

path = getattr(ecos_mod, '__file__', None)
if not path or not os.path.isfile(path):
  raise SystemExit(f"[ERROR] cannot locate ecos.ecos file: {path!r}")

with open(path, 'r', encoding='utf-8') as f:
  src = f.read()

marker = "SciPy >= 1.15 may hand us sparse *arrays*"
if marker in src:
  print(f"[INFO] ECOS patch already present: {path}")
  raise SystemExit(0)

# Best-effort patch: insert sparse-array -> spmatrix conversion after issparse checks.
needle = "if A is not None and not sparse.issparse(A):\n        raise TypeError(\"A is required to be a sparse matrix\")\n"
insert = needle + "\n    # SciPy >= 1.15 may hand us sparse *arrays* (e.g. csc_array), which are\n    # not subclasses of spmatrix and do not provide get_shape(). Convert them\n    # explicitly to CSC spmatrix for compatibility with the underlying wrapper.\n    if G is not None and not sparse.isspmatrix(G):\n        warn(\"Converting G sparse array to a CSC matrix; may take a while.\")\n        G = sparse.csc_matrix(G)\n    if A is not None and not sparse.isspmatrix(A):\n        warn(\"Converting A sparse array to a CSC matrix; may take a while.\")\n        A = sparse.csc_matrix(A)\n"

if needle not in src:
  raise SystemExit(f"[WARN] ECOS patch skipped (unexpected ecos.py layout): {path}")

src2 = src.replace(needle, insert)
with open(path, 'w', encoding='utf-8') as f:
  f.write(src2)

print(f"[INFO] ECOS patch applied: {path}")
PY
fi

if [[ "$NEUPAN_EXTRAS" == "1" ]]; then
  echo "[INFO] Installing common NeuPAN extras (matplotlib/pillow/scikit-learn)"
  python -m pip install -q matplotlib pillow scikit-learn
else
  echo "[INFO] Skipping common NeuPAN extras (NEUPAN_EXTRAS=$NEUPAN_EXTRAS)"
fi

if [[ -n "$EXTRA_PKGS" ]]; then
  echo "[INFO] Installing extra python packages: $EXTRA_PKGS"
  # shellcheck disable=SC2086
  python -m pip install $EXTRA_PKGS
fi

# Ensure ROS/colcon-related python packages available in the venv
INSTALL_ROS_PY_PKGS="${INSTALL_ROS_PY_PKGS:-1}"
if [[ "$INSTALL_ROS_PY_PKGS" == "1" ]]; then
  echo "[INFO] Installing ROS/colcon python packages (catkin_pkg)"
  python -m pip install -q catkin_pkg || true
else
  echo "[INFO] Skipping ROS/colcon python packages (INSTALL_ROS_PY_PKGS=$INSTALL_ROS_PY_PKGS)"
fi

echo "[INFO] Verifying imports & versions"
python - <<'PY'
import sys
from importlib import metadata as importlib_metadata
import os

def version_of(module_name: str) -> str:
    try:
        m = __import__(module_name)
        v = getattr(m, '__version__', None)
        if v:
            return str(v)
    except Exception:
        pass
    try:
        return importlib_metadata.version(module_name)
    except Exception:
        return '(unknown)'

pkgs = [
  'numpy','scipy','torch','yaml','cvxpy','cvxpylayers','diffcp','ecos',
  'colorama','rich','gctl'
]

if os.environ.get('NEUPAN_EXTRAS', '1') == '1':
  pkgs.extend(['matplotlib', 'PIL', 'sklearn'])

print('python', sys.version)
for p in pkgs:
  try:
    m = __import__(p)
    print(f'{p} {version_of(p)}')
    if os.environ.get('SHOW_PY_PATHS', '0') == '1':
      path = getattr(m, '__file__', None)
      if path:
        print(f'  -> {path}')
  except Exception as e:
    print(f'{p} import failed:', repr(e))
    raise

try:
    __import__('catkin_pkg')
    print(f"catkin_pkg {version_of('catkin_pkg')}")
except Exception as e:
    print('[WARN] catkin_pkg not available:', repr(e))

print('OK')
PY

echo "[INFO] Done. Next steps:" \
  && echo "  - source $WS_DIR/install/setup.bash" \
  && echo "  - source $VENV_DIR/bin/activate" \
  && echo "  - (tip) activate venv last so python3 points to venv" \
  && echo "  - (optional) run ./scripts/complete_build.sh or ./scripts/neupan_build.sh"
#严格按照/src下放neupan插件    /scripts下放脚本