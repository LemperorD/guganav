#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MPC_PACKAGE_DIR="${REPO_ROOT}/src/guga_controller/mpc_controller"
PY_SIM_DIR="${MPC_PACKAGE_DIR}/test/py_sim"
GENERATED_ROOT="${ACADOS_GENERATED_ROOT:-${MPC_PACKAGE_DIR}/generated}"
GENERATED_OCP_DIR="${GENERATED_ROOT}/omni/omni_ocp"
GENERATED_SOLVER_FILES=(
  "${GENERATED_OCP_DIR}/acados_solver_omni.h"
  "${GENERATED_OCP_DIR}/acados_solver_omni.c"
)

set +u
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/env_humble.sh"
set -u

if ! python3 -c "import acados_template" >/dev/null 2>&1; then
  echo "Python package acados_template is not available." >&2
  echo "Run scripts/ci/install_deps_humble.sh with INSTALL_ACADOS_PYTHON=1." >&2
  exit 1
fi

export PYTHONPATH="${PY_SIM_DIR}:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
python3 "${PY_SIM_DIR}/c_codegen/c_codegen_omni.py"

for generated_file in "${GENERATED_SOLVER_FILES[@]}"; do
  if [ ! -f "${generated_file}" ]; then
    echo "acados solver generation did not create ${generated_file}" >&2
    exit 1
  fi
done

echo "Generated stable omni solver C API in ${GENERATED_OCP_DIR}"
