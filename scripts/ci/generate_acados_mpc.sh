#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MPC_PACKAGE_DIR="${REPO_ROOT}/src/guga_controller/mpc_controller"
PY_SIM_DIR="${MPC_PACKAGE_DIR}/test/py_sim"
GENERATED_SOLVER_HEADER="${MPC_PACKAGE_DIR}/generated/omni/omni_ocp/acados_solver_omni.h"

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

if [ ! -f "${GENERATED_SOLVER_HEADER}" ]; then
  echo "acados solver generation did not create ${GENERATED_SOLVER_HEADER}" >&2
  exit 1
fi

echo "Generated ${GENERATED_SOLVER_HEADER}"
