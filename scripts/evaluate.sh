#!/usr/bin/env bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

usage() {
  cat <<'EOF'
Usage:
  scripts/evaluate.sh <reality|simulation> [options] [launch_arg:=value ...]

Options:
  --save                 Save CSV and JSON files (default: do not save)
  --no-save              Do not save files
  --output DIR           Result directory used with --save
  --namespace NS         Robot namespace
  --no-gui               Run without the live dashboard
  -h, --help             Show this help

Examples:
  scripts/evaluate.sh reality
  scripts/evaluate.sh reality --save --output output/evaluate/straight_line
  scripts/evaluate.sh simulation
  scripts/evaluate.sh simulation --save --output output/evaluate/pid

The live dashboard is enabled and recording is disabled by default. The
navigation stack must already be running. With --save, Ctrl-C writes the final
summary.json.
EOF
}

mode=${1:-}
if [ "$mode" = "-h" ] || [ "$mode" = "--help" ]; then
  usage
  exit 0
fi
if [ "$mode" != "reality" ] && [ "$mode" != "simulation" ]; then
  usage >&2
  exit 2
fi
shift

timestamp=$(date +%Y%m%d_%H%M%S)
output_dir=""
namespace=""
namespace_set=false
save_data=false
show_visualization=true
positionals=()
extra_launch_args=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --save)
      save_data=true
      shift
      ;;
    --no-save)
      save_data=false
      shift
      ;;
    --no-gui)
      show_visualization=false
      shift
      ;;
    --output)
      if [ "$#" -lt 2 ]; then
        echo "--output requires a directory." >&2
        exit 2
      fi
      output_dir=$2
      shift 2
      ;;
    --namespace)
      if [ "$#" -lt 2 ]; then
        echo "--namespace requires a value." >&2
        exit 2
      fi
      namespace=$2
      namespace_set=true
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *:=*)
      extra_launch_args+=("$1")
      shift
      ;;
    --*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      positionals+=("$1")
      shift
      ;;
  esac
done

# Keep the original positional output_dir/namespace form working.
if [ -z "$output_dir" ] && [ "${#positionals[@]}" -ge 1 ]; then
  output_dir=${positionals[0]}
fi
if [ "$namespace_set" = false ] && [ "${#positionals[@]}" -ge 2 ]; then
  namespace=${positionals[1]}
  namespace_set=true
fi
if [ "${#positionals[@]}" -gt 2 ]; then
  echo "Too many positional arguments." >&2
  usage >&2
  exit 2
fi

output_dir=${output_dir:-"$WS/output/evaluate/${mode}_${timestamp}"}

if [ "$mode" = "simulation" ]; then
  if [ "$namespace_set" = false ]; then
    namespace=red_standard_robot1
  fi
  use_sim_time=true
  use_ground_truth=true
else
  use_sim_time=false
  use_ground_truth=false
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Missing $WS/install/setup.bash; build the workspace first." >&2
  exit 1
fi
set +u
source "$WS/install/setup.bash"
set -u

if ! ros2 pkg prefix guga_evaluate >/dev/null 2>&1; then
  echo "guga_evaluate is not installed. Run:" >&2
  echo "  colcon build --symlink-install --packages-select guga_evaluate" >&2
  exit 1
fi

if [ "$save_data" = true ]; then
  mkdir -p "$output_dir"
  output_dir=$(readlink -f "$output_dir")
fi

echo "Starting guga evaluation"
echo "  mode:       $mode"
echo "  namespace:  ${namespace:-<root>}"
echo "  dashboard:  $show_visualization"
echo "  recording:  $save_data"
if [ "$save_data" = true ]; then
  echo "  output:     $output_dir"
fi

launch_args=(
  mode:="$mode"
  output_dir:="$output_dir"
  workspace:="$WS"
  use_sim_time:="$use_sim_time"
  use_ground_truth:="$use_ground_truth"
  save_data:="$save_data"
  show_visualization:="$show_visualization"
)
if [ -n "$namespace" ]; then
  launch_args+=(namespace:="$namespace")
fi

exec ros2 launch guga_evaluate evaluate.launch.py \
  "${launch_args[@]}" "${extra_launch_args[@]}"
