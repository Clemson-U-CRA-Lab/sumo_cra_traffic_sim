#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

controller="${1:-PreviewNN}"
leading_vehicle_profile="${2:-Hwy}"
max_vehicle_count="${3:-300}"
vehicle_step="${4:-10}"
print_level="${5:-quiet}"

valid_controller=false
for candidate in MPC NN PreviewNN TerminalFCN IDM; do
    if [[ "$controller" == "$candidate" ]]; then
        valid_controller=true
        break
    fi
done

if [[ "$valid_controller" != true ]]; then
    echo "Unsupported controller: $controller"
    echo "Usage: ./runtime_analysis.sh [MPC|NN|PreviewNN|TerminalFCN|IDM] [Hwy|Nyc|Ftp|US06|FTPsec1|FTPsec2|FTPsec3] [max_vehicle_count] [vehicle_step] [quiet|info|debug]"
    exit 1
fi

if (( vehicle_step <= 0 )); then
    echo "vehicle_step must be positive."
    exit 1
fi

if (( max_vehicle_count < vehicle_step )); then
    echo "max_vehicle_count must be at least vehicle_step."
    exit 1
fi

echo "Runtime analysis configuration:"
echo "  Controller: $controller"
echo "  Leading profile: $leading_vehicle_profile"
echo "  Max vehicle count: $max_vehicle_count"
echo "  Vehicle step: $vehicle_step"
echo "  Print level: $print_level"

for ((veh_num=vehicle_step; veh_num<=max_vehicle_count; veh_num+=vehicle_step)); do
    echo "Running controller=$controller profile=$leading_vehicle_profile num_sv=$veh_num"
    python3 scripts/run_sim.py \
        --num_sv "$veh_num" \
        --print_level "$print_level" \
        "$leading_vehicle_profile" \
        "$controller"
done

echo "Runtime analysis complete."
