#!/usr/bin/env bash
set -euo pipefail

# It reads one YAML profile, generates an OMNeT++ ini fragment, then runs OMNeT++.

# Directory containing this runner script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Roots of the project and OMNeT++ /Simu5G workspace
PROJ="${SUMO_CRA_TRAFFIC_SIM:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
WS="${OMNETPP_SIMU5G_WS:-$HOME/omnetpp_simu5g_ws}"

# SUMO 1.22 virtual environment path
SUMO122_VENV="${SUMO122_VENV:-$HOME/sumo122_venv}"

# YAML config file 
V2X_CONFIG="${V2X_CONFIG:-networksim/veins_live_model/config/attack_dos.yaml}"

# OMNeT++ scenario directory, where omnetpp.ini lives and runtime CSV outputs are written too
SCENARIO_DIR="$PROJ/networksim/veins_live_model/simulations/v2x_controller_live"

# Genereated .ini path file
GENERATED_INI="$SCENARIO_DIR/generated/live_generated.ini"

# NED path tells OMNeT++ where to find all module/network definitions
NED_PATH="$PROJ/networksim/veins_live_model/simulations:$WS/simu5g-1.4.4/emulation:$WS/simu5g-1.4.4/src:$WS/inet-4.6.0/src:$WS/veins-5.3.1/src/veins:$WS/veins-5.3.1/subprojects/veins_inet/src/veins_inet"

export PATH="$SUMO122_VENV/bin:$PATH"
export SUMO_HOME="${SUMO_HOME:-/usr/share/sumo}"
export PYTHONPATH="$SUMO_HOME/tools:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PROJ/networksim/veins_live_model/src:$WS/inet-4.6.0/src:$WS/veins-5.3.1/src:$WS/veins-5.3.1/subprojects/veins_inet/src:$WS/simu5g-1.4.4/src:${LD_LIBRARY_PATH:-}"

# Use repo venv Python if it exists.
if [[ -x "$PROJ/.venv/bin/python3" ]]; then
  PYTHON_BIN="$PROJ/.venv/bin/python3"
else
  PYTHON_BIN="python3"
fi

# check if in OMNeT++/Simu5G environment
if ! command -v opp_run >/dev/null 2>&1; then
  echo "ERROR: opp_run not found."
  echo "Run this script from inside the Simu5G OMNeT++ shell:"
  echo ""
  echo "  cd $WS"
  echo "  opp_env shell simu5g-latest"
  echo "  cd $PROJ"
  echo "  V2X_CONFIG=$V2X_CONFIG ./networksim/veins_live_model/scripts/run_v2x_controller_live.sh"
  exit 1
fi

# Move to project root before generating the config
cd "$PROJ"
echo "Generating OMNeT++ config from YAML:"
echo "  $V2X_CONFIG"

# Genereates .ini from YAML config file
V2X_CONFIG="$V2X_CONFIG" "$PYTHON_BIN" "$SCRIPT_DIR/generate_v2x_ini.py"

# Move into the OMNeT++ scenario directory before running opp_run
cd "$SCENARIO_DIR"
echo "Running OMNeT++:"
echo "  Base ini:      $SCENARIO_DIR/omnetpp.ini"
echo "  Generated ini: $GENERATED_INI"
echo "  Config:        General"

# Run the OMNeT++ simulation
opp_run \
  -u Cmdenv \
  -n "$NED_PATH" \
  -l INET \
  -l veins \
  -l veins_inet \
  -l simu5g \
  -l v2x_live_model \
  -f omnetpp.ini \
  -f "$GENERATED_INI" \
  --debug-on-errors=false \
  --cmdenv-interactive=false