#!/usr/bin/env bash
set -e

WS="$HOME/omnetpp_simu5g_ws"

export PATH="$HOME/sumo122_venv/bin:$PATH"
export SUMO_HOME=/usr/share/sumo
export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

export LD_LIBRARY_PATH="$WS/inet-4.6.0/src:$WS/veins-5.3.1/src:$WS/veins-5.3.1/subprojects/veins_inet/src:$WS/simu5g-1.4.4/src:$LD_LIBRARY_PATH"

cd "$WS/simu5g-1.4.4/simulations/nr/cars"

opp_run -u Cmdenv \
  -n "$WS/simu5g-1.4.4/simulations:$WS/simu5g-1.4.4/emulation:$WS/simu5g-1.4.4/src:$WS/inet-4.6.0/src:$WS/veins-5.3.1/src/veins:$WS/veins-5.3.1/subprojects/veins_inet/src/veins_inet" \
  -l INET \
  -l veins \
  -l veins_inet \
  -l simu5g \
  -f omnetpp.ini \
  --debug-on-errors=false \
  --cmdenv-interactive=false \
  --sim-time-limit=5s \
  '--**.launchConfig=xmldoc("heterogeneous.launchd.xml")'
