#!/usr/bin/env bash
set -e

WS="$HOME/omnetpp_simu5g_ws"
PROJ="$HOME/projects/sumo_cra_traffic_sim"

cd "$PROJ/networksim/veins_live_model/src"

opp_makemake -f --deep --make-so -o v2x_live_model \
  -I"$WS/inet-4.6.0/src" \
  -I"$WS/veins-5.3.1/src" \
  -I"$WS/veins-5.3.1/subprojects/veins_inet/src" \
  -I"$WS/simu5g-1.4.4/src" \
  -L"$WS/inet-4.6.0/src" \
  -L"$WS/veins-5.3.1/src" \
  -L"$WS/veins-5.3.1/subprojects/veins_inet/src" \
  -L"$WS/simu5g-1.4.4/src" \
  -lINET \
  -lveins \
  -lveins_inet \
  -lsimu5g

make -j"$(nproc)"
