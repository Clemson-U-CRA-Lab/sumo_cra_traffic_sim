#!/usr/bin/env bash
set -e

WS="$HOME/omnetpp_simu5g_ws"
PROJ="$HOME/projects/sumo_cra_traffic_sim"

export PATH="$HOME/sumo122_venv/bin:$PATH"
export SUMO_HOME=/usr/share/sumo
export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

export LD_LIBRARY_PATH="$PROJ/networksim/veins_live_model/src:$WS/inet-4.6.0/src:$WS/veins-5.3.1/src:$WS/veins-5.3.1/subprojects/veins_inet/src:$WS/simu5g-1.4.4/src:$LD_LIBRARY_PATH"

SCENARIO_DIR="$PROJ/networksim/veins_live_model/simulations/v2x_controller_live"

NED_PATH="$PROJ/networksim/veins_live_model/simulations:$WS/simu5g-1.4.4/emulation:$WS/simu5g-1.4.4/src:$WS/inet-4.6.0/src:$WS/veins-5.3.1/src/veins:$WS/veins-5.3.1/subprojects/veins_inet/src/veins_inet"

cd "$SCENARIO_DIR"

opp_run \
  -u Cmdenv \
  -n "$NED_PATH" \
  -l INET \
  -l veins \
  -l veins_inet \
  -l simu5g \
  -l v2x_live_model \
  -f omnetpp.ini \
  --debug-on-errors=false \
  --cmdenv-interactive=false \
  --sim-time-limit=30s \
  '--**.controllerBridgeEnabled=true' \
  '--**.controllerHost="127.0.0.1"' \
  '--**.controllerPort=5555' \
  '--**.commandLogFile="v2x_live_controller_commands.csv"' \
  '--**.useSimu5gDeliveredNv0=true' \
  '--**.networkDegradationEnabled=false' \
  '--**.networkBaseDelay=0s' \
  '--**.networkAttackDelay=0s' \
  '--**.networkAttackJitter=0s' \
  '--**.networkAttackDropProbability=0' \
  '--**.networkMessageLogFile="v2x_live_network_messages.csv"' \
  '--**.amcMode="D2D"' \
  '--*.car[*].cellularNic.d2dInitialMode=true' \
  '--*.gNodeB*.cellularNic.nrPhy.enableD2DCqiReporting=true' \
  '--**.usePreconfiguredTxParams=false' \
  '--*.car[*].numApps=1' \
  '--*.car[0].app[0].typename="V2XStatusSenderApp"' \
  '--*.car[0].app[0].destAddress="car[1]"' \
  '--*.car[0].app[0].localPort=4000' \
  '--*.car[0].app[0].destPort=4000' \
  '--*.car[0].app[0].sendInterval=0.1s' \
  '--*.car[0].app[0].startTime=0.5s' \
  '--*.car[0].app[0].packetSizeBytes=300' \
  '--*.car[1].app[0].typename="V2XStatusReceiverApp"' \
  '--*.car[1].app[0].localPort=4000' \
  '--*.car[1].app[0].packetLogFile="v2x_simu5g_delivered_packets.csv"'
