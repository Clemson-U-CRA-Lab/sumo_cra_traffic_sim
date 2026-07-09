# DoS Attack Simulation README

The main config file is `networksim/veins_live_model/config/attack_dos.yaml`.
This YAML controls DoS enabled/disabled, fake-agent count, packet rate, packet size, attack timing, and simulation time.

The single-run script is `networksim/veins_live_model/scripts/run_v2x_controller_live.sh`.

The sweep script is `networksim/veins_live_model/scripts/run_dos_sweep.sh`.

Before running, start three terminals.

Terminal 1 starts Veins/SUMO:
source ~/sumo122_venv/bin/activate
export PATH="$HOME/sumo122_venv/bin:$PATH"
cd ~/omnetpp_simu5g_ws
~/omnetpp_simu5g_ws/veins-5.3.1/bin/veins_launchd -vv -c "$HOME/sumo122_venv/bin/sumo"

Terminal 3 starts the Python MPC server:
cd ~/projects/sumo_cra_traffic_sim
source .venv/bin/activate
python3 scripts/live_mpc_controller_server.py

Terminal 2 enters the Simu5G shell:
cd ~/omnetpp_simu5g_ws
opp_env shell simu5g-latest

Go to the repo once inside chell:
cd ~/projects/sumo_cra_traffic_sim
./networksim/veins_live_model/scripts/build_v2x_live_model.sh

To run one DoS scenario using the current YAML settings: `V2X_CONFIG=networksim/veins_live_model/config/attack_dos.yaml ./networksim/veins_live_model/scripts/run_v2x_controller_live.sh`.
A single run writes logs and `v2x_dos_timeseries.png` to `networksim/veins_live_model/simulations/v2x_controller_live/`.

To run the full sweep: `./networksim/veins_live_model/scripts/run_dos_sweep.sh`.
The sweep script automatically changes the fake-agent count, runs each scenario, generates the graph, and saves the graph.
Sweep graphs are saved to `networksim/veins_live_model/simulations/v2x_controller_live/sweep_results/`.

To change which attacker counts are tested, edit `AGENTS_LIST` inside `run_dos_sweep.sh`,

Use `0` for baseline/no DoS; any value above `0` enables DoS
Graph files are named by attacker count, such as `v2x_dos_timeseries_baseline.png`, `v2x_dos_timeseries_100.png`, and `v2x_dos_timeseries_800.png`.

One-way delay is calculated as `receive_time - send_time` using real V2X packets from `v2x_simu5g_delivered_packets.csv`.