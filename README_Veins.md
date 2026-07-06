1. Known-Good Environment

This setup has been tested with:

OMNeT++: 6.4.0
INET: 4.6.0
Simu5G: 1.4.4
Veins: 5.3.1
SUMO: 1.22.0
Python: 3.x

Expected folder layout:

~/projects/sumo_cra_traffic_sim
~/omnetpp_simu5g_ws
~/sumo122_venv

The run scripts assume this layout. If your paths are different, update the path variables inside:

networksim/veins_live_model/scripts/run_v2x_controller_live_base.sh
networksim/veins_live_model/scripts/run_v2x_controller_live_stress.sh
networksim/veins_live_model/scripts/build_v2x_live_model.sh

2. System-Level Prerequisites

On Ubuntu / WSL, install basic build and Python tools:

sudo apt update

sudo apt install -y \
  build-essential \
  clang \
  gdb \
  make \
  cmake \
  git \
  python3 \
  python3-pip \
  python3-venv \
  python3-dev \
  libxml2-dev \
  zlib1g-dev \
  default-jre \
  default-jdk

Optional but useful:

sudo apt install -y xdg-utils

3. Install SUMO 1.22.0

The current model was validated using SUMO 1.22.0.

Create a dedicated SUMO virtual environment:

python3 -m venv ~/sumo122_venv
source ~/sumo122_venv/bin/activate
python3 -m pip install --upgrade pip

Install SUMO 1.22 Python package and tools:

pip install eclipse-sumo==1.22.0
pip install traci==1.22.0
pip install sumolib==1.22.0

Verify the SUMO binary:

which sumo
sumo --version

Expected binary path:

/home/<user>/sumo122_venv/bin/sumo

Expected version:

Eclipse SUMO sumo Version 1.22.0

For this project, each run should use:

source ~/sumo122_venv/bin/activate
export PATH="$HOME/sumo122_venv/bin:$PATH"

4. Install OMNeT++ / INET / Simu5G / Veins

This project was run inside the workspace:

~/omnetpp_simu5g_ws

The working shell is entered with:

cd ~/omnetpp_simu5g_ws
opp_env shell simu5g-latest

The prompt should look like:

omnetpp-6.4.0+inet-4.6.0+simu5g-1.4.4

The workspace should contain:

~/omnetpp_simu5g_ws/inet-4.6.0
~/omnetpp_simu5g_ws/simu5g-1.4.4
~/omnetpp_simu5g_ws/veins-5.3.1

Quick checks:

ls ~/omnetpp_simu5g_ws
ls ~/omnetpp_simu5g_ws/inet-4.6.0
ls ~/omnetpp_simu5g_ws/simu5g-1.4.4
ls ~/omnetpp_simu5g_ws/veins-5.3.1

5. Project Python Environment

From the repo root:

cd ~/projects/sumo_cra_traffic_sim

python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip

Install the Python packages needed by the MPC controller and plotting scripts. The exact set depends on the original project, but at minimum:

pip install numpy pandas matplotlib scipy casadi

If the original project has a requirements.txt, use:

pip install -r requirements.txt

The Python MPC server is:

scripts/live_mpc_controller_server.py

The adapter around the original controller logic is:

scripts/_live_controller_adapter.py

6. Important Project Files
OMNeT++ / Simu5G scenario
networksim/veins_live_model/simulations/v2x_controller_live/

Main files:

Highway.ned
V2XLiveStateLogger.ned
V2XStatusApps.ned
V2XNoiseApps.ned
omnetpp.ini
C++ OMNeT++ modules
networksim/veins_live_model/src/

Main files:

V2XLiveStateLogger.cc
V2XStatusApps.cc
V2XStatusRegistry.h
V2XNoiseApps.cc
Scripts
networksim/veins_live_model/scripts/

Main scripts:

build_v2x_live_model.sh
run_v2x_controller_live_base.sh
run_v2x_controller_live_stress.sh
compare_v2x_live_runs.py
plot_v2x_controller_comparison.py
SUMO scenario

The live Veins launch config points to the two-vehicle V2X SUMO scenario:

sumo/v2x/v2x_2veh.sumocfg
sumo/v2x/map_2veh.rou.xml
sumo/v2x/map_highSpd.net.xml

7. Network Model Used

The current V2X network model is:

UDP application packets
over Simu5G NR D2D / sidelink-style UE-to-UE communication
with INET packet handling
and Veins/SUMO mobility

Baseline case:

car[0] / nv0
→ V2XStatusSenderApp
→ UDP packet over Simu5G D2D path
→ V2XStatusReceiverApp on car[1] / nv1
→ latest delivered nv0 packet used by MPC

Stress case:

same V2X status traffic
+
high-rate UDP noise traffic
+
lower D2D CQI settings

This creates real packet delay and backlog inside the OMNeT++ / INET / Simu5G simulation rather than manually injecting delay into Python.

Conservative wording for reports:

The model uses UDP V2X status packets over Simu5G NR D2D / sidelink-style UE-to-UE communication.


8. Simulation Runtime

The original SUMO-only scenario runs for about 80 seconds.

The live OMNeT++ run scripts should therefore use:

--sim-time-limit=80s

Check this with:

grep -n "sim-time-limit" \
  networksim/veins_live_model/scripts/run_v2x_controller_live_base.sh \
  networksim/veins_live_model/scripts/run_v2x_controller_live_stress.sh

Expected:

--sim-time-limit=80s

If needed, patch both scripts:

cd ~/projects/sumo_cra_traffic_sim

python3 - <<'PY'
from pathlib import Path

scripts = [
    Path("networksim/veins_live_model/scripts/run_v2x_controller_live_base.sh"),
    Path("networksim/veins_live_model/scripts/run_v2x_controller_live_stress.sh"),
]

for path in scripts:
    text = path.read_text()
    text = text.replace("--sim-time-limit=30s", "--sim-time-limit=80s")
    path.write_text(text)
    print(f"Updated {path} to 80s")
PY

9. Three-Terminal Run Procedure

Use three terminals.

Run them in this order:

Terminal 1: Veins / SUMO bridge
Terminal 3: Python MPC controller server
Terminal 2: OMNeT++ / INET / Simu5G simulation

Terminal 1 — Start Veins / SUMO Bridge
source ~/sumo122_venv/bin/activate
export PATH="$HOME/sumo122_venv/bin:$PATH"

cd ~/omnetpp_simu5g_ws

~/omnetpp_simu5g_ws/veins-5.3.1/bin/veins_launchd -vv -c "$HOME/sumo122_venv/bin/sumo"

What this does:

- Activates SUMO 1.22.0.
- Starts the Veins launch daemon.
- Waits for OMNeT++ to connect.
- Launches SUMO when the OMNeT++ simulation starts.

Leave this terminal running.

If port 9999 is stuck:

pkill -f veins_launchd || true
pkill -f sumo || true
pkill -f sumo-gui || true
ss -ltnp | grep 9999 || echo "port 9999 is free"

Terminal 3 — Start Python MPC Server
cd ~/projects/sumo_cra_traffic_sim
source .venv/bin/activate

python3 scripts/live_mpc_controller_server.py

What this does:

- Starts a local Python TCP server.
- Receives nv0/nv1 state from the OMNeT++ C++ bridge.
- Calls the MPC controller logic.
- Returns the nv1 acceleration command.

Leave this terminal running.

Terminal 2 — Enter Simu5G Shell
cd ~/omnetpp_simu5g_ws
opp_env shell simu5g-latest

Then:

cd ~/projects/sumo_cra_traffic_sim

What this does:

- Enters the correct OMNeT++ / INET / Simu5G environment.
- Makes opp_run and the required libraries available.

10. Build Custom OMNeT++ Modules

Run this in Terminal 2:

cd ~/projects/sumo_cra_traffic_sim

./networksim/veins_live_model/scripts/build_v2x_live_model.sh

What this does:

- Rebuilds the custom OMNeT++ shared library.
- Compiles V2XLiveStateLogger.cc.
- Compiles V2XStatusApps.cc.
- Compiles V2XNoiseApps.cc.
- Produces libv2x_live_model.so.

Rebuild whenever any C++ or NED file changes.

11. Run Baseline Scenario

In Terminal 2:

cd ~/projects/sumo_cra_traffic_sim

rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_vehicle_states.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_controller_commands.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_delivered_packets.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_noise_packets.csv

./networksim/veins_live_model/scripts/run_v2x_controller_live_base.sh

What this does:

- Runs the clean baseline case.
- Uses real Simu5G-delivered nv0 packets.
- Does not add noise traffic.
- MPC controls nv1 through the live OMNeT++ / Veins / SUMO loop.

Save baseline outputs:

mkdir -p networksim/veins_live_model/results/baseline

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_vehicle_states.csv \
   networksim/veins_live_model/results/baseline/

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_controller_commands.csv \
   networksim/veins_live_model/results/baseline/

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_delivered_packets.csv \
   networksim/veins_live_model/results/baseline/

12. Run Stress Scenario

In Terminal 2:

cd ~/projects/sumo_cra_traffic_sim

rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_vehicle_states.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_controller_commands.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_delivered_packets.csv
rm -f networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_noise_packets.csv

./networksim/veins_live_model/scripts/run_v2x_controller_live_stress.sh

What this does:

- Runs the same vehicle and MPC scenario as baseline.
- Keeps nv0 → nv1 V2X status packets active.
- Adds high-rate UDP noise traffic over the same Simu5G D2D path.
- Uses worse D2D / CQI settings.
- Causes real packet backlog and delay in Simu5G / INET.

Save stress outputs:

mkdir -p networksim/veins_live_model/results/stress

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_vehicle_states.csv \
   networksim/veins_live_model/results/stress/

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_controller_commands.csv \
   networksim/veins_live_model/results/stress/

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_delivered_packets.csv \
   networksim/veins_live_model/results/stress/

cp networksim/veins_live_model/simulations/v2x_controller_live/v2x_simu5g_noise_packets.csv \
   networksim/veins_live_model/results/stress/

13. Verify Run Length

After running baseline or stress, check that the controller log reaches close to 80 seconds:

tail -5 networksim/veins_live_model/simulations/v2x_controller_live/v2x_live_controller_commands.csv

The first column is sim_time. It should approach 80

14. Compare Baseline vs Stress

Run:

python3 networksim/veins_live_model/scripts/compare_v2x_live_runs.py

This writes:

networksim/veins_live_model/results/baseline_vs_stress_summary.csv

View the summary:

cat networksim/veins_live_model/results/baseline_vs_stress_summary.csv

The comparison includes:

controller rows
Simu5G packet rows
direct fallback rows
controller_ok rows
actuation_ok rows
average nv0 information age
maximum nv0 information age
delivered status packets
average status packet delay
maximum status packet delay
noise packet statistics

Expected qualitative result:

Baseline:
- low packet delay
- low nv0 information age
- no noise packets

Stress:
- higher packet delay
- fewer delivered status packets
- higher nv0 information age
- many delivered noise packets

15. Generate Presentation Plots

Run:

python3 networksim/veins_live_model/scripts/plot_v2x_controller_comparison.py \
  --make-both \
  --xmin 0 \
  --xmax 80

This creates:

networksim/veins_live_model/results/controller_vehicle_baseline_vs_stress_perceived_gap.png
networksim/veins_live_model/results/controller_vehicle_baseline_vs_stress_perceived_gap.pdf

networksim/veins_live_model/results/controller_vehicle_baseline_vs_stress_true_gap.png
networksim/veins_live_model/results/controller_vehicle_baseline_vs_stress_true_gap.pdf

Open the main plot:

xdg-open networksim/veins_live_model/results/controller_vehicle_baseline_vs_stress_perceived_gap.png

Presentation interpretation:

The perceived-gap plot shows what the controller believes based on the latest delivered nv0 packet.
Under stress, stale packets cause the controller's perceived gap to degrade.
The true-gap plot shows the physical gap between vehicles.

16. Expected Logs

The main output CSVs are:

v2x_live_vehicle_states.csv
v2x_live_controller_commands.csv
v2x_simu5g_delivered_packets.csv
v2x_simu5g_noise_packets.csv

Important columns in v2x_live_controller_commands.csv:

sim_time
nv0_source
direct_nv0_x
controller_nv0_x
nv0_info_age
nv1_x
nv1_speed
controller_ok
acc_cmd_nv1
actuation_ok
target_speed_nv1

Important columns in v2x_simu5g_delivered_packets.csv:

receive_time
seq
send_time
delay
x
y
speed
accel

Important columns in v2x_simu5g_noise_packets.csv:

receive_time
seq
send_time
delay

