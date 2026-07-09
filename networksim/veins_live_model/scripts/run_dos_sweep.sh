#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$HOME/projects/sumo_cra_traffic_sim"
cd "$REPO_ROOT"

CONFIG="networksim/veins_live_model/config/attack_dos.yaml"
SCENARIO_DIR="networksim/veins_live_model/simulations/v2x_controller_live"
SWEEP_DIR="$SCENARIO_DIR/sweep_results"

mkdir -p "$SWEEP_DIR"

# 0 = baseline, no DoS.
# 100-800 = DoS enabled with that many fake agents.
AGENTS_LIST=(0 100 200 300 400 500 600 700 800)

update_yaml() {
    local agents="$1"

    python3 - "$CONFIG" "$agents" <<'PY'
from pathlib import Path
import sys

config_path = Path(sys.argv[1])
agents = int(sys.argv[2])

lines = config_path.read_text().splitlines()
out = []

in_dos_block = False

for line in lines:
    stripped = line.strip()

    if stripped == "dos_attack:":
        in_dos_block = True
        out.append(line)
        continue

    # End dos_attack block when the next top-level section starts.
    if in_dos_block and line and not line.startswith(" ") and stripped.endswith(":"):
        in_dos_block = False

    if in_dos_block:
        if stripped.startswith("enabled:"):
            if agents == 0:
                out.append("  enabled: false")
            else:
                out.append("  enabled: true")
            continue

        if stripped.startswith("fake_agent_count:"):
            out.append(f"  fake_agent_count: {agents}")
            continue

        if stripped.startswith("packets_per_agent_per_second:"):
            out.append("  packets_per_agent_per_second: 10")
            continue

    out.append(line)

config_path.write_text("\n".join(out) + "\n")

if agents == 0:
    print(f"Updated {config_path}: baseline run, dos_attack.enabled=false")
else:
    print(f"Updated {config_path}: dos_attack.enabled=true, fake_agent_count={agents}, packets_per_agent_per_second=10")
PY
}

for AGENTS in "${AGENTS_LIST[@]}"; do
    echo "============================================================"

    if [ "$AGENTS" -eq 0 ]; then
        RUN_LABEL="baseline"
        echo "Running baseline sweep run with DoS disabled"
    else
        RUN_LABEL="$AGENTS"
        echo "Running DoS sweep with fake_agent_count = $AGENTS"
    fi

    echo "============================================================"

    update_yaml "$AGENTS"

    # Clean old per-run outputs so old CSVs do not contaminate the next graph.
    # The simulation will recreate the CSVs for the current run.
    # We do NOT save/copy these CSVs into sweep_results.
    rm -f "$SCENARIO_DIR/v2x_simu5g_delivered_packets.csv"
    rm -f "$SCENARIO_DIR/v2x_dos_packets.csv"
    rm -f "$SCENARIO_DIR/v2x_live_controller_commands.csv"
    rm -f "$SCENARIO_DIR/v2x_live_vehicle_states.csv"
    rm -f "$SCENARIO_DIR/v2x_live_network_messages.csv"
    rm -f "$SCENARIO_DIR/v2x_dos_timeseries.png"

    # Run one OMNeT++/Simu5G scenario.
    V2X_CONFIG="$CONFIG" ./networksim/veins_live_model/scripts/run_v2x_controller_live.sh

    # Generate plot from the current run's CSVs in SCENARIO_DIR.
    python3 networksim/veins_live_model/scripts/plot_v2x_dos_timeseries.py

    # Save only the graph, renamed by attack count.
    OUT_GRAPH="$SWEEP_DIR/v2x_dos_timeseries_${RUN_LABEL}.png"
    cp "$SCENARIO_DIR/v2x_dos_timeseries.png" "$OUT_GRAPH"

    echo "Saved graph:"
    echo "$OUT_GRAPH"
done

echo "============================================================"
echo "DoS sweep complete."
echo "Graphs saved under:"
echo "$SWEEP_DIR"
echo "============================================================"