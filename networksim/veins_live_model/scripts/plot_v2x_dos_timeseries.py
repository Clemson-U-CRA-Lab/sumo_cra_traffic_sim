from pathlib import Path
import csv
import math
import sys
import matplotlib.pyplot as plt

# Re-running this script overwrites the old plot instead of creating a new one.

REPO_ROOT = Path(__file__).resolve().parents[3]

SCENARIO_DIR = REPO_ROOT / "networksim/veins_live_model/simulations/v2x_controller_live"

# Allow plotting from another run folder if passed as an argument.
if len(sys.argv) > 1:
    SCENARIO_DIR = Path(sys.argv[1]).resolve()

STATUS_FILE = SCENARIO_DIR / "v2x_simu5g_delivered_packets.csv"
DOS_FILE = SCENARIO_DIR / "v2x_dos_packets.csv"
OUT_FILE = SCENARIO_DIR / "v2x_dos_timeseries.png"

if not STATUS_FILE.exists():
    raise FileNotFoundError(f"Missing status packet file: {STATUS_FILE}")

dos_enabled = DOS_FILE.exists()

# Graphs 1 and 2: real V2X status packets only
status_send_times = []
status_one_way_delays = []
status_estimated_rtts = []

# Graph 3: total packet send rate, real V2X + DoS packets
all_send_times = []

with STATUS_FILE.open() as f:
    reader = csv.DictReader(f)

    for row in reader:
        send_time = float(row["send_time"])
        receive_time = float(row["receive_time"])

        # Calculate delay
        one_way_delay = receive_time - send_time

        # Current system does not use ACK/echo packets yet, so this is estimated RTT, not true measured RTT.
        estimated_rtt = 2.0 * one_way_delay

        # Use send_time on x-axis so all graphs align by packet generation time.
        status_send_times.append(send_time)
        status_one_way_delays.append(one_way_delay)
        status_estimated_rtts.append(estimated_rtt)

        # Graph 3 includes real V2X packets in total channel load.
        all_send_times.append(send_time)

# DoS packets

dos_packet_count = 0

if DOS_FILE.exists():
    with DOS_FILE.open() as f:
        reader = csv.DictReader(f)

        for row in reader:
            send_time = float(row["send_time"])
            all_send_times.append(send_time)
            dos_packet_count += 1

# Graph 3: total packet send rate per second
packet_counts = {}

for t in all_send_times:
    second = int(math.floor(t))
    packet_counts[second] = packet_counts.get(second, 0) + 1

packet_seconds = sorted(packet_counts.keys())
packets_per_sec = [packet_counts[second] for second in packet_seconds]

# Make the step plot hold its last value through the final bin.
if packet_seconds:
    step_x = packet_seconds + [packet_seconds[-1] + 1]
    step_y = packets_per_sec + [packets_per_sec[-1]]
else:
    step_x = []
    step_y = []

# Common x-axis range
all_time_candidates = []

if status_send_times:
    all_time_candidates.extend(status_send_times)

if step_x:
    all_time_candidates.extend(step_x)

if not all_time_candidates:
    raise RuntimeError("No packet timing data found to plot.")

xmin = min(all_time_candidates)
xmax = max(all_time_candidates)

#Plotting
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

if dos_enabled:
    fig.suptitle("V2X DoS Attack Time-Series Analysis", fontsize=14)
else:
    fig.suptitle("V2X Baseline Time-Series Analysis", fontsize=14)

# Graph 1: estimated RTT for real V2X status packets only.
axes[0].plot(
    status_send_times,
    status_estimated_rtts,
    marker="o",
    markersize=2,
    linewidth=0.8,
    label="Estimated RTT, real V2X only",
)
axes[0].set_title("Estimated RTT over time")
axes[0].set_ylabel("Estimated RTT (s)")
axes[0].grid(True)
axes[0].legend()

# Graph 2: one-way delay for real V2X status packets only.
axes[1].plot(
    status_send_times,
    status_one_way_delays,
    marker="o",
    markersize=2,
    linewidth=0.8,
    label="One-way delay, real V2X only",
)
axes[1].set_title("Real V2X one-way packet delay over time")
axes[1].set_ylabel("Delay (s)")
axes[1].grid(True)
axes[1].legend()

# Graph 3: total packet send rate, real V2X + DoS packets.
axes[2].step(
    step_x,
    step_y,
    where="post",
    label="Total packet send rate",
)
axes[2].set_title("Total packet send rate over time")
axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Packets/sec, log scale")
axes[2].set_yscale("log")

# Cleaner log-grid: major gridlines only.
axes[2].grid(True, which="major", linestyle="-", alpha=0.55)
axes[2].grid(False, which="minor")
axes[2].legend()

# Match x-axis across all three plots.
for ax in axes:
    ax.set_xlim(xmin, xmax)

axes[0].set_xlabel("Time (s)")
axes[1].set_xlabel("Time (s)")

plt.tight_layout()
plt.savefig(OUT_FILE, dpi=200)
plt.close()