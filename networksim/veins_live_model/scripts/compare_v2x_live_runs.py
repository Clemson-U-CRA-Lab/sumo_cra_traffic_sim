#!/usr/bin/env python3
import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def summarize_run(name):
    run_dir = RESULTS / name

    commands = read_csv(run_dir / "v2x_live_controller_commands.csv")
    packets = read_csv(run_dir / "v2x_simu5g_delivered_packets.csv")

    nv0_ages = [float(r["nv0_info_age"]) for r in commands]
    packet_delays = [float(r["delay"]) for r in packets]

    controller_ok = sum(1 for r in commands if r["controller_ok"] == "1")
    actuation_ok = sum(1 for r in commands if r["actuation_ok"] == "1")
    simu5g_rows = sum(1 for r in commands if r["nv0_source"] == "simu5g_packet")
    direct_rows = sum(1 for r in commands if r["nv0_source"] == "direct")

    summary = {
        "run": name,
        "controller_rows": len(commands),
        "simu5g_packet_rows": simu5g_rows,
        "direct_fallback_rows": direct_rows,
        "controller_ok_rows": controller_ok,
        "actuation_ok_rows": actuation_ok,
        "avg_nv0_info_age": mean(nv0_ages),
        "max_nv0_info_age": max(nv0_ages) if nv0_ages else 0.0,
        "delivered_status_packets": len(packets),
        "avg_status_packet_delay": mean(packet_delays),
        "max_status_packet_delay": max(packet_delays) if packet_delays else 0.0,
    }

    noise_path = run_dir / "v2x_simu5g_noise_packets.csv"
    if noise_path.exists():
        noise = read_csv(noise_path)
        noise_delays = [float(r["delay"]) for r in noise]
        summary["delivered_noise_packets"] = len(noise)
        summary["avg_noise_packet_delay"] = mean(noise_delays)
        summary["max_noise_packet_delay"] = max(noise_delays) if noise_delays else 0.0
    else:
        summary["delivered_noise_packets"] = 0
        summary["avg_noise_packet_delay"] = 0.0
        summary["max_noise_packet_delay"] = 0.0

    return summary


def main():
    summaries = [summarize_run("baseline"), summarize_run("stress")]

    out_path = RESULTS / "baseline_vs_stress_summary.csv"

    fields = [
        "run",
        "controller_rows",
        "simu5g_packet_rows",
        "direct_fallback_rows",
        "controller_ok_rows",
        "actuation_ok_rows",
        "avg_nv0_info_age",
        "max_nv0_info_age",
        "delivered_status_packets",
        "avg_status_packet_delay",
        "max_status_packet_delay",
        "delivered_noise_packets",
        "avg_noise_packet_delay",
        "max_noise_packet_delay",
    ]

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    print(f"Wrote {out_path}")
    print()

    for s in summaries:
        print(f"=== {s['run']} ===")
        for key in fields[1:]:
            print(f"{key}: {s[key]}")
        print()


if __name__ == "__main__":
    main()
