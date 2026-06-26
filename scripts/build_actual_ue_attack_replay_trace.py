#!/usr/bin/env python3

import argparse
import csv
import math
import re
import sys
from pathlib import Path

csv.field_size_limit(sys.maxsize)

TARGET_MODULE_CONTAINS = "server.app[0]"


def as_float(x):
    try:
        if x is None:
            return None
        x = str(x).strip()
        if not x:
            return None
        y = float(x)
        return y if math.isfinite(y) else None
    except Exception:
        return None


def float_list(x):
    vals = []
    if not x:
        return vals

    for p in re.split(r"[,\s]+", str(x).strip()):
        y = as_float(p)
        if y is not None:
            vals.append(y)

    return vals


def read_schedule(path):
    rows = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            row["message_id"] = int(row["message_id"])
            row["send_time"] = float(row["send_time"])
            rows.append(row)

    rows.sort(key=lambda r: r["message_id"])
    return rows


def extract_target_delay_vector(vector_csv):
    candidates = []

    with open(vector_csv, newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            if row.get("type") != "vector":
                continue

            module = row.get("module", "")
            name = row.get("name", "")
            vecvalue = row.get("vecvalue", "")

            blob = f"{module} {name}".lower()

            if TARGET_MODULE_CONTAINS not in blob:
                continue

            if "delay" not in blob:
                continue

            if not vecvalue.strip():
                continue

            values = [v for v in float_list(vecvalue) if 0.0 <= v <= 10.0]

            if not values:
                continue

            candidates.append((module, name, values))

    if not candidates:
        raise SystemExit(
            "ERROR: Could not find a nonempty target-flow delay vector from server.app[0]. "
            "Run the inspection command again and send me the output."
        )

    module, name, values = candidates[0]

    print("Using target-flow Simu5G delay vector:")
    print(" module:", module)
    print(" name:", name)
    print(" samples:", len(values))

    return values, module, name


def build_trace(schedule, delays, out_path, module, name):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    delivered = 0
    dropped = 0

    with out_path.open("w", newline="") as f:
        fieldnames = [
            "message_id",
            "send_time",
            "receive_time",
            "delivered",
            "delay_seconds",
            "drop_reason",
            "source",
            "receiver",
            "network_model",
            "simu5g_delay_module",
            "simu5g_delay_vector",
        ]

        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for i, msg in enumerate(schedule):
            send_time = msg["send_time"]

            if i >= len(delays):
                writer.writerow({
                    "message_id": msg["message_id"],
                    "send_time": f"{send_time:.6f}",
                    "receive_time": "",
                    "delivered": "false",
                    "delay_seconds": "",
                    "drop_reason": "target_message_not_received_in_simu5g",
                    "source": msg.get("sender_id", "nv0"),
                    "receiver": msg.get("receiver_id", "nv1"),
                    "network_model": "Simu5G_ActualUeAttack",
                    "simu5g_delay_module": module,
                    "simu5g_delay_vector": name,
                })
                dropped += 1
                continue

            delay = delays[i]
            receive_time = send_time + delay

            writer.writerow({
                "message_id": msg["message_id"],
                "send_time": f"{send_time:.6f}",
                "receive_time": f"{receive_time:.6f}",
                "delivered": "true",
                "delay_seconds": f"{delay:.6f}",
                "drop_reason": "",
                "source": msg.get("sender_id", "nv0"),
                "receiver": msg.get("receiver_id", "nv1"),
                "network_model": "Simu5G_ActualUeAttack",
                "simu5g_delay_module": module,
                "simu5g_delay_vector": name,
            })
            delivered += 1

    print(f"Wrote: {out_path}")
    print(f"scheduled messages: {len(schedule)}")
    print(f"target delay samples from Simu5G: {len(delays)}")
    print(f"delivered in replay trace: {delivered}")
    print(f"dropped in replay trace: {dropped}")

    used = delays[: min(len(schedule), len(delays))]
    if used:
        print(f"delay min:  {min(used):.6f} s")
        print(f"delay mean: {sum(used) / len(used):.6f} s")
        print(f"delay max:  {max(used):.6f} s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--schedule", required=True)
    parser.add_argument("--vectors", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    schedule = read_schedule(args.schedule)
    delays, module, name = extract_target_delay_vector(args.vectors)
    build_trace(schedule, delays, args.out, module, name)


if __name__ == "__main__":
    main()
