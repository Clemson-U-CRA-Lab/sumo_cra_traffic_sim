#!/usr/bin/env python3

import argparse
from pathlib import Path
import numpy as np
import pandas as pd


def load(path):
    df = pd.read_csv(path).copy()
    tcol = "Sim Time [sec]"
    df = df[df[tcol] >= 0].copy()
    df = df.sort_values(tcol)
    df = df.drop_duplicates(tcol, keep="last")
    df = df.reset_index(drop=True)
    df["gap [m]"] = df["v0_dist [m]"] - df["v1_dist [m]"]
    return df


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", default="networksim/results/baseline/sumo_controller_log.csv")
    ap.add_argument("--attack", required=True)
    args = ap.parse_args()

    base = load(args.baseline)
    atk = load(args.attack)

    t = base["Sim Time [sec]"].values

    def interp(df, col):
        return np.interp(t, df["Sim Time [sec]"].values, df[col].values)

    metrics = {}

    for col in ["v1_spd [m/s]", "v1_accCmd [m/s2]", "gap [m]"]:
        b = base[col].values
        a = interp(atk, col)
        diff = a - b

        metrics[col] = {
            "max_abs_error": float(np.max(np.abs(diff))),
            "mean_abs_error": float(np.mean(np.abs(diff))),
            "rmse": float(np.sqrt(np.mean(diff ** 2))),
        }

    atk_gap = interp(atk, "gap [m]")
    atk_acc = interp(atk, "v1_accCmd [m/s2]")
    atk_speed = interp(atk, "v1_spd [m/s]")

    print("Attack file:", args.attack)
    print()
    print("Tracking error metrics attack minus baseline:")
    for col, vals in metrics.items():
        print(f"\n{col}")
        for k, v in vals.items():
            print(f"  {k}: {v}")

    print("\nSafety/control stress metrics:")
    print("  min attack gap [m]:", float(np.min(atk_gap)))
    print("  max attack gap [m]:", float(np.max(atk_gap)))
    print("  max abs attack acc cmd [m/s2]:", float(np.max(np.abs(atk_acc))))
    print("  min attack speed [m/s]:", float(np.min(atk_speed)))
    print("  max attack speed [m/s]:", float(np.max(atk_speed)))

    danger = atk_gap <= 0.5
    print("  timesteps with gap <= 0.5m:", int(np.sum(danger)))

    if np.any(danger):
        first_idx = np.where(danger)[0][0]
        print("  first severe gap time [s]:", float(t[first_idx]))


if __name__ == "__main__":
    main()
