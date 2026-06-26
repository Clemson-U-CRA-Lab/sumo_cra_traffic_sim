#!/usr/bin/env python3

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


BASELINE_CTRL = Path("networksim/results/baseline/sumo_controller_log.csv")
ATTACK_CTRL = Path("networksim/results/simu5g_actual_ue_attack_extreme/sumo_controller_log.csv")
ATTACK_TRACE = Path("networksim/results/simu5g_actual_ue_attack_extreme/network_degradation_trace.csv")

OUTDIR = Path("networksim/results/simu5g_actual_ue_attack_extreme")
OUTDIR.mkdir(parents=True, exist_ok=True)


def load_controller(path):
    df = pd.read_csv(path).copy()
    tcol = "Sim Time [sec]"
    df = df[df[tcol] >= 0].copy()
    df = df.sort_values(tcol)
    df = df.drop_duplicates(subset=[tcol], keep="last")
    df = df.reset_index(drop=True)

    if "v0_dist [m]" in df.columns and "v1_dist [m]" in df.columns:
        df["gap [m]"] = df["v0_dist [m]"] - df["v1_dist [m]"]

    return df


def load_trace(path):
    df = pd.read_csv(path).copy()
    scheduled = df[df["status"] == "scheduled"].copy()
    delivered = df[df["status"] == "delivered"].copy()
    dropped = df[df["status"] == "dropped"].copy()
    return scheduled, delivered, dropped


def interp(df, t, ycol):
    return np.interp(t, df["Sim Time [sec]"].values, df[ycol].values)


def gap_values(df):
    return (df["v0_dist [m]"] - df["v1_dist [m]"]).values


def save_combined_controller_comparison(base_df, attack_df):
    tcol = "Sim Time [sec]"
    t = base_df[tcol].values

    base_speed = interp(base_df, t, "v1_spd [m/s]")
    attack_speed = interp(attack_df, t, "v1_spd [m/s]")

    base_gap = np.interp(t, base_df[tcol].values, gap_values(base_df))
    attack_gap = np.interp(t, attack_df[tcol].values, gap_values(attack_df))

    base_cmd = interp(base_df, t, "v1_accCmd [m/s2]")
    attack_cmd = interp(attack_df, t, "v1_accCmd [m/s2]")

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    axs[0].plot(t, base_speed, "--", label="Baseline")
    axs[0].plot(t, attack_speed, label="Simu5G actual UE attack extreme")
    axs[0].set_ylabel("v1 speed [m/s]")
    axs[0].set_title("Controller/vehicle comparison: baseline vs Simu5G actual UE attack")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, base_gap, "--", label="Baseline")
    axs[1].plot(t, attack_gap, label="Simu5G actual UE attack extreme")
    axs[1].set_ylabel("Gap [m]")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, base_cmd, "--", label="Baseline")
    axs[2].plot(t, attack_cmd, label="Simu5G actual UE attack extreme")
    axs[2].set_ylabel("v1 acc cmd [m/s²]")
    axs[2].set_xlabel("Sim time [s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.savefig(OUTDIR / "baseline_vs_actual_ue_attack_extreme_combined.png", dpi=160)
    plt.close()


def save_difference_plot(base_df, attack_df):
    tcol = "Sim Time [sec]"
    t = base_df[tcol].values

    base_speed = interp(base_df, t, "v1_spd [m/s]")
    attack_speed = interp(attack_df, t, "v1_spd [m/s]")

    base_gap = np.interp(t, base_df[tcol].values, gap_values(base_df))
    attack_gap = np.interp(t, attack_df[tcol].values, gap_values(attack_df))

    base_cmd = interp(base_df, t, "v1_accCmd [m/s2]")
    attack_cmd = interp(attack_df, t, "v1_accCmd [m/s2]")

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    axs[0].plot(t, attack_speed - base_speed)
    axs[0].set_ylabel("Δ speed [m/s]")
    axs[0].set_title("Simu5G actual UE attack extreme minus baseline")
    axs[0].grid(True)

    axs[1].plot(t, attack_gap - base_gap)
    axs[1].set_ylabel("Δ gap [m]")
    axs[1].grid(True)

    axs[2].plot(t, attack_cmd - base_cmd)
    axs[2].set_ylabel("Δ acc cmd [m/s²]")
    axs[2].set_xlabel("Sim time [s]")
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(OUTDIR / "baseline_vs_actual_ue_attack_extreme_difference.png", dpi=160)
    plt.close()


def save_zoomed_difference_plot(base_df, attack_df, start=18.0, end=58.0):
    tcol = "Sim Time [sec]"
    t = base_df[tcol].values
    mask = (t >= start) & (t <= end)

    base_speed = interp(base_df, t, "v1_spd [m/s]")
    attack_speed = interp(attack_df, t, "v1_spd [m/s]")

    base_gap = np.interp(t, base_df[tcol].values, gap_values(base_df))
    attack_gap = np.interp(t, attack_df[tcol].values, gap_values(attack_df))

    base_cmd = interp(base_df, t, "v1_accCmd [m/s2]")
    attack_cmd = interp(attack_df, t, "v1_accCmd [m/s2]")

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    axs[0].plot(t[mask], (attack_speed - base_speed)[mask])
    axs[0].set_ylabel("Δ speed [m/s]")
    axs[0].set_title(f"Zoomed attack difference: {start:.0f}s to {end:.0f}s")
    axs[0].grid(True)

    axs[1].plot(t[mask], (attack_gap - base_gap)[mask])
    axs[1].set_ylabel("Δ gap [m]")
    axs[1].grid(True)

    axs[2].plot(t[mask], (attack_cmd - base_cmd)[mask])
    axs[2].set_ylabel("Δ acc cmd [m/s²]")
    axs[2].set_xlabel("Sim time [s]")
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(OUTDIR / "baseline_vs_actual_ue_attack_extreme_difference_zoom.png", dpi=160)
    plt.close()


def save_network_delay_plot(scheduled, delivered):
    plt.figure(figsize=(10, 5))
    plt.plot(scheduled["send_time"], scheduled["delay_seconds"])
    plt.xlabel("Send time [s]")
    plt.ylabel("Model-generated replay delay [s]")
    plt.title("Simu5G actual UE attack extreme delay vs send time")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUTDIR / "actual_ue_attack_extreme_delay_vs_send_time.png", dpi=160)
    plt.close()

    merged = pd.merge(
        scheduled[["message_id", "send_time", "delivery_time", "delay_seconds"]],
        delivered[["message_id", "controller_receive_time", "message_age_seconds"]],
        on="message_id",
        how="inner",
    )

    if not merged.empty:
        merged["control_cycle_wait_seconds"] = (
            merged["controller_receive_time"] - merged["delivery_time"]
        )

        plt.figure(figsize=(11, 6))
        plt.plot(merged["send_time"], merged["delay_seconds"], label="Raw Simu5G model delay")
        plt.plot(merged["send_time"], merged["control_cycle_wait_seconds"], label="Controller wait after arrival")
        plt.plot(merged["send_time"], merged["message_age_seconds"], label="Total message age at controller")
        plt.xlabel("Message send time [s]")
        plt.ylabel("Time [s]")
        plt.title("Timestamp decomposition: Simu5G actual UE attack extreme")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(OUTDIR / "actual_ue_attack_extreme_timestamp_decomposition.png", dpi=160)
        plt.close()


def save_packet_summary(scheduled, delivered, dropped):
    labels = ["Scheduled", "Delivered", "Dropped"]
    values = [len(scheduled), len(delivered), len(dropped)]

    plt.figure(figsize=(7, 5))
    plt.bar(labels, values)
    plt.ylabel("Packet count")
    plt.title("Simu5G actual UE attack extreme packet summary")
    plt.tight_layout()
    plt.savefig(OUTDIR / "actual_ue_attack_extreme_packet_summary.png", dpi=160)
    plt.close()


def print_summary(scheduled, delivered, dropped):
    delays = scheduled["delay_seconds"].dropna().astype(float).values

    print("Packet summary")
    print(" scheduled:", len(scheduled))
    print(" delivered:", len(delivered))
    print(" dropped:", len(dropped))

    if len(delays):
        print("Delay summary")
        print(" min:", float(np.min(delays)))
        print(" mean:", float(np.mean(delays)))
        print(" max:", float(np.max(delays)))


def main():
    if not BASELINE_CTRL.exists():
        raise FileNotFoundError(f"Missing baseline controller log: {BASELINE_CTRL}")
    if not ATTACK_CTRL.exists():
        raise FileNotFoundError(f"Missing attack controller log: {ATTACK_CTRL}")
    if not ATTACK_TRACE.exists():
        raise FileNotFoundError(f"Missing attack replay trace: {ATTACK_TRACE}")

    base_df = load_controller(BASELINE_CTRL)
    attack_df = load_controller(ATTACK_CTRL)
    scheduled, delivered, dropped = load_trace(ATTACK_TRACE)

    save_combined_controller_comparison(base_df, attack_df)
    save_difference_plot(base_df, attack_df)
    save_zoomed_difference_plot(base_df, attack_df)
    save_network_delay_plot(scheduled, delivered)
    save_packet_summary(scheduled, delivered, dropped)
    print_summary(scheduled, delivered, dropped)

    print("Saved plots to:", OUTDIR)
    for p in sorted(OUTDIR.glob("*.png")):
        print(" -", p)


if __name__ == "__main__":
    main()
