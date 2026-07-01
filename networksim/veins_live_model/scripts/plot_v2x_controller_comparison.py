#!/usr/bin/env python3
import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


def load_controller_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)

    numeric_cols = [
        "sim_time",
        "direct_nv0_x",
        "controller_nv0_x",
        "nv1_x",
        "nv1_speed",
        "nv1_accel",
        "acc_cmd_nv1",
        "nv0_info_age",
        "target_speed_nv1",
    ]

    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    return df.dropna(subset=["sim_time"])


def build_gap(df: pd.DataFrame, gap_mode: str):
    if gap_mode == "perceived":
        return df["controller_nv0_x"] - df["nv1_x"], "Controller-perceived gap [m]"
    elif gap_mode == "true":
        return df["direct_nv0_x"] - df["nv1_x"], "True vehicle gap [m]"
    else:
        raise ValueError(f"Unknown gap mode: {gap_mode}")


def plot_controller_comparison(
    baseline_path: Path,
    stress_path: Path,
    output_path: Path,
    title: str,
    stress_label: str,
    gap_mode: str,
    attack_start: float,
    attack_end: float,
    xmin: float | None,
    xmax: float | None,
):
    baseline = load_controller_csv(baseline_path)
    stress = load_controller_csv(stress_path)

    baseline_gap, gap_ylabel = build_gap(baseline, gap_mode)
    stress_gap, _ = build_gap(stress, gap_mode)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)

    baseline_style = {
        "linestyle": "--",
        "linewidth": 2.0,
        "label": "Baseline",
    }

    stress_style = {
        "linewidth": 2.0,
        "label": stress_label,
    }

    # 1. nv1 speed
    axes[0].plot(
        baseline["sim_time"],
        baseline["nv1_speed"],
        **baseline_style,
    )
    axes[0].plot(
        stress["sim_time"],
        stress["nv1_speed"],
        **stress_style,
    )
    axes[0].set_ylabel("v1 speed [m/s]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.25)

    # 2. gap
    axes[1].plot(
        baseline["sim_time"],
        baseline_gap,
        **baseline_style,
    )
    axes[1].plot(
        stress["sim_time"],
        stress_gap,
        **stress_style,
    )
    axes[1].set_ylabel(gap_ylabel)
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.25)

    # Add zero line for perceived gap because negative perceived gap matters
    if gap_mode == "perceived":
        axes[1].axhline(0, linewidth=1.0, linestyle=":", alpha=0.7)

    # 3. acceleration command
    axes[2].plot(
        baseline["sim_time"],
        baseline["acc_cmd_nv1"],
        **baseline_style,
    )
    axes[2].plot(
        stress["sim_time"],
        stress["acc_cmd_nv1"],
        **stress_style,
    )
    axes[2].set_ylabel("v1 accel command [m/s²]")
    axes[2].set_xlabel("sim time [s]")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.25)

    # Stress-window shading
    if attack_start is not None and attack_end is not None:
        for ax in axes:
            ax.axvspan(attack_start, attack_end, alpha=0.12)

        y_top = axes[0].get_ylim()[1]
        axes[0].text(
            (attack_start + attack_end) / 2.0,
            y_top,
            "stress window",
            ha="center",
            va="top",
            fontsize=9,
        )

    if xmin is not None or xmax is not None:
        axes[2].set_xlim(left=xmin, right=xmax)

    fig.suptitle(title, fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    fig.savefig(output_path.with_suffix(".pdf"), bbox_inches="tight")

    print(f"Wrote {output_path}")
    print(f"Wrote {output_path.with_suffix('.pdf')}")


def main():
    root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--baseline",
        type=Path,
        default=root / "results" / "baseline" / "v2x_live_controller_commands.csv",
    )

    parser.add_argument(
        "--stress",
        type=Path,
        default=root / "results" / "stress" / "v2x_live_controller_commands.csv",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=root / "results" / "controller_vehicle_baseline_vs_stress_perceived_gap.png",
    )

    parser.add_argument(
        "--title",
        default="Effect of Simu5G Network Stress on MPC Vehicle Control",
    )

    parser.add_argument(
        "--stress-label",
        default="Simu5G UE stress attack",
    )

    parser.add_argument(
        "--gap-mode",
        choices=["perceived", "true"],
        default="perceived",
        help="perceived = delayed packet nv0 - nv1; true = direct physical nv0 - nv1",
    )

    parser.add_argument("--attack-start", type=float, default=5.0)
    parser.add_argument("--attack-end", type=float, default=10.0)

    parser.add_argument("--xmin", type=float, default=None)
    parser.add_argument("--xmax", type=float, default=None)

    parser.add_argument(
        "--make-both",
        action="store_true",
        help="Generate both perceived-gap and true-gap plots.",
    )

    args = parser.parse_args()

    if args.make_both:
        perceived_output = root / "results" / "controller_vehicle_baseline_vs_stress_perceived_gap.png"
        true_output = root / "results" / "controller_vehicle_baseline_vs_stress_true_gap.png"

        plot_controller_comparison(
            baseline_path=args.baseline,
            stress_path=args.stress,
            output_path=perceived_output,
            title=args.title + " — Controller-Perceived Gap",
            stress_label=args.stress_label,
            gap_mode="perceived",
            attack_start=args.attack_start,
            attack_end=args.attack_end,
            xmin=args.xmin,
            xmax=args.xmax,
        )

        plot_controller_comparison(
            baseline_path=args.baseline,
            stress_path=args.stress,
            output_path=true_output,
            title=args.title + " — True Physical Gap",
            stress_label=args.stress_label,
            gap_mode="true",
            attack_start=args.attack_start,
            attack_end=args.attack_end,
            xmin=args.xmin,
            xmax=args.xmax,
        )

    else:
        plot_controller_comparison(
            baseline_path=args.baseline,
            stress_path=args.stress,
            output_path=args.output,
            title=args.title,
            stress_label=args.stress_label,
            gap_mode=args.gap_mode,
            attack_start=args.attack_start,
            attack_end=args.attack_end,
            xmin=args.xmin,
            xmax=args.xmax,
        )


if __name__ == "__main__":
    main()
