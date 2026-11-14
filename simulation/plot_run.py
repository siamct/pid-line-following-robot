"""
Headless runs that compare controller settings and save plots for the README.

    python plot_run.py            writes PNGs into ../docs and prints a results table

Three things are produced:
    sim_paths.png   the path driven with P-only, PD and PID gains over the track
    sim_error.png   line-position error against time for the same three runs
    kp_sweep.png    RMS error and times the line was lost against Kp
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from simulator import Simulator, Gains  # noqa: E402

DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs")
RUN_SECONDS = 30.0

CASES = [
    ("P only  (Kp 0.8)", Gains(kp=0.8, ki=0.0, kd=0.0)),
    ("PD  (Kp 0.8, Kd 0.06)", Gains(kp=0.8, ki=0.0, kd=0.06)),
    ("PID  (Kp 0.8, Ki 0.2, Kd 0.06)", Gains(kp=0.8, ki=0.2, kd=0.06)),
]


def run_cases():
    results = []
    for name, gains in CASES:
        sim = Simulator(gains)
        metrics = sim.run(RUN_SECONDS)
        results.append((name, sim, metrics))
    return results


def plot_paths(results):
    fig, axes = plt.subplots(1, len(results), figsize=(5.2 * len(results), 4.0), sharey=True)
    for ax, (name, sim, m) in zip(axes, results):
        tx = [p[0] for p in sim.track.points] + [sim.track.points[0][0]]
        ty = [p[1] for p in sim.track.points] + [sim.track.points[0][1]]
        ax.plot(tx, ty, color="black", linewidth=6, solid_capstyle="round", alpha=0.85, label="tape")
        px = [p[0] for p in sim.path]
        py = [p[1] for p in sim.path]
        ax.plot(px, py, color="#e07a2f", linewidth=1.2, label="robot path")
        ax.set_title("%s\nRMS error %.3f, max deviation %.0f mm, laps %d" % (
            name, m["rms_error"], m["max_deviation_mm"], m["laps"]), fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(0, 1.6)
        ax.set_ylim(0, 1.0)
        ax.set_xlabel("x (m)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("y (m)")
    axes[0].legend(loc="lower right", fontsize=8)
    fig.suptitle("Simulated path over %.0f s with different controller terms" % RUN_SECONDS, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "sim_paths.png"), dpi=130)
    plt.close(fig)


def plot_errors(results, seconds=12.0):
    fig, axes = plt.subplots(len(results), 1, figsize=(9, 2.1 * len(results)), sharex=True)
    for ax, (name, sim, m) in zip(axes, results):
        t = [h[0] for h in sim.history if h[0] <= seconds]
        e = [h[3] for h in sim.history if h[0] <= seconds]
        ax.plot(t, e, linewidth=0.9, color="#1f77b4")
        ax.axhline(0, color="black", linewidth=0.6)
        ax.set_ylim(-1.1, 1.1)
        ax.set_ylabel("error")
        ax.set_title(name, fontsize=9, loc="left")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Line position error (-1 left, +1 right) for the first %.0f s" % seconds, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "sim_error.png"), dpi=130)
    plt.close(fig)


def plot_kp_sweep():
    kps = [0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    rows = []
    for kp in kps:
        p_only = Simulator(Gains(kp=kp, ki=0.0, kd=0.0)).run(20.0)
        pid = Simulator(Gains(kp=kp, ki=0.2, kd=0.06)).run(20.0)
        rows.append((kp, p_only["rms_error"], p_only["line_lost"], pid["rms_error"], pid["line_lost"]))

    fig, ax1 = plt.subplots(figsize=(7.5, 3.6))
    ax1.plot(kps, [r[1] for r in rows], marker="o", color="#d62728", label="P only: RMS error")
    ax1.plot(kps, [r[3] for r in rows], marker="o", color="#1f77b4", label="PID (Ki 0.2, Kd 0.06): RMS error")
    ax1.set_xlabel("Kp")
    ax1.set_ylabel("RMS line-position error")
    ax1.set_ylim(bottom=0)
    ax1.grid(alpha=0.3)
    ax2 = ax1.twinx()
    ax2.bar([k - 0.02 for k in kps], [r[2] for r in rows], width=0.04, color="#d62728", alpha=0.35, label="P only: times lost")
    ax2.bar([k + 0.02 for k in kps], [r[4] for r in rows], width=0.04, color="#1f77b4", alpha=0.35, label="PID: times lost")
    ax2.set_ylabel("times the line was lost in 20 s")
    ax2.set_ylim(bottom=0)
    lines, labels = ax1.get_legend_handles_labels()
    bars, blabels = ax2.get_legend_handles_labels()
    ax1.legend(lines + bars, labels + blabels, fontsize=8, loc="upper right")
    fig.suptitle("Effect of proportional gain, with and without the I and D terms", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "kp_sweep.png"), dpi=130)
    plt.close(fig)
    return rows


def main():
    os.makedirs(DOCS, exist_ok=True)
    results = run_cases()
    plot_paths(results)
    plot_errors(results)
    sweep = plot_kp_sweep()

    print("%-34s %6s %7s %9s %10s %7s %9s" % ("case", "laps", "RMS", "max err", "max dev mm", "lost", "speed m/s"))
    for name, _, m in results:
        print("%-34s %6d %7.3f %9.3f %10.1f %7d %9.3f" % (
            name, m["laps"], m["rms_error"], m["max_error"], m["max_deviation_mm"], m["line_lost"], m["mean_speed_mps"]))
    print()
    print("Kp sweep:  Kp   P-only rms / lost   PID rms / lost")
    for kp, r1, l1, r2, l2 in sweep:
        print("          %.1f   %.3f / %d          %.3f / %d" % (kp, r1, l1, r2, l2))


if __name__ == "__main__":
    main()
