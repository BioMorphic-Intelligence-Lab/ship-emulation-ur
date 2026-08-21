"""
Quick visual inspection of vessel_motion_clean.csv before robot testing.

Usage:
    python -m ship_emulation.analyze_data
    python -m ship_emulation.analyze_data --csv path/to/other.csv

    # Analyse only a 180 s window starting at t=2184 s (e.g. the phase2 emulation window):
    python -m ship_emulation.analyze_data --start-time 2184 --duration 180

    # Preview augmented data (m→mm, 5× angular scale, 10 deg roll overlay at 0.1 Hz):
    python -m ship_emulation.analyze_data --linear-scale 1000 --angular-scale 5 \
        --overlay-roll-amp 10 --overlay-roll-freq 0.1
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.interpolate import UnivariateSpline
from scipy.spatial.transform import Rotation

POSITION_COLS = ["linear_x", "linear_y", "linear_z"]
ROTATION_COLS = ["rotation_x", "rotation_y", "rotation_z"]
TIME_COL = "time"

DEFAULT_CSV = Path(__file__).parent / "2b_vessel_motion_clean.csv"


def load(path: Path):
    import csv, io
    with open(path, encoding="utf-8-sig") as f:
        raw = f.read()
    # Some exports wrap every row in double quotes; strip them so DictReader
    # can parse the comma-separated columns normally.
    lines = [line.strip().strip('"') for line in raw.splitlines() if line.strip()]
    reader = csv.DictReader(io.StringIO("\n".join(lines)))
    rows = [{k: float(v) for k, v in row.items()} for row in reader]
    return {col: np.array([r[col] for r in rows]) for col in rows[0]}


def slice_window(data, t_start, duration):
    """Return a copy of data restricted to [t_start, t_start + duration]."""
    t = data[TIME_COL]
    t_end = t_start + duration
    mask = (t >= t_start) & (t <= t_end)
    if not mask.any():
        raise ValueError(
            f"No samples in [{t_start:.3f}, {t_end:.3f}] s "
            f"(data spans [{t[0]:.3f}, {t[-1]:.3f}] s)"
        )
    return {col: arr[mask] for col, arr in data.items()}


def apply_augmentation(data, linear_scale, angular_scale, overlay, fade_in_duration=0.0):
    data = dict(data)
    for col in POSITION_COLS:
        data[col] = data[col] * linear_scale
    for col in ROTATION_COLS:
        data[col] = data[col] * angular_scale
    if overlay is not None:
        t = data[TIME_COL]
        # Raised-cosine envelope matching SinusoidalOverlaySource — ramps from 0 to 1.
        if fade_in_duration > 0.0:
            tau = t - t[0]
            envelope = np.where(
                tau < fade_in_duration,
                0.5 * (1.0 - np.cos(np.pi * tau / fade_in_duration)),
                1.0,
            )
        else:
            envelope = 1.0
        amps_freqs = [
            ("rotation_x", overlay["roll_amp"], overlay["roll_freq"]),
            ("rotation_y", overlay["pitch_amp"], overlay["pitch_freq"]),
            ("rotation_z", overlay["yaw_amp"], overlay["yaw_freq"]),
        ]
        for col, amp, freq in amps_freqs:
            if amp != 0.0:
                data[col] = data[col] + envelope * amp * np.sin(2 * math.pi * freq * t)
    return data


def apply_rigid_body_offset(data, ox, oy, oz):
    """Translate COG motion to a body-frame offset point (intrinsic XYZ Euler convention)."""
    data = dict(data)
    eulers = np.stack([data["rotation_x"], data["rotation_y"], data["rotation_z"]], axis=1)
    R = Rotation.from_euler('xyz', eulers, degrees=True).as_matrix()  # (N, 3, 3)
    delta = (R @ np.array([ox, oy, oz]))  # (N, 3)
    data["linear_x"] = data["linear_x"] + delta[:, 0]
    data["linear_y"] = data["linear_y"] + delta[:, 1]
    data["linear_z"] = data["linear_z"] + delta[:, 2]
    return data


def print_stats(data, pos_unit, rot_unit):
    print(f"\n{'Column':<14} {'units':>6} {'min':>10} {'max':>10} {'mean':>10} {'std':>10}")
    print("-" * 66)
    for col in POSITION_COLS:
        v = data[col]
        print(f"{col:<14} {pos_unit:>6} {v.min():>10.4f} {v.max():>10.4f} {v.mean():>10.4f} {v.std():>10.4f}")
    for col in ROTATION_COLS:
        v = data[col]
        print(f"{col:<14} {rot_unit:>6} {v.min():>10.4f} {v.max():>10.4f} {v.mean():>10.4f} {v.std():>10.4f}")
    t = data[TIME_COL]
    dt = np.diff(t)
    print(f"\nTime range : {t[0]:.1f} s → {t[-1]:.1f} s  ({len(t)} samples)")
    print(f"Step size  : min={dt.min():.4f} s  max={dt.max():.4f} s  median={np.median(dt):.4f} s")


def apply_smoothing(data, s_linear, s_angular):
    t = data[TIME_COL]
    smoothed = dict(data)
    for col in POSITION_COLS:
        smoothed[col] = UnivariateSpline(t, data[col], s=s_linear)(t)
    for col in ROTATION_COLS:
        smoothed[col] = UnivariateSpline(t, data[col], s=s_angular)(t)
    return smoothed


def print_rate_stats(data, pos_unit, rot_unit):
    _, linear_rate, angular_rate, per_axis_linear, per_axis_angular = compute_rates(data)
    lunit = f"{pos_unit}/s"
    aunit = f"{rot_unit}/s"
    print(f"\n{'Rate':<20} {'units':>8} {'min':>10} {'max':>10} {'mean':>10} {'std':>10}")
    print("-" * 74)
    print(f"{'linear_rate (‖v‖)':<20} {lunit:>8} {linear_rate.min():>10.4f} {linear_rate.max():>10.4f} {linear_rate.mean():>10.4f} {linear_rate.std():>10.4f}")
    for col, values in per_axis_linear.items():
        label = f"  {col}"
        print(f"{label:<20} {lunit:>8} {values.min():>10.4f} {values.max():>10.4f} {values.mean():>10.4f} {values.std():>10.4f}")
    print(f"{'angular_rate (max|ω|)':<20} {aunit:>8} {angular_rate.min():>10.4f} {angular_rate.max():>10.4f} {angular_rate.mean():>10.4f} {angular_rate.std():>10.4f}")
    for col, values in per_axis_angular.items():
        label = f"  {col}"
        print(f"{label:<20} {aunit:>8} {values.min():>10.4f} {values.max():>10.4f} {values.mean():>10.4f} {values.std():>10.4f}")


def compute_rates(data):
    t = data[TIME_COL]
    dt = np.diff(t)
    pos = np.stack([data[c] for c in POSITION_COLS], axis=1)
    rot = np.stack([data[c] for c in ROTATION_COLS], axis=1)
    linear_rate = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    angular_rate = np.max(np.abs(np.diff(rot, axis=0)), axis=1) / dt
    per_axis_linear  = {col: np.diff(data[col]) / dt for col in POSITION_COLS}
    per_axis_angular = {col: np.diff(data[col]) / dt for col in ROTATION_COLS}
    return t[1:], linear_rate, angular_rate, per_axis_linear, per_axis_angular


def plot(data, pos_unit, rot_unit, title_suffix, smoothed_data=None, save_path=None):
    t = data[TIME_COL]
    t_rate, linear_rate, angular_rate, _, _ = compute_rates(data)

    title = "Vessel Motion Inspection"
    if title_suffix:
        title += f"  [{title_suffix}]"

    fig = plt.figure(figsize=(16, 14))
    fig.suptitle(title, fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax_pos = fig.add_subplot(gs[0, :])
    colors = {}
    for col in POSITION_COLS:
        line, = ax_pos.plot(t, data[col], label=col, linewidth=0.6, alpha=0.4 if smoothed_data else 1.0)
        colors[col] = line.get_color()
    if smoothed_data:
        for col in POSITION_COLS:
            ax_pos.plot(t, smoothed_data[col], color=colors[col], linewidth=1.2, label=f"{col} (smoothed)")
    ax_pos.set_title("Position")
    ax_pos.set_ylabel(pos_unit)
    ax_pos.legend(loc="upper right", fontsize=8)
    ax_pos.grid(True, alpha=0.3)

    ax_rot = fig.add_subplot(gs[1, :])
    for col in ROTATION_COLS:
        line, = ax_rot.plot(t, data[col], label=col, linewidth=0.6, alpha=0.4 if smoothed_data else 1.0)
        colors[col] = line.get_color()
    if smoothed_data:
        for col in ROTATION_COLS:
            ax_rot.plot(t, smoothed_data[col], color=colors[col], linewidth=1.2, label=f"{col} (smoothed)")
    ax_rot.set_title("Rotation")
    ax_rot.set_ylabel(rot_unit)
    ax_rot.set_xlabel("time (s)")
    ax_rot.legend(loc="upper right", fontsize=8)
    ax_rot.grid(True, alpha=0.3)

    ax_xy = fig.add_subplot(gs[2, 0])
    ax_xy.plot(data["linear_x"], data["linear_y"], linewidth=0.4, color="steelblue")
    ax_xy.scatter(data["linear_x"][0], data["linear_y"][0], color="green", s=30, zorder=5, label="start")
    ax_xy.scatter(data["linear_x"][-1], data["linear_y"][-1], color="red", s=30, zorder=5, label="end")
    ax_xy.set_title("XY trajectory (top-down)")
    ax_xy.set_xlabel(f"linear_x ({pos_unit})")
    ax_xy.set_ylabel(f"linear_y ({pos_unit})")
    ax_xy.legend(fontsize=8)
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3)

    ax_xz = fig.add_subplot(gs[2, 1])
    ax_xz.plot(data["linear_x"], data["linear_z"], linewidth=0.4, color="darkorange")
    ax_xz.scatter(data["linear_x"][0], data["linear_z"][0], color="green", s=30, zorder=5, label="start")
    ax_xz.scatter(data["linear_x"][-1], data["linear_z"][-1], color="red", s=30, zorder=5, label="end")
    ax_xz.set_title("XZ trajectory (side view)")
    ax_xz.set_xlabel(f"linear_x ({pos_unit})")
    ax_xz.set_ylabel(f"linear_z ({pos_unit})")
    ax_xz.legend(fontsize=8)
    ax_xz.set_aspect("equal")
    ax_xz.grid(True, alpha=0.3)

    ax_lr = fig.add_subplot(gs[3, 0])
    ax_lr.plot(t_rate, linear_rate, linewidth=0.6, color="steelblue", alpha=0.4 if smoothed_data else 1.0, label="raw")
    ax_lr.set_title("Linear rate of change (‖Δpos‖/Δt)")
    ax_lr.set_xlabel("time (s)")
    ax_lr.set_ylabel(f"{pos_unit}/s")
    ax_lr.grid(True, alpha=0.3)
    p95 = np.percentile(linear_rate, 95)
    ax_lr.axhline(p95, color="red", linestyle="--", linewidth=0.8, label=f"p95={p95:.1f}")
    if smoothed_data:
        _, s_linear_rate, _, _, _ = compute_rates(smoothed_data)
        ax_lr.plot(t_rate, s_linear_rate, linewidth=1.2, color="steelblue", label="smoothed")
        s_p95 = np.percentile(s_linear_rate, 95)
        ax_lr.axhline(s_p95, color="navy", linestyle="--", linewidth=0.8, label=f"smoothed p95={s_p95:.1f}")
    ax_lr.legend(fontsize=8)

    ax_ar = fig.add_subplot(gs[3, 1])
    ax_ar.plot(t_rate, angular_rate, linewidth=0.6, color="darkorange", alpha=0.4 if smoothed_data else 1.0, label="raw")
    ax_ar.set_title("Angular rate of change (max|Δrot|/Δt)")
    ax_ar.set_xlabel("time (s)")
    ax_ar.set_ylabel(f"{rot_unit}/s")
    ax_ar.grid(True, alpha=0.3)
    p95 = np.percentile(angular_rate, 95)
    ax_ar.axhline(p95, color="red", linestyle="--", linewidth=0.8, label=f"p95={p95:.1f}")
    if smoothed_data:
        _, _, s_angular_rate, _, _ = compute_rates(smoothed_data)
        ax_ar.plot(t_rate, s_angular_rate, linewidth=1.2, color="darkorange", label="smoothed")
        s_p95 = np.percentile(s_angular_rate, 95)
        ax_ar.axhline(s_p95, color="saddlebrown", linestyle="--", linewidth=0.8, label=f"smoothed p95={s_p95:.1f}")
    ax_ar.legend(fontsize=8)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\nPlot saved to {save_path}")
    else:
        plt.show()


def compute_psd(data):
    """Welch power spectral density for all 6 DOF channels. Returns ({col: (freqs, psd)}, fs)."""
    from scipy.signal import welch as scipy_welch
    t = data[TIME_COL]
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    results = {}
    for col in POSITION_COLS + ROTATION_COLS:
        signal = data[col] - data[col].mean()
        freqs, psd = scipy_welch(signal, fs=fs, nperseg=min(4096, len(signal) // 4))
        results[col] = (freqs, psd)
    return results, fs


def print_psd_stats(psd_results, fs, thresholds=(0.05, 0.1, 0.2, 0.5, 1.0)):
    nyquist = fs / 2.0
    thresholds = [thr for thr in thresholds if thr < nyquist]

    print(f"\n--- Fourier / PSD Analysis ---")
    print(f"Sampling rate: {fs:.3f} Hz  |  Nyquist: {nyquist:.3f} Hz")
    print(f"\nFraction of power ABOVE threshold (high-frequency content indicator):")
    header = f"{'Channel':<14} {'Dom. freq (Hz)':>16} {'Period (s)':>12}"
    for thr in thresholds:
        header += f"  {f'>{thr:.2f}Hz':>9}"
    print(header)
    print("-" * (44 + 11 * len(thresholds)))

    for col in POSITION_COLS + ROTATION_COLS:
        freqs, psd = psd_results[col]
        total = psd.sum()
        if total == 0:
            print(f"{col:<14}  (all zeros — skip)")
            continue
        idx = np.argmax(psd[1:]) + 1  # skip DC
        dom_freq = freqs[idx]
        dom_period = 1.0 / dom_freq if dom_freq > 0 else float("inf")
        line = f"{col:<14} {dom_freq:>16.4f} {dom_period:>12.2f}"
        for thr in thresholds:
            pct = 100.0 * psd[freqs >= thr].sum() / total
            line += f"  {pct:>8.1f}%"
        print(line)

    # Recommendation based on 0.5 Hz cutoff (UR arm practical tracking limit)
    print()
    noisy = []
    for col in POSITION_COLS + ROTATION_COLS:
        freqs, psd = psd_results[col]
        total = psd.sum()
        if total == 0:
            continue
        pct = 100.0 * psd[freqs >= 0.5].sum() / total
        if pct > 5.0:
            noisy.append((col, pct))

    if noisy:
        print("Channels with >5% power above 0.5 Hz:")
        for col, pct in noisy:
            print(f"  {col}: {pct:.1f}%")
        print("→ Low-pass filter recommended before robot arm tracking.")
    else:
        print("All channels have <5% power above 0.5 Hz.")
        print("→ Low-pass filter likely not required for robot arm tracking.")


def plot_psd(psd_results, fs, lpf_cutoff=None, save_path=None):
    """Plot Welch PSD (log-Y) for all 6 DOF channels in a 2×3 grid."""
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle("Power Spectral Density (Welch) — Vessel Motion", fontsize=14, fontweight="bold")

    col_titles = {
        "linear_x": "Surge (linear_x)", "linear_y": "Sway (linear_y)",
        "linear_z": "Heave (linear_z)", "rotation_x": "Roll (rotation_x)",
        "rotation_y": "Pitch (rotation_y)", "rotation_z": "Yaw (rotation_z)",
    }
    colors = {c: ("steelblue" if c in POSITION_COLS else "darkorange")
              for c in POSITION_COLS + ROTATION_COLS}

    for ax, col in zip(axes.flat, POSITION_COLS + ROTATION_COLS):
        freqs, psd = psd_results[col]
        mask = freqs > 0
        ax.semilogy(freqs[mask], psd[mask], linewidth=0.8, color=colors[col])
        if lpf_cutoff:
            ax.axvline(lpf_cutoff, color="red", linestyle="--", linewidth=1.2,
                       label=f"LPF {lpf_cutoff} Hz")
            ax.legend(fontsize=8)
        ax.set_title(col_titles.get(col, col), fontsize=10)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("PSD")
        ax.grid(True, which="both", alpha=0.3)

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"PSD plot saved to {save_path}")
    else:
        plt.show()
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Inspect vessel motion CSV data.")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, metavar="FILE")
    parser.add_argument("--start-time", type=float, default=None, metavar="S",
                        help="Start of analysis window in CSV time [s] (default: beginning of file)")
    parser.add_argument("--duration", type=float, default=None, metavar="S",
                        help="Length of analysis window [s] (default: to end of file)")
    parser.add_argument("--save", type=Path, default=None, metavar="OUT.png",
                        help="Save plot to file instead of showing interactively")
    parser.add_argument("--linear-scale", type=float, default=1.0, metavar="FACTOR",
                        help="Scale applied to x/y/z (e.g. 1000 to preview robot output in mm)")
    parser.add_argument("--angular-scale", type=float, default=1.0, metavar="FACTOR",
                        help="Scale applied to roll/pitch/yaw")
    parser.add_argument("--overlay-roll-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-roll-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--overlay-pitch-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-pitch-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--overlay-yaw-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-yaw-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--fade-in-duration", type=float, default=0.0, metavar="S",
                        help="Apply the same raised-cosine fade-in ramp to the overlay as "
                             "SinusoidalOverlaySource does (match phase2 FADE_IN_DURATION)")
    parser.add_argument("--smooth-linear", type=float, default=0.0, metavar="S",
                        help="Spline smoothing factor for linear channels in mm² (0 = disabled)")
    parser.add_argument("--smooth-angular", type=float, default=0.0, metavar="S",
                        help="Spline smoothing factor for angular channels in deg² (0 = disabled)")
    parser.add_argument("--offset-x", type=float, default=0.0, metavar="M",
                        help="Body-frame X offset from COG to target point (same units as pose data)")
    parser.add_argument("--offset-y", type=float, default=0.0, metavar="M",
                        help="Body-frame Y offset from COG to target point")
    parser.add_argument("--offset-z", type=float, default=0.0, metavar="M",
                        help="Body-frame Z offset from COG to target point")
    parser.add_argument("--fourier", action="store_true",
                        help="Run Fourier/PSD analysis and show power spectrum plots")
    parser.add_argument("--lpf-cutoff", type=float, default=None, metavar="HZ",
                        help="Mark a low-pass filter cutoff frequency on the PSD plot")
    parser.add_argument("--save-fourier", type=Path, default=None, metavar="OUT.png",
                        help="Save PSD plot to file instead of showing interactively")
    args = parser.parse_args(argv)

    if not args.csv.exists():
        sys.exit(f"File not found: {args.csv}")

    has_overlay = any([args.overlay_roll_amp, args.overlay_pitch_amp, args.overlay_yaw_amp])
    overlay = {
        "roll_amp": args.overlay_roll_amp, "roll_freq": args.overlay_roll_freq,
        "pitch_amp": args.overlay_pitch_amp, "pitch_freq": args.overlay_pitch_freq,
        "yaw_amp": args.overlay_yaw_amp, "yaw_freq": args.overlay_yaw_freq,
    } if has_overlay else None

    pos_unit = {1.0: "m", 1000.0: "mm"}.get(args.linear_scale, f"m×{args.linear_scale}")
    rot_unit = "deg"

    title_parts = []
    if args.linear_scale != 1.0:
        title_parts.append(f"linear×{args.linear_scale}")
    if args.angular_scale != 1.0:
        title_parts.append(f"angular×{args.angular_scale}")
    if overlay:
        parts = []
        if args.overlay_roll_amp:
            parts.append(f"roll {args.overlay_roll_amp}°@{args.overlay_roll_freq}Hz")
        if args.overlay_pitch_amp:
            parts.append(f"pitch {args.overlay_pitch_amp}°@{args.overlay_pitch_freq}Hz")
        if args.overlay_yaw_amp:
            parts.append(f"yaw {args.overlay_yaw_amp}°@{args.overlay_yaw_freq}Hz")
        if args.fade_in_duration > 0.0:
            parts.append(f"fade {args.fade_in_duration:.0f}s")
        title_parts.append("overlay: " + ", ".join(parts))

    print(f"Loading {args.csv} ...")
    data = load(args.csv)

    if args.start_time is not None or args.duration is not None:
        t_all = data[TIME_COL]
        t_start = args.start_time if args.start_time is not None else float(t_all[0])
        duration = args.duration if args.duration is not None else float(t_all[-1]) - t_start
        try:
            data = slice_window(data, t_start, duration)
        except ValueError as e:
            sys.exit(str(e))
        title_parts.append(f"t={t_start:.1f}s + {duration:.1f}s")
        print(f"Window: t={t_start:.3f} s → {t_start + duration:.3f} s  ({len(data[TIME_COL])} samples)")

    data = apply_augmentation(data, args.linear_scale, args.angular_scale, overlay,
                              fade_in_duration=args.fade_in_duration)

    has_offset = any(v != 0.0 for v in (args.offset_x, args.offset_y, args.offset_z))
    if has_offset:
        data = apply_rigid_body_offset(data, args.offset_x, args.offset_y, args.offset_z)
        title_parts.append(f"offset ({args.offset_x:.3g}, {args.offset_y:.3g}, {args.offset_z:.3g})")

    print_stats(data, pos_unit, rot_unit)
    print("\nRaw velocity stats:")
    print_rate_stats(data, pos_unit, rot_unit)

    smoothed_data = None
    if args.smooth_linear > 0 or args.smooth_angular > 0:
        smoothed_data = apply_smoothing(data, args.smooth_linear, args.smooth_angular)
        title_parts.append(f"smooth lin={args.smooth_linear:.3g} ang={args.smooth_angular:.3g}")
        print(f"\nSmoothed (linear s={args.smooth_linear:.3g}, angular s={args.smooth_angular:.3g}) — sampled at original timestamps:")
        print_stats(smoothed_data, pos_unit, rot_unit)
        print("\nSmoothed velocity stats:")
        print_rate_stats(smoothed_data, pos_unit, rot_unit)

    plot(data, pos_unit, rot_unit, title_suffix=", ".join(title_parts),
         smoothed_data=smoothed_data, save_path=args.save)

    if args.fourier:
        psd_results, fs = compute_psd(data)
        print_psd_stats(psd_results, fs)
        plot_psd(psd_results, fs, lpf_cutoff=args.lpf_cutoff, save_path=args.save_fourier)


if __name__ == "__main__":
    import sys as _sys
    _DEFAULTS = [
        "--csv", "ship_emulation/1_vessel_motion_com_1m_translated.csv",
        #"--start-time", "2184",   # analyse the phase-2 emulation window
        #"--duration",   "180",
        #"--linear-scale", "1000", # m → mm
        #"--angular-scale", "1.0",
        #"--offset-x", "0.0", # forward of COG (bow direction, total length 103 m)
        #"--offset-y", "8.0", # side of ship (beam direction, total 16 m)
        #"--offset-z", "8.0", # vertical offset (total height of ship, 16 m, draft 6.7 m)
        # "--overlay-roll-amp", "10.0",
        # "--overlay-roll-freq", "0.1",
        # "--overlay-pitch-amp", "15.0",
        # "--overlay-pitch-freq", "0.07",
        # "--overlay-yaw-amp", "0.0",
        # "--overlay-yaw-freq", "0.1",
        # "--fade-in-duration", "10",   # match phase2 FADE_IN_DURATION
        # "--smooth-linear", "0.0", # 0 removes smoothing
        # "--fourier",
        # "--lpf-cutoff", "0.5",
        # "--smooth-angular", "1e2",
        # "--save", "vessel_motion.png",
        # "--save-fourier", "vessel_motion_psd.png",
    ]
    main(_sys.argv[1:] or _DEFAULTS)
