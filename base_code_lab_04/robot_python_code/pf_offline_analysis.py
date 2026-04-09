import math
import random
import sys
import types
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Optional dependency stubs so offline analysis can run without hardware packages.
if "serial" not in sys.modules:
    try:
        import serial  # type: ignore  # noqa: F401
    except Exception:
        sys.modules["serial"] = types.ModuleType("serial")

if "cv2" not in sys.modules:
    try:
        import cv2  # type: ignore  # noqa: F401
    except Exception:
        cv2_stub = types.ModuleType("cv2")
        aruco_stub = types.ModuleType("aruco")
        setattr(cv2_stub, "aruco", aruco_stub)
        sys.modules["cv2"] = cv2_stub
        sys.modules["cv2.aruco"] = aruco_stub

import data_handling
import parameters
import particle_filter
import robot_python_code


@dataclass
class PFRunResult:
    name: str
    time: np.ndarray
    mean_states: np.ndarray  # shape: (N, 3), columns = x, y, theta
    particle_snapshots: dict  # key: time index, value: np.ndarray shape (num_particles, 2)


def draw_map(ax, map_obj):
    for wall in map_obj.wall_list:
        ax.plot([wall.corner1.x, wall.corner2.x], [wall.corner1.y, wall.corner2.y], "k", linewidth=1.2)
    ax.set_xlim(map_obj.plot_range[0], map_obj.plot_range[1])
    ax.set_ylim(map_obj.plot_range[2], map_obj.plot_range[3])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")


def to_particle_xy(particle_set):
    x = [p.state.x for p in particle_set.particle_list]
    y = [p.state.y for p in particle_set.particle_list]
    return np.column_stack([x, y]) if len(x) > 0 else np.zeros((0, 2))


def choose_snapshot_indices(n_steps, n_snapshots):
    if n_steps <= 0:
        return []
    if n_steps <= n_snapshots:
        return list(range(n_steps))
    idx = np.linspace(0, n_steps - 1, n_snapshots)
    return sorted({int(round(v)) for v in idx})


def run_pf_on_log(
    filename,
    run_name,
    use_correction=True,
    known_start_state=True,
    initial_state=particle_filter.State(0.68,0.53,0.0),
    initial_stdev=particle_filter.State(0.1, 0.1, 0.1),
    seed=1,
    n_snapshot_samples=8,
):
    random.seed(seed)
    np.random.seed(seed)

    map_obj = particle_filter.Map(parameters.wall_corner_list)
    pf_data = data_handling.get_file_data_for_pf(filename)
    if len(pf_data) < 2:
        raise ValueError("PF log must contain at least 2 rows.")

    pf = particle_filter.ParticleFilter(
        parameters.num_particles,
        map_obj,
        initial_state=initial_state,
        state_stdev=initial_stdev,
        known_start_state=known_start_state,
        encoder_counts_0=pf_data[0][2].encoder_counts,
    )

    time = [pf_data[0][0]]
    means = [[pf.state_estimate.x, pf.state_estimate.y, pf.state_estimate.theta]]
    snapshots = {}

    snapshot_indices = set(choose_snapshot_indices(len(pf_data), n_snapshot_samples))
    if 0 in snapshot_indices:
        snapshots[0] = to_particle_xy(pf.particle_set)

    for k in range(1, len(pf_data)):
        row = pf_data[k]
        delta_t = pf_data[k][0] - pf_data[k - 1][0]
        u_t = np.array([row[2].encoder_counts, row[2].steering])
        z_t = row[2]

        if use_correction:
            pf.update(u_t, z_t, delta_t)
        else:
            # Prediction-only path for Part 4 debugging.
            pf.prediction(u_t, delta_t)
            pf.particle_set.update_mean_state()
            pf.state_estimate = pf.particle_set.mean_state
            pf.state_estimate_list.append(pf.state_estimate.deepcopy())

        time.append(pf_data[k][0])
        means.append([pf.state_estimate.x, pf.state_estimate.y, pf.state_estimate.theta])
        if k in snapshot_indices:
            snapshots[k] = to_particle_xy(pf.particle_set)

    return PFRunResult(
        name=run_name,
        time=np.asarray(time, dtype=float),
        mean_states=np.asarray(means, dtype=float),
        particle_snapshots=snapshots,
    )


def try_extract_camera_truth_xy(filename, expected_len):
    try:
        data_dict = robot_python_code.DataLoader(filename).load()
    except Exception:
        return None, None

    camera_list = data_dict.get("camera_sensor_signal", [])
    if len(camera_list) != expected_len:
        return None, None

    truth_xy = np.full((expected_len, 2), np.nan, dtype=float)
    valid = np.zeros(expected_len, dtype=bool)
    for i, row in enumerate(camera_list):
        if not isinstance(row, (list, tuple)) or len(row) < 2:
            continue
        x = float(row[0])
        y = float(row[1])
        # In this codebase, [0, 0, ...] is usually an invalid/no-camera placeholder.
        if abs(x) < 1e-12 and abs(y) < 1e-12:
            continue
        truth_xy[i, 0] = x
        truth_xy[i, 1] = y
        valid[i] = True

    if np.sum(valid) < 5:
        return None, None
    return truth_xy, valid


def plot_xy_trajectories(
    map_obj,
    runs,
    truth_xy=None,
    truth_valid=None,
    manual_truth_enabled=False,
    manual_truth_start=None,
    manual_truth_end=None,
    manual_truth_label="manual truth line",
):
    fig, ax = plt.subplots(figsize=(7, 6))
    draw_map(ax, map_obj)

    for run in runs:
        ax.plot(run.mean_states[:, 0], run.mean_states[:, 1], linewidth=2.0, label=run.name)
        ax.plot(run.mean_states[0, 0], run.mean_states[0, 1], "o", markersize=5)
        ax.plot(run.mean_states[-1, 0], run.mean_states[-1, 1], "s", markersize=5)

    if truth_xy is not None and truth_valid is not None and np.any(truth_valid):
        ax.plot(truth_xy[truth_valid, 0], truth_xy[truth_valid, 1], "k--", linewidth=1.5, label="camera truth proxy")

    # Manual ground truth proxy for straight-line tests.
    if manual_truth_enabled and manual_truth_start is not None and manual_truth_end is not None:
        # Use the shortest run length so the reference line aligns with plotted estimates.
        N = min(run.mean_states.shape[0] for run in runs)
        tx = np.linspace(manual_truth_start[0], manual_truth_end[0], N)
        ty = np.linspace(manual_truth_start[1], manual_truth_end[1], N)
        ax.plot(tx, ty, linestyle="--", color="0.7", linewidth=2.0, label=manual_truth_label)

    ax.set_title("PF Offline XY Trajectories")
    ax.legend()
    fig.tight_layout()

def plot_particle_cloud_overlay(map_obj, run_result, max_snapshots=6):
    fig, ax = plt.subplots(figsize=(7, 6))
    draw_map(ax, map_obj)

    indices = sorted(run_result.particle_snapshots.keys())
    if len(indices) > max_snapshots:
        keep = choose_snapshot_indices(len(indices), max_snapshots)
        indices = [indices[i] for i in keep]

    colors = plt.cm.viridis(np.linspace(0.15, 0.95, max(1, len(indices))))
    for i, idx in enumerate(indices):
        pts = run_result.particle_snapshots[idx]
        if pts.size == 0:
            continue
        ax.scatter(pts[:, 0], pts[:, 1], s=8, alpha=0.16, color=colors[i], label=f"cloud@k={idx}")

    ax.plot(
        run_result.mean_states[:, 0],
        run_result.mean_states[:, 1],
        color="tab:red",
        linewidth=2.0,
        label=f"{run_result.name} mean",
    )
    ax.set_title(f"Particle Cloud Overlay ({run_result.name})")
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()


def plot_error_vs_time(runs):
    fig, ax = plt.subplots(figsize=(8, 4))
    if len(runs) < 2:
        ax.text(0.5, 0.5, "Need at least 2 runs for discrepancy plot.", ha="center", va="center")
        ax.set_axis_off()
        return

    base = runs[0]
    compare = runs[1]
    n = min(len(base.time), len(compare.time))
    dx = compare.mean_states[:n, 0] - base.mean_states[:n, 0]
    dy = compare.mean_states[:n, 1] - base.mean_states[:n, 1]
    e_compare = np.sqrt(dx * dx + dy * dy)
    ax.plot(
        base.time[:n],
        e_compare,
        linewidth=2.2,
        color="tab:blue",
        label=f"{compare.name} discrepancy to {base.name}",
    )
    ax.set_title("Estimator Discrepancy vs Time")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Discrepancy (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()


def plot_error_metrics_table(runs):
    fig, ax = plt.subplots(figsize=(8, 2.2))
    ax.axis("off")

    if len(runs) < 2:
        ax.text(0.5, 0.5, "Need at least 2 runs for metrics table.", ha="center", va="center")
        return

    base = runs[0]
    compare = runs[1]
    n = min(len(base.time), len(compare.time))
    dx = compare.mean_states[:n, 0] - base.mean_states[:n, 0]
    dy = compare.mean_states[:n, 1] - base.mean_states[:n, 1]
    e = np.sqrt(dx * dx + dy * dy)

    rmse = float(np.sqrt(np.mean(e * e)))
    mean_e = float(np.mean(e))
    max_e = float(np.max(e))

    row_label = f"{compare.name} vs {base.name}"
    table = ax.table(
        cellText=[[f"{rmse:.4f}", f"{mean_e:.4f}", f"{max_e:.4f}"]],
        rowLabels=[row_label],
        colLabels=["RMSE (m)", "Mean Error (m)", "Max Error (m)"],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.5)
    ax.set_title("Error Metrics Table", pad=10)
    fig.tight_layout()


def print_run_summary(run):
    x0, y0 = run.mean_states[0, 0], run.mean_states[0, 1]
    x1, y1 = run.mean_states[-1, 0], run.mean_states[-1, 1]
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    dtheta = particle_filter.angle_wrap(run.mean_states[-1, 2] - run.mean_states[0, 2])
    print(f"[{run.name}]")
    print(f"  start: ({x0:.4f}, {y0:.4f}, {run.mean_states[0,2]:.4f})")
    print(f"  end:   ({x1:.4f}, {y1:.4f}, {run.mean_states[-1,2]:.4f})")
    print(f"  travel distance (m): {d:.4f}")
    print(f"  delta theta (rad):   {dtheta:.4f}")


if __name__ == "__main__":
    # Change to your own trial file(s) as you collect data.
    data_file = "./data/new_kidnapped.pkl"

    # Manual truth line overlay toggle.
    # Set True for the datasets where you have hand-measured start/end points.
    use_manual_truth_line = False
    manual_truth_start = (0.68, 0.53)  # (x, y) in meters
    manual_truth_end = (1.88, 0.53)    # (x, y) in meters
    manual_truth_label = "manual truth (measured start-end)"

    # Optional per-file auto configuration:
    # manual_truth_by_file = {
    #     "robot_data_35_0_05_03_26_18_08_09.pkl": ((0.68, 0.53), (1.88, 0.53)),
    # }
    manual_truth_by_file = {}

    if not Path(data_file).exists():
        raise FileNotFoundError(f"Data file not found: {data_file}")

    data_filename = Path(data_file).name
    if data_filename in manual_truth_by_file:
        use_manual_truth_line = True
        manual_truth_start, manual_truth_end = manual_truth_by_file[data_filename]

    map_obj = particle_filter.Map(parameters.wall_corner_list)

    pred_only = run_pf_on_log(
        filename=data_file,
        run_name="Prediction-only",
        use_correction=False,
        known_start_state=True,
        seed=1,
    )
    full_pf = run_pf_on_log(
        filename=data_file,
        run_name="Full PF (prediction+correction)",
        use_correction=True,
        known_start_state=True,
            seed=1,
    )

    print_run_summary(pred_only)
    print_run_summary(full_pf)

    truth_xy, truth_valid = try_extract_camera_truth_xy(data_file, len(full_pf.time))
    if truth_xy is None:
        print("No usable camera truth found in log. XY plot will skip camera truth overlay.")

    plot_xy_trajectories(
        map_obj,
        [pred_only, full_pf],
        truth_xy=truth_xy,
        truth_valid=truth_valid,
        manual_truth_enabled=use_manual_truth_line,
        manual_truth_start=manual_truth_start,
        manual_truth_end=manual_truth_end,
        manual_truth_label=manual_truth_label,
    )
    plot_particle_cloud_overlay(map_obj, full_pf, max_snapshots=6)
    plot_error_vs_time([pred_only, full_pf])
    plot_error_metrics_table([pred_only, full_pf])

    plt.show()
