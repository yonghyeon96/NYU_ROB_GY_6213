"""
offline_analysis.py
====================
Replay a recorded .pkl log and compare:
  - Dead Reckoning
  - Vanilla ICP
  - WP-ICP

Usage
-----
  python offline_analysis.py <path_to_pkl> <x0> <y0> <theta0>

  e.g.
  python offline_analysis.py ./data/straight_trial1.pkl 0.30 0.30 0.0
"""

import sys
import os
import math
import random
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import parameters
import data_handling
import motion_models
import map_builder
import feature_extraction
import icp
import wp_icp


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _draw_map_walls(ax):
    for seg in parameters.wall_corner_list:
        x1, y1, x2, y2 = seg
        ax.plot([x1, x2], [y1, y2], 'k-', lw=2, zorder=1)


def _pos_error(pose_a, pose_b):
    return math.sqrt((pose_a[0] - pose_b[0])**2 + (pose_a[1] - pose_b[1])**2)


def _valid_values(values):
    return [float(v) for v in values if math.isfinite(float(v))]


def _positive_values(values):
    return [float(v) for v in values if math.isfinite(float(v)) and float(v) > 0.0]


def _mean_or_nan(values):
    values = _valid_values(values)
    return float(np.mean(values)) if values else float('nan')


def _std_or_nan(values):
    values = _valid_values(values)
    return float(np.std(values)) if values else float('nan')


def _percentile_or_nan(values, percentile):
    values = _valid_values(values)
    return float(np.percentile(values, percentile)) if values else float('nan')


def _fmt(value, decimals=3):
    return f"{value:.{decimals}f}" if math.isfinite(float(value)) else "N/A"


def _fmt_pose(pose):
    return f"[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]"


def _save_fig(fig, out_dir, filename):
    path = os.path.join(out_dir, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def _write_summary_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'dead_reckoning', 'standard_icp', 'wp_icp'])
        for row in rows:
            writer.writerow(row)


def _write_summary_md(path, rows):
    with open(path, 'w') as f:
        f.write('| Metric | Dead Reckoning | Standard ICP | WP-ICP |\n')
        f.write('|--------|----------------|--------------|--------|\n')
        for metric, dr_value, icp_value, wp_value in rows:
            f.write(f'| {metric} | {dr_value} | {icp_value} | {wp_value} |\n')


def _write_timeseries_csv(path, ts, dr_icp_disc, icp_wp_disc,
                          icp_iters, wp_c_iters, wp_l_iters,
                          icp_runtime_ms, wp_runtime_ms,
                          wp_gamma_c, wp_gamma_l):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_s',
            'dr_to_icp_discrepancy_m',
            'icp_to_wp_icp_discrepancy_m',
            'standard_icp_iterations',
            'wp_icp_corner_iterations',
            'wp_icp_line_iterations',
            'wp_icp_total_iterations',
            'standard_icp_runtime_ms',
            'wp_icp_runtime_ms',
            'wp_icp_gamma_c',
            'wp_icp_gamma_l',
        ])
        for k in range(1, len(ts)):
            writer.writerow([
                f'{ts[k]:.6f}',
                f'{dr_icp_disc[k]:.6f}',
                f'{icp_wp_disc[k]:.6f}',
                icp_iters[k - 1],
                wp_c_iters[k - 1],
                wp_l_iters[k - 1],
                wp_c_iters[k - 1] + wp_l_iters[k - 1],
                f'{icp_runtime_ms[k - 1]:.6f}',
                f'{wp_runtime_ms[k - 1]:.6f}',
                f'{wp_gamma_c[k - 1]:.6f}',
                f'{wp_gamma_l[k - 1]:.6f}',
            ])


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(filename, initial_pose):
    print(f"\n{'='*60}")
    print(f"  File         : {filename}")
    print(f"  Initial pose : {initial_pose}")
    print(f"{'='*60}\n")

    # ---- load data ----
    rows = data_handling.get_replay_data(filename)
    print(f"Loaded {len(rows)} timesteps.")

    if len(rows) < 2:
        print("Not enough data.")
        return

    # ---- build map ----
    map_feats = map_builder.build_map_features(
        parameters.wall_corner_list, parameters.map_interpolation_step
    )
    print(f"Map: {len(map_feats.map_corners)} corners, "
          f"{len(map_feats.map_line_points)} line pts, "
          f"{map_feats.full_tree.n} full-tree pts\n")

    # ---- initialise motion model (noise disabled for deterministic replay) ----
    random.seed(0)
    init_encoder = rows[0][2].encoder_counts
    dr_model  = motion_models.MyMotionModel(list(initial_pose), init_encoder)
    icp_model = motion_models.MyMotionModel(list(initial_pose), init_encoder)
    wp_model  = motion_models.MyMotionModel(list(initial_pose), init_encoder)

    # ---- storage ----
    dr_traj,  icp_traj,  wp_traj   = [list(initial_pose)], [list(initial_pose)], [list(initial_pose)]
    icp_pose  = list(initial_pose)
    wp_pose   = list(initial_pose)

    icp_iters,  wp_c_iters,  wp_l_iters  = [], [], []
    icp_runtime, wp_runtime               = [], []
    icp_errors                            = []
    wp_gamma_c,  wp_gamma_l               = [], []
    icp_converge, wp_converge             = 0, 0
    dropped_icp,  dropped_wp             = 0, 0

    # ---- replay loop ----
    for i in range(1, len(rows)):
        t_cur,  ctrl_cur,  sensor_cur  = rows[i]
        t_prev, _,         _           = rows[i - 1]
        delta_t = max(t_cur - t_prev, 1e-3)

        enc = sensor_cur.encoder_counts
        steer = ctrl_cur[1]

        # --- Dead Reckoning ---
        dr_pose = dr_model.step_update(enc, steer, delta_t, use_noise=False)
        dr_traj.append(list(dr_pose))

        # --- warm start from motion model (ICP branch) ---
        icp_warm = icp_model.step_update(enc, steer, delta_t, use_noise=False)

        # --- scan points ---
        scan_pts, _, _ = feature_extraction.lidar_to_sensor_frame(sensor_cur)

        # =================== Vanilla ICP ===================
        if len(scan_pts) == 0:
            dropped_icp += 1
            icp_traj.append(list(icp_pose))
            icp_iters.append(0)
            icp_runtime.append(0.0)
            icp_errors.append(float('nan'))
        else:
            res_icp = icp.update(scan_pts, icp_warm, map_feats)
            if res_icp.converged:
                icp_pose = res_icp.pose
                icp_model.state = list(icp_pose)   # sync model to ICP correction
                icp_converge += 1
            else:
                icp_pose = list(icp_warm)
                icp_model.state = list(icp_pose)
            icp_traj.append(list(icp_pose))
            icp_iters.append(res_icp.iterations)
            icp_runtime.append(res_icp.runtime_sec)
            icp_errors.append(res_icp.mean_error)

        # =================== WP-ICP ===================
        wp_warm = wp_model.step_update(enc, steer, delta_t, use_noise=False)

        if len(scan_pts) == 0:
            dropped_wp += 1
            wp_traj.append(list(wp_pose))
            wp_c_iters.append(0); wp_l_iters.append(0)
            wp_runtime.append(0.0)
            wp_gamma_c.append(0.0); wp_gamma_l.append(0.0)
        else:
            fs = feature_extraction.extract_features(sensor_cur)
            res_wp = wp_icp.update(fs, wp_warm, map_feats)
            if res_wp.converged:
                wp_pose = res_wp.pose
                wp_model.state = list(wp_pose)
                wp_converge += 1
            else:
                wp_pose = list(wp_warm)
                wp_model.state = list(wp_pose)
            wp_traj.append(list(wp_pose))
            wp_c_iters.append(res_wp.corner_iterations)
            wp_l_iters.append(res_wp.line_iterations)
            wp_runtime.append(res_wp.runtime_sec)
            wp_gamma_c.append(res_wp.corner_confidence)
            wp_gamma_l.append(res_wp.line_confidence)

    n = len(rows)
    trial_steps = n - 1
    print(f"{'Method':<32} {'Converged':>10} {'Dropped':>8}")
    print(f"{'Standard ICP (Vanilla ICP)':<32} {icp_converge:>10}/{trial_steps}  {dropped_icp:>6}")
    print(f"{'WP-ICP':<32} {wp_converge:>10}/{trial_steps}  {dropped_wp:>6}")

    icp_arr = np.array(icp_traj)
    wp_arr  = np.array(wp_traj)
    dr_arr  = np.array(dr_traj)
    ts = [rows[k][0] - rows[0][0] for k in range(len(rows))]

    # discrepancy vs ICP as proxy reference
    icp_wp_disc = [_pos_error(icp_arr[k], wp_arr[k]) for k in range(len(icp_arr))]
    dr_icp_disc = [_pos_error(dr_arr[k],  icp_arr[k]) for k in range(len(icp_arr))]
    wp_total_iters = np.array(wp_c_iters) + np.array(wp_l_iters)
    icp_runtime_ms = np.array(icp_runtime) * 1000.0
    wp_runtime_ms = np.array(wp_runtime) * 1000.0

    out_dir  = 'plots'
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(filename))[0]
    report_dir = os.path.join(out_dir, base)
    os.makedirs(report_dir, exist_ok=True)

    # ---- plots ----
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(f'Offline Analysis - {os.path.basename(filename)}', fontsize=13, y=0.98)

    # --- 1. Trajectory ---
    ax1 = fig.add_subplot(2, 3, 1)
    _draw_map_walls(ax1)
    ax1.plot(dr_arr[:,0],  dr_arr[:,1],  'g--', lw=1.2, alpha=0.8, label='Dead Reckoning')
    ax1.plot(icp_arr[:,0], icp_arr[:,1], 'b-',  lw=1.5, label='Standard ICP')
    ax1.plot(wp_arr[:,0],  wp_arr[:,1],  'r-',  lw=1.5, label='WP-ICP')
    ax1.plot(*initial_pose[:2], 'ko', ms=8, label='start')
    ax1.set_aspect('equal'); ax1.legend(fontsize=7); ax1.set_title('Trajectory')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.grid(True, alpha=0.3)

    # --- 2. Position discrepancy ---
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(ts, dr_icp_disc, 'g--', lw=1.2, alpha=0.8, label='|DR - ICP|')
    ax2.plot(ts, icp_wp_disc, 'r-',  lw=1.2, label='|ICP - WP-ICP|')
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('position diff (m)')
    ax2.set_title('Position Discrepancy vs ICP'); ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

    # --- 3. ICP iterations ---
    ax3 = fig.add_subplot(2, 3, 3)
    bins = range(0, parameters.icp_max_iterations + 2)
    ax3.hist(icp_iters, bins=bins, alpha=0.6, color='blue',
             label=f'Standard ICP (mean={np.mean(icp_iters):.1f})')
    ax3.hist(wp_total_iters, bins=bins, alpha=0.6, color='red',
             label=f'WP-ICP total (mean={np.mean(wp_total_iters):.1f})')
    ax3.set_xlabel('iterations'); ax3.set_ylabel('count')
    ax3.set_title('ICP Iteration Histogram'); ax3.legend(fontsize=7); ax3.grid(True, alpha=0.3)

    # --- 4. Runtime ---
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(ts[1:], icp_runtime_ms, 'b-', lw=1.0, label='Standard ICP')
    ax4.plot(ts[1:], wp_runtime_ms,  'r-', lw=1.0, label='WP-ICP')
    ax4.set_xlabel('time (s)'); ax4.set_ylabel('runtime (ms)')
    ax4.set_title('Runtime per Scan'); ax4.legend(fontsize=7); ax4.grid(True, alpha=0.3)

    # --- 5. WP-ICP confidence ---
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(ts[1:], wp_gamma_c, 'orange', lw=1.2, label='Γ_C (corner)')
    ax5.plot(ts[1:], wp_gamma_l, 'steelblue', lw=1.2, label='Γ_L (line)')
    ax5.set_ylim(-0.05, 1.05)
    ax5.set_xlabel('time (s)'); ax5.set_ylabel('confidence')
    ax5.set_title('WP-ICP Confidence'); ax5.legend(fontsize=7); ax5.grid(True, alpha=0.3)

    # --- 6. WP-ICP corner vs line iterations ---
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(ts[1:], wp_c_iters, 'orange',    lw=1.0, label='corner iters')
    ax6.plot(ts[1:], wp_l_iters, 'steelblue', lw=1.0, label='line iters')
    ax6.set_xlabel('time (s)'); ax6.set_ylabel('iterations')
    ax6.set_title('WP-ICP Corner vs Line Iterations'); ax6.legend(fontsize=7); ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    out_path = os.path.join(out_dir, f'offline_{base}.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    overview_path = os.path.join(report_dir, 'offline_overview.png')
    fig.savefig(overview_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"\nOverview plot saved: {out_path}")
    print(f"Report overview saved: {overview_path}")

    # ---- report-ready individual figures ----
    fig_traj, ax = plt.subplots(figsize=(7, 6))
    _draw_map_walls(ax)
    ax.plot(dr_arr[:,0], dr_arr[:,1], 'g--', lw=1.3, alpha=0.8, label='Dead Reckoning')
    ax.plot(icp_arr[:,0], icp_arr[:,1], 'b-', lw=1.6, label='Standard ICP')
    ax.plot(wp_arr[:,0], wp_arr[:,1], 'r-', lw=1.6, label='WP-ICP')
    ax.plot(*initial_pose[:2], 'ko', ms=7, label='start')
    ax.set_aspect('equal'); ax.set_title('Trajectory Comparison')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    traj_path = _save_fig(fig_traj, report_dir, 'trajectory_comparison.png')

    fig_disc, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts, dr_icp_disc, 'g--', lw=1.2, alpha=0.8, label='|Dead Reckoning - Standard ICP|')
    ax.plot(ts, icp_wp_disc, 'r-', lw=1.2, label='|Standard ICP - WP-ICP|')
    ax.set_title('Position Discrepancy')
    ax.set_xlabel('time (s)'); ax.set_ylabel('discrepancy (m)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    disc_path = _save_fig(fig_disc, report_dir, 'position_discrepancy.png')

    fig_iter, ax = plt.subplots(figsize=(8, 4))
    ax.hist(icp_iters, bins=bins, alpha=0.6, color='blue',
            label=f'Standard ICP (mean={_mean_or_nan(icp_iters):.1f})')
    ax.hist(wp_total_iters, bins=bins, alpha=0.6, color='red',
            label=f'WP-ICP total (mean={_mean_or_nan(wp_total_iters):.1f})')
    ax.set_title('Iteration Histogram')
    ax.set_xlabel('iterations'); ax.set_ylabel('count')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    iter_path = _save_fig(fig_iter, report_dir, 'iteration_histogram.png')

    fig_runtime, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts[1:], icp_runtime_ms, 'b-', lw=1.0, label='Standard ICP')
    ax.plot(ts[1:], wp_runtime_ms, 'r-', lw=1.0, label='WP-ICP')
    ax.set_title('Runtime Per Scan')
    ax.set_xlabel('time (s)'); ax.set_ylabel('runtime (ms)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    runtime_path = _save_fig(fig_runtime, report_dir, 'runtime_per_scan.png')

    fig_conf, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts[1:], wp_gamma_c, 'orange', lw=1.2, label='corner confidence')
    ax.plot(ts[1:], wp_gamma_l, 'steelblue', lw=1.2, label='line confidence')
    ax.set_ylim(-0.05, 1.05)
    ax.set_title('WP-ICP Confidence')
    ax.set_xlabel('time (s)'); ax.set_ylabel('confidence')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    conf_path = _save_fig(fig_conf, report_dir, 'wp_icp_confidence.png')

    fig_feat_iter, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts[1:], wp_c_iters, 'orange', lw=1.0, label='corner iterations')
    ax.plot(ts[1:], wp_l_iters, 'steelblue', lw=1.0, label='line iterations')
    ax.set_title('WP-ICP Corner vs Line Iterations')
    ax.set_xlabel('time (s)'); ax.set_ylabel('iterations')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    feat_iter_path = _save_fig(fig_feat_iter, report_dir, 'wp_icp_corner_line_iterations.png')

    print("Report figures saved:")
    for path in [traj_path, disc_path, iter_path, runtime_path, conf_path, feat_iter_path]:
        print(f"  {path}")

    # ---- summary table ----
    valid_icp_iters = _positive_values(icp_iters)
    valid_wp_total_iters = _positive_values(wp_total_iters)
    valid_wpc = _positive_values(wp_c_iters)
    valid_wpl = _positive_values(wp_l_iters)
    valid_icp_runtime_ms = _positive_values(icp_runtime_ms)
    valid_wp_runtime_ms = _positive_values(wp_runtime_ms)
    valid_icp_err = _valid_values(icp_errors)
    icp_conv_rate = 100.0 * icp_converge / max(trial_steps, 1)
    wp_conv_rate = 100.0 * wp_converge / max(trial_steps, 1)
    mean_dr_icp_disc = _mean_or_nan(dr_icp_disc)
    mean_icp_wp_disc = _mean_or_nan(icp_wp_disc)
    final_dr_icp_disc = dr_icp_disc[-1]
    final_icp_wp_disc = icp_wp_disc[-1]
    min_total_conf = min((c + l for c, l in zip(wp_gamma_c, wp_gamma_l)), default=float('nan'))

    summary_rows = [
        ['Convergence rate', 'N/A', f'{icp_conv_rate:.1f} %', f'{wp_conv_rate:.1f} %'],
        ['Dropped scans', 'N/A', str(dropped_icp), str(dropped_wp)],
        ['Mean iterations', 'N/A', _fmt(_mean_or_nan(valid_icp_iters), 2), _fmt(_mean_or_nan(valid_wp_total_iters), 2)],
        ['Mean corner iterations', 'N/A', 'N/A', _fmt(_mean_or_nan(valid_wpc), 2)],
        ['Mean line iterations', 'N/A', 'N/A', _fmt(_mean_or_nan(valid_wpl), 2)],
        ['Mean runtime (ms/scan)', 'N/A', _fmt(_mean_or_nan(valid_icp_runtime_ms), 2), _fmt(_mean_or_nan(valid_wp_runtime_ms), 2)],
        ['Runtime std. dev. (ms)', 'N/A', _fmt(_std_or_nan(valid_icp_runtime_ms), 2), _fmt(_std_or_nan(valid_wp_runtime_ms), 2)],
        ['95th percentile runtime (ms)', 'N/A', _fmt(_percentile_or_nan(valid_icp_runtime_ms, 95), 2), _fmt(_percentile_or_nan(valid_wp_runtime_ms, 95), 2)],
        ['Mean discrepancy to Standard ICP (m)', _fmt(mean_dr_icp_disc, 4), '0.0000', _fmt(mean_icp_wp_disc, 4)],
        ['Final discrepancy to Standard ICP (m)', _fmt(final_dr_icp_disc, 4), '0.0000', _fmt(final_icp_wp_disc, 4)],
        ['Average confidence', 'N/A', 'N/A', f'corner={_fmt(_mean_or_nan(wp_gamma_c), 3)}, line={_fmt(_mean_or_nan(wp_gamma_l), 3)}'],
        ['Minimum total confidence', 'N/A', 'N/A', _fmt(min_total_conf, 3)],
        ['Mean ICP match error (m)', 'N/A', _fmt(_mean_or_nan(valid_icp_err), 4), 'N/A'],
        ['Final pose [x, y, theta]', _fmt_pose(dr_arr[-1]), _fmt_pose(icp_arr[-1]), _fmt_pose(wp_arr[-1])],
    ]

    summary_csv_path = os.path.join(report_dir, 'summary_table.csv')
    summary_md_path = os.path.join(report_dir, 'summary_table.md')
    timeseries_csv_path = os.path.join(report_dir, 'timeseries_metrics.csv')
    _write_summary_csv(summary_csv_path, summary_rows)
    _write_summary_md(summary_md_path, summary_rows)
    _write_timeseries_csv(
        timeseries_csv_path, ts, dr_icp_disc, icp_wp_disc,
        icp_iters, wp_c_iters, wp_l_iters,
        icp_runtime_ms, wp_runtime_ms, wp_gamma_c, wp_gamma_l,
    )

    print(f"\n{'='*60}")
    print(f"  Summary")
    print(f"{'='*60}")
    print(f"  Timesteps          : {n}")
    print(f"  Standard ICP convergence : {icp_conv_rate:.1f}% ({icp_converge}/{trial_steps})")
    print(f"  WP-ICP convergence       : {wp_conv_rate:.1f}% ({wp_converge}/{trial_steps})")
    print(f"  Standard ICP mean iters  : {_fmt(_mean_or_nan(valid_icp_iters), 2)}")
    print(f"  WP-ICP total iters       : {_fmt(_mean_or_nan(valid_wp_total_iters), 2)}")
    print(f"  WP corner iters          : {_fmt(_mean_or_nan(valid_wpc), 2)}")
    print(f"  WP line iters            : {_fmt(_mean_or_nan(valid_wpl), 2)}")
    print(f"  Standard ICP runtime     : {_fmt(_mean_or_nan(valid_icp_runtime_ms), 2)} +/- {_fmt(_std_or_nan(valid_icp_runtime_ms), 2)} ms")
    print(f"  WP-ICP runtime           : {_fmt(_mean_or_nan(valid_wp_runtime_ms), 2)} +/- {_fmt(_std_or_nan(valid_wp_runtime_ms), 2)} ms")
    print(f"  Standard ICP 95% runtime : {_fmt(_percentile_or_nan(valid_icp_runtime_ms, 95), 2)} ms")
    print(f"  WP-ICP 95% runtime       : {_fmt(_percentile_or_nan(valid_wp_runtime_ms, 95), 2)} ms")
    print(f"  Mean |DR - ICP|          : {_fmt(mean_dr_icp_disc, 4)} m")
    print(f"  Mean |ICP - WP-ICP|      : {_fmt(mean_icp_wp_disc, 4)} m")
    print(f"  Final |DR - ICP|         : {_fmt(final_dr_icp_disc, 4)} m")
    print(f"  Final |ICP - WP-ICP|     : {_fmt(final_icp_wp_disc, 4)} m")
    print(f"  Avg Gamma_C              : {_fmt(_mean_or_nan(wp_gamma_c), 3)}")
    print(f"  Avg Gamma_L              : {_fmt(_mean_or_nan(wp_gamma_l), 3)}")
    print(f"  Min Gamma_C + Gamma_L    : {_fmt(min_total_conf, 3)}")
    print(f"  Final DR  pose           : {_fmt_pose(dr_arr[-1])}")
    print(f"  Final ICP pose           : {_fmt_pose(icp_arr[-1])}")
    print(f"  Final WP  pose           : {_fmt_pose(wp_arr[-1])}")
    print(f"\nSummary CSV saved: {summary_csv_path}")
    print(f"Summary Markdown saved: {summary_md_path}")
    print(f"Timeseries CSV saved: {timeseries_csv_path}")

    return {
        'dr_traj': dr_traj, 'icp_traj': icp_traj, 'wp_traj': wp_traj,
        'icp_iters': icp_iters, 'wp_c_iters': wp_c_iters, 'wp_l_iters': wp_l_iters,
        'icp_runtime': icp_runtime, 'wp_runtime': wp_runtime,
        'wp_gamma_c': wp_gamma_c, 'wp_gamma_l': wp_gamma_l,
        'dr_icp_disc': dr_icp_disc, 'icp_wp_disc': icp_wp_disc,
        'summary_rows': summary_rows,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print(__doc__)
        print("  Example: python offline_analysis.py ./data/straight_trial1.pkl 0.30 0.30 0.0")
        sys.exit(1)

    pkl_path = sys.argv[1]
    x0, y0, theta0 = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

    if not os.path.exists(pkl_path):
        print(f"File not found: {pkl_path}")
        sys.exit(1)

    run(pkl_path, [x0, y0, theta0])
