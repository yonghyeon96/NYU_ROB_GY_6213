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
import time as _time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import parameters
import data_handling
import motion_models
import map_builder
import feature_extraction
import icp
import wp_icp
from icp import _angle_wrap


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _draw_map_walls(ax):
    for seg in parameters.wall_corner_list:
        x1, y1, x2, y2 = seg
        ax.plot([x1, x2], [y1, y2], 'k-', lw=2, zorder=1)


def _pos_error(pose_a, pose_b):
    return math.sqrt((pose_a[0] - pose_b[0])**2 + (pose_a[1] - pose_b[1])**2)


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
    icp_errors, wp_errors                 = [], []
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
    print(f"{'Method':<18} {'Converged':>10} {'Dropped':>8}")
    print(f"{'Vanilla ICP':<18} {icp_converge:>10}/{n-1}  {dropped_icp:>6}")
    print(f"{'WP-ICP':<18} {wp_converge:>10}/{n-1}  {dropped_wp:>6}")

    icp_arr = np.array(icp_traj)
    wp_arr  = np.array(wp_traj)
    dr_arr  = np.array(dr_traj)

    # discrepancy vs ICP as proxy reference
    icp_wp_disc = [_pos_error(icp_arr[k], wp_arr[k]) for k in range(len(icp_arr))]
    dr_icp_disc = [_pos_error(dr_arr[k],  icp_arr[k]) for k in range(len(icp_arr))]

    # ---- plots ----
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(f'Offline Analysis — {os.path.basename(filename)}', fontsize=13, y=0.98)

    # --- 1. Trajectory ---
    ax1 = fig.add_subplot(2, 3, 1)
    _draw_map_walls(ax1)
    ax1.plot(dr_arr[:,0],  dr_arr[:,1],  'g--', lw=1.2, alpha=0.8, label='Dead Reckoning')
    ax1.plot(icp_arr[:,0], icp_arr[:,1], 'b-',  lw=1.5, label='Vanilla ICP')
    ax1.plot(wp_arr[:,0],  wp_arr[:,1],  'r-',  lw=1.5, label='WP-ICP')
    ax1.plot(*initial_pose[:2], 'ko', ms=8, label='start')
    ax1.set_aspect('equal'); ax1.legend(fontsize=7); ax1.set_title('Trajectory')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.grid(True, alpha=0.3)

    # --- 2. Position discrepancy ---
    ax2 = fig.add_subplot(2, 3, 2)
    ts = [rows[k][0] - rows[0][0] for k in range(len(rows))]
    ax2.plot(ts, dr_icp_disc, 'g--', lw=1.2, alpha=0.8, label='|DR - ICP|')
    ax2.plot(ts, icp_wp_disc, 'r-',  lw=1.2, label='|ICP - WP-ICP|')
    ax2.set_xlabel('time (s)'); ax2.set_ylabel('position diff (m)')
    ax2.set_title('Position Discrepancy vs ICP'); ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

    # --- 3. ICP iterations ---
    ax3 = fig.add_subplot(2, 3, 3)
    bins = range(0, parameters.icp_max_iterations + 2)
    ax3.hist(icp_iters,  bins=bins, alpha=0.6, color='blue',   label=f'Vanilla ICP (μ={np.mean(icp_iters):.1f})')
    ax3.hist(np.array(wp_c_iters) + np.array(wp_l_iters),
             bins=bins, alpha=0.6, color='red',
             label=f'WP-ICP total (μ={np.mean(np.array(wp_c_iters)+np.array(wp_l_iters)):.1f})')
    ax3.set_xlabel('iterations'); ax3.set_ylabel('count')
    ax3.set_title('ICP Iteration Histogram'); ax3.legend(fontsize=7); ax3.grid(True, alpha=0.3)

    # --- 4. Runtime ---
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(ts[1:], np.array(icp_runtime)*1000,  'b-',  lw=1.0, label='Vanilla ICP')
    ax4.plot(ts[1:], np.array(wp_runtime)*1000,   'r-',  lw=1.0, label='WP-ICP')
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

    out_dir  = 'plots'
    os.makedirs(out_dir, exist_ok=True)
    base     = os.path.splitext(os.path.basename(filename))[0]
    out_path = os.path.join(out_dir, f'offline_{base}.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved: {out_path}")

    # ---- summary table ----
    print(f"\n{'='*60}")
    print(f"  Summary")
    print(f"{'='*60}")
    print(f"  Timesteps          : {n}")
    valid_icp  = [v for v in icp_iters  if v > 0]
    valid_wpc  = [v for v in wp_c_iters if v > 0]
    valid_wpl  = [v for v in wp_l_iters if v > 0]
    valid_err  = [v for v in icp_errors if math.isfinite(v)]
    print(f"  ICP mean iters     : {np.mean(valid_icp):.1f}"  if valid_icp  else "  ICP mean iters     : N/A")
    print(f"  WP corner iters    : {np.mean(valid_wpc):.1f}"  if valid_wpc  else "  WP corner iters    : N/A")
    print(f"  WP line iters      : {np.mean(valid_wpl):.1f}"  if valid_wpl  else "  WP line iters      : N/A")
    print(f"  ICP mean error (m) : {np.mean(valid_err):.4f}" if valid_err  else "  ICP mean error     : N/A")
    print(f"  ICP  runtime (ms)  : {np.mean(icp_runtime)*1000:.2f} ± {np.std(icp_runtime)*1000:.2f}")
    print(f"  WP   runtime (ms)  : {np.mean(wp_runtime)*1000:.2f} ± {np.std(wp_runtime)*1000:.2f}")
    print(f"  Avg Γ_C            : {np.mean(wp_gamma_c):.3f}")
    print(f"  Avg Γ_L            : {np.mean(wp_gamma_l):.3f}")
    print(f"  Final DR  pose     : [{dr_arr[-1,0]:.3f}, {dr_arr[-1,1]:.3f}, {dr_arr[-1,2]:.3f}]")
    print(f"  Final ICP pose     : [{icp_arr[-1,0]:.3f}, {icp_arr[-1,1]:.3f}, {icp_arr[-1,2]:.3f}]")
    print(f"  Final WP  pose     : [{wp_arr[-1,0]:.3f}, {wp_arr[-1,1]:.3f}, {wp_arr[-1,2]:.3f}]")

    return {
        'dr_traj': dr_traj, 'icp_traj': icp_traj, 'wp_traj': wp_traj,
        'icp_iters': icp_iters, 'wp_c_iters': wp_c_iters, 'wp_l_iters': wp_l_iters,
        'icp_runtime': icp_runtime, 'wp_runtime': wp_runtime,
        'wp_gamma_c': wp_gamma_c, 'wp_gamma_l': wp_gamma_l,
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
