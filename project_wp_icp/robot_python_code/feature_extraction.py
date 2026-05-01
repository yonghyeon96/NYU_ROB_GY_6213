import math
import numpy as np
from dataclasses import dataclass, field

import parameters


@dataclass
class FeatureSet:
    raw_points: np.ndarray           # (N, 2) sensor frame
    scan_ranges: np.ndarray          # (N,)
    scan_angles: np.ndarray          # (N,) radians, ascending
    clusters: list                   # list of np.ndarray (K, 2)
    corners: np.ndarray              # (Nc, 2) sensor frame
    line_points: np.ndarray          # (Nl, 2) sensor frame


# ---------------------------------------------------------------------------
# 2-1  Raw LiDAR -> sensor-frame points
# ---------------------------------------------------------------------------

def lidar_to_sensor_frame(robot_sensor_signal):
    """Return (scan_points, scan_ranges, scan_angles) in sensor frame, sorted by angle."""
    angles_hw = robot_sensor_signal.angles
    dists_hw  = robot_sensor_signal.distances

    pts, ranges, angles = [], [], []
    for a_hw, d_hw in zip(angles_hw, dists_hw):
        d = robot_sensor_signal.convert_hardware_distance(d_hw)
        theta = robot_sensor_signal.convert_hardware_angle(a_hw)

        if d < parameters.scan_min_range_m:
            continue
        if not math.isfinite(d) or not math.isfinite(theta):
            continue
        if d > parameters.lidar_max_range_m:
            continue

        pts.append([d * math.cos(theta), d * math.sin(theta)])
        ranges.append(d)
        angles.append(theta)

    if len(pts) == 0:
        empty = np.empty((0, 2))
        return empty, np.array([]), np.array([])

    order = np.argsort(angles)
    return (
        np.array(pts)[order],
        np.array(ranges)[order],
        np.array(angles)[order],
    )


# ---------------------------------------------------------------------------
# 2-2  ARC clustering  (paper Eq. 1-3)
# ---------------------------------------------------------------------------

def arc_clustering(scan_points, scan_ranges, scan_angles):
    """
    ΔS = R × (α × 2π/360),  R = min(r_i, r_{i-1})
    λ  = N × ΔS
    split when ‖p_i - p_{i-1}‖ > λ
    """
    if len(scan_points) == 0:
        return []

    alpha_rad = parameters.lidar_angular_resolution_deg * 2 * math.pi / 360
    N = parameters.arc_noise_scale_N
    min_size = parameters.arc_min_cluster_size

    clusters = []
    current = [0]

    for i in range(1, len(scan_points)):
        R = min(scan_ranges[i], scan_ranges[i - 1])
        lam = N * R * alpha_rad
        dist = np.linalg.norm(scan_points[i] - scan_points[i - 1])

        if dist > lam:
            clusters.append(current)
            current = [i]
        else:
            current.append(i)
    clusters.append(current)

    # discard small clusters (outliers)
    return [
        scan_points[idx]
        for idx in clusters
        if len(idx) >= min_size
    ]


# ---------------------------------------------------------------------------
# 2-3  Split & Merge  (paper Eq. 4)
# ---------------------------------------------------------------------------

def _line_coeffs(p1, pk):
    """
    Solve  [[x1,y1],[xk,yk]] @ [a,b] = [-1,-1]
    for line  ax + by + 1 = 0  passing through p1 and pk.
    Returns (a, b) or None if singular (identical / collinear edge case).
    """
    A = np.array([[p1[0], p1[1]],
                  [pk[0], pk[1]]], dtype=float)
    b = np.array([-1.0, -1.0])
    try:
        coeffs = np.linalg.solve(A, b)
        return coeffs[0], coeffs[1]
    except np.linalg.LinAlgError:
        return None


def _perp_distances(pts, a, b):
    """Point-to-line distance: |ax + by + 1| / sqrt(a²+b²)  (paper Eq. 4)."""
    denom = math.sqrt(a * a + b * b)
    if denom < 1e-12:
        return np.zeros(len(pts))
    return np.abs(pts[:, 0] * a + pts[:, 1] * b + 1.0) / denom


def _split_merge(pts, corners_out, line_pts_out):
    """
    Recursive split-and-merge.
    Appends corner sensor-frame points to corners_out,
    line cluster points to line_pts_out.
    """
    if len(pts) < parameters.split_merge_min_points:
        line_pts_out.extend(pts.tolist())
        return

    coeffs = _line_coeffs(pts[0], pts[-1])
    if coeffs is None:
        # degenerate segment — treat as line
        line_pts_out.extend(pts.tolist())
        return

    a, b = coeffs
    dists = _perp_distances(pts, a, b)
    idx_max = int(np.argmax(dists))
    d_max = dists[idx_max]

    if d_max > parameters.split_merge_threshold_dc:
        corners_out.append(pts[idx_max].tolist())
        _split_merge(pts[:idx_max + 1], corners_out, line_pts_out)
        _split_merge(pts[idx_max:],     corners_out, line_pts_out)
    else:
        line_pts_out.extend(pts.tolist())


def split_and_merge(clusters):
    corners_out   = []
    line_pts_out  = []
    for cluster in clusters:
        _split_merge(cluster, corners_out, line_pts_out)

    corners = np.array(corners_out, dtype=float) if corners_out else np.empty((0, 2))
    line_pts = np.array(line_pts_out, dtype=float) if line_pts_out else np.empty((0, 2))
    return corners, line_pts


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def extract_features(robot_sensor_signal):
    scan_points, scan_ranges, scan_angles = lidar_to_sensor_frame(robot_sensor_signal)

    if len(scan_points) == 0:
        empty = np.empty((0, 2))
        return FeatureSet(empty, np.array([]), np.array([]), [], empty, empty)

    clusters = arc_clustering(scan_points, scan_ranges, scan_angles)
    corners, line_pts = split_and_merge(clusters)

    return FeatureSet(
        raw_points=scan_points,
        scan_ranges=scan_ranges,
        scan_angles=scan_angles,
        clusters=clusters,
        corners=corners,
        line_points=line_pts,
    )


# ---------------------------------------------------------------------------
# Plot helper
# ---------------------------------------------------------------------------

def plot_features(feature_set, ax=None, title='Feature Extraction'):
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(7, 7))

    # raw points (light grey)
    if len(feature_set.raw_points):
        ax.scatter(feature_set.raw_points[:, 0], feature_set.raw_points[:, 1],
                   s=3, c='lightgrey', label='raw scan', zorder=1)

    # clusters (colour-coded)
    cmap = cm.get_cmap('tab10')
    for i, cl in enumerate(feature_set.clusters):
        color = cmap(i % 10)
        ax.scatter(cl[:, 0], cl[:, 1], s=10, color=color, alpha=0.7, zorder=2)

    # line points (blue)
    if len(feature_set.line_points):
        ax.scatter(feature_set.line_points[:, 0], feature_set.line_points[:, 1],
                   s=12, c='steelblue', label=f'line pts ({len(feature_set.line_points)})', zorder=3)

    # corners (red star)
    if len(feature_set.corners):
        ax.scatter(feature_set.corners[:, 0], feature_set.corners[:, 1],
                   s=120, c='red', marker='*', label=f'corners ({len(feature_set.corners)})', zorder=4)

    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(title)
    ax.set_xlabel('x (m) — sensor frame')
    ax.set_ylabel('y (m) — sensor frame')
    ax.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax.axvline(0, color='k', lw=0.5, alpha=0.3)
    ax.grid(True, alpha=0.2)

    if standalone:
        plt.tight_layout()
        return fig, ax
    return ax


# ---------------------------------------------------------------------------
# Offline verification using an existing .pkl file
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import sys
    import os
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # find a pkl file to test with
    data_dir = './data'
    pkl_files = sorted(f for f in os.listdir(data_dir) if f.endswith('.pkl'))
    if not pkl_files:
        print("No .pkl files found in ./data — run the robot first.")
        sys.exit(1)

    filename = os.path.join(data_dir, pkl_files[0])
    print(f"Using: {filename}")

    import data_handling
    rows = data_handling.get_file_data_for_pf(filename)
    print(f"Total timesteps: {len(rows)}")

    # pick a few frames spread across the log
    indices = [len(rows) // 4, len(rows) // 2, 3 * len(rows) // 4]

    fig, axes = plt.subplots(1, len(indices), figsize=(6 * len(indices), 6))
    if len(indices) == 1:
        axes = [axes]

    for ax, idx in zip(axes, indices):
        sensor = rows[idx][2]
        fs = extract_features(sensor)
        plot_features(fs, ax=ax,
                      title=f'Frame {idx}  corners={len(fs.corners)}  line_pts={len(fs.line_points)}')
        print(f"  Frame {idx:4d}: raw={len(fs.raw_points):4d}  "
              f"clusters={len(fs.clusters):3d}  "
              f"corners={len(fs.corners):3d}  "
              f"line_pts={len(fs.line_points):4d}")

    plt.tight_layout()
    out = 'plots/feature_extraction_verify.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f"\nSaved: {out}")
