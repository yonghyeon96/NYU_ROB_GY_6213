import numpy as np
from dataclasses import dataclass, field
from scipy.spatial import KDTree

import parameters


@dataclass
class MapFeatures:
    map_corners: np.ndarray        # (N, 2) — wall endpoints
    map_line_points: np.ndarray    # (M, 2) — interpolated points along walls
    map_line_midpoints: np.ndarray # (K, 2) — one midpoint per wall segment
    corner_tree: KDTree
    line_tree: KDTree              # map_line_points + map_line_midpoints
    full_tree: KDTree              # all of the above combined


def build_map_features(wall_corner_list, interpolation_step):
    corners_set = {}
    line_points_list = []
    midpoints_list = []

    for seg in wall_corner_list:
        x1, y1, x2, y2 = seg

        # --- corners: deduplicate by rounded key ---
        for px, py in [(x1, y1), (x2, y2)]:
            key = (
                round(px / parameters.map_duplicate_tol) * parameters.map_duplicate_tol,
                round(py / parameters.map_duplicate_tol) * parameters.map_duplicate_tol,
            )
            corners_set[key] = (px, py)

        # --- midpoint ---
        midpoints_list.append([(x1 + x2) / 2.0, (y1 + y2) / 2.0])

        # --- interpolated line points ---
        length = np.hypot(x2 - x1, y2 - y1)
        if length < 1e-9:
            continue
        n_pts = max(int(np.floor(length / interpolation_step)), 1)
        ts = np.linspace(0.0, 1.0, n_pts + 1)
        for t in ts:
            line_points_list.append([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])

    map_corners = np.array(list(corners_set.values()), dtype=float)
    map_line_points = np.array(line_points_list, dtype=float)
    map_line_midpoints = np.array(midpoints_list, dtype=float)

    # line_tree: interpolated points + midpoints (paper Section 3.2)
    line_targets = np.vstack([map_line_points, map_line_midpoints])
    full_targets = np.vstack([map_line_points, map_line_midpoints, map_corners])

    return MapFeatures(
        map_corners=map_corners,
        map_line_points=map_line_points,
        map_line_midpoints=map_line_midpoints,
        corner_tree=KDTree(map_corners),
        line_tree=KDTree(line_targets),
        full_tree=KDTree(full_targets),
    )


def plot_map_features(features, ax=None):
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(10, 5))

    ax.scatter(
        features.map_line_points[:, 0],
        features.map_line_points[:, 1],
        s=4, c='steelblue', alpha=0.6, label=f'line points ({len(features.map_line_points)})',
    )
    ax.scatter(
        features.map_line_midpoints[:, 0],
        features.map_line_midpoints[:, 1],
        s=60, c='orange', marker='D', zorder=4,
        label=f'midpoints ({len(features.map_line_midpoints)})',
    )
    ax.scatter(
        features.map_corners[:, 0],
        features.map_corners[:, 1],
        s=80, c='red', marker='^', zorder=5,
        label=f'corners ({len(features.map_corners)})',
    )

    # draw wall outlines
    for seg in parameters.wall_corner_list:
        x1, y1, x2, y2 = seg
        ax.plot([x1, x2], [y1, y2], 'k-', lw=1.5, alpha=0.4)

    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Map Features')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, alpha=0.3)

    if standalone:
        plt.tight_layout()
        return fig, ax
    return ax


# --- validation checks ---

def validate_map_features(features):
    errors = []
    warnings = []

    if len(features.map_corners) == 0:
        errors.append("map_corners is empty")
    if len(features.map_line_points) == 0:
        errors.append("map_line_points is empty")
    if len(features.map_line_midpoints) != len(parameters.wall_corner_list):
        errors.append(
            f"midpoints count {len(features.map_line_midpoints)} "
            f"!= wall segment count {len(parameters.wall_corner_list)}"
        )

    # check no points outside bounding box of wall_corner_list
    all_pts = np.vstack([features.map_corners, features.map_line_points, features.map_line_midpoints])
    seg_arr = np.array(parameters.wall_corner_list)
    x_min = min(seg_arr[:, [0, 2]].min(), 0) - 0.01
    x_max = seg_arr[:, [0, 2]].max() + 0.01
    y_min = min(seg_arr[:, [1, 3]].min(), 0) - 0.01
    y_max = seg_arr[:, [1, 3]].max() + 0.01

    out = np.any(
        (all_pts[:, 0] < x_min) | (all_pts[:, 0] > x_max) |
        (all_pts[:, 1] < y_min) | (all_pts[:, 1] > y_max)
    )
    if out:
        warnings.append("Some points lie outside the wall bounding box — check segment coordinates")

    # expected spacing check
    if len(features.map_line_points) > 1:
        dists = np.linalg.norm(np.diff(features.map_line_points, axis=0), axis=1)
        max_gap = dists.max()
        if max_gap > parameters.map_interpolation_step * 1.5:
            warnings.append(f"Largest gap between line points: {max_gap:.3f} m — possible interpolation issue")

    return errors, warnings


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    features = build_map_features(parameters.wall_corner_list, parameters.map_interpolation_step)

    print("=== Map Feature Summary ===")
    print(f"  map_corners      : {len(features.map_corners)} points")
    print(f"  map_line_points  : {len(features.map_line_points)} points")
    print(f"  map_line_midpoints: {len(features.map_line_midpoints)} points")
    print(f"  line_tree size   : {features.line_tree.n} points")
    print(f"  full_tree size   : {features.full_tree.n} points")

    errors, warnings = validate_map_features(features)
    if errors:
        print("\n[ERROR]")
        for e in errors:
            print(f"  - {e}")
    if warnings:
        print("\n[WARNING]")
        for w in warnings:
            print(f"  - {w}")
    if not errors and not warnings:
        print("\n[PASS] All validation checks passed.")

    plot_map_features(features)
    plt.show()
