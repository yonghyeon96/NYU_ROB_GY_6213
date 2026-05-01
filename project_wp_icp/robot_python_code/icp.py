import math
import time
import numpy as np
from dataclasses import dataclass
from typing import Optional

import parameters


@dataclass
class ICPResult:
    pose: list           # [x, y, theta] world frame
    iterations: int
    converged: bool
    mean_error: float
    num_matches: int
    runtime_sec: float


def _rotation_matrix(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _angle_wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _weighted_svd(source, target, weights):
    """
    Compute rigid transform (R, t) minimising Σ wᵢ ‖R·pᵢ + t − qᵢ‖²
    Paper Equations (5)-(9).

    source, target : (N, 2) arrays of matched points
    weights        : (N,) array, 0 for outliers

    Returns (R_2x2, t_2, theta_rad) or None on failure.
    """
    w_sum = weights.sum()
    if w_sum < 1e-12:
        return None

    # weighted centroids  (Eq. 7)
    p_bar = (weights[:, None] * source).sum(axis=0) / w_sum
    q_bar = (weights[:, None] * target).sum(axis=0) / w_sum

    # centred coordinates
    X = (source - p_bar).T          # (2, N)
    Y = (target - q_bar).T          # (2, N)
    W = np.diag(weights)            # (N, N)

    # cross-covariance  (Eq. 8)
    S = X @ W @ Y.T                 # (2, 2)

    U, _, Vt = np.linalg.svd(S)
    V = Vt.T

    # optimal rotation  (Eq. 9)
    R = V @ U.T

    # reflection check
    if np.linalg.det(R) < 0:
        V[:, -1] *= -1
        R = V @ U.T

    # optimal translation  (Eq. 6)
    t = q_bar - R @ p_bar

    theta = math.atan2(R[1, 0], R[0, 0])
    return R, t, theta


def update(scan_points, initial_pose, map_features):
    """
    Vanilla ICP: match all scan points to map full_tree.

    scan_points   : (N, 2) sensor-frame points
    initial_pose  : [x, y, theta]
    map_features  : MapFeatures (from map_builder)

    Returns ICPResult.
    """
    t_start = time.perf_counter()

    if len(scan_points) == 0:
        return ICPResult(
            pose=list(initial_pose), iterations=0,
            converged=False, mean_error=float('inf'),
            num_matches=0, runtime_sec=0.0,
        )

    x, y, theta = float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])

    prev_mean_error = float('inf')
    num_matches = 0

    for it in range(parameters.icp_max_iterations):

        # --- transform scan to world frame ---
        R_cur = _rotation_matrix(theta)
        world_pts = scan_points @ R_cur.T + np.array([x, y])

        # --- nearest-neighbour query ---
        dists, idxs = map_features.full_tree.query(world_pts)

        # --- weight assignment (paper: w_i = 0 for outliers) ---
        weights = np.where(dists <= parameters.icp_reject_threshold, 1.0, 0.0)
        n_valid = int(weights.sum())

        if n_valid < parameters.icp_min_matches:
            break

        target_pts = map_features.full_tree.data[idxs]

        # --- weighted SVD ---
        result = _weighted_svd(world_pts, target_pts, weights)
        if result is None:
            break

        R_delta, t_delta, theta_delta = result

        # --- pose update ---
        xy_new = R_delta @ np.array([x, y]) + t_delta
        x, y   = float(xy_new[0]), float(xy_new[1])
        theta  = _angle_wrap(theta + theta_delta)

        num_matches = n_valid
        mean_error  = float((weights * dists).sum() / max(n_valid, 1))

        # --- convergence check ---
        pose_change = math.sqrt(t_delta[0]**2 + t_delta[1]**2 + theta_delta**2)
        error_change = abs(prev_mean_error - mean_error)

        if pose_change < parameters.icp_convergence_tol and \
           error_change < parameters.icp_error_tol:
            it += 1
            break

        prev_mean_error = mean_error

    else:
        it = parameters.icp_max_iterations

    # --- sanity checks ---
    pose = [x, y, theta]
    converged = (num_matches >= parameters.icp_min_matches and
                 math.isfinite(x) and math.isfinite(y) and math.isfinite(theta))

    if not converged or \
       math.sqrt((x - initial_pose[0])**2 + (y - initial_pose[1])**2) > parameters.icp_max_pose_jump_m or \
       abs(_angle_wrap(theta - initial_pose[2])) > parameters.icp_max_heading_jump_rad:
        pose = list(initial_pose)
        converged = False

    return ICPResult(
        pose=pose,
        iterations=it,
        converged=converged,
        mean_error=mean_error if num_matches > 0 else float('inf'),
        num_matches=num_matches,
        runtime_sec=time.perf_counter() - t_start,
    )


# ---------------------------------------------------------------------------
# Synthetic verification
# ---------------------------------------------------------------------------

def _run_synthetic_test():
    import map_builder

    print("=== ICP Synthetic Transform Test ===")
    features = map_builder.build_map_features(
        parameters.wall_corner_list, parameters.map_interpolation_step
    )

    # sample map points as a "perfect scan"
    rng = np.random.default_rng(0)
    map_pts = features.map_line_points
    sample_idx = rng.choice(len(map_pts), size=min(60, len(map_pts)), replace=False)
    clean_world_pts = map_pts[sample_idx]

    results = []
    test_cases = [
        ("small translation",      [0.05, 0.03, 0.0],     [0.00, 0.00, 0.00]),
        ("small rotation",         [0.00, 0.00, 0.08],    [0.00, 0.00, 0.00]),
        ("translation + rotation", [0.07, -0.04, 0.06],   [0.00, 0.00, 0.00]),
        ("larger offset",          [0.12, 0.08, 0.10],    [0.00, 0.00, 0.00]),
    ]

    all_passed = True
    for name, true_pose, warm_start in test_cases:
        # build sensor-frame scan from known world pose
        tx, ty, tt = true_pose
        R_true = _rotation_matrix(tt)
        # sensor_pts @ R_true.T + [tx,ty] = clean_world_pts
        # => sensor_pts = (clean_world_pts - [tx,ty]) @ R_true
        sensor_pts = (clean_world_pts - np.array([tx, ty])) @ R_true

        result = update(sensor_pts, warm_start, features)

        ex = abs(result.pose[0] - tx)
        ey = abs(result.pose[1] - ty)
        et = abs(_angle_wrap(result.pose[2] - tt))
        ok = result.converged and ex < 0.02 and ey < 0.02 and et < 0.02

        status = "PASS" if ok else "FAIL"
        if not ok:
            all_passed = False

        print(f"  [{status}] {name}")
        print(f"          true=({tx:.3f},{ty:.3f},{tt:.3f})  "
              f"est=({result.pose[0]:.3f},{result.pose[1]:.3f},{result.pose[2]:.3f})  "
              f"err=({ex:.4f},{ey:.4f},{et:.4f})  "
              f"iters={result.iterations}  matches={result.num_matches}")

    print()
    if all_passed:
        print("[PASS] All synthetic tests passed.")
    else:
        print("[FAIL] Some tests failed — check ICP parameters.")
    return all_passed


if __name__ == '__main__':
    _run_synthetic_test()
