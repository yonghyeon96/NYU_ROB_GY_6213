import math
import time
import numpy as np
from dataclasses import dataclass
from typing import Optional, List

import parameters
from icp import _rotation_matrix, _angle_wrap, _weighted_svd, ICPResult


@dataclass
class WPICPResult:
    pose: list                        # [x, y, theta] world frame
    corner_iterations: int
    line_iterations: int
    converged: bool
    runtime_sec: float
    corner_confidence: float          # Γ_C
    line_confidence: float            # Γ_L
    corner_matches: int
    line_matches: int
    corner_pose: Optional[list]       # [x, y, theta] or None
    line_pose: Optional[list]         # [x, y, theta] or None


# ---------------------------------------------------------------------------
# Internal: single-feature ICP
# ---------------------------------------------------------------------------

def _feature_icp(scan_feat_pts, initial_pose, kd_tree,
                 max_iter, reject_thr, min_matches,
                 convergence_tol, error_tol):
    """
    ICP for one feature type (corner or line).
    scan_feat_pts : (N, 2) sensor-frame feature points
    kd_tree       : KDTree of the corresponding map feature targets
    Returns (pose, iterations, num_matches, mean_error, ok)
    """
    if len(scan_feat_pts) == 0 or kd_tree.n == 0:
        return list(initial_pose), 0, 0, float('inf'), False

    x, y, theta = float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])
    prev_mean_error = float('inf')
    num_matches = 0
    mean_error  = float('inf')

    for it in range(max_iter):
        R_cur    = _rotation_matrix(theta)
        world_pts = scan_feat_pts @ R_cur.T + np.array([x, y])

        dists, idxs = kd_tree.query(world_pts)

        weights  = np.where(dists <= reject_thr, 1.0, 0.0)
        n_valid  = int(weights.sum())

        if n_valid < min_matches:
            break

        target_pts = kd_tree.data[idxs]
        result = _weighted_svd(world_pts, target_pts, weights)
        if result is None:
            break

        R_delta, t_delta, theta_delta = result

        xy_new = R_delta @ np.array([x, y]) + t_delta
        x, y   = float(xy_new[0]), float(xy_new[1])
        theta  = _angle_wrap(theta + theta_delta)

        num_matches = n_valid
        mean_error  = float((weights * dists).sum() / max(n_valid, 1))

        pose_change  = math.sqrt(t_delta[0]**2 + t_delta[1]**2 + theta_delta**2)
        error_change = abs(prev_mean_error - mean_error)

        if pose_change < convergence_tol and error_change < error_tol:
            it += 1
            break

        prev_mean_error = mean_error

    else:
        it = max_iter

    ok = (num_matches >= min_matches and
          math.isfinite(x) and math.isfinite(y) and math.isfinite(theta))
    return [x, y, theta], it, num_matches, mean_error, ok


# ---------------------------------------------------------------------------
# Pose fusion  (paper Eq. 11, angle_wrap variant)
# ---------------------------------------------------------------------------

def _fuse_poses(corner_pose, line_pose, gamma_c, gamma_l):
    """
    Weighted average: alpha = Γ_C / (Γ_C + Γ_L)
    Translation: linear interpolation
    Rotation   : angle_wrap interpolation (numerically stable at ±π)
    """
    alpha = gamma_c / (gamma_c + gamma_l)

    x     = alpha * corner_pose[0] + (1 - alpha) * line_pose[0]
    y     = alpha * corner_pose[1] + (1 - alpha) * line_pose[1]
    theta = corner_pose[2] + (1 - alpha) * _angle_wrap(line_pose[2] - corner_pose[2])
    theta = _angle_wrap(theta)
    return [x, y, theta]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def update(feature_set, initial_pose, map_features):
    """
    WP-ICP: parallel corner ICP + line ICP, confidence-weighted fusion.

    feature_set  : FeatureSet (from feature_extraction)
    initial_pose : [x, y, theta]
    map_features : MapFeatures (from map_builder)

    Returns WPICPResult.
    """
    t_start = time.perf_counter()

    # downsampling for line points if configured
    line_pts = feature_set.line_points
    if parameters.wp_icp_line_downsample_step > 1 and len(line_pts) > 0:
        line_pts = line_pts[::parameters.wp_icp_line_downsample_step]

    # total counts for confidence denominator (pre-downsampling)
    total_corners    = max(len(feature_set.corners), 1)
    total_line_pts   = max(len(feature_set.line_points), 1)

    # ---- Corner ICP ----
    corner_pose, corner_iters, corner_matches, _, corner_ok = _feature_icp(
        scan_feat_pts = feature_set.corners,
        initial_pose  = initial_pose,
        kd_tree       = map_features.corner_tree,
        max_iter      = parameters.icp_max_iterations,
        reject_thr    = parameters.icp_reject_threshold,
        min_matches   = parameters.wp_icp_min_corner_matches,
        convergence_tol = parameters.icp_convergence_tol,
        error_tol     = parameters.icp_error_tol,
    )
    # sanity: jump check
    if corner_ok:
        dx = math.sqrt((corner_pose[0]-initial_pose[0])**2 + (corner_pose[1]-initial_pose[1])**2)
        da = abs(_angle_wrap(corner_pose[2] - initial_pose[2]))
        if dx > parameters.icp_max_pose_jump_m or da > parameters.icp_max_heading_jump_rad:
            corner_ok = False

    # ---- Line ICP ----
    line_pose, line_iters, line_matches, _, line_ok = _feature_icp(
        scan_feat_pts = line_pts,
        initial_pose  = initial_pose,
        kd_tree       = map_features.line_tree,
        max_iter      = parameters.icp_max_iterations,
        reject_thr    = parameters.icp_reject_threshold,
        min_matches   = parameters.wp_icp_min_line_matches,
        convergence_tol = parameters.icp_convergence_tol,
        error_tol     = parameters.icp_error_tol,
    )
    if line_ok:
        dx = math.sqrt((line_pose[0]-initial_pose[0])**2 + (line_pose[1]-initial_pose[1])**2)
        da = abs(_angle_wrap(line_pose[2] - initial_pose[2]))
        if dx > parameters.icp_max_pose_jump_m or da > parameters.icp_max_heading_jump_rad:
            line_ok = False

    # ---- Confidence (paper Eq. 10) ----
    gamma_c = corner_matches / total_corners if corner_ok else 0.0
    gamma_l = line_matches   / total_line_pts if line_ok  else 0.0

    # ---- Pose fusion / fallback ----
    if gamma_c + gamma_l == 0.0:
        # both failed
        final_pose = list(initial_pose)
        converged  = False

    elif not corner_ok:
        final_pose = line_pose
        converged  = True

    elif not line_ok:
        final_pose = corner_pose
        converged  = True

    else:
        # both valid — check if they agree
        pose_diff = math.sqrt(
            (corner_pose[0] - line_pose[0])**2 +
            (corner_pose[1] - line_pose[1])**2
        )
        if pose_diff > parameters.icp_max_pose_jump_m:
            # large disagreement: trust higher-confidence result
            final_pose = corner_pose if gamma_c >= gamma_l else line_pose
        else:
            final_pose = _fuse_poses(corner_pose, line_pose, gamma_c, gamma_l)
        converged = True

    return WPICPResult(
        pose             = final_pose,
        corner_iterations= corner_iters,
        line_iterations  = line_iters,
        converged        = converged,
        runtime_sec      = time.perf_counter() - t_start,
        corner_confidence= gamma_c,
        line_confidence  = gamma_l,
        corner_matches   = corner_matches,
        line_matches     = line_matches,
        corner_pose      = corner_pose if corner_ok else None,
        line_pose        = line_pose   if line_ok   else None,
    )


# ---------------------------------------------------------------------------
# Synthetic verification
# ---------------------------------------------------------------------------

def _run_synthetic_test():
    import map_builder
    from feature_extraction import FeatureSet

    print("=== WP-ICP Synthetic Test ===")
    features = map_builder.build_map_features(
        parameters.wall_corner_list, parameters.map_interpolation_step
    )

    rng = np.random.default_rng(42)
    map_line = features.map_line_points
    map_corn = features.map_corners

    # sample "line" and "corner" scan points from map
    n_line = min(40, len(map_line))
    n_corn = min(4,  len(map_corn))
    line_idx = rng.choice(len(map_line), size=n_line, replace=False)
    corn_idx = rng.choice(len(map_corn), size=n_corn, replace=False)

    true_x, true_y, true_theta = 0.08, -0.05, 0.06

    R_true = _rotation_matrix(true_theta)
    # sensor_pts = (world_pts - t) @ R_true
    sensor_line = (map_line[line_idx] - np.array([true_x, true_y])) @ R_true
    sensor_corn = (map_corn[corn_idx] - np.array([true_x, true_y])) @ R_true

    empty = np.empty((0, 2))

    test_cases = [
        ("corner only (line empty)",  sensor_corn, empty),
        ("line only (corner empty)",  empty,       sensor_line),
        ("both valid",                sensor_corn, sensor_line),
    ]

    all_passed = True
    for name, c_pts, l_pts in test_cases:
        fs = FeatureSet(
            raw_points=np.vstack([c_pts, l_pts]) if len(c_pts) and len(l_pts)
                       else (c_pts if len(c_pts) else l_pts),
            scan_ranges=np.array([]), scan_angles=np.array([]),
            clusters=[], corners=c_pts, line_points=l_pts,
        )
        result = update(fs, [0.0, 0.0, 0.0], features)

        if result.converged:
            ex = abs(result.pose[0] - true_x)
            ey = abs(result.pose[1] - true_y)
            et = abs(_angle_wrap(result.pose[2] - true_theta))
            ok = ex < 0.02 and ey < 0.02 and et < 0.02
        else:
            ok = False

        status = "PASS" if ok else "FAIL"
        if not ok:
            all_passed = False

        print(f"  [{status}] {name}")
        print(f"          est=({result.pose[0]:.3f},{result.pose[1]:.3f},{result.pose[2]:.3f})  "
              f"Γ_C={result.corner_confidence:.2f}  Γ_L={result.line_confidence:.2f}  "
              f"c_iter={result.corner_iterations}  l_iter={result.line_iterations}")

    # both-failed case
    fs_fail = FeatureSet(
        raw_points=empty, scan_ranges=np.array([]), scan_angles=np.array([]),
        clusters=[], corners=empty, line_points=empty,
    )
    result_fail = update(fs_fail, [0.1, 0.2, 0.3], features)
    ok_fail = not result_fail.converged and result_fail.pose == [0.1, 0.2, 0.3]
    status = "PASS" if ok_fail else "FAIL"
    if not ok_fail:
        all_passed = False
    print(f"  [{status}] both failed → returns initial_pose")

    print()
    if all_passed:
        print("[PASS] All WP-ICP synthetic tests passed.")
    else:
        print("[FAIL] Some tests failed.")
    return all_passed


if __name__ == '__main__':
    _run_synthetic_test()
