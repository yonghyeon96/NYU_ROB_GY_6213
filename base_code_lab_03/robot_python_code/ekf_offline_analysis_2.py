import math
import pickle
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import parameters
from extended_kalman_filter import ExtendedKalmanFilter


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def is_valid_camera_measurement(z: np.ndarray) -> bool:
    # current validity check (0,0,0) means invalid
    return not (abs(z[0]) < 1e-12 and abs(z[1]) < 1e-12 and abs(z[2]) < 1e-12)


def load_log_pkl(pkl_path: str) -> dict:
    with open(pkl_path, "rb") as f:
        return pickle.load(f)


def extract_arrays(log: dict):
    """
      - time: list[float]
      - robot_sensor_signal: list[RobotSensorSignal]  -> .encoder_counts, .steering
      - camera_sensor_signal: list[list length 6]     -> x=0, y=1, theta=5
    """
    t = np.asarray(log["time"], dtype=float)

    robot_sig = log["robot_sensor_signal"]
    enc = np.asarray([rs.encoder_counts for rs in robot_sig], dtype=float)
    steer = np.asarray([rs.steering for rs in robot_sig], dtype=float)

    cam_sig = log["camera_sensor_signal"]
    cam_x = np.asarray([cs[0] for cs in cam_sig], dtype=float)
    cam_y = np.asarray([cs[1] for cs in cam_sig], dtype=float)
    cam_th = np.asarray([cs[5] for cs in cam_sig], dtype=float)
    cam_th[0] = 0.0

    # (Optional) If you intentionally want to override only the first heading:
    # cam_th[0] = 0.0

    dt = np.diff(t, prepend=t[0])
    if len(dt) > 1:
        dt0 = float(np.median(dt[1:]))
        dt[0] = dt0 if dt0 > 0 else float(np.mean(dt[1:]))

    return t, dt, enc, steer, cam_x, cam_y, cam_th


# -----------------------------
# EKF runner (Method B: ekf.update)
# -----------------------------
def run_ekf_with_update(
    t, dt, enc, steer, cam_x, cam_y, cam_th,
    correction_on: bool,
    x0: np.ndarray,
    Sigma0: np.ndarray,
    debug: bool = False,
    debug_inputs: bool = False,
):
    ekf = ExtendedKalmanFilter(
        x_0=x0.tolist(),
        Sigma_0=Sigma0,
        encoder_counts_0=float(enc[0]) if len(enc) else 0.0,
    )

    N = len(t)
    X = np.zeros((N, 3), dtype=float)
    P = np.zeros((N, 3, 3), dtype=float)

    Z = np.zeros((N, 3), dtype=float)
    Z_valid = np.zeros((N,), dtype=bool)

    correction_applied = 0

    if debug_inputs:
        print("\n--- Input sanity ---")
        print("N:", N)
        print("time range:", float(t[0]), "->", float(t[-1]))
        print("dt min/max/mean:", float(dt.min()), float(dt.max()), float(dt.mean()))
        print("encoder_counts min/max:", float(enc.min()), float(enc.max()))
        print("steering min/max:", float(steer.min()), float(steer.max()))

    for k in range(N):
        u_t = np.array([enc[k], steer[k]], dtype=float)
        z_t = np.array([cam_x[k], cam_y[k], cam_th[k]], dtype=float)

        Z[k] = z_t
        Z_valid[k] = is_valid_camera_measurement(z_t)

        # prediction-only => z_in=None
        # correction-on   => z_in=z_t when valid
        z_in = z_t if (correction_on and Z_valid[k]) else None
        if z_in is not None:
            correction_applied += 1

        ekf.update(u_t, z_in, float(dt[k]))

        X[k] = np.asarray(ekf.state_mean, dtype=float)
        P[k] = np.asarray(ekf.state_covariance, dtype=float)

        if debug and (k < 5 or k % 25 == 0):
            print(f"\n[k={k}] z_used={z_in is not None}")
            print(f" state=({X[k,0]:.3f}, {X[k,1]:.3f}, {X[k,2]:.3f})")
            print(f" Pdiag=({P[k,0,0]:.4g}, {P[k,1,1]:.4g}, {P[k,2,2]:.4g})")

    if debug_inputs:
        print("\n--- Correction usage ---")
        print("correction_on:", correction_on)
        print("valid camera frames:", int(np.sum(Z_valid)), "/", len(Z_valid))
        print("correction applied:", correction_applied, "/", len(Z_valid))

    return X, P, Z, Z_valid


# -----------------------------
# Metrics
# -----------------------------
def position_error(X, Z, Z_valid):
    dx = X[Z_valid, 0] - Z[Z_valid, 0]
    dy = X[Z_valid, 1] - Z[Z_valid, 1]
    return np.sqrt(dx * dx + dy * dy)


def final_position_error(X, Z, Z_valid):
    idx = np.where(Z_valid)[0]
    if len(idx) == 0:
        return None
    k = idx[-1]
    dx = X[k, 0] - Z[k, 0]
    dy = X[k, 1] - Z[k, 1]
    return float(np.sqrt(dx * dx + dy * dy))


def rmse(e):
    if len(e) == 0:
        return None
    return float(np.sqrt(np.mean(e * e)))


def pct_reduction(before: float, after: float) -> float | None:
    if before is None or after is None or abs(before) < 1e-12:
        return None
    return float((before - after) / before * 100.0)


# -----------------------------
# Summary Plots (ONLY 2 figures)
#   1) Trajectory (unknown cases only)
#   2) Position error vs time (unknown cases only)
# -----------------------------
def plot_unknown_trajectory_summary(
    Z, Z_valid,
    unknown_runs: list,
    title: str = "Unknown-start EKF: Trajectory (Correction-On)"
):
    """
    unknown_runs: list of dicts with keys:
      - name
      - X_corr, P_corr (optional), etc.
    """
    plt.figure()
    plt.plot(Z[Z_valid, 0], Z[Z_valid, 1], label="camera (truth proxy)")

    # Only plot correction-on trajectories for unknown cases (clean report figure)
    for r in unknown_runs:
        Xc = r["X_corr"]
        plt.plot(Xc[:, 0], Xc[:, 1], label=f"corr-on: {r['name']}")

    plt.axis("equal")
    plt.ylim(-0.07, 0.07)
    plt.title(title)
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()


def plot_unknown_error_summary(
    t, Z_valid,
    unknown_runs: list,
    title: str = "Unknown-start EKF: Position Error vs Time",
    show_pred_only: bool = True
):
    """
    unknown_runs: list of dicts with keys:
      - name
      - e_pred (optional if show_pred_only)
      - e_corr
    """
    if np.sum(Z_valid) == 0:
        print("No valid camera frames; skipping error summary plot.")
        return

    t_valid = t[Z_valid]

    plt.figure()

    for r in unknown_runs:
        if show_pred_only:
            plt.plot(t_valid, r["e_pred"], label=f"pred-only: {r['name']}")
        plt.plot(t_valid, r["e_corr"], label=f"corr-on: {r['name']}")

    plt.title(title)
    plt.xlabel("time (s)")
    plt.ylabel("distance (m)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()


# -----------------------------
# Initializer helpers (Known vs Unknown)
# -----------------------------
def make_known_start(cam_x0, cam_y0, cam_th0) -> np.ndarray:
    return np.array([float(cam_x0), float(cam_y0), float(cam_th0)], dtype=float)


def make_unknown_start(known_x0: np.ndarray, dx: float, dy: float, dtheta_deg: float) -> np.ndarray:
    dth = math.radians(dtheta_deg)
    return np.array(
        [known_x0[0] + dx, known_x0[1] + dy, wrap_angle(known_x0[2] + dth)],
        dtype=float
    )


def make_P0_from_offsets(dx: float, dy: float, dtheta_deg: float) -> np.ndarray:
    """
    Choose Sigma0 that 'accommodates' the distance between initial guess and actual pose.
    We use diagonal covariance with std dev roughly matching the offsets.
    """
    sigma_x = max(abs(dx), 0.02)          # >= 2 cm
    sigma_y = max(abs(dy), 0.01)          # >= 1 cm
    sigma_th = max(abs(math.radians(dtheta_deg)), math.radians(5.0))  # >= 5 deg
    return np.diag([sigma_x**2, sigma_y**2, sigma_th**2]).astype(float)


# -----------------------------
# Main (auto-run)
# -----------------------------
if __name__ == "__main__":

    # Pick your straight-line data file here
    PKL_FILE = "./data/robot_data_48_0_25_02_26_11_59_11.pkl"
    DEBUG = False
    DEBUG_INPUTS_ONCE = True

    # Only 2 summary plots will be created
    MAKE_SUMMARY_PLOTS = True
    SHOW_PRED_ONLY_IN_ERROR_PLOT = True   # set False if you want cleaner convergence-only plot

    if not Path(PKL_FILE).exists():
        raise FileNotFoundError(f"PKL file not found: {PKL_FILE}")

    log = load_log_pkl(PKL_FILE)
    t, dt, enc, steer, cam_x, cam_y, cam_th = extract_arrays(log)

    known_x0 = make_known_start(cam_x[0], cam_y[0], cam_th[0])

    # --- Define 1 known + 3 unknown cases ---
    cases = [
        {
            "name": "KNOWN",
            "is_known": True,
            "x0": known_x0,
            "P0": np.diag([1e-4, 1e-4, 1e-3]).astype(float),
        },
        {
            "name": "U1(dx=+0.30, dy=+0.05, dth=+10deg)",
            "is_known": False,
            "x0": make_unknown_start(known_x0, dx=0.30, dy=0.05, dtheta_deg=10.0),
            "P0": make_P0_from_offsets(dx=0.30, dy=0.05, dtheta_deg=10.0),
        },
        {
            "name": "U2(dx=+0.00, dy=+0.08, dth=-15deg)",
            "is_known": False,
            "x0": make_unknown_start(known_x0, dx=0.00, dy=0.08, dtheta_deg=-15.0),
            "P0": make_P0_from_offsets(dx=0.00, dy=0.08, dtheta_deg=-15.0),
        },
        {
            "name": "U3(dx=-0.20, dy=+0.03, dth=+25deg)",
            "is_known": False,
            "x0": make_unknown_start(known_x0, dx=-0.20, dy=0.03, dtheta_deg=25.0),
            "P0": make_P0_from_offsets(dx=-0.20, dy=0.03, dtheta_deg=25.0),
        },
    ]

    results = []
    unknown_runs_for_plots = []

    # Run input sanity once (not per case)
    if DEBUG_INPUTS_ONCE:
        _ = run_ekf_with_update(
            t, dt, enc, steer, cam_x, cam_y, cam_th,
            correction_on=False,
            x0=cases[0]["x0"],
            Sigma0=cases[0]["P0"],
            debug=False,
            debug_inputs=True
        )

    # We'll capture Z/Z_valid once from any run (they are the same per file)
    Z_ref = None
    Z_valid_ref = None

    for case in cases:
        name = case["name"]
        x0 = case["x0"]
        P0 = case["P0"]

        print("\n" + "=" * 80)
        print(f"CASE: {name}")
        print(f" x0 = ({x0[0]:.3f}, {x0[1]:.3f}, {x0[2]:.3f})")
        print(f" P0 diag = ({P0[0,0]:.4g}, {P0[1,1]:.4g}, {P0[2,2]:.4g})")

        # pred-only
        X_pred, P_pred, Z, Z_valid = run_ekf_with_update(
            t, dt, enc, steer, cam_x, cam_y, cam_th,
            correction_on=False,
            x0=x0,
            Sigma0=P0,
            debug=DEBUG,
            debug_inputs=False
        )

        # corr-on
        X_corr, P_corr, Z, Z_valid = run_ekf_with_update(
            t, dt, enc, steer, cam_x, cam_y, cam_th,
            correction_on=True,
            x0=x0,
            Sigma0=P0,
            debug=DEBUG,
            debug_inputs=False
        )

        if Z_ref is None:
            Z_ref = Z
            Z_valid_ref = Z_valid

        # metrics
        e_pred = position_error(X_pred, Z, Z_valid)
        e_corr = position_error(X_corr, Z, Z_valid)

        rmse_pred = rmse(e_pred)
        rmse_corr = rmse(e_corr)

        f_pred = final_position_error(X_pred, Z, Z_valid)
        f_corr = final_position_error(X_corr, Z, Z_valid)

        rmse_impr = pct_reduction(rmse_pred, rmse_corr)
        final_impr = pct_reduction(f_pred, f_corr)

        print("\n--- Metrics ---")
        print(f" RMSE pred-only : {rmse_pred:.6f}")
        print(f" RMSE corr-on   : {rmse_corr:.6f}")
        print(f" RMSE reduction : {rmse_impr:.1f}%")
        print(f" Final pred-only: {f_pred:.6f}")
        print(f" Final corr-on  : {f_corr:.6f}")
        print(f" Final reduction: {final_impr:.1f}%")

        # covariance min/max (keep output as requested)
        print("\n--- Covariance min/max ---")
        print(" valid camera frames:", int(np.sum(Z_valid)), "/", len(Z_valid))
        print(" Pxx pred min/max:", float(P_pred[:, 0, 0].min()), float(P_pred[:, 0, 0].max()))
        print(" Pyy pred min/max:", float(P_pred[:, 1, 1].min()), float(P_pred[:, 1, 1].max()))
        print(" Ptt pred min/max:", float(P_pred[:, 2, 2].min()), float(P_pred[:, 2, 2].max()))
        print(" Pxx corr min/max:", float(P_corr[:, 0, 0].min()), float(P_corr[:, 0, 0].max()))
        print(" Pyy corr min/max:", float(P_corr[:, 1, 1].min()), float(P_corr[:, 1, 1].max()))
        print(" Ptt corr min/max:", float(P_corr[:, 2, 2].min()), float(P_corr[:, 2, 2].max()))

        results.append({
            "case": name,
            "is_known": case["is_known"],
            "rmse_pred": rmse_pred,
            "rmse_corr": rmse_corr,
            "rmse_reduction_pct": rmse_impr,
            "final_pred": f_pred,
            "final_corr": f_corr,
            "final_reduction_pct": final_impr,
        })

        # Save only UNKNOWN runs for the 2 summary plots
        if not case["is_known"]:
            unknown_runs_for_plots.append({
                "name": name,
                "X_pred": X_pred,
                "X_corr": X_corr,
                "P_corr": P_corr,
                "e_pred": e_pred,
                "e_corr": e_corr,
            })

    # Summary print (keep known + unknown)
    print("\n" + "=" * 80)
    print("SUMMARY (Known + 3 Unknown cases)")
    for r in results:
        tag = "KNOWN" if r["is_known"] else "UNKNOWN"
        print(
            f"- [{tag}] {r['case']}: "
            f"RMSE pred {r['rmse_pred']:.4f} -> corr {r['rmse_corr']:.4f} "
            f"({r['rmse_reduction_pct']:.1f}% red), "
            f"Final {r['final_pred']:.4f} -> {r['final_corr']:.4f} "
            f"({r['final_reduction_pct']:.1f}% red)"
        )

    # Only 2 plots total (unknown-only)
    if MAKE_SUMMARY_PLOTS and (Z_ref is not None):
        plot_unknown_trajectory_summary(
            Z_ref, Z_valid_ref, unknown_runs_for_plots,
            title="Unknown-start EKF: Trajectory (Correction-On only)"
        )
        plot_unknown_error_summary(
            t, Z_valid_ref, unknown_runs_for_plots,
            title="Unknown-start EKF: Position Error vs Time",
            show_pred_only=SHOW_PRED_ONLY_IN_ERROR_PLOT
        )

        plt.show()