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
    debug: bool = True
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

    # Sanity debug (inputs)
    if debug:
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

        # ✅ Method B:
        #  - prediction-only: z_in=None
        #  - correction-on  : z_in=z_t]
        """if k == 0:
            z_in = None
        else:
            z_in = z_t if (correction_on and Z_valid[k]) else None"""
        
        z_in = z_t if (correction_on and Z_valid[k]) else None

        if z_in is not None:
            correction_applied += 1

        ekf.update(u_t, z_in, float(dt[k]))

        X[k] = np.asarray(ekf.state_mean, dtype=float)
        P[k] = np.asarray(ekf.state_covariance, dtype=float)



    if debug:
        print("\n--- Correction usage ---")
        print("correction_on:", correction_on)
        print("valid camera frames:", int(np.sum(Z_valid)), "/", len(Z_valid))
        print("correction applied:", correction_applied, "/", len(Z_valid))
    if debug and (k < 10 or k % 20 == 0):
        print(f"\n[k={k}]")
        print(f" dt = {dt[k]:.3f}")
        print(f" encoder = {enc[k]:.1f}, steering = {steer[k]:.2f}")
        print(f" z_used = {z_in is not None}")
        print(f" state = ({X[k,0]:.3f}, {X[k,1]:.3f}, {X[k,2]:.3f})")
        print(f" P diag = ({P[k,0,0]:.4g}, {P[k,1,1]:.4g}, {P[k,2,2]:.4g})")

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


# -----------------------------
# Plots
# -----------------------------
def plot_trajectory_with_ellipses(X_pred, P_pred, X_corr, P_corr, Z, Z_valid,
                                  title: str,
                                  ellipse_every: int = 10,
                                  ellipse_sigma: float = 2.0):
    plt.figure()
    plt.plot(Z[Z_valid, 0], Z[Z_valid, 1], label="camera (truth proxy)")
    plt.plot(X_pred[:, 0], X_pred[:, 1], label="prediction only")
    plt.plot(X_corr[:, 0], X_corr[:, 1], label="correction on")

    
    # confidence ellipse (correction covariance 기준)
    for k in range(ellipse_every, len(X_corr), ellipse_every):
        Pxy = parameters.covariance_plot_scale * P_corr[k, 0:2, 0:2]
        vals, vecs = np.linalg.eigh(Pxy)
        if np.any(vals < 0):
            continue

        a = ellipse_sigma * math.sqrt(vals[1])
        b = ellipse_sigma * math.sqrt(vals[0])
        angle = math.atan2(vecs[1, 1], vecs[0, 1])

        th = np.linspace(0, 2 * math.pi, 60)
        ex = a * np.cos(th)
        ey = b * np.sin(th)

        R = np.array([[math.cos(angle), -math.sin(angle)],
                      [math.sin(angle),  math.cos(angle)]], dtype=float)
        epts = (R @ np.vstack([ex, ey])).T
        epts[:, 0] += X_corr[k, 0]
        epts[:, 1] += X_corr[k, 1]
        plt.plot(epts[:, 0], epts[:, 1], linewidth=0.8)
    
    plt.axis("equal")
    plt.title(title)
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()


def plot_covariance_split(P_pred, P_corr, t):

    # (1) Position covariance
    plt.figure()
    plt.plot(t, P_pred[:, 0, 0], label="Pxx pred-only")
    plt.plot(t, P_corr[:, 0, 0], label="Pxx corr-on")
    plt.plot(t, P_pred[:, 1, 1], label="Pyy pred-only")
    plt.plot(t, P_corr[:, 1, 1], label="Pyy corr-on")
    plt.title("Covariance over time (position)")
    plt.xlabel("time (s)")
    plt.ylabel("variance(m^2)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()

    # (2) Heading covariance
    plt.figure()
    plt.plot(t, P_pred[:, 2, 2], label="Ptt pred-only")
    plt.plot(t, P_corr[:, 2, 2], label="Ptt corr-on")
    plt.title("Covariance over time (heading angle)")
    plt.xlabel("time (s)")
    plt.ylabel("variance(rad^2)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()


def plot_error_over_time(e_pred, e_corr, t_valid):
    plt.figure()
    plt.plot(t_valid, e_pred, label="pos error pred-only")
    plt.plot(t_valid, e_corr, label="pos error corr-on")
    plt.title("Position error vs time (camera-valid frames)")
    plt.xlabel("time (s)")
    plt.ylabel("distance(m)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()


# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":

    PKL_FILE = "./data/robot_data_60_20_25_02_26_16_16_45.pkl"  
    DEBUG = True  


    UNKNOWN_START_DX = 0.0
    UNKNOWN_START_DY = 0.0
    UNKNOWN_START_DTHETA = 0.0
    UNKNOWN_START_COV_MULT = 1.0  
    # ============================

    if not Path(PKL_FILE).exists():
        raise FileNotFoundError(f"PKL file not found: {PKL_FILE}")

    log = load_log_pkl(PKL_FILE)
    t, dt, enc, steer, cam_x, cam_y, cam_th = extract_arrays(log)

    x0 = np.array([
        cam_x[0] + UNKNOWN_START_DX,
        cam_y[0] + UNKNOWN_START_DY,
        wrap_angle(0.0 + UNKNOWN_START_DTHETA),
    ], dtype=float)

    """x0 = np.array([
        cam_x[0] + UNKNOWN_START_DX,
        cam_y[0] + UNKNOWN_START_DY,
        wrap_angle(cam_th[0] + UNKNOWN_START_DTHETA),
    ], dtype=float)"""

    Sigma0 = parameters.I3 * float(UNKNOWN_START_COV_MULT)

    # Run prediction-only
    X_pred, P_pred, Z, Z_valid = run_ekf_with_update(
        t, dt, enc, steer, cam_x, cam_y, cam_th,
        correction_on=False,
        x0=x0,
        Sigma0=Sigma0,
        debug=DEBUG
    )

    # Run correction-on
    X_corr, P_corr, Z, Z_valid = run_ekf_with_update(
        t, dt, enc, steer, cam_x, cam_y, cam_th,
        correction_on=True,
        x0=x0,
        Sigma0=Sigma0,
        debug=DEBUG
    )

    # Metrics (camera valid frames only)
    e_pred = position_error(X_pred, Z, Z_valid)
    e_corr = position_error(X_corr, Z, Z_valid)

    rmse_pred = rmse(e_pred)
    rmse_corr = rmse(e_corr)

    f_pred = final_position_error(X_pred, Z, Z_valid)
    f_corr = final_position_error(X_corr, Z, Z_valid)

    print("\n=== Metrics ===")
    print("RMSE pred-only:", rmse_pred)
    print("RMSE corr-on  :", rmse_corr)
    print("Final error pred-only:", f_pred)
    print("Final error corr-on  :", f_corr)

    # Min/Max debug for covariance
    print("\n=== Covariance min/max ===")
    print("valid camera frames:", int(np.sum(Z_valid)), "/", len(Z_valid))
    print("Pxx pred min/max:", float(P_pred[:, 0, 0].min()), float(P_pred[:, 0, 0].max()))
    print("Pyy pred min/max:", float(P_pred[:, 1, 1].min()), float(P_pred[:, 1, 1].max()))
    print("Ptt pred min/max:", float(P_pred[:, 2, 2].min()), float(P_pred[:, 2, 2].max()))
    print("Pxx corr min/max:", float(P_corr[:, 0, 0].min()), float(P_corr[:, 0, 0].max()))
    print("Pyy corr min/max:", float(P_corr[:, 1, 1].min()), float(P_corr[:, 1, 1].max()))
    print("Ptt corr min/max:", float(P_corr[:, 2, 2].min()), float(P_corr[:, 2, 2].max()))

    # Plots
    plot_trajectory_with_ellipses(
        X_pred, P_pred, X_corr, P_corr,
        Z, Z_valid,
        title="Offline EKF Comparison"
    )
    plot_covariance_split(P_pred, P_corr, t)

    # Error plot
    if np.sum(Z_valid) > 0:
        t_valid = t[Z_valid]
        plot_error_over_time(e_pred, e_corr, t_valid)

    plt.show()