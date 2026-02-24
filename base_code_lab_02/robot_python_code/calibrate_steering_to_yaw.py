# calibrate_steering_to_yaw.py
#
# Use your logged trials to fit a mapping
#       w = f_w(alpha) ≈ k * alpha
# and estimate a (possibly constant) variance sigma_w^2.
#
# Run from robot_python_code/ with:
#     python3 calibrate_steering_to_yaw.py

import math
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# 1. Enter your steering experiments here
#    One entry per trial:
#    file name, steering command (alpha), trial duration (sec), final yaw angle (deg)
# ----------------------------------------------------------------------

trials = [
    # ---------- positive steering ----------
    # alpha = 5
    ("robot_data_0_0_10_02_26_17_37_54.pkl",  5, 4.0,  20),
    ("robot_data_0_0_10_02_26_18_26_09.pkl",  5, 3.0,  17),
    ("robot_data_0_0_10_02_26_18_35_32.pkl",  5, 3.0,  20),

    # alpha = 10
    ("robot_data_0_0_10_02_26_17_38_42.pkl", 10, 4.0,  35),
    ("robot_data_0_0_10_02_26_18_36_55.pkl", 10, 4.0,  36),
    ("robot_data_0_0_10_02_26_18_39_42.pkl", 10, 4.0,  34),

    # alpha = 15
    ("robot_data_0_0_10_02_26_17_41_40.pkl", 15, 4.0,  40),
    ("robot_data_0_0_10_02_26_18_41_41.pkl", 15, 4.0,  43),
    ("robot_data_0_0_10_02_26_18_45_03.pkl", 15, 4.0,  42),

    # alpha = 20
    ("robot_data_0_0_10_02_26_17_29_34.pkl", 20, 4.0,  35),
    ("robot_data_0_0_10_02_26_18_46_46.pkl", 20, 4.0,  36),
    ("robot_data_38_20_10_02_26_18_48_02.pkl", 20, 4.0,  39),

    # ---------- negative steering ----------
    # alpha = -5
    ("robot_data_0_0_10_02_26_17_30_37.pkl", -5, 4.0, -15),
    ("robot_data_0_0_10_02_26_18_55_24.pkl", -5, 4.0, -13),
    ("robot_data_0_0_10_02_26_18_58_24.pkl", -5, 4.0, -14),

    # alpha = -10
    ("robot_data_0_0_10_02_26_17_31_53.pkl", -10, 4.0, -40),
    ("robot_data_0_0_10_02_26_18_59_26.pkl", -10, 4.0, -50),
    ("robot_data_0_0_10_02_26_19_00_56.pkl", -10, 4.0, -51),
    ("robot_data_0_0_10_02_26_19_01_53.pkl", -10, 4.0, -52),

    # alpha = -15
    ("robot_data_0_0_10_02_26_17_33_47.pkl", -15, 4.0, -60),
    ("robot_data_0_0_10_02_26_19_03_20.pkl", -15, 4.0, -71),
    ("robot_data_0_0_10_02_26_19_04_06.pkl", -15, 4.0, -53),
    ("robot_data_0_0_10_02_26_19_07_32.pkl", -15, 4.0, -58),
    ("robot_data_0_0_10_02_26_19_08_35.pkl", -15, 4.0, -54),
    ("robot_data_0_0_10_02_26_19_11_58.pkl", -15, 4.0, -55),

    # alpha = -20
    ("robot_data_0_0_10_02_26_17_34_41.pkl", -20, 4.0, -75),
    ("robot_data_0_0_10_02_26_19_09_42.pkl", -20, 4.0, -74),
    ("robot_data_0_0_10_02_26_19_10_49.pkl", -20, 4.0, -75),
]

# ----------------------------------------------------------------------
# 2. Convert to yaw rates
# ----------------------------------------------------------------------

alphas = []
ws = []

print("Per-trial yaw rates (rad/s):")
for fname, alpha, T, theta_deg in trials:
    theta_rad = math.radians(theta_deg)
    w = theta_rad / T            # yaw rate rad/s
    alphas.append(alpha)
    ws.append(w)
    print(f"{fname:40s} alpha={alpha:>4}, T={T:4.1f}s, theta={theta_deg:6.1f}°, w={w:7.4f} rad/s")

alphas = np.asarray(alphas, dtype=float)
ws = np.asarray(ws, dtype=float)

# ----------------------------------------------------------------------
# 3. Fit w = k * alpha (through origin, least squares)
# ----------------------------------------------------------------------

k = (alphas @ ws) / (alphas @ alphas)
ws_hat = k * alphas
errors = ws - ws_hat
sigma_w2 = np.mean(errors**2)

print("\nFitted global model:")
print(f"  w = {k:.4e} * alpha")
print(f"  sigma_w^2 (constant) ≈ {sigma_w2:.4e} (rad^2/s^2)")

# Optional: separate positive / negative fits (just for info)
mask_pos = alphas > 0
mask_neg = alphas < 0
if mask_pos.any():
    k_pos = (alphas[mask_pos] @ ws[mask_pos]) / (alphas[mask_pos] @ alphas[mask_pos])
    print(f"  positive steering: w ≈ {k_pos:.4e} * alpha")
if mask_neg.any():
    k_neg = (alphas[mask_neg] @ ws[mask_neg]) / (alphas[mask_neg] @ alphas[mask_neg])
    print(f"  negative steering: w ≈ {k_neg:.4e} * alpha")

# ----------------------------------------------------------------------
# 4. Plots
# ----------------------------------------------------------------------

# (a) yaw rate vs steering, with fitted line
plt.figure()
plt.scatter(alphas, ws, label='Measured w')
alpha_line = np.linspace(alphas.min() * 1.1, alphas.max() * 1.1, 200)
plt.plot(alpha_line, k * alpha_line, label='Fitted w = k·alpha')
plt.xlabel('Steering command alpha')
plt.ylabel('Yaw rate w (rad/s)')
plt.title('Yaw rate vs steering command')
plt.grid(True)
plt.legend()

# (b) squared error vs steering, with constant variance line
plt.figure()
plt.scatter(alphas, errors**2, label='Squared error')
plt.hlines(sigma_w2, alpha_line.min(), alpha_line.max(), linestyles='dashed', label='σ_w² (constant)')
plt.xlabel('Steering command alpha')
plt.ylabel('Squared error (rad²/s²)')
plt.title('Distance error vs steering (for w)')
plt.grid(True)
plt.legend()

plt.show()
