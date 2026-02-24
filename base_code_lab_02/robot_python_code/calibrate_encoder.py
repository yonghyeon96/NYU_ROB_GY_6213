import os
import numpy as np
import math
import matplotlib.pyplot as plt

import robot_python_code  # lab's internal module

# hand recorded distance

true_distances = {
    # new straight-line X_G, Y_G data (all speed 36)
    "robot_data_34_0_10_02_26_19_21_42.pkl": math.hypot(0.0,   0.295),  # ≈ 0.295
    "robot_data_0_0_10_02_26_19_24_15.pkl":  math.hypot(0.0,   0.294),  # ≈ 0.294
    "robot_data_0_0_10_02_26_19_25_18.pkl":  math.hypot(0.0,   0.305),  # ≈ 0.305

    "robot_data_0_0_10_02_26_19_28_13.pkl":  math.hypot(0.035, 0.845),  # ≈ 0.846
    "robot_data_0_0_10_02_26_19_29_28.pkl":  math.hypot(0.03,  0.82),   # ≈ 0.821
    "robot_data_0_0_10_02_26_19_31_42.pkl":  math.hypot(0.043, 0.86),   # ≈ 0.861

    "robot_data_0_0_10_02_26_19_33_02.pkl":  math.hypot(0.11,  1.265),  # ≈ 1.269
    "robot_data_0_0_10_02_26_19_36_45.pkl":  math.hypot(0.09,  1.32),   # ≈ 1.323
    "robot_data_0_0_10_02_26_19_38_19.pkl":  math.hypot(0.09,  1.41),   # ≈ 1.413

    "robot_data_0_0_10_02_26_19_39_50.pkl":  math.hypot(0.145, 1.835),  # ≈ 1.840
    "robot_data_0_0_10_02_26_19_42_09.pkl":  math.hypot(0.25,  1.81),   # ≈ 1.827
    "robot_data_0_0_10_02_26_19_43_27.pkl":  math.hypot(0.22,  1.805),  # ≈ 1.818
}

BASE_DIR = os.path.dirname(__file__)
DATA_DIR = os.path.join(BASE_DIR, "data")

encoder_vals = []
distance_vals = []

# --------------------------------------------------
# 2. Load encoder counts via DataLoader (same as data_handling.py)
# --------------------------------------------------
for fname, dist in true_distances.items():
    filepath = os.path.join(DATA_DIR, fname)
    if not os.path.exists(filepath):
        print(f"Skipping {fname} (file not found at {filepath})")
        continue

    # Use the lab's DataLoader
    data_loader = robot_python_code.DataLoader(filepath)
    data_dict = data_loader.load()

    # data_dict keys: ['time', 'control_signal', 'robot_sensor_signal', 'camera_sensor_signal']
    robot_sensor_signal_list = data_dict['robot_sensor_signal']

    # same pattern as get_file_data()
    encoder_count_list = [row.encoder_counts for row in robot_sensor_signal_list]

    # total encoder change over the trial
    encoder_delta = encoder_count_list[-1] - encoder_count_list[0]

    encoder_vals.append(encoder_delta)
    distance_vals.append(dist)

encoder_vals = np.array(encoder_vals, dtype=float)
distance_vals = np.array(distance_vals, dtype=float)

print("Encoder deltas:", encoder_vals)
print("Measured distances (m):", distance_vals)

if len(encoder_vals) == 0:
    raise RuntimeError("No valid trials loaded – check filenames and DATA_DIR.")

# --------------------------------------------------
# 3. Fit distance model: s = a * e  (line through origin)
# --------------------------------------------------
a = (encoder_vals @ distance_vals) / (encoder_vals @ encoder_vals)

print(f"\nFitted distance model:")
print(f"s = {a:.6e} * encoder_count")

s_pred = a * encoder_vals

# --------------------------------------------------
# 4. Fit variance model: sigma_s^2 = alpha * s
# --------------------------------------------------
errors = distance_vals - s_pred
errors_sq = errors ** 2

alpha = (s_pred @ errors_sq) / (s_pred @ s_pred)

print(f"\nFitted variance model:")
print(f"sigma_s^2 = {alpha:.6e} * s")

# --------------------------------------------------
# 5. Plot results
# --------------------------------------------------
plt.figure()
plt.scatter(encoder_vals, distance_vals, label="Measured distance")
e_line = np.linspace(0, encoder_vals.max() * 1.1, 100)
plt.plot(e_line, a * e_line, label="Fitted line", linewidth=2)
plt.xlabel("Encoder count delta")
plt.ylabel("Distance (m)")
plt.title("Encoder count vs distance")
plt.legend()
plt.grid(True)

plt.figure()
plt.scatter(encoder_vals, errors_sq, label="Squared error")
plt.plot(e_line, alpha * (a * e_line), label="Variance model", linewidth=2)
plt.xlabel("Encoder count delta")
plt.ylabel("Squared error (m²)")
plt.title("Distance error vs encoder count")
plt.legend()
plt.grid(True)

plt.show()
