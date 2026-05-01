# External libraries
import math
import numpy as np

# UDP parameters
localIP = "192.168.0.199" # Put your laptop computer's IP here 199
arduinoIP = "192.168.0.200" # Put your arduino's IP here 200
localPort = 4010
arduinoPort = 4010
bufferSize = 1024

# Camera parameters
camera_id = 0
marker_length = 0.071
camera_matrix = np.array([[1.41089024e+03, 0.00000000e+00 ,5.34757040e+02],
 [0.00000000e+00 ,1.40977771e+03, 4.63300611e+02],
 [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]], dtype=np.float32)
dist_coeffs = np.array([-0.32511173, -0.09273864 ,-0.00295959 , 0.00111094 , 0.2446519 ], dtype=np.float32)

# Robot parameters
num_robot_sensors = 2 # encoder, steering
num_robot_control_signals = 2 # speed, steering

# Logging parameters
max_num_lines_before_write = 1
filename_start = './data/robot_data'
data_name_list = ['time', 'control_signal', 'robot_sensor_signal', 'state_mean', 'wp_icp_debug']

# Experiment trial parameters
trial_time = 10000 # milliseconds
extra_trial_log_time = 2000 # milliseconds

# KF parameters
I3 = np.array([[1, 0, 0],[0, 1, 0], [0, 0, 1]])
covariance_plot_scale = 100

# PF parameters, modify the map and num particles as you see fit.
num_particles = 200
# Lidar likelihood variance sigma^2 in m^2 (not sigma).
distance_variance_m2 = 0.10
# Backward-compatible alias used by existing code.
distance_variance = distance_variance_m2
lidar_max_range_m = 12.0
encoder_counts_to_distance = 3.05e-4  # m / encoder_count
steering_to_w = -1.4357e-2  # rad/s per steering command unit
distance_variance_gain = 1.79e-4  # motion model variance gain for traveled distance
rotational_velocity_variance = 3.35e-3  # (rad/s)^2
# Map coordinates from map.jpeg.
# Units are metres. Origin is the lower-left corner in the photo:
# +x points right, +y points up.
wall_corner_list = [
    [0.00, 0.00, 2.09, 0.00],  # A -> B: bottom-left wall, 209 cm
    [2.09, 0.00, 2.09, 0.61],  # B -> C: inner vertical step, 61 cm
    [2.09, 0.61, 3.04, 0.61],  # C -> D: lower-right wall, 95 cm
    [3.04, 0.61, 3.04, 2.01],  # D -> E: right wall, 140 cm
    [3.04, 2.01, 0.00, 2.01],  # E -> F: top wall, 304 cm
    [0.00, 2.01, 0.00, 0.00],  # F -> A: left wall, 201 cm
]

# Initial pose of the LiDAR centre. Edit before each trial if the physical
# start marker is moved or if the LiDAR centre is not exactly on the marker.
initial_pose = [0.30, 1.00, 0.0]   # [x_m, y_m, theta_rad]

# --- WP-ICP parameters ---

# RPLidar spec (verify actual angular resolution from spec sheet)
lidar_angular_resolution_deg = 1.0

# Feature extraction (values from paper experiments)
arc_noise_scale_N = 15
arc_min_cluster_size = 5
split_merge_threshold_dc = 0.10      # m  (paper d_c = 10 cm)
split_merge_min_points = 3           # minimum cluster size to recurse
scan_min_range_m = 0.05

# ICP (reject threshold from paper: 50 cm)
icp_max_iterations = 50
icp_convergence_tol = 1e-4
icp_error_tol = 1e-5
icp_reject_threshold = 0.5           # m
icp_min_matches = 5
icp_max_pose_jump_m = 1.0
icp_max_heading_jump_rad = 0.8

# WP-ICP
wp_icp_min_corner_matches = 2
wp_icp_min_line_matches = 5
wp_icp_line_downsample_step = 1      # 1 = no downsampling

# Map interpolation (paper value: 10 cm)
map_interpolation_step = 0.10        # m
map_duplicate_tol = 1e-6
