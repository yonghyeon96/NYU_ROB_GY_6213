import time
import numpy as np

import parameters
import robot_python_code
import motion_models
import map_builder
import feature_extraction
import icp
import wp_icp

# ---------------------------------------------------------------------------
# Localisation mode:  'icp'  or  'wp_icp'
# ---------------------------------------------------------------------------
LOCALIZATION_MODE = 'wp_icp'

# Set this before starting the control loop (metres, radians).
INITIAL_POSE = list(parameters.initial_pose)   # edit parameters.py before each trial


class Robot:

    def __init__(self):
        self.connected_to_hardware = False
        self.running_trial         = False
        self.msg_sender            = None
        self.msg_receiver          = None

        # sensor / logging
        self.data_logger        = robot_python_code.DataLogger(
            parameters.filename_start, parameters.data_name_list)
        self.robot_sensor_signal = robot_python_code.RobotSensorSignal([0, 0, 0])

        # map features (built once at startup)
        self.map_features = map_builder.build_map_features(
            parameters.wall_corner_list, parameters.map_interpolation_step)
        print(f"[Robot] Map built: {self.map_features.full_tree.n} full-tree pts")

        # motion model
        self.motion_model  = None    # initialised on first valid sensor msg
        self.last_time     = None
        self.current_pose  = list(INITIAL_POSE)

        # debug info stored each step (for logging and GUI)
        self.debug_info = {}

        # last scan / feature data exposed for GUI overlay
        self.last_scan_pts    = np.empty((0, 2))
        self.last_feature_set = None

    # -----------------------------------------------------------------------
    # UDP plumbing (unchanged interface from lab04)
    # -----------------------------------------------------------------------

    def setup_udp_connection(self, udp_communication):
        self.msg_sender   = robot_python_code.MsgSender(
            time.perf_counter(), parameters.num_robot_control_signals, udp_communication)
        self.msg_receiver = robot_python_code.MsgReceiver(
            time.perf_counter(), parameters.num_robot_sensors, udp_communication)
        print("[Robot] UDP connected.")

    def eliminate_udp_connection(self):
        self.msg_sender   = None
        self.msg_receiver = None
        print("[Robot] UDP disconnected.")

    # -----------------------------------------------------------------------
    # State estimation
    # -----------------------------------------------------------------------

    def update_state_estimate(self, delta_t):
        sensor = self.robot_sensor_signal

        # --- initialise motion model on first call ---
        if self.motion_model is None:
            self.motion_model = motion_models.MyMotionModel(
                list(INITIAL_POSE), sensor.encoder_counts)
            return   # nothing to update yet

        # --- warm start from encoder ---
        warm = self.motion_model.step_update(
            sensor.encoder_counts, sensor.steering, delta_t)

        # --- convert LiDAR to sensor-frame points ---
        scan_pts, _, _ = feature_extraction.lidar_to_sensor_frame(sensor)
        self.last_scan_pts = scan_pts

        if len(scan_pts) == 0:
            self.last_feature_set = None
            self.current_pose = list(warm)
            self.motion_model.state = list(warm)
            self.debug_info = {'dropped_scan': True, 'mode': LOCALIZATION_MODE}
            return

        # --- run chosen localisation ---
        if LOCALIZATION_MODE == 'icp':
            self.last_feature_set = None
            result = icp.update(scan_pts, warm, self.map_features)
            if result.converged:
                self.current_pose = result.pose
            else:
                self.current_pose = list(warm)
            self.debug_info = {
                'mode': 'icp',
                'iterations': result.iterations,
                'runtime_sec': result.runtime_sec,
                'num_matches': result.num_matches,
                'mean_error': result.mean_error,
                'converged': result.converged,
                'dropped_scan': False,
            }

        else:   # 'wp_icp'
            fs = feature_extraction.extract_features(sensor)
            self.last_feature_set = fs
            result = wp_icp.update(fs, warm, self.map_features)
            if result.converged:
                self.current_pose = result.pose
            else:
                self.current_pose = list(warm)
            self.debug_info = {
                'mode': 'wp_icp',
                'corner_iterations': result.corner_iterations,
                'line_iterations':   result.line_iterations,
                'runtime_sec':       result.runtime_sec,
                'corner_confidence': result.corner_confidence,
                'line_confidence':   result.line_confidence,
                'corner_matches':    result.corner_matches,
                'line_matches':      result.line_matches,
                'num_scan_points':   len(scan_pts),
                'num_corners':       len(fs.corners),
                'num_line_points':   len(fs.line_points),
                'converged':         result.converged,
                'dropped_scan':      False,
            }

        # sync motion model to corrected pose
        self.motion_model.state = list(self.current_pose)

    # -----------------------------------------------------------------------
    # Control loop  (called repeatedly by GUI)
    # -----------------------------------------------------------------------

    def control_loop(self, cmd_speed=0, cmd_steering_angle=0, logging_switch_on=False):
        now = time.perf_counter()

        # receive sensor msg
        if self.msg_receiver is not None:
            self.robot_sensor_signal = self.msg_receiver.receive_robot_sensor_signal(
                self.robot_sensor_signal)

        # compute delta_t
        delta_t = (now - self.last_time) if self.last_time is not None else 0.1
        delta_t = max(delta_t, 1e-3)
        self.last_time = now

        # update pose estimate
        self.update_state_estimate(delta_t)

        # send control signal
        control_signal = [cmd_speed, cmd_steering_angle]
        if self.msg_sender is not None:
            self.msg_sender.send_control_signal(control_signal)

        # log  (state_mean = current pose, wp_icp_debug = debug_info dict)
        self.data_logger.log(
            logging_switch_on,
            now,
            control_signal,
            self.robot_sensor_signal,
            self.current_pose,
            self.debug_info,
        )
