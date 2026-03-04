# External libraries
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Local libraries
import parameters
import data_handling

# Main class
class ExtendedKalmanFilter:
    def __init__(self, x_0, Sigma_0, encoder_counts_0):
        self.state_mean = x_0
        self.state_covariance = Sigma_0
        self.predicted_state_mean = [0,0,0]
        self.predicted_state_covariance = parameters.I3 * 1.0
        self.last_encoder_counts = encoder_counts_0

    """
    u_t = [encoder_counts, steering_angle_command]
    z_t = [x_cam, y_cam, theta_cam]
    delta_t = time step size in seconds
    """

    # Call the prediction and correction steps
    def update(self, u_t, z_t, delta_t):
        # 1) Prediction
        self.prediction_step(u_t, delta_t)
        # 2) Correction
        # IF NO MEASUREMENT, skip correction and set variables to predicted values
        if z_t is None:
            self.state_mean = self.predicted_state_mean
            self.state_covariance = self.predicted_state_covariance
            return
        try:
            if len(z_t) < 3:
                self.state_mean = self.predicted_state_mean
                self.state_covariance = self.predicted_state_covariance
                return
            z_arr = np.asarray(z_t, dtype=float)
            if np.any(np.isnan(z_arr)):
                self.state_mean = self.predicted_state_mean
                self.state_covariance = self.predicted_state_covariance
                return
        except Exception:
            self.state_mean = self.predicted_state_mean
            self.state_covariance = self.predicted_state_covariance
            return 
        # DO CORRECTION STEP       
        self.correction_step(z_arr)

    """
    EKF prediction:
        mean: x_pred = g_function(x_prev, u_t, delta_t)
        covariance: G_x = get_G_x(x_prev, u_t, delta_t)
    """

    # Set the EKF's predicted state mean and covariance matrix
    def prediction_step(self, u_t, delta_t):
        # 1) Non-linear motion model 
        x_prev = np.asarray(self.state_mean, dtype=float)
        x_pred, s = self.g_function(x_prev, u_t, delta_t)
        # 2) Jacobians
        Gx = self.get_G_x(x_prev, s)
        Gu = self.get_G_u(x_prev, delta_t)
        # 3) Predicted covariance
        R = self.get_R(s)
        # 4) Predicted state mean and covariance
        Sigma_prev = np.asarray(self.state_covariance, dtype=float)
        Sigma_pred = Gx @ Sigma_prev @ Gx.T + Gu @ R @ Gu.T
        # 5) Set predicted mean and covariance
        self.predicted_state_mean = np.asarray(x_pred, dtype=float)
        self.predicted_state_covariance = np.asarray(Sigma_pred, dtype=float)


    """
    EKF Correction:
    y = z_t - h_function(x_pred)
    S = H @ Sigma_pred @ H.T + Q
    K = Sigma_pred @ H.T @ S^-1
    x = x_pred + K @ y
    Sigma = (I - K @ H) @ Sigma_pred
    """
    # Set the EKF's corrected state mean and covariance matrix
    def correction_step(self, z_t):
        # 1) Measurement prediction
        x_pred = np.asarray(self.predicted_state_mean, dtype=float)
        Sigma_pred = np.asarray(self.predicted_state_covariance, dtype=float)
        # 2) Measurement Jacobian
        z = np.asarray(z_t, dtype=float)
        h = np.asarray(self.get_h_function(x_pred), dtype=float)
        H = np.asarray(self.get_H(), dtype=float)
        # 3) Measurement covariance
        Q = np.asarray(self.get_Q(), dtype=float)
        # 4) Kalman gain and correction (innovation)
        y = z - h
        y[2] = (y[2] + math.pi) % (2 * math.pi) - math.pi
        # 5) innovation covariance
        S = H @ Sigma_pred @ H.T + Q
        # 6) Kalman gain
        K = Sigma_pred @ H.T @ np.linalg.inv(S)
        # 7) Corrected state mean and covariance
        x_new = x_pred + K @ y
        x_new[2] = (x_new[2] + math.pi) % (2 * math.pi) - math.pi
        # 8) Set corrected mean and covariance
        I = parameters.I3
        Sigma_new = (I - K @ H) @ Sigma_pred
        # 9) Update EKF state
        self.state_mean = x_new
        self.state_covariance = Sigma_new
        
    # Function to calculate distance from encoder counts
    def distance_travelled_s(self, encoder_counts):
        return 3.05e-4 * encoder_counts
            
    # Function to calculate rotational velocity from steering and dist travelled or speed
    
    def rotational_velocity_w(self, steering_angle_command):        
        k = 1.4357e-02
        return k * steering_angle_command

    # The nonlinear transition equation that provides new states from past states
    def g_function(self, x_tm1, u_t, delta_t):
        x_tm1 = np.asarray(x_tm1, dtype=float)
        encoder_counts = float(u_t[0])
        steering = float(u_t[1])

        delta_e = encoder_counts - float(self.last_encoder_counts)
        self.last_encoder_counts = encoder_counts

        # delta encoder -> distance
        s = self.distance_travelled_s(delta_e)
        # steering -> yaw rate
        w = self.rotational_velocity_w(steering)

        x, y, theta = x_tm1

        x_new = x + s * math.cos(theta)
        y_new = y + s * math.sin(theta)
        theta_new = theta + w * delta_t

        # theta normalize to [-pi, pi] (optional normalization)
        theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi

        x_t = np.array([x_new, y_new, theta_new], dtype=float)
        return x_t, s
    
    # The nonlinear measurement function
    def get_h_function(self, x_t):
        return np.asarray(x_t, dtype=float)
    
    # This function returns a matrix with the partial derivatives dg/dx
    # g outputs x_t, y_t, theta_t, and we take derivatives wrt inputs x_tm1, y_tm1, theta_tm1
    def get_G_x(self, x_tm1, s):       
        x_tm1 = np.asarray(x_tm1, dtype=float)
        theta = float(x_tm1[2])

        Gx = np.array([
            [1.0, 0.0, -s * math.sin(theta)],
            [0.0, 1.0,  s * math.cos(theta)],
            [0.0, 0.0, 1.0]
        ], dtype=float)
        return Gx

    # This function returns a matrix with the partial derivatives dg/du
    def get_G_u(self, x_tm1, delta_t):                
        x_tm1 = np.asarray(x_tm1, dtype=float)
        theta = float(x_tm1[2])

        a = 3.05e-4      # ds/d(delta_encoder)
        k = 1.4357e-02   # dw/d(steering)

        Gu = np.array([
            [a * math.cos(theta), 0.0],
            [a * math.sin(theta), 0.0],
            [0.0,               k * delta_t]
        ], dtype=float)
        return Gu

    # This function returns a matrix with the partial derivatives dh_t/dx_t
    def get_H(self):
        H = np.identity(3)
        return H
    
    # This function returns the R_t matrix which contains transition function covariance terms.
    def get_R(self, s):
        alpha = 1.79e-4
        var_s = alpha * abs(float(s))
        var_w = 3.35e-3

        a = 3.05e-4
        k = 1.4357e-02

        # avoid divide-by-zero
        var_enc = var_s / (a * a) if a != 0 else 0.0
        var_steer = var_w / (k * k) if k != 0 else 0.0

        R = np.array([
            [var_enc,   0.0],
            [0.0,    var_steer]
        ], dtype=float)
        return R

    # This function returns the Q_t matrix which contains measurement covariance terms.
    def get_Q(self):
        ########## WE CAN TUNE THESE VALUES #############
        sigma_x = 0.02          # meters
        sigma_y = 0.02          # meters
        sigma_theta = 0.05      # radians
        ##################################################

        Q = np.array([
            [sigma_x**2, 0.0,        0.0],
            [0.0,        sigma_y**2, 0.0],
            [0.0,        0.0,        sigma_theta**2]
        ], dtype=float)
        return Q

class KalmanFilterPlot:

    def __init__(self):
        self.dir_length = 0.1
        fig, ax = plt.subplots()
        self.ax = ax
        self.fig = fig

    def update(self, state_mean, state_covaraiance):
        plt.clf()

        # Plot covariance ellipse
        lambda_, v = np.linalg.eig(state_covaraiance)
        lambda_ = np.sqrt(lambda_)
        xy = (state_mean[0], state_mean[1])
        angle=np.rad2deg(np.arctan2(*v[:,0][::-1]))
        ell = Ellipse(xy, alpha=0.5, facecolor='red',width=lambda_[0], height=lambda_[1], angle = angle)
        ax = self.fig.gca()
        ax.add_artist(ell)
        
        # Plot state estimate
        plt.plot(state_mean[0], state_mean[1],'ro')
        plt.plot([state_mean[0], state_mean[0]+ self.dir_length*math.cos(state_mean[2]) ], [state_mean[1], state_mean[1]+ self.dir_length*math.sin(state_mean[2]) ],'r')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.axis([-0.25, 1.3, -1, 1])
        plt.grid()
        plt.draw()
        plt.pause(0.1)


# Code to run your EKF offline with a data file.
def offline_efk():

    # Get data to filter
    filename = './data/robot_data_60_20_25_02_26_16_16_45.pkl'
    ekf_data = data_handling.get_file_data_for_kf(filename)

    # Instantiate PF with no initial guess
    x_0 = [ekf_data[0][3][0]+.5, ekf_data[0][3][1], ekf_data[0][3][5]]
    Sigma_0 = parameters.I3
    encoder_counts_0 = ekf_data[0][2].encoder_counts
    extended_kalman_filter = ExtendedKalmanFilter(x_0, Sigma_0, encoder_counts_0)

    # Create plotting tool for ekf
    kalman_filter_plot = KalmanFilterPlot()

    # Loop over sim data
    for t in range(1, len(ekf_data)):
        row = ekf_data[t]
        delta_t = ekf_data[t][0] - ekf_data[t-1][0] # time step size
        ##### prediction step #####
        u_t = np.array([row[2].encoder_counts, row[2].steering]) # robot_sensor_signal
        ##### correction step #####
        z_t = np.array([row[3][0],row[3][1],row[3][5]]) # camera_sensor_signal

        # Run the EKF for a time step
        extended_kalman_filter.update(u_t, z_t, delta_t)
        kalman_filter_plot.update(extended_kalman_filter.state_mean, extended_kalman_filter.state_covariance[0:2,0:2])


####### MAIN #######
if True:
    offline_efk()
