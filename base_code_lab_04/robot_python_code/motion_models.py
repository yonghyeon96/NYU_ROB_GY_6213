# External Libraries
import math
import random

import parameters


def variance_distance_travelled_s(distance):
    return parameters.distance_variance_gain * distance


def distance_travelled_s(encoder_counts):
    return parameters.encoder_counts_to_distance * encoder_counts


def variance_rotational_velocity_w(distance):
    return parameters.rotational_velocity_variance


def rotational_velocity_w(steering_angle_command):
    return parameters.steering_to_w * steering_angle_command


class MyMotionModel:

    def __init__(self, initial_state, last_encoder_count):
        self.state = initial_state
        self.last_encoder_count = last_encoder_count

    def step_update(self, encoder_counts, steering_angle_command, delta_t):
        delta_e = encoder_counts - self.last_encoder_count
        self.last_encoder_count = encoder_counts

        s = distance_travelled_s(delta_e)
        w = rotational_velocity_w(steering_angle_command)

        var_s = variance_distance_travelled_s(abs(s))
        std_s = math.sqrt(var_s) if var_s > 0 else 0.0
        s_sample = s + random.gauss(0.0, std_s)

        var_w = variance_rotational_velocity_w(abs(s))
        std_w = math.sqrt(var_w) if var_w > 0 else 0.0
        w_sample = w + random.gauss(0.0, std_w)

        x, y, theta = self.state
        x_new = x + s_sample * math.cos(theta)
        y_new = y + s_sample * math.sin(theta)
        theta_new = theta + w_sample * delta_t

        self.state = [x_new, y_new, theta_new]
        return self.state

    def traj_propagation(self, time_list, encoder_count_list, steering_angle_list):
        x_list = [self.state[0]]
        y_list = [self.state[1]]
        theta_list = [self.state[2]]
        self.last_encoder_count = encoder_count_list[0]
        for i in range(1, len(encoder_count_list)):
            delta_t = time_list[i] - time_list[i - 1]
            new_state = self.step_update(encoder_count_list[i], steering_angle_list[i], delta_t)
            x_list.append(new_state[0])
            y_list.append(new_state[1])
            theta_list.append(new_state[2])
        return x_list, y_list, theta_list

    def generate_simulated_traj(self, duration):
        delta_t = 0.1

        t_list = [0.0]
        x_list = [self.state[0]]
        y_list = [self.state[1]]
        theta_list = [self.state[2]]

        t = 0.0
        encoder_counts = self.last_encoder_count

        while t < duration:
            delta_counts = random.randint(40, 60)
            encoder_counts += delta_counts
            steering_angle_command = 0

            new_state = self.step_update(encoder_counts, steering_angle_command, delta_t)
            x_list.append(new_state[0])
            y_list.append(new_state[1])
            theta_list.append(new_state[2])

            t += delta_t
            t_list.append(t)

        return t_list, x_list, y_list, theta_list
