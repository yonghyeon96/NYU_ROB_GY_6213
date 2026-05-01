# External Libraries
import math
import random

# Motion Model constants


# A function for obtaining variance in distance travelled as a function of distance travelled
def variance_distance_travelled_s(distance):
    # Add student code here
    # Variance model: sigma_s^2 = alpha * s
    alpha = 1.79e-4
    var_s = alpha * distance

    return var_s

# Function to calculate distance from encoder counts
def distance_travelled_s(encoder_counts):
    # Add student code here
    # Add student code here
    s = 3.05e-4 * encoder_counts  #Linear model, y = ax

    return s

# A function for obtaining variance in distance travelled as a function of distance travelled
def variance_rotational_velocity_w(distance):
    # Add student code here
    var_w = 3.35e-3

    return var_w

def rotational_velocity_w(steering_angle_command):
    # Add student code here
 
    k = 1.4357e-02
    
#   Asymmetric fit
#    if steering_angle_command >= 0:
#        k = 1.0611e-02
#    else:
#        k = 1.7143e-02
        
    w = k * steering_angle_command

    return w

# This class is an example structure for implementing your motion model.
class MyMotionModel:

    # Constructor, change as you see fit.
    def __init__(self, initial_state, last_encoder_count):
        self.state = initial_state
        self.last_encoder_count = last_encoder_count

    # This is the key step of your motion model, which implements x_t = f(x_{t-1}, u_t)
    def step_update(self, encoder_counts, steering_angle_command, delta_t):
        # Add student code here
        # 1) encoder delta since last step
        delta_e = encoder_counts - self.last_encoder_count
        self.last_encoder_count = encoder_counts

        # 2) forward distance travelled in this step (meters)
        s = distance_travelled_s(delta_e)

        # 3) yaw rate from steering command (rad/s)
        w = rotational_velocity_w(steering_angle_command)
        var_s = variance_distance_travelled_s(abs(s))
        std_s = math.sqrt(var_s) if var_s > 0 else 0.0
        s_sample = s + random.gauss(0.0, std_s)

        var_w = variance_rotational_velocity_w(abs(s))
        std_w = math.sqrt(var_w) if var_w > 0 else 0.0
        w_sample = w + random.gauss(0.0, std_w)

        # 4) previous state
        x, y, theta = self.state

        # 5) propagate state: simple unicycle approximation
        # move s along current heading, then update heading by w * dt
        x_new = x + s_sample * math.cos(theta)
        y_new = y + s_sample * math.sin(theta)
        theta_new = theta + w_sample * delta_t

        self.state = [x_new, y_new, theta_new]
        return self.state
    
    # This is a great tool to take in data from a trial and iterate over the data to create 
    # a robot trajectory in the global frame, using your motion model.
    def traj_propagation(self, time_list, encoder_count_list, steering_angle_list):
        x_list = [self.state[0]]
        y_list = [self.state[1]]
        theta_list = [self.state[2]]
        self.last_encoder_count = encoder_count_list[0]
        for i in range(1, len(encoder_count_list)):
            delta_t = time_list[i] - time_list[i-1]
            new_state = self.step_update(encoder_count_list[i], steering_angle_list[i], delta_t)
            x_list.append(new_state[0])
            y_list.append(new_state[1])
            theta_list.append(new_state[2])

        return x_list, y_list, theta_list
    

    # Coming soon
    def generate_simulated_traj(self, duration):
        delta_t = 0.1

        t_list = [0.0]
        x_list = [self.state[0]]
        y_list = [self.state[1]]
        theta_list = [self.state[2]]

        t = 0.0
        encoder_counts = self.last_encoder_count

        while t < duration:
            # --- fake some controls for this time step ---

            # pretend the encoder count keeps increasing roughly linearly
            # (you can tune these numbers if you want it faster/slower)
            delta_counts = random.randint(40, 60)
            encoder_counts += delta_counts

            # choose a random steering command from a small set
            steering_angle_command = 0 #random.choice([-20, -10, -5, 0, 5, 10, 20])

            # --- propagate the state using your step_update ---
            new_state = self.step_update(encoder_counts,
                                        steering_angle_command,
                                        delta_t)
            x_list.append(new_state[0])
            y_list.append(new_state[1])
            theta_list.append(new_state[2])

            t += delta_t
            t_list.append(t)

        return t_list, x_list, y_list, theta_list
            