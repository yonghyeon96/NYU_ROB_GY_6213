# External libraries
import copy
import matplotlib.pyplot as plt
import math
import numpy as np
import random

# Local libraries
import parameters
import data_handling

# Helper function to make sure all angles are between -pi and pi
def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    return angle

# Helper class to store and manipulate your states.
class State:

    # Constructor
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # Get the euclidean distance between 2 states
    def distance_to(self, other_state):
        return math.sqrt(math.pow(self.x - other_state.x, 2) + math.pow(self.y - other_state.y, 2))
        
    # Get the distance squared between two states
    def distance_to_squared(self, other_state):
        return math.pow(self.x - other_state.x, 2) + math.pow(self.y - other_state.y, 2)

    # return a deep copy of the state.
    def deepcopy(self):
        return copy.deepcopy(self)
        
    # Print the state
    def print(self):
        print("State: ",self.x, self.y, self.theta)


# Class to store walls as objects (specifically when represented as line segments in a 2D map.)
class Wall:

    # Constructor
    def __init__(self, wall_corners):
        self.corner1 = State(wall_corners[0], wall_corners[1], 0)
        self.corner2 = State(wall_corners[2], wall_corners[3], 0)
        self.corner1_mm = State(wall_corners[0] * 1000, wall_corners[1] * 1000, 0)
        self.corner2_mm = State(wall_corners[2] * 1000, wall_corners[3] * 1000, 0)
        
        self.m = (wall_corners[3] - wall_corners[1])/(0.0001 + wall_corners[2] -  wall_corners[0])
        self.b = wall_corners[3] - self.m * wall_corners[2]
        self.b_mm =  wall_corners[3] * 1000 - self.m * wall_corners[2] * 1000
        self.length = self.corner1.distance_to(self.corner2)
        self.length_mm_squared = self.corner1_mm.distance_to_squared(self.corner2_mm)
        
        if self.m > 1000:
            self.vertical = True
        else:
            self.vertical = False
        if abs(self.m) < 0.1:
            self.horizontal = True
        else:
            self.horizontal = False


# A class to store 2D maps
class Map:
    def __init__(self, wall_corner_list):
        self.wall_list = []
        for wall_corners in wall_corner_list:
            self.wall_list.append(Wall(wall_corners))
        min_x = 999999
        max_x = -99999
        min_y = 999999
        max_y = -99999
        for wall in self.wall_list:
            min_x = min(min_x, min(wall.corner1.x, wall.corner2.x))
            max_x = max(max_x, max(wall.corner1.x, wall.corner2.x))
            min_y = min(min_y, min(wall.corner1.y, wall.corner2.y))
            max_y = max(max_y, max(wall.corner1.y, wall.corner2.y))
        border = 0.5
        self.plot_range = [min_x - border, max_x + border, min_y - border, max_y + border]
        
        self.particle_range = [min_x , max_x , min_y, max_y]

    # Function to calculate the distance between any state and its closest wall, accounting for directon of the state.
    def closest_distance_to_walls(self, state):
        closest_distance = 999999999999
        for wall in self.wall_list:
            closest_distance = self.get_distance_to_wall(state, wall, closest_distance)
        
        return closest_distance
        
    # Function to get distance to a wall from a state, in the direction of the state's theta angle.
    # Or return the distance currently believed to be the closest if its closer.
    def get_distance_to_wall(self, state, wall, closest_distance):
        # Ray/segment intersection.
        x0 = state.x
        y0 = state.y
        ray_dx = math.cos(state.theta)
        ray_dy = math.sin(state.theta)

        x1 = wall.corner1.x
        y1 = wall.corner1.y
        x2 = wall.corner2.x
        y2 = wall.corner2.y
        seg_dx = x2 - x1
        seg_dy = y2 - y1

        den = ray_dx * seg_dy - ray_dy * seg_dx
        if abs(den) < 1e-10:
            return closest_distance  # Parallel or collinear.

        px = x1 - x0
        py = y1 - y0

        t = (px * seg_dy - py * seg_dx) / den  # Distance along ray (ray direction is unit length).
        u = (px * ray_dy - py * ray_dx) / den  # Position on segment [0, 1].

        if t >= 0 and 0 <= u <= 1:
            return min(closest_distance, t)
        return closest_distance


# Class to hold a particle
class Particle:
    
    def __init__(self):
        self.state = State(0, 0, 0)
        self.weight = 1
        
    # Function to create a new random particle state within a range
    def randomize_uniformly(self, xy_range):
        x = random.uniform(xy_range[0], xy_range[1])
        y = random.uniform(xy_range[2], xy_range[3])
        theta = random.uniform(-math.pi, math.pi)
        self.state = State(x, y, theta)
        self.weight = 1

    # Function to create a new random particle state with a normal distribution
    def randomize_around_initial_state(self, initial_state, state_stdev):
        x = random.gauss(initial_state.x, state_stdev.x)
        y = random.gauss(initial_state.y, state_stdev.y)
        theta = angle_wrap(random.gauss(initial_state.theta, state_stdev.theta))
        self.state = State(x, y, theta)
        self.weight = 1
        
    # Function to take a particle and "randomly" propagate it forward according to a motion model.
    def propagate_state(self, last_state, delta_encoder_counts, steering, delta_t):
        s_mean = parameters.encoder_counts_to_distance * delta_encoder_counts
        w_mean = parameters.steering_to_w * steering

        s_var = parameters.distance_variance_gain * abs(s_mean)
        s_std = math.sqrt(max(0, s_var))
        w_std = math.sqrt(max(0, parameters.rotational_velocity_variance))

        s_sample = s_mean + random.gauss(0, s_std)
        w_sample = w_mean + random.gauss(0, w_std)

        x = last_state.x + s_sample * math.cos(last_state.theta)
        y = last_state.y + s_sample * math.sin(last_state.theta)
        theta = angle_wrap(last_state.theta + w_sample * delta_t)
        self.state = State(x, y, theta)
        
    # Function to determine a particles weight based how well the lidar measurement matches up with the map.
    def calculate_weight(self, lidar_signal, map):
        valid_ray_count = 0
        log_weight_sum = 0
        min_prob = 1e-12

        for i in range(len(lidar_signal.angles)):
            measured_distance = lidar_signal.convert_hardware_distance(lidar_signal.distances[i])
            if measured_distance <= 0:
                continue

            measured_distance = min(measured_distance, parameters.lidar_max_range_m)
            beam_angle = self.state.theta + lidar_signal.convert_hardware_angle(lidar_signal.angles[i])
            beam_state = State(self.state.x, self.state.y, angle_wrap(beam_angle))
            expected_distance = map.closest_distance_to_walls(beam_state)
            # If the ray does not intersect the map, treat it as a max-range return
            # instead of dropping that measurement.
            if not math.isfinite(expected_distance) or expected_distance > parameters.lidar_max_range_m:
                expected_distance = parameters.lidar_max_range_m

            prob = max(min_prob, self.gaussian(expected_distance, measured_distance))
            log_weight_sum += math.log(prob)
            valid_ray_count += 1

        if valid_ray_count == 0:
            self.weight = min_prob
        else:
            # Use geometric mean to avoid numerical underflow while preserving ranking.
            self.weight = max(min_prob, math.exp(log_weight_sum / valid_ray_count))
        
    # Return the normal distribution function output.
    def gaussian(self, expected_distance, distance):
        variance_m2 = parameters.distance_variance_m2
        if variance_m2 <= 0:
            variance_m2 = 1e-6
        return math.exp(-math.pow(expected_distance - distance, 2) / (2 * variance_m2))

    # Deep copy the particle
    def deepcopy(self):
        return copy.deepcopy(self)
        
    # Print the particle
    def print(self):
        print("Particle: ", self.state.x, self.state.y, self.state.theta, " w: ", self.weight)


# This class holds the collection of particles.
class ParticleSet:
    
    # Constructor, which calls the known start or unknown start initialization.
    def __init__(self, num_particles, xy_range, initial_state, state_stdev, known_start_state):
        self.num_particles = num_particles
        self.particle_list = []
        if known_start_state:
            self.generate_initial_state_particles(initial_state, state_stdev)
        else:
            self.generate_uniform_random_particles(xy_range)
        self.mean_state = State(0, 0, 0)
        self.update_mean_state()
        
    # Function to reset particles and random locations in the workspace.
    def generate_uniform_random_particles(self, xy_range):
        for i in range(self.num_particles):
            random_particle = Particle()
            random_particle.randomize_uniformly(xy_range)
            self.particle_list.append(random_particle)

    # Function to reset particles, normally distributed around the initial state. 
    def generate_initial_state_particles(self, initial_state, state_stdev):
        for i in range(self.num_particles):
            random_particle = Particle()
            random_particle.randomize_around_initial_state(initial_state, state_stdev)
            self.particle_list.append(random_particle)

    # Function to resample the particles set, i.e. make a new one with more copies of particles with higher weights.  
    def resample(self, max_weight):
        if self.num_particles == 0:
            return

        if max_weight <= 0:
            for p in self.particle_list:
                p.weight = 1
            return

        resampled_particles = []
        index = random.randint(0, self.num_particles - 1)
        beta = 0
        for _ in range(self.num_particles):
            beta += random.uniform(0, 2 * max_weight)
            while beta > self.particle_list[index].weight:
                beta -= self.particle_list[index].weight
                index = (index + 1) % self.num_particles
            sampled_particle = self.particle_list[index].deepcopy()
            sampled_particle.weight = 1
            resampled_particles.append(sampled_particle)

        self.particle_list = resampled_particles
            
    # Calculate the mean state. 
    def update_mean_state(self):
        if len(self.particle_list) == 0:
            self.mean_state = State(0, 0, 0)
            return

        total_weight = 0
        for particle in self.particle_list:
            total_weight += max(0, particle.weight)

        if total_weight <= 0:
            total_weight = len(self.particle_list)
            weights = [1.0] * len(self.particle_list)
        else:
            weights = [max(0, p.weight) for p in self.particle_list]

        mean_x = 0
        mean_y = 0
        mean_cos_theta = 0
        mean_sin_theta = 0
        for i, particle in enumerate(self.particle_list):
            w = weights[i] / total_weight
            mean_x += w * particle.state.x
            mean_y += w * particle.state.y
            mean_cos_theta += w * math.cos(particle.state.theta)
            mean_sin_theta += w * math.sin(particle.state.theta)

        mean_theta = math.atan2(mean_sin_theta, mean_cos_theta)

        self.mean_state.x = mean_x
        self.mean_state.y = mean_y
        self.mean_state.theta = mean_theta
        
    # Print the particle set. Useful for debugging.
    def print_particles(self):
        for particle in self.particle_list:
            particle.print()
        print()

# Class to hold the particle filter and its functions.
class ParticleFilter:
    
    # Constructor
    def __init__(self, num_particles, map, initial_state, state_stdev, known_start_state, encoder_counts_0):
        self.map = map
        self.particle_set = ParticleSet(num_particles, map.particle_range, initial_state, state_stdev, known_start_state)
        self.state_estimate = self.particle_set.mean_state
        self.state_estimate_list = []
        self.last_time = 0
        self.last_encoder_counts = encoder_counts_0

    # Update the states given new measurements
    def update(self, odometery_signal, measurement_signal, delta_t):
        self.prediction(odometery_signal, delta_t)
        if len(measurement_signal.angles)>0:
            self.correction(measurement_signal)
        self.particle_set.update_mean_state()
        self.state_estimate = self.particle_set.mean_state
        self.state_estimate_list.append(self.state_estimate.deepcopy())

    # Predict the current state from the last state.
    def prediction(self, odometry_signal, delta_t):
        encoder_counts = odometry_signal[0]
        steering = odometry_signal[1]

        delta_encoder_counts = encoder_counts - self.last_encoder_counts
        self.last_encoder_counts = encoder_counts

        for particle in self.particle_set.particle_list:
            last_state = particle.state.deepcopy()
            particle.propagate_state(last_state, delta_encoder_counts, steering, delta_t)
        return
        
    # Corrrect the predicted states.
    def correction(self, measurement_signal):
        max_weight = 0
        for particle in self.particle_set.particle_list:
            particle.calculate_weight(measurement_signal, self.map)
            max_weight = max(max_weight, particle.weight)

        self.particle_set.resample(max_weight)
        
    # Output to terminal the mean state.
    def print_state_estimate(self):
        print("Mean state: ", self.particle_set.mean_state.x, self.particle_set.mean_state.y, self.particle_set.mean_state.theta)
    

# Class to help with plotting PF data.
class ParticleFilterPlot:

    # Constructor
    def __init__(self, map):
        self.dir_length = 0.1
        fig, ax = plt.subplots()
        self.ax = ax
        self.fig = fig
        self.map = map

    # Clear and update the plot with new PF data
    def update(self, state_mean, particle_set, lidar_signal, hold_show_plot):
        plt.clf()
        
        # Plot walls
        for wall in self.map.wall_list:
            plt.plot([wall.corner1.x, wall.corner2.x],[wall.corner1.y, wall.corner2.y],'k')

        # Plot lidar
        for i in range(len(lidar_signal.angles)):
            distance = lidar_signal.convert_hardware_distance(lidar_signal.distances[i])
            angle = lidar_signal.convert_hardware_angle(lidar_signal.angles[i]) + state_mean.theta
            x_ray = [state_mean.x, state_mean.x + distance * math.cos(angle)]
            y_ray = [state_mean.y, state_mean.y + distance * math.sin(angle)]
            plt.plot(x_ray, y_ray, 'r')


        # Plot state estimate
        plt.plot(state_mean.x, state_mean.y,'ro')
        plt.plot([state_mean.x, state_mean.x+ self.dir_length*math.cos(state_mean.theta) ], [state_mean.y, state_mean.y+ self.dir_length*math.sin(state_mean.theta) ],'r')
        x_particles, y_particles = self.to_plot_data(particle_set)
        plt.plot(x_particles, y_particles, 'g.')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.axis(self.map.plot_range)
        plt.grid()
        if hold_show_plot:
            plt.show()
        else:
            plt.draw()
            plt.pause(0.1)

    # Helper function to make the particles easy to plot.
    def to_plot_data(self, particle_set):
        x_list = []
        y_list = []
        for p in particle_set.particle_list:
            x_list.append(p.state.x)
            y_list.append(p.state.y)
        return x_list, y_list
        

# Function used to test your PF offline with logged data.
def offline_pf():
    
    # Make a map of walls
    map = Map(parameters.wall_corner_list)

    # Get data to filter
    filename = './data/new_kidnapped.pkl'
    pf_data = data_handling.get_file_data_for_pf(filename)

    # Instantiate PF with no initial guess
    particle_filter = ParticleFilter(parameters.num_particles, map, initial_state = State(0.68, 0.53, 0.0), state_stdev = State(0.1,0.1,0.1), known_start_state=True, encoder_counts_0=pf_data[0][2].encoder_counts)
    start_estimate = particle_filter.state_estimate.deepcopy()

    # Create plotting tool for particles
    particle_filter_plot = ParticleFilterPlot(map)

    # Loop over pf data
    for t in range(1, len(pf_data)):
        row = pf_data[t]
        delta_t = pf_data[t][0] - pf_data[t-1][0] # time step size
        u_t = np.array([row[2].encoder_counts, row[2].steering]) # robot_sensor_signal
        z_t = row[2] # lidar_sensor_signal

        # Run the PF for a time step
        particle_filter.update(u_t, z_t, delta_t)
        particle_filter_plot.update(particle_filter.particle_set.mean_state, particle_filter.particle_set, z_t, False)

    particle_filter_plot.update(particle_filter.particle_set.mean_state, particle_filter.particle_set, z_t, False)

    # Debug summary for localization drift/translation checks.
    end_estimate = particle_filter.state_estimate.deepcopy()
    dx = end_estimate.x - start_estimate.x
    dy = end_estimate.y - start_estimate.y
    total_distance = math.sqrt(dx * dx + dy * dy)
    dtheta = angle_wrap(end_estimate.theta - start_estimate.theta)
    print("PF Debug Summary")
    print(" start_estimate:", round(start_estimate.x, 4), round(start_estimate.y, 4), round(start_estimate.theta, 4))
    print(" end_estimate:  ", round(end_estimate.x, 4), round(end_estimate.y, 4), round(end_estimate.theta, 4))
    print(" delta_xy (m):  ", round(dx, 4), round(dy, 4))
    print(" distance (m):  ", round(total_distance, 4))
    print(" delta_theta(rad):", round(dtheta, 4))

        


####### MAIN #######
if __name__ == '__main__':
    offline_pf()
