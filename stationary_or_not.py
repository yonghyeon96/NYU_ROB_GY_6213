# External libraries
import math

# The images we used for this data
images = [
'1478020244189375590_jpg',
'1478020244690239779_jpg',
'1478020245189639963_jpg',
]

# Data format for bounding boxes xmin, ymin, xmax, ymax
car_1_data = [
[140,236,164,260],
[140,236,163,266],
[139,236,163,266],
]

# Data format for bounding boxes xmin, ymin, xmax, ymax
car_2_data = [
[106,238,141,280],
[93,237,133,278],
[84,238,126,278],
]

# Fabricated function p(stationary_t | stationary_tm1)
def conditional_prob_stationary_given_stationary():
    return 0.8

# Fabricated function p(stationary_t | moving_tm1)
def conditional_prob_stationary_given_moving():
    return 0.2

# Fabricated function p(moving_t | stationary_tm1)
def conditional_prob_moving_given_stationary():
    return 0.2

# Fabricated function p(moving_t | moving_tm1)
def conditional_prob_moving_given_moving():
    return 0.8

# Fabricated function p(z_t | stationary_t)
def conditional_prob_z_vel_given_stationary(z_vel):
    mu = 0
    sigma = 1.5
    p = 1/sigma/math.sqrt(2*math.pi) * math.exp( -0.5* math.pow((mu - z_vel)/sigma,2) )

    return p

# Fabricated function p(z_t | moving_t)
def conditional_prob_z_vel_given_moving(z_vel):
    mu = - 10
    sigma = 5
    p = 1/sigma/math.sqrt(2*math.pi) * math.exp( -0.5* math.pow((mu - z_vel)/sigma,2) )

    return p


# Bayes filter
def stationary_bayes_filter(data_set):

    # Unknown state to begin with
    p_stationary_list = [0.5]

    # Loop over data
    for i in range(1, len(data_set)):
        
        # Get measurement z for current time step
        row_tm1 = data_set[i-1]
        row_t = data_set[i]
        x_pos_t = (row_t[0] + row_t[2])/2
        x_pos_tm1 = (row_tm1[0] + row_tm1[2])/2
        z_vel = x_pos_t - x_pos_tm1

        # Look at last time steps
        p_stationary_tm1 = p_stationary_list[i-1]
        p_moving_tm1 = 1 - p_stationary_tm1

        # Prediction
        p_stationary_predicted = p_stationary_tm1 * conditional_prob_stationary_given_stationary() + p_moving_tm1 * conditional_prob_stationary_given_moving()
        p_moving_predicted = p_stationary_tm1 * conditional_prob_moving_given_stationary() + p_moving_tm1 * conditional_prob_moving_given_moving()

        # Corrrection 
        p_stationary = conditional_prob_z_vel_given_stationary(z_vel) * p_stationary_predicted
        p_moving = conditional_prob_z_vel_given_moving(z_vel) * p_moving_predicted

        # Normalization
        normalizer = p_stationary + p_moving
        p_stationary = p_stationary / normalizer
        p_moving = p_moving / normalizer

        p_stationary_list.append(p_stationary)
    
    return p_stationary_list


############################# MAIN #######################################
p_stationary_list_1 = stationary_bayes_filter(car_1_data)
print("Probability of car 1 being stationary for different time steps: ")
for p in p_stationary_list_1:
    print(" ",p)
print("")

p_stationary_list_2 = stationary_bayes_filter(car_2_data)
print("Probability of car 2 being stationary for different time steps: ")
for p in p_stationary_list_2:
    print(" ",p)
print("")