#!/usr/env python
'''
    This script stores the parameters needed for the simulation
'''
import math
import time as tim 
'''
    Probabilistic roadmap 
'''
N_SAMPLE = 1000  # number of sample_points
N_KNN = 20  # number of edge from one sampled point
MAX_EDGE_LEN = 10.0  # [m] Maximum edge length

'''
    Simulation properties
'''
T = 60.0  # max simulation time
avoidance_algorithm = 'potential'#'potential' # 'potential' or 'rvo'
dt = 0.05  # [s]
# n_arrived = 0
time = 0.0

'''
    Robot properties
'''
n_robots = 2
robot_size = 2.0  # [m]
reference_speed = 10.0 # [m/s]
k = 0.1  # look forward gain
if avoidance_algorithm == 'rvo':
    safety_distance = 6 # [m]
    Lfc = 8.0  # look-ahead distance
elif avoidance_algorithm == 'potential':
    safety_distance = 4.0 # [m] from walls for the PRM
    Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed proportional gain
Kd = 0.01
Ki = 1
k_omega = 1.0   # angular velocity proportional gain
L = 2.0  # [m] wheel base of vehicle
r_w = 0.15 # wheel radius

'''
    Kalman properties
'''
sigmaX0 = 5
sigmaY0 = 5
sigmaPsi0 =  30*math.pi/180

sigmaUWBx = 0.3
sigmaUWBy = 0.3
sigmaCx = 0.3
sigmaCy = 0.3
sigmaCyaw = math.pi/180
sigmaGPSx = 0.3
sigmaGPSy = 0.3
sigmaR    = 0.2

kalman_mhe = True
kalman_centralized = True

'''
    Default values for showing the animations and saving the simulations
'''
show_animation = True
show_animation_roadmap = False
show_velocity_profile = False
show_graph = False
save_video = True
writer_dpi = 200
writing_folder = '.'
timestr = tim.strftime("%m-%d_%H-%M")
video_name = writing_folder + '/simulation_' + str(n_robots) + '_' + avoidance_algorithm + '_' + timestr + '.mp4'
save_images = False
