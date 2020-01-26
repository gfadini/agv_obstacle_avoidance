#!/usr/env python
'''
    This script stores the parameters needed for the simulation
'''
import math
import time
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
avoidance_algorithm = 'rvo'#'potential' # 'potential' or 'rvo'
dt = 0.05  # [s]
# n_arrived = 0
# time = 0.0

'''
    Robot properties
'''
n_robots = 3
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
sigmaX0 = 3
sigmaY0 = 3
sigmaPsi0 =  30*math.pi/180
# sigmaRW = 0.2
# sigmaLW = 0.2
sigmaUWBx = 0.2
sigmaUWBy = 0.2
sigmaCx = 0.3
sigmaCy = 0.3
sigmaCyaw = math.pi/180
sigmaGPSx = 0.2
sigmaGPSy = 0.2
sigmaR    = 0.1

kalman_mhe = True
kalman_centralized = True

'''
    Default values for showing the animations and saving the simulations
'''
show_animation = True
show_animation_roadmap = False
show_velocity_profile = False
show_graph = False
save_video = False
writer_dpi = 200
writing_folder = '.'
timestr = time.strftime("%m-%d_%H-%M")
video_name = writing_folder + '/simulation_' + str(n_robots) + '_' + avoidance_algorithm + '_' + timestr + '.mp4'
save_images = False
