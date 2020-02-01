'''
    This script stores the parameters needed for the simulation
'''
import math
import time as tim
import sys
'''
    Probabilistic roadmap 
'''
N_SAMPLE = 1000  # number of sample_points
N_KNN = 20  # number of edge from one sampled point
MAX_EDGE_LEN = 10.0  # [m] Maximum edge length

'''
    Parsing command line options
'''
if len(sys.argv) > 1:
    _, *cli_parameters = sys.argv
    print('Overriding default with: ', *cli_parameters)
else:
    cli_parameters = None

'''
    Simulation properties
'''
T = 60.0  # max simulation time

avoidance_algorithm = 'potential'# 'potential' or 'rvo'
n_robots = 2
map_case = 'plant' # 'polygon', 'plant'
dt = 0.05  # [s]
time = 0.0


def check_string(x):
    return isinstance(x, str)
def check_bool(x):
    return isinstance(x, bool)
def check_int(x):
    try:
        int(x)
    except:
        return False
    return int(x) >= 0
def check_float(x):
    return isinstance(x, float) and float(x) >= 0

# if the cli parameters are not default, override
if not cli_parameters is None:
    str_parameters = list(filter(check_string, cli_parameters))
    int_parameters = list(filter(check_int, cli_parameters))

    # changing the avoidance method
    if 'potential' in str_parameters:
        avoidance_algorithm = 'potential'
    elif 'rvo' in str_parameters:
        avoidance_algorithm = 'rvo'
    # changing the map method
    if 'plant' in str_parameters:
        map_case = 'plant'
    elif 'polygon' in cli_parameters:
        map_case = 'polygon'
    # kalman modifiers, defaults are False, check next section
    if 'central' in str_parameters:
        kalman_centralized = True
    if 'mhe' in str_parameters:
        kalman_mhe = True

    try:
        n_robots = int(max(int_parameters))
    except:
        print('default robot number: ', n_robots)

'''
    Robot properties
'''
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
sigmaPsi0 =  20*math.pi/180

sigmaUWBx = 0.3
sigmaUWBy = 0.3
sigmaCx = 0.5
sigmaCy = 0.5
sigmaCyaw = 5*math.pi/180
sigmaGPSx = 0.3
sigmaGPSy = 0.3
sigmaR    = 0.2
try:
    kalman_mhe = kalman_mhe
except:
    kalman_mhe = False
try:
    kalman_centralized = kalman_centralized
except:
    kalman_centralized = False
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