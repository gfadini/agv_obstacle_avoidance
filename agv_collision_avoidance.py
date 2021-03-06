#!/bin/ipython -i
'''
Distributed AGV collision avoidance system
author: Gabriele Fadini and Mattia Piazza
'''
import math
import numpy as np
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import time as tim
import networkx as nx
import subprocess
import random
import sys
import os
import matplotlib.pyplot as plt
if os.name == 'nt':
    plt.rcParams['animation.ffmpeg_path'] ='C:\\ffmpeg\\bin\\ffmpeg.exe'
from matplotlib.animation import FFMpegWriter
from wall import wall, Map
from lidar import lidar
from simulation_parameters import *
from utils import *
from planning import *
from graph import graph
from swarm import Swarm

if sys.version_info < (3, 0):
    raise  Exception('Unsupported python version')

'''
    INITIALIZATION
'''
# initialize the swarm
swarm = Swarm()

if map_case == 'polygon':
    # POLYGON TEST, doesn't need planning
    poly, swarm.robots = polygon(n_robots, radius = 25, n_edges = 100)
    plant, possible_points = Map().from_data(*poly)
    # plant.add_wall([5,5],[-5,-5])
elif map_case == 'plant':
    # WITH A MAP, PRELOADED, needs planning
    plant, possible_points = Map().from_data(*map_2)
    # WITH A MAP, FROM IMAGE, needs planning
    # the map must be in the cwd for the script to work properly
    # plant, possible_points = load_from_image('wall_from_image/plant.png')
    '''
        PATH PLANNING
    '''
    probabilistic_roadmap(swarm, plant, possible_points)
else:
    print('Map case not understood')

# save the map in the swarm routine
swarm.plant = plant

if kalman_centralized:
    swarm.central_kalman = centralized_kal(swarm)

'''
    SIMULATION
'''
# Initialize the graph
swarm.generate_graph()

# Initialize the animation object
metadata = dict(title='Distributed AGV collision avoidance',
                artist='Fadini & Piazza',
                comment='Now on file!')
writer = FFMpegWriter(fps=int(1/dt), metadata = metadata)

print(HEADER + '*'*25 + '  PARAMETERS SUMMARY  ' + '*'*25 + ENDC)
print('\tT = {:0.2f}s\n\tN_robots = {:1}\n\tAlgorithm = {:2}\n\tMap = {:3}\n\tCentralized Kalman filter = {:4}\n\tMHE filter = {:5}'.format(
    T, n_robots, avoidance_algorithm, map_case, str(kalman_centralized), str(mhe_filter)))
print(OKGREEN + '*'*24 + ' ALL READY PRESS ENTER  ' + '*'*24 + ENDC)


_ = input()

tic()
timestamp = 0
time = 0.0

plt.figure('Simulation')
plt.interactive(True)

print(HEADER + '*'*24 + '  STARTING SIMULATION  ' + '*'*24 + ENDC)

with writer.saving(plt.figure('Simulation'), video_name, writer_dpi):
    while time <= T and not swarm.all_arrived:
       
        print_informations(time, dt, swarm, interval = dt)

        swarm.G.compute(swarm.robots)
        swarm.evolve_dynamics(time)
        plt.cla()
        simulation_plot(swarm, time, plot_lidar = True, plot_path = True, plot_target = True, plot_trajectory = True)

        if show_animation:
            plt.show()
            plt.pause(dt)
        if save_images:
            plt.savefig('./simulation/simulation{:0}.jpg'.format(timestamp))
            timestamp = int(timestamp + 1)
        if save_video:
            writer.grab_frame()
        if show_graph:
            swarm.G.show_graph()
        if show_velocity_profile:
            plot_velocity_profile(swarm.robots[0])

        time = time + dt

'''
    POSTPROCESSING
'''
print_performance(swarm)
plot_kalman_error(swarm)
plot_MHE_error(swarm)
plot_filtered_trajectory(swarm)
plot_filtered_state(swarm)
compare_KalMHE(swarm)
print_RMSE(swarm)