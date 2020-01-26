import random
import math
import numpy as np
import scipy.spatial
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import os
if os.name == 'nt':
    plt.rcParams['animation.ffmpeg_path'] ='C:\\ffmpeg\\bin\\ffmpeg.exe'
import random
import sys
from wall import wall, Map
from lidar import lidar
import time as tim
import networkx as nx
from simulation_parameters import *
from utils import *
from planning import *
from robot import Robots
from graph import graph
import subprocess
from CentralizedKalman import centralized_kal, Jacobian

class Swarm():

    def __init__(self):
        self.robots = []
        self.G = None
        self.plant = None
        self.all_arrived = False
        self.possible_collision = False
        if kalman_centralized:
            self.central_kalman = []
    
    def generate_graph(self):
        '''
        Initializes the graph after the robots are added
        '''
        self.G = graph(self.robots)
    
    def update_graph(self):
        self.G.compute(self.robots)
    
    def evolve_dynamics(self):

        self.possible_collision = False

        for index, agent in enumerate(self.robots):

            agent.save_history(time)

            if agent.arrived == False:
                if agent.target_index == None:
                    if avoidance_algorithm == 'potential':
                        agent.target_index = calc_target_index(agent, agent.path.x, agent.path.y)
                    elif avoidance_algorithm == 'rvo':
                        agent.target_index = calc_target_index_RVO(agent, agent.path.x, agent.path.y, 0)
                
                # STOPPING CONDITION
                if agent.target_index == len(agent.path.x) - 1 and agent.target_speed > 0:
                    agent.target_speed = max(agent.target_speed - 0.3 * dt, 0)
                
                if avoidance_algorithm == 'rvo':
                    agent.lidar.scan(self.plant)
                    agent.pure_pursuit_controlRVO(agent.path.x, agent.path.y, agent.target_index)
                    if agent.docking == False:
                        agent.RVO_update(self.robots, self.G.adjacency_matrix)
                    agent.compute_controls_RVO()
                    agent.update_diff_kine()
                    agent.kalman.filter()
                    if hasattr(agent, 'MHE'):
                        agent.MHE.filter()
                elif avoidance_algorithm == 'potential':
                    agent.lidar.scan(self.plant)
                    PIDControl(agent)
                    delta = pure_pursuit_control(agent, agent.path.x, agent.path.y, agent.target_index)
                    agent.update_derivatives(self, delta, flocking = False, giroscopic = True, potential = True, map_potential = True)
                    # agent.update(delta)
                    agent.compute_controls()
                    agent.update_diff_kine()
                    agent.kalman.filter()
                    if hasattr(agent, 'MHE'):
                        agent.MHE.filter()
                else:
                    raise(NotImplementedError)
                
            elif sum([agents.arrived for agents in self.robots]) == n_robots:
                toc()
                print('All robots arrived at their goal')
                self.all_arrived = True
                return # exits the main cycle

            near_agents = agent.nearby_obs(self.robots, self.G.adjacency_matrix)
            if len(near_agents) != 0:
                self.possible_collision, T_coll_flag = agent.future_collision_detector(near_agents)

        # Error if one agent cannot reach goal
            assert agent.last_index >= agent.target_index, FAIL + "Cannot reach the final goal" + ENDC
        
        if hasattr(self, 'central_kalman'):
            self.central_kalman.filter()
