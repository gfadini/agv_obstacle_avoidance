import numpy as np
import math
import networkx as nx
import matplotlib.pyplot as plt
from simulation_parameters import *

class graph():
    '''
    Represents the graph of the connections between drones while moving in the map
    Such graph is undirected, the compute of the adjacency matrix and laplacian can be called dynamically
    '''
    def __init__(self, swarm):
        if swarm == None:
            raise ValueError
        else:
            self.adjacency_matrix = np.zeros([len(swarm), len(swarm)])
            self.laplacian_matrix = np.zeros([len(swarm), len(swarm)])
            self.degree_matrix = np.zeros([len(swarm), len(swarm)])
            # self.compute(swarm)

    def show_graph(self):
        '''
        Plots a representation of the interconnections of the graph
        '''
        G = nx.from_numpy_matrix(np.array(self.adjacency_matrix))
        plt.figure('Graph')
        plt.cla()
        nx.draw(G, with_labels = True, pos = nx.spring_layout(G))
        plt.show()
        if show_animation: # replotting to the realtime simulation if present
            plt.figure('Simulation')
    
    def compute(self, swarm):
        '''
        Depending on the swarm state, updates all the connectivity matrices of the graph
        '''
        n_agents = len(swarm)
        if n_agents < 2:
            pass
        else:
            for i, _ in enumerate(swarm):
                j = i + 1
                while j < n_agents:
                    distance = math.sqrt((swarm[i].state.x - swarm[j].state.x)**2 + (swarm[i].state.y - swarm[j].state.y)**2)
                    if distance < 10 :#swarm[i].lidar.range / 2:
                        self.adjacency_matrix[i, j] = 1
                        self.adjacency_matrix[j, i] = 1
                        self.laplacian_matrix[i, j] = -1
                        self.laplacian_matrix[j, i] = -1
                    else:
                        self.adjacency_matrix[i, j] = 0
                        self.adjacency_matrix[j, i] = 0
                        self.laplacian_matrix[i, j] = 0
                        self.laplacian_matrix[j, i] = 0
                    j += 1
                self.degree_matrix[i, i] = np.sum(self.adjacency_matrix[i, :])
                self.laplacian_matrix[i, i] = self.degree_matrix[i, i]