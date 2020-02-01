import numpy as np
import math
from simulation_parameters import *
import sympy as sym

class Enc():
    def __init__(self):
        self.rate = 100
        self.Ticks = 360
        self.Quanta = 2* math.pi / self.Ticks
        self.theta = np.array([0,0])
        self.delta_theta_dt = np.array([0,0])
        sigTik = 3
        self.Q = np.diag([(sigTik*self.Quanta/3)**2, (sigTik*self.Quanta/3)**2])
        self.R = np.array([[0,0],[0,0]])
        self.H = np.array([[0,0,0],[0,0,0]])

    def measure(self, agent):
        error = np.round( (np.random.rand(2) - 0.5) * 2 ) * self.Quanta
        self.delta_theta_dt = np.floor((dt*agent.state.u)/(self.Quanta))*self.Quanta + error
        self.theta = self.theta + self.delta_theta_dt
        return self.delta_theta_dt

class GPS():
    def __init__(self):
        self.rate = 5
        self.H = np.array([[1,0,0],[0,1,0]])
        self.R = np.diag([sigmaGPSx**2,sigmaGPSy**2])
    def measure(self,state):
        return self.H @ state + np.sqrt(self.R) @ np.random.randn( self.R.shape[0] )
    
class UWB():
    def __init__(self):
        self.rate = 5
        self.H = np.array([[1,0,0],[0,1,0]])
        self.R = np.diag([sigmaUWBx**2,sigmaUWBy**2])
    def measure(self,state):
        return self.H @ state + np.sqrt(self.R) @ np.random.randn( self.R.shape[0] )

class Camera():
    def __init__(self):
        self.rate = 2
        self.H = np.diag([1,1,1]) 
        self.R = np.diag([sigmaCx**2,sigmaCy**2,sigmaCyaw**2])
    def measure(self,state):
        return self.H @ state + np.sqrt(self.R) @ np.random.randn( self.R.shape[0] )
    def h_funct(self, agent):
        ii = agent.indexSwarm
        xi = sym.symbols('x_' + str(ii))
        yi = sym.symbols('y_' + str(ii))
        ti = sym.symbols('t_' + str(ii))
        hai = sym.sympify([xi, yi, ti])
        return hai

class Range():
    def __init__(self):
        self.rate = 5
        self.H = np.diag([1]) # not true
        self.R = sigmaR**2 
        self.sigmaR = sigmaR
    def measure(self,swarm,agent):
        z = []
        idxi = agent.indexSwarm
        xi = agent.state.x
        yi = agent.state.y
        G = swarm.G.adjacency_matrix
        for idxj, val in enumerate(G[idxi,]):
            if val != 0:
                xj = swarm.robots[idxj].state.x
                yj = swarm.robots[idxj].state.y
                zij = ((xi-xj)**2+(yi-yj)**2)**(1/2) + np.random.randn() * (self.sigmaR)**(1/2)
                z.extend([zij])
        return z
    def h_funct(self, swarm, agent):
        ii = agent.indexSwarm
        xi = sym.symbols('x_' + str(ii))
        yi = sym.symbols('y_' + str(ii))
        ti = sym.symbols('t_' + str(ii))
        hri = []
        G = swarm.G.adjacency_matrix
        for idxj, val in enumerate(G[ii,]):
            if val != 0:
                xj = sym.symbols('x_' + str(idxj))
                yj = sym.symbols('y_' + str(idxj))
                hrij = sym.sympify([((xi-xj)**2+(yi-yj)**2)**(1/2)])
                hri.extend(hrij)
        self.R = sigmaR**2 * np.eye(len(hri))
        return hri