import numpy as np
import math
from simulation_parameters import *
from utils import *
import matplotlib.pyplot as plt
import kalman 
from scipy.linalg import block_diag
from kalman import kal, kal_hist
from sensors import Enc, GPS, UWB, Camera, Range
import sympy as sym

def Jacobian(v_str, f_list):
    # vars = sym.symbols(v_str)
    vars = v_str
    f = sym.sympify(f_list)
    J = sym.zeros(len(f),len(vars))
    for i, fi in enumerate(f):
        for j, s in enumerate(vars):
            J[i,j] = sym.diff(fi, s)
    return J

class centralized_kal():

    def __init__(self, swarm):
        self.swarm = swarm
        self.G = swarm.G
        self.XK = np.array([])
        self.XK1 = np.array([])
        self.PK = np.array([])
        self.PK1 = np.array([])
        self.R = np.array([])
        self.H = np.array([])
        self.S = np.array([])
        self.W = np.array([])
        self.z = np.array([])
        self.u_est_dt = np.array([])
        self.kalmanTime = 0
        full_states = np.array([])
        is_first = True
        for agent in self.swarm.robots:
            state = np.array([agent.state.x, agent.state.y, agent.state.yaw])
            full_states = np.append(full_states,state)
            if is_first:
                full_covariance = agent.kalman.Pk
                is_first = False
            else:
                full_covariance = block_diag(full_covariance, agent.kalman.Pk)
        self.PK = full_covariance # np.diag(full_covariance)
        self.XK = full_states + self.PK @ np.random.randn(3*len(swarm.robots)) 
        
        self.hist = kal_hist(self.XK,self.PK,full_states )

    def control_estimation(self):
        U_est = np.array([])
        for agent in self.swarm.robots:
            U_est = np.append( U_est , agent.kalman.Enc.measure(agent) )
        self.u_est_dt = U_est

    def state_prediction(self):
        is_first = True
        for agent in self.swarm.robots:
            theta = self.XK[2 + 3*agent.indexSwarm]
            B = np.array([[math.cos(theta),0],[math.sin(theta),0],[0,1]]) @ np.array([[L,L],[1,-1]]) * r_w/(2*L)
            if is_first:
                Btot = B
                is_first = False
            else:
                Btot = block_diag(Btot, B)
        self.XK1 = self.XK + Btot @ self.u_est_dt

    def covariance_prediction(self):
        is_first = True
        for agent in self.swarm.robots:
            theta = self.XK[2 + 3*agent.indexSwarm]
            A  = np.array([[1, 0, -math.sin(theta) * r_w/2 * ( self.u_est_dt[0] + self.u_est_dt[1] )], 
                           [0, 1,  math.cos(theta) * r_w/2 * ( self.u_est_dt[0] + self.u_est_dt[1] )],
                           [0, 0,    1 ]]
                          )
            B = np.array([[ math.cos(theta) * r_w/2 , math.cos(theta) * r_w/2 ],
                          [ math.sin(theta) * r_w/2 , math.sin(theta) * r_w/2 ],
                          [               r_w/(2*L) ,             - r_w/(2*L) ]])
            if is_first:
                Atot = A
                Btot = B
                Qtot = agent.kalman.Enc.Q
                is_first = False
            else:
                Atot = block_diag( Atot, A)
                Btot = block_diag( Btot, B)
                Qtot = block_diag( Qtot, agent.kalman.Enc.Q ) 
        PK = self.PK
        PK1 = Atot @ PK @ Atot.T + Btot @ Qtot @ Btot.T
        self.PK1= PK1
    

    def observation_and_measure(self):
        h =[]
        var = []
        # z = []
        z = np.array([])
        val = []
        # R = []
        is_first = True
        for agent in self.swarm.robots:
            ii = agent.indexSwarm
            xi = sym.symbols('x_' + str(ii))
            yi = sym.symbols('y_' + str(ii))
            ti = sym.symbols('t_' + str(ii))
            var.extend([xi,yi,ti])
            hai = agent.kalman.Camera.h_funct(agent)
            h.extend(hai)
            state = np.array([agent.state.x, agent.state.y, agent.state.yaw])
            zai = agent.kalman.Camera.measure(state)
            # z.extend(zai)
            z = np.append(z,zai)
            if is_first:
                R = agent.kalman.Camera.R
                is_first = False
            else:
                R = block_diag(R, agent.kalman.Camera.R)

            hri = agent.kalman.Range.h_funct(self.swarm, agent)
            zri = agent.kalman.Range.measure(self.swarm, agent)
            h.extend(hri)
            z = np.append(z,zri)
            if hri != []:
                R = block_diag(R, agent.kalman.Range.R)
        
        for jj, value in enumerate(self.XK1):
            val.extend([ ( var[jj] , value ) ]) 
        H = Jacobian(var,h) #symbolic
        H = (H.subs(val)).evalf()
        H = np.array(np.array(H), np.float)
        self.H = H
        self.R = R
        self.z = z # np.array(z)

    def covariance_innovation(self):
        self.S = self.H @ self.PK1 @ self.H.T + self.R 

    def gain(self):
        self.W = self.PK1 @ self.H.T @ np.linalg.inv(self.S)

    def state_update(self):
        self.XK = self.XK1 + self.W @ (self.z - self.H @ self.XK1)
    
    def covariance_update(self):
        n = self.PK.shape[0]
        self.PK = (np.eye(n) - self.W @ self.H) @ self.PK1

    def update_without_measure(self):
        self.XK = self.XK1
        self.PK = self.PK1

    def history_save(self):
        self.hist.xk.extend( (self.XK.flatten()).tolist() )
        self.hist.Pk.extend( (self.PK.flatten()).tolist() )
        self.hist.Time.extend( [self.kalmanTime] ) 
        full_states = np.array([]) 
        for agent in self.swarm.robots:
            state = np.array([agent.state.x, agent.state.y, agent.state.yaw])
            full_states = np.append(full_states,state)
        self.hist.Dxk.extend( ((self.XK - full_states).flatten()).tolist() )

    def filter(self):
        self.control_estimation()
        self.state_prediction()
        self.covariance_prediction()
        meas_flag = ( int(self.kalmanTime/dt) % int( (1/self.swarm.robots[0].kalman.Range.rate)/dt)  == 0 and self.kalmanTime ) != 0
        if meas_flag :
            print(BOLD + OKGREEN + 'Centralized Kalman filter collecting measures' + ENDC)
            self.observation_and_measure()
            self.covariance_innovation()
            self.gain()
            self.state_update()
            self.covariance_update()
        else:
            self.update_without_measure()

        self.kalmanTime += dt
        self.history_save()