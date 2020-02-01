import numpy as np
import math
from simulation_parameters import *
from sensors import Enc, GPS, UWB, Camera, Range
import matplotlib.pyplot as plt
import sympy as sym

class kal_hist():
    def __init__(self,xk,Pk,real_state):
        self.xk = (xk.flatten()).tolist()
        self.Pk = (Pk.flatten()).tolist()
        self.Time = [0]
        self.Dxk = ((xk - real_state).flatten()).tolist()
        self.z = []

class kal():

    def __init__(self, agent):
        self.agent = agent
        self.xk = np.array([self.agent.state.x, 
                            self.agent.state.y, 
                            self.agent.state.yaw]) + ( 
                            np.diag([sigmaX0,sigmaY0,sigmaPsi0]) @ np.random.randn(3) )
        self.xk1 = self.xk
        self.Pk = np.diag([sigmaX0**2,sigmaY0**2,sigmaPsi0**2])
        self.Pk1 = self.Pk
        self.Enc = Enc()
        self.sensors = [Camera()] #[GPS(), UWB(), Camera()]
        self.sensors_measuring_at_T = self.sensors
        self.Range = Range()
        self.Camera = Camera()
        self.R = [] # redefined later
        self.H = [] # redefined later
        self.S = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.W = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.z = self.xk
        self.u_est_dt = np.array([0,0])
        self.kalmanTime = 0
        real_state = np.array([self.agent.state.x, self.agent.state.y, self.agent.state.yaw])
        self.hist = kal_hist(self.xk,self.Pk,real_state )

    def control_estimation(self):
        self.u_est_dt = self.Enc.measure(self.agent)
        # sqrt cause the element of the matrix are the covariances squared

    def state_prediction(self):
        theta = self.xk[2]
        B = np.array([[math.cos(theta),0],[math.sin(theta),0],[0,1]]) @ np.array([[L,L],[1,-1]]) * r_w/(2*L)
        self.xk1 = self.xk + B @ self.u_est_dt

    def covariance_prediction(self):
        theta = self.xk[2]
        A  = np.array([[1, 0, -math.sin(theta) * r_w/2 * ( self.u_est_dt[0] + self.u_est_dt[1] )], 
                       [0, 1,  math.cos(theta) * r_w/2 * ( self.u_est_dt[0] + self.u_est_dt[1] )],
                       [0, 0,                1]]
                      )
        B = np.array([[ math.cos(theta) * r_w/2 , math.cos(theta) * r_w/2 ],
                      [ math.sin(theta) * r_w/2 , math.sin(theta) * r_w/2 ],
                      [               r_w/(2*L) ,             - r_w/(2*L) ]])
        Pk = self.Pk
        Q  = self.Enc.Q 
        Pk1 = A @ Pk @ A.T + B @ Q @ B.T
        self.Pk1= Pk1
    
    
    def is_measuring(self):
        meas_list = []
        meas_flag = False
        for sensor in self.sensors:
            if  ( int(self.kalmanTime/dt) % int( (1/sensor.rate)/dt) ) == 0:
                meas_list.append(sensor)
                meas_flag = True
                print('Distributed measuring')
        self.sensors_measuring_at_T = meas_list
        return meas_flag

    def observation_and_measure(self):
        H = np.array([])
        R = np.array([])
        hh = 0
        state = np.array([self.agent.state.x, self.agent.state.y, self.agent.state.yaw])
        z = np.array([])
        for sensor in self.sensors_measuring_at_T:
            hh = hh + sensor.H.shape[0]
            H = np.append( H , sensor.H.flatten() )
            R = np.append( R , np.diag(sensor.R) ) 
            z = np.append( z , sensor.measure(state) ) 
        self.H = H.reshape( ( hh, 3 ) )
        self.R = np.diag(R)
        self.z = z

    def covariance_innovation(self):
        self.S = self.H @ self.Pk1 @ self.H.T + self.R 

    def gain(self):
        self.W = self.Pk1 @ self.H.T @ np.linalg.inv(self.S)

    def state_update(self):
        self.xk = self.xk1 + self.W @ (self.z - self.H @ self.xk1)
    
    def covariance_update(self):
        self.Pk = (np.eye(3) - self.W @ self.H) @ self.Pk1

    def update_without_measure(self):
        self.xk = self.xk1
        self.Pk = self.Pk1

    def history_save(self):
        self.hist.xk.extend( (self.xk.flatten()).tolist() )
        self.hist.Pk.extend( (self.Pk.flatten()).tolist() )
        self.hist.Time.extend( [self.kalmanTime] ) 
        state = np.array([self.agent.state.x, self.agent.state.y, self.agent.state.yaw])
        self.hist.Dxk.extend( ((self.xk - state).flatten()).tolist() )
        self.hist.z.append( (self.z).tolist() )

    def filter(self):
        self.control_estimation()
        self.state_prediction()
        self.covariance_prediction()

        if self.is_measuring():
            self.observation_and_measure()
            self.covariance_innovation()
            self.gain()
            self.state_update()
            self.covariance_update()
        else:
            self.update_without_measure()
        self.kalmanTime += dt
        self.history_save()

    def ellipse_plot(self):
        Pxy = self.Pk[0:2,0:2]
        eigval, eigvec = np.linalg.eig(Pxy)

        if eigval[0] >= eigval[1]:
            bigind = 0
            smallind = 1
        else:
            bigind = 1
            smallind = 0
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        a = math.sqrt(eigval[bigind]) * 3
        b = math.sqrt(eigval[smallind]) * 3
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]

        angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
        rot = np.array([[math.cos(angle), math.sin(angle)],
                        [-math.sin(angle), math.cos(angle)]])
        fx = rot @ (np.array([x, y]))
        px = np.array(fx[0, :] + self.xk[0]).flatten()
        py = np.array(fx[1, :] + self.xk[1]).flatten()
        plt.plot(px, py, "-", color = self.agent.color)