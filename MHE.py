from casadi import *
import math 
import numpy as np 
from simulation_parameters import *
import matplotlib.pyplot as plt
from terminalplot import plot
import sympy as sym
from sensors import Enc, GPS, UWB, Camera, Range

class MHE_hist():
    def __init__(self):
        self.xk = []
        self.Time = []
        self.Dxk = []
        
class MHE():
    def __init__(self, agent):
        self.agent = agent
        self.Enc = Enc()
        self.Camera = Camera()
        self.MHETime = 0
        self.hist = MHE_hist()
        self.xkest = self.agent.kalman.xk.tolist()

        # States
        self.nstates = 3
        _x = MX.sym('_x')
        _y = MX.sym('_y')
        _th = MX.sym('_th')
        self.x = vertcat(_x, _y, _th)
        
        # # Controls
        self.ncontrols = 2
        w__r = MX.sym('w__r')
        w__l = MX.sym('w__l')
        self.u = vertcat(w__r,w__l)
        
        # Model equations
        xdot = vertcat(cos(_th) * r_w/2 * (w__r + w__l),
                        sin(_th) * r_w/2 * (w__r + w__l), 
                        r_w/(2*L) * (w__r - w__l) 
                        )
        self.Q = np.linalg.inv(self.Enc.Q**(1/2))
        self.R = np.linalg.inv(self.Camera.R**(1/2))
        
        # Objective term
        O =  0  
        
        # Dynamics simulated with 
        # Fixed step Runge-Kutta 4 integrator
        M = 4 # RK4 steps per interval
        DT = dt/M
        f = Function('f', [self.x, self.u], [xdot, O])
        X0 = MX.sym('X0', self.nstates)
        U = MX.sym('U', self.ncontrols)
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT/2 * k1, U)
            k3, k3_q = f(X + DT/2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
            Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        self.F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])
        
        # Start with an empty NLP
        self.w=[]
        self.w0 = []
        self.lbw = []
        self.ubw = []
        self.J = 0
        self.g=[]
        self.lbg = []
        self.ubg = []

        # Define Horizon
        self.Horizon = 6 # measures
        self.enoughMeasure = False
        
        # Define vector of control estimated measurements
        self.u_tilde = [0,0] # np.array([])
        
        # Define vector of position and attitude measurements
        self.y_tilde = [] # np.array([])

        self.KalX = self.agent.kalman.xk.tolist()
        self.KalP = ( self.agent.kalman.Pk.flatten() ).tolist()
        self.state_guess = self.xkest

    def measurement_acquisition(self):
        if self.enoughMeasure:
            self.u_tilde = self.u_tilde[self.ncontrols:]
            self.u_tilde.extend(( self.agent.kalman.u_est_dt/dt ).tolist())
            self.KalX = self.KalX[self.nstates:]
            self.KalX.extend( self.agent.kalman.xk.tolist() )
            self.KalP = self.KalP[(self.nstates**2):]
            self.KalP.extend( ( self.agent.kalman.Pk.flatten() ).tolist() )
            self.y_tilde = self.y_tilde[self.nstates:]
            if self.agent.kalman.is_measuring():
                self.y_tilde.extend(( self.agent.kalman.z ).tolist())
            else:
                self.y_tilde.extend( [ 'nope' , 'nope', 'nope'] )
        else:
            self.u_tilde.extend(( self.agent.kalman.u_est_dt/dt ).tolist()) 
            self.KalX.extend( self.agent.kalman.xk.tolist())
            self.KalP.extend( ( self.agent.kalman.Pk.flatten()).tolist())
            if self.agent.kalman.is_measuring():
                self.state_guess =  ( self.agent.kalman.z ).tolist() 
                self.y_tilde.extend(( self.agent.kalman.z ).tolist())
            else:
                self.y_tilde.extend([ 'nope' , 'nope', 'nope'])
        if len(self.u_tilde) >= (self.Horizon+1)*self.ncontrols:
            self.enoughMeasure = True

    def BuildMinimizationProblem(self):
        # Start with an empty NLP
        self.w=[]
        self.w0 = []
        self.lbw = []
        self.ubw = []
        self.J = 0
        self.g=[]
        self.lbg = []
        self.ubg = []
        xlim = 300
        clim = 300
        # "Lift" initial conditions
        Xk = MX.sym('X0', self.nstates)
        self.w += [Xk]
        self.lbw += self.nstates*[-xlim]
        self.ubw += self.nstates*[+xlim] 

        xkal = np.array(self.KalX[self.nstates:(2*self.nstates)])
        Pkal = np.linalg.inv((np.array(self.KalP[:(self.nstates**2)]).reshape((self.nstates,self.nstates))))
        self.J += ( xkal - Xk).T @ Pkal @ (xkal - Xk)

        # Formulate the NLP
        for k in range(0,self.Horizon):
            # New NLP variable for the control
            Uk = MX.sym('U_' + str(k),self.ncontrols)
            self.w   += [Uk]
            self.lbw += self.ncontrols*[-clim]
            self.ubw += self.ncontrols*[+clim]
            sIdx = k*self.ncontrols
            eIdx = sIdx + self.ncontrols

            # Add Lagrange term
            sIdx = k*self.ncontrols
            eIdx = sIdx + self.ncontrols
            u_tilde_vec = np.array( self.u_tilde[sIdx:eIdx] )
            self.J += ( u_tilde_vec - Uk).T @ self.Q @ ( u_tilde_vec - Uk)

            # Integrate till the end of the interval
            Fk = self.F(x0=Xk, p=Uk)
            Xk_end = Fk['xf']
            
            # New NLP variable for state at end of interval
            Xk = MX.sym('X_' + str(k+1), self.nstates)
            self.w   += [Xk]
            self.lbw += self.nstates*[-xlim] 
            self.ubw += self.nstates*[+xlim]
            if self.y_tilde[ k*self.nstates ] != 'nope':
                sIdx = k*self.nstates
                eIdx = sIdx + self.nstates
                y_tilde_vec = np.array(self.y_tilde[sIdx:eIdx])
                self.J += ( y_tilde_vec - Xk).T @ self.R @ (y_tilde_vec - Xk)

            # Add equality constraint
            self.g   += [Xk_end-Xk]
            self.lbg += self.nstates*[0]
            self.ubg += self.nstates*[0]

        # Create an NLP solver
        prob = {'f': self.J, 'x': vertcat(*self.w), 'g': vertcat(*self.g)}
        opts={}
        opts["verbose_init"] = False
        opts["verbose"] = False
        opts["print_time"] = False
        opts["ipopt.print_level"] = 0
        solver = nlpsol('solver', 'ipopt', prob, opts)

        # Solve the NLP
        sol = solver(lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg)
        w_opt = sol['x'].full().flatten()

        # Plot the solution
        x1_opt = w_opt[0::5]
        x2_opt = w_opt[1::5]
        x3_opt = w_opt[2::5]
        u1_opt = w_opt[3::5]
        u2_opt = w_opt[4::5]

        self.state_guess = [x1_opt[1], x2_opt[1], x3_opt[1]]

        self.xkest = [x1_opt[-1], x2_opt[-1], x3_opt[-1]]
    
    def save_hist(self):
        self.hist.xk.extend(self.xkest)
        self.hist.Time.extend([self.MHETime])
        state = np.array([self.agent.state.x, self.agent.state.y, self.agent.state.yaw])
        xk = np.array(self.xkest)
        self.hist.Dxk.extend(((xk - state).flatten()).tolist())

    def filter(self):
        self.measurement_acquisition()
        if self.enoughMeasure:
            self.BuildMinimizationProblem()
        else:
            self.xkest = self.KalX[:self.nstates]
        self.save_hist()
        self.MHETime += dt
