import random 
import matplotlib.pyplot as plt
import math
import numpy as np
from simulation_parameters import *

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

def random_color():
    random_number = random.randint(0, 2**24 - 1)
    hex_number = format(random_number,'x')
    hex_number = '#' + hex_number
    if len(hex_number) == 7:
        return hex_number
    else:
        return random_color()

def plot_arrow(x, y, yaw, fc = 'r', length=0.1, width=2.0, ec="k"):
    '''
    Plot arrow
    '''
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def plot_velocity_profile(agent):
    plt.figure('Velocity profile')
    plt.cla()
    plt.title("Speed [km/h]")
    plt.plot(agent.history.time, [iv * 3.6 for iv in agent.history.v], color = agent.color)
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()
    if show_animation: # replotting to the realtime simulation if present
        plt.figure('Simulation')

def print_informations(time, dt, swarm, interval = 1):
    if int(time/dt) % int(interval/dt) == 0:
        print(OKBLUE + 30*'-' + ' t = {:0.4} s '.format(time) + 30*'-' + ENDC)
        for agent in swarm.robots:
            if agent.state.yaw > math.pi:
                attitude = agent.state.yaw - 2*math.pi
            elif agent.state.yaw <  - math.pi:
                attitude = agent.state.yaw + 2*math.pi
            else:
                attitude = agent.state.yaw
            print(str(BOLD + "Robot {0:}" + ENDC + " reached: [{1:^.4}, {2:^.4}] with attitude {3:^.4}°, {4:^.4}% route").format(agent.indexSwarm, agent.state.x, agent.state.y,
                attitude*180/math.pi, float(100*agent.nearest_point_index/agent.last_index)))
        if swarm.possible_collision:
            print(WARNING + 'Possible collision detected' + ENDC)

def tic(): 
    #Homemade version of matlab tic and toc functions 
    import time 
    global startTime_for_tictoc 
    startTime_for_tictoc = time.time() 
 
def toc(): 
    import time 
    if 'startTime_for_tictoc' in globals(): 
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else: 
        print("Toc: start time not set")

def simulation_plot(swarm, time, plot_start_end = True, plot_lidar = False, plot_path = False, plot_target = False, plot_trajectory = False):

    for wall in swarm.plant.walls:
        wall.plot()

    for agent in swarm.robots:
        if plot_start_end:
            plt.plot(agent.path.x[0], agent.path.y[0], "^", color = agent.color)
            plt.plot(agent.path.x[-1], agent.path.y[-1], "*", color = agent.color)
        if plot_trajectory:
            plt.plot(agent.history.x, agent.history.y, color = agent.color, label="trajectory", linewidth = 0.7, alpha = 0.5)
        if plot_target:
            # show target index on planned path
            plt.plot(agent.path.x[agent.target_index], agent.path.y[agent.target_index], "x", color = agent.color, alpha = 1, label="target")
        if plot_path:
            # show planned path
            plt.plot(agent.path.x, agent.path.y, ".", markersize = 0.5, color = agent.color, alpha = 1, label="course")
        if plot_lidar:
            agent.lidar.plotscan()
        plot_arrow(agent.state.x, agent.state.y, agent.state.yaw, agent.color)
        plt.axis("equal")
        plt.grid(True)
        plt.title('Collision avoidance simulation \n t={:0.4}s'.format(time))

        plot_cov_ellipse = True
        if plot_cov_ellipse:
            agent.kalman.ellipse_plot()
        
    plt.xlim(min(swarm.plant.ox), max(swarm.plant.ox))        
    plt.ylim(min(swarm.plant.oy), max(swarm.plant.oy))


def plot_kalman_error(swarm):

    plt.figure('Kalman error in position estimation')
    for robot in swarm.robots:
        # diff_x = robot.kalman.hist.Dxk[0::3]
        # diff_y = robot.kalman.hist.Dxk[1::3]
        diff_xyt = np.array( robot.kalman.hist.Dxk )
        diff_x = diff_xyt[0::3]
        diff_y = diff_xyt[1::3]
        dist = np.sqrt(diff_x**2 + diff_y**2)
        plt.plot( robot.kalman.hist.Time, dist, "-" , color = robot.color)
        plt.ylabel('Error in position estimation')
        plt.xlabel('Time')
        plt.show()
    
    if not swarm.central_kalman is None:
        plt.figure('Centralized Kalman error')
        n = 3*n_robots
        for i, robot in enumerate(swarm.robots):
            mx = i*3
            my = mx + 1
            # diff_x = swarm.central_kalman.hist.Dxk[mx::n]
            # diff_y = swarm.central_kalman.hist.Dxk[my::n]
            diff_x = np.array(swarm.central_kalman.hist.Dxk[mx::n])
            diff_y = np.array(swarm.central_kalman.hist.Dxk[my::n])
            dist = np.sqrt(diff_x**2 + diff_y**2)
            plt.ylabel('Error in x direction - Centralized Kalman')
            plt.xlabel('Time')
            plt.plot(swarm.central_kalman.hist.Time, dist, "-" , color = robot.color)
        plt.show()

def plot_MHE_error(swarm):

    plt.figure('MHE error in position estimation')
    for robot in swarm.robots:
        diff_xyt = np.array( robot.MHE.hist.Dxk )
        diff_x = diff_xyt[0::3]
        diff_y = diff_xyt[1::3]
        dist = np.sqrt(diff_x**2 + diff_y**2)
        plt.plot( robot.MHE.hist.Time, dist, "-" , color = robot.color)
        plt.ylabel('Error in position estimation')
        plt.xlabel('Time')
        plt.show()

def plot_filteredtrajectory(swarm):

    plt.figure('Trajectory estimation')
    for robot in swarm.robots:
        plt.plot(robot.history.x, robot.history.y,'-.',color = robot.color,label="trajectory", linewidth = 0.7, alpha = 0.5)
        _xytMHE = np.array( robot.MHE.hist.xk )
        _xMHE = _xytMHE[0::3]
        _yMHE = _xytMHE[1::3]
        plt.plot( _xMHE, _yMHE , "-" , color = robot.color)
        _xytKAL = np.array( robot.kalman.hist.xk )
        _xKAL = _xytKAL[0::3]
        _yKAL = _xytKAL[1::3]
        plt.plot( _xKAL, _yKAL , "--" , color = robot.color)


        plt.ylabel('y')
        plt.xlabel('y')
        plt.show()

def plot_filteredtrajectory1(swarm):
    for robot in swarm.robots:
        plt.figure('Trajectory estimation subplots' +  str(robot.indexSwarm))
        _xytMHE = np.array( robot.MHE.hist.xk )
        _xMHE = _xytMHE[0::3]
        _yMHE = _xytMHE[1::3]
        _thMHE= _xytMHE[2::3]
        tMHE  = robot.MHE.hist.Time
        _xytKAL = np.array( robot.kalman.hist.xk )
        _xKAL = _xytKAL[0::3]
        _yKAL = _xytKAL[1::3]
        _thKAL= _xytKAL[2::3]
        tKAL  = robot.kalman.hist.Time
        _xGT  = robot.history.x
        _yGT  = robot.history.y
        _thGT = robot.history.yaw
        tGT   = robot.kalman.hist.Time
        _xytz = np.array( robot.kalman.hist.xk )
        _xz   = _xytz[0::3]
        _yz   = _xytz[1::3]
        _thz  = _xytz[2::3]
        tz    = np.arange(len(_xz)) * 1/robot.kalman.Camera.rate # np.arange(robot.kalman.hist.Time[-1],1/robot.kalman.Camera.rate)

        plt.subplot(3, 1, 1)
        plt.plot( tGT, _xGT,'-',color = robot.color) 
        plt.plot( tMHE, _xMHE,'-.',color = robot.color) 
        plt.plot( tKAL, _xKAL,'--',color = robot.color) 
        # plt.plot( tz, _xz,'.',color = robot.color) 

        plt.subplot(3, 1, 2)
        plt.plot( tGT, _yGT,'-',color = robot.color) 
        plt.plot( tMHE, _yMHE,'-.',color = robot.color) 
        plt.plot( tKAL, _yKAL,'--',color = robot.color) 
        # plt.plot( tz, _yz,'.',color = robot.color) 
        
        plt.subplot(3, 1, 3)
        plt.plot( tGT, _thGT,'-',color = robot.color) 
        plt.plot( tMHE, _thMHE,'-.',color = robot.color) 
        plt.plot( tKAL, _thKAL,'--',color = robot.color) 
        # plt.plot( tz, _thz,'.',color = robot.color) 
        
        plt.show()