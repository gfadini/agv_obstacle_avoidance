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
    plt.title("Velocity profile")
    plt.plot(agent.history.time, [iv * 3.6 for iv in agent.history.v], color = agent.color)
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)
    plt.show()
    if show_animation: # replotting to the realtime simulation if present
        plt.figure('Simulation')

def print_informations(time, dt, swarm, interval = 1):
    if int(time/dt) % int(interval/dt) == 0:
        print(OKBLUE + 30*'-' + ' t = {:0.2f} s '.format(time) + 30*'-' + ENDC)
        for agent in swarm.robots:
            if agent.state.yaw > math.pi:
                attitude = agent.state.yaw - 2*math.pi
            elif agent.state.yaw <  - math.pi:
                attitude = agent.state.yaw + 2*math.pi
            else:
                attitude = agent.state.yaw
            print(str(BOLD + "Robot {0:}" + ENDC + " reached: [{1:^.2f}, {2:^.2f}] with attitude {3:^.1f}°, {4:^.2f}% route").format(agent.indexSwarm, agent.state.x, agent.state.y,
                attitude*180/math.pi, float(100*agent.nearest_point_index/agent.last_index)))
            if  (int(agent.kalman.kalmanTime/dt) % int((1/agent.kalman.sensors[-1].rate)/dt)) == 0:
                print(str(BOLD + "Robot {0:}" + ENDC + OKGREEN + ' received new measures' + ENDC).format(agent.indexSwarm))
        if swarm.possible_collision:
            print(WARNING + 'Possible collision detected' + ENDC)

def print_performance(swarm):
    print(OKBLUE + '*'*29 + '  PERFORMANCE  ' + '*'*29 + ENDC)
    mean_velocity = 0
    std_velocity = 0
    for i, robot in enumerate(swarm.robots):
        mean_velocity = np.mean(robot.history.v) + mean_velocity
        std_velocity = np.std(robot.history.v)
    mean_velocity = mean_velocity/(i+1)
    std_velocity = std_velocity/(i+1)
    print('Swarm mean speed is {:0.2f} m/s, reference speed was {:0.2f} m/s'.format(mean_velocity, reference_speed))
    print('Swarm mean standard deviation on the speed is {:0.2f} m/s'.format(std_velocity))
    print('_'*72)
    
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
        plt.title('Collision avoidance simulation \n t={:0.2f}s'.format(time))

        plot_cov_ellipse = True
        if plot_cov_ellipse:
            agent.kalman.ellipse_plot()
        
    plt.xlim(min(swarm.plant.ox), max(swarm.plant.ox))
    plt.ylim(min(swarm.plant.oy), max(swarm.plant.oy))


def plot_kalman_error(swarm):

    plt.figure('Kalman error in postion estimation [m]')
    for robot in swarm.robots:
        # diff_x = robot.kalman.hist.Dxk[0::3]
        # diff_y = robot.kalman.hist.Dxk[1::3]
        diff_xyt = np.array(robot.kalman.hist.Dxk)
        diff_x = diff_xyt[0::3]
        diff_y = diff_xyt[1::3]
        dist = np.sqrt(diff_x**2 + diff_y**2)
        plt.plot(robot.kalman.hist.Time, dist, "-" , color = robot.color, label='Position error robot'+str(robot.indexSwarm))
        plt.ylabel('Error in postion estimation [m]')
        plt.xlabel('Time [s]')
        plt.grid(True)
        plt.legend()
        plt.show()
    
    if hasattr(swarm, 'central_kalman'):
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
            plt.ylabel('Error in x direction - Centralized Kalman [m]')
            plt.xlabel('Time [s]')
            plt.plot(swarm.central_kalman.hist.Time, dist, "-" , color = robot.color, label='Position error robot'+str(robot.indexSwarm))
        plt.grid(True)
        plt.legend()
        plt.show()
    else:
        print(FAIL + 'No central kalman is initialized, nothing to plot' + ENDC)

def plot_MHE_error(swarm):
    if kalman_mhe:
        plt.figure('MHE error in postion estimation [m]')
        for robot in swarm.robots:
            diff_xyt = np.array(robot.MHE.hist.Dxk)
            diff_x = diff_xyt[0::3]
            diff_y = diff_xyt[1::3]
            dist = np.sqrt(diff_x**2 + diff_y**2)
            plt.plot(robot.MHE.hist.Time, dist, "-" , color = robot.color, label='Position error robot'+str(robot.indexSwarm))
            plt.ylabel('Error in postion estimation [m]')
            plt.xlabel('Time [s]')
            plt.show()
    else:
        print(FAIL + 'No MHE is initialized, nothing to plot' + ENDC)

def plot_filtered_trajectory(swarm):

    if kalman_mhe:
        plt.figure('Trajectory estimation')
        for robot in swarm.robots:
            _xytMHE = np.array(robot.MHE.hist.xk)
            _xMHE = _xytMHE[0::3]
            _yMHE = _xytMHE[1::3]
            plt.plot(_xMHE, _yMHE , "-." , color = robot.color, label='MHE robot'+str(robot.indexSwarm))
            _xytKAL = np.array(robot.kalman.hist.xk)
            _xKAL = _xytKAL[0::3]
            _yKAL = _xytKAL[1::3]
            plt.plot(_xKAL, _yKAL , "--" , color = robot.color, label='Kalman robot'+str(robot.indexSwarm))
            _xGT  = robot.history.x
            _yGT  = robot.history.y
            plt.plot(_xGT, _yGT,'-',color = robot.color, label='Ground truth'+str(robot.indexSwarm))

            plt.xlabel('x')
            plt.ylabel('y')
            plt.grid(True)
            plt.legend()
            plt.show()
    else:
        print(FAIL + 'No MHE is initialized, nothing to plot' + ENDC)

def plot_filtered_state(swarm):
    
    if kalman_mhe:
        for robot in swarm.robots:
            _xytMHE = np.array(robot.MHE.hist.xk)
            _xMHE = _xytMHE[0::3]
            _yMHE = _xytMHE[1::3]
            _thMHE= _xytMHE[2::3]
            tMHE  = robot.MHE.hist.Time
            _xytKAL = np.array(robot.kalman.hist.xk)
            _xKAL = _xytKAL[0::3]
            _yKAL = _xytKAL[1::3]
            _thKAL= _xytKAL[2::3]
            tKAL  = robot.kalman.hist.Time
            _xGT  = robot.history.x
            _yGT  = robot.history.y
            _thGT = robot.history.yaw
            tGT   = robot.history.time
            _xytz = np.array(robot.kalman.hist.xk)
            _xz   = _xytz[0::3]
            _yz   = _xytz[1::3]
            _thz  = _xytz[2::3]
            tz    = np.arange(len(_xz)) * 1/robot.kalman.Camera.rate

            plt.figure('Trajectory estimation subplots' +  str(robot.indexSwarm))

            plt.subplot(3, 1, 1)
            plt.plot(tGT, _xGT,'-',color = robot.color, label='Ground truth')
            plt.plot(tMHE, _xMHE,'-.',color = robot.color, label='MHE')
            plt.plot(tKAL, _xKAL,'--',color = robot.color, label='Kalman')
            plt.ylabel('x [m]')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(tGT, _yGT,'-',color = robot.color, label='Ground truth')
            plt.plot(tMHE, _yMHE,'-.',color = robot.color, label='MHE')
            plt.plot(tKAL, _yKAL,'--',color = robot.color, label='Kalman')
            plt.ylabel('y [m]')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(tGT, _thGT,'-',color = robot.color, label='Ground truth')
            plt.plot(tMHE, _thMHE,'-.',color = robot.color, label='MHE')
            plt.plot(tKAL, _thKAL,'--',color = robot.color, label='Kalman')
            plt.ylabel('$\\theta$ [rad]')
            plt.xlabel('Time [s]')
            plt.grid(True)
            plt.legend()

            plt.show()
    else:
        print(FAIL + 'No MHE is initialized, nothing to plot' + ENDC)


def compare_KalMHE(swarm):

    if kalman_mhe:
        for robot in swarm.robots:
            plt.figure('Error in position estimation Kalman vs MHE for robot ' +  str(robot.indexSwarm))
            diff_xyt = np.array(robot.MHE.hist.Dxk)
            diff_x = diff_xyt[0::3]
            diff_y = diff_xyt[1::3]
            dist = np.sqrt(diff_x**2 + diff_y**2)
            plt.plot(robot.MHE.hist.Time, dist, "-" , color = robot.color, label='MHE robot'+str(robot.indexSwarm))
            diff_xyt = np.array(robot.kalman.hist.Dxk)
            diff_x = diff_xyt[0::3]
            diff_y = diff_xyt[1::3]
            dist = np.sqrt(diff_x**2 + diff_y**2)
            plt.plot(robot.kalman.hist.Time, dist, ":" , color = robot.color, label='Kalman robot'+str(robot.indexSwarm))
            plt.ylabel('Error in postion estimation [m]')
            plt.xlabel('Time [s]')
            plt.grid(True)
            plt.legend()
            plt.show()
    else:
        print(FAIL + 'No MHE is initialized, nothing to plot' + ENDC)

def print_RMSE(swarm):
    print(OKBLUE + '*'*29 + '   BENCHMARK  ' + '*'*29 + ENDC)
    n = 3*n_robots
    for i, robot in enumerate(swarm.robots):

        diff_xytKAL = np.array( robot.kalman.hist.Dxk)
        diff_xKAL = diff_xytKAL[0::3]
        diff_yKAL = diff_xytKAL[1::3]
        diff_tKAL = diff_xytKAL[2::3]

        if kalman_mhe:
            diff_xytMHE = np.array( robot.MHE.hist.Dxk)
            diff_xMHE = diff_xytMHE[0::3]
            diff_yMHE = diff_xytMHE[1::3]
            diff_tMHE = diff_xytMHE[2::3]

        mx = i*3
        my = mx + 1
        mt = mx + 2
        if kalman_centralized:
            diff_xKALC = np.array(swarm.central_kalman.hist.Dxk[mx::n])
            diff_yKALC = np.array(swarm.central_kalman.hist.Dxk[my::n])
            diff_tKALC = np.array(swarm.central_kalman.hist.Dxk[mt::n])

        RMSE_xKAL = math.sqrt(np.sum(diff_xKAL**2)/len(diff_xKAL))
        RMSE_yKAL = math.sqrt(np.sum(diff_yKAL**2)/len(diff_yKAL))
        RMSE_tKAL = math.sqrt(np.sum(diff_tKAL**2)/len(diff_tKAL))
        
        if kalman_centralized:
            RMSE_xKALC = math.sqrt(np.sum(diff_xKALC**2)/len(diff_xKALC))
            RMSE_yKALC = math.sqrt(np.sum(diff_yKALC**2)/len(diff_yKALC))
            RMSE_tKALC = math.sqrt(np.sum(diff_tKALC**2)/len(diff_tKALC))
        if kalman_mhe:
            RMSE_xMHE  = math.sqrt(np.sum(diff_xMHE**2)/len(diff_xMHE))
            RMSE_yMHE  = math.sqrt(np.sum(diff_yMHE**2)/len(diff_yMHE))
            RMSE_tMHE  = math.sqrt(np.sum(diff_tMHE**2)/len(diff_tMHE))

        print(BOLD + 'Robot ' + str(robot.indexSwarm) + ENDC + ' RMSE VALUES:')
        print('\tDISTRIBUTED KALMAN x: {:0.2f} m'.format(RMSE_xKAL))
        if kalman_centralized:
            print('\tCENTRALISED KALMAN x: {:0.2f} m'.format(RMSE_xKALC))
        if kalman_mhe:
            print('\t               MHE x: {:0.2f} m'.format(RMSE_xMHE))
        print('\tDISTRIBUTED KALMAN y: {:0.2f} m'.format(RMSE_yKAL))
        if kalman_centralized:
            print('\tCENTRALISED KALMAN y: {:0.2f} m'.format(RMSE_yKALC))
        if kalman_mhe:
            print('\t               MHE y: {:0.2f} m'.format(RMSE_yMHE))
        print('\tDISTRIBUTED KALMAN θ: {:0.2f} rad'.format(RMSE_tKAL))
        if kalman_centralized:
            print('\tCENTRALISED KALMAN θ: {:0.2f} rad'.format(RMSE_tKALC))
        if kalman_mhe:
            print('\t               MHE θ: {:0.2f} rad'.format(RMSE_tMHE))

        if not kalman_centralized:
            print(WARNING + 'Warning Centralized Kalman wasn\'t initialized, skipping' + ENDC)
        if not kalman_mhe:
            print(WARNING + 'Warning MHE wasn\'t initialized, skipping' + ENDC)
        print('_'*72)