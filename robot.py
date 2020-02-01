from simulation_parameters import *
import math
import numpy as np
from lidar import *
from forces import force_giroscopic, force_potential, wall_potential
from utils import random_color
from kalman import kal, kal_hist
from MHE import MHE

class State:
    '''
    Includes the state of the robot and other useful quantites to recall during the motion
    '''
    def __init__(self, x=0.0, y=0.0, yaw = 0.0, v = 0.0, xdot = 0.0, ydot = 0.0, omega=0.0, vdot = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.xdot = xdot
        self.ydot = ydot
        self.omega = omega
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))
        self.vdot = vdot
        self.omegadot = 0
        self.u = np.array([0,0])

class Log:
    '''
    Saves some useful state quanties for the plotting and further computations
    '''
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.time = []

class obst():
    def __init__(self, x, y, psi, v):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v

def calc_distance(state, point_x, point_y):
    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)

def calc_target_index(agent, rx, ry):
    dx = [agent.state.rear_x - icx for icx in rx]
    dy = [agent.state.rear_y - icy for icy in ry]
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    min_d = min(d)
    if calc_distance(agent.state, agent.path.x[-1], agent.path.y[-1]) <= 1 or math.isnan(agent.state.x) or math.isnan(agent.state.y):
        agent.arrived = True
        agent.state.u = agent.state.u * 0
    ind = d.index(min_d)
    agent.nearest_point_index = ind
    agent.target_index = ind + 1

    if ind + 2 < len(agent.path.x):
        return ind + 2
    else:
        return ind + 1

def calc_target_index_RVO(agent, rx, ry, pind):
    dx = [agent.state.rear_x - icx for icx in rx[pind:]]
    dy = [agent.state.rear_y - icy for icy in ry[pind:]]
    d = [math.sqrt(idx ** 2 + idy ** 2)-Lfc for (idx, idy) in zip(dx, dy)]
    d_cut = [x for x in d if x >= 0]

    if calc_distance(agent.state, agent.path.x[-1], agent.path.y[-1]) <= 1 or math.isnan(agent.state.x) or math.isnan(agent.state.y):
        agent.arrived = True
    
    if d_cut == [] or d == []:
        i_plus = 0
    else:
        min_d = min(d_cut)
        i_plus = d.index(min_d)
    ind = i_plus + pind

    if ind >= len(rx)-1 :
        ind = len(rx)-1
        agent.docking = True
        

    agent.nearest_point_index = ind 
    agent.target_index = ind 

    return ind

class Points():
    def __init__(self, rx = [], ry = []):
        self.x = rx
        self.y = ry

class Robots:
    '''
    Is the agent that will move into the environment, coordinating with the other entities of the swarm, if they are present.
    If it is initialized as a Swarm() entity, it will have a unique indexSwarm.
    It contains:
        - current State() object
        - a path that comes from the planning or is predetermined in a Points() class which represents the points to follow
        - On the path the agent will try to try to reach a target_index of the planning which will be updated
        - a lidar instance that represents the point cloud reading of the environment
        - a Kalman filter algorithm to estimate the actual position
    '''
    def __init__(self):
        self.state = State()
        self.path = Points()
        self.nearest_point_index = 0
        self.last_index = 1
        self.target_index = None
        self.history = Log()
        self.L = L
        self.target_speed = reference_speed / 3.6 # [km/h]
        self.arrived = False
        self.color = random_color()
        self.lidar = lidar(self)
        self.communication_range = NotImplemented
        self.vicinity_range = NotImplemented
        self.indexSwarm = 0
        self.integral_error = 0
        self.V_ref = np.array([0,0])
        self.kalman = kal(self)
        if kalman_mhe:
            self.MHE = MHE(self)
        self.docking = False
    
    def update(self, delta):
        self.state.x = self.state.x + self.state.xdot * dt
        self.state.y = self.state.y + self.state.ydot * dt
        self.state.yaw = self.state.yaw + (self.state.omega) * dt
        self.state.v = self.state.v + self.state.vdot * dt
        self.state.rear_x = self.state.x - ((self.L / 2) * math.cos(self.state.yaw)) 
        self.state.rear_y = self.state.y - ((self.L / 2) * math.sin(self.state.yaw))
        # if self.state.yaw > 2 * np.pi:
        #     self.state.yaw = self.state.yaw - 2 * np.pi
        # elif self.state.yaw < 0:
        #     self.state.yaw = self.state.yaw + 2 * np.pi
        self.compute_controls()

    def update_derivatives(self, swarm, delta, flocking = False, giroscopic = True, potential = False, map_potential = True):
        # interacting = list(int(not agent.arrived) for agent in swarm)
        # selection_matrix = np.matrix(list(interacting for _ in swarm))
        # print(interacting, selection_matrix)
        X = np.matrix(list(agent.state.x for agent in swarm.robots))
        Y = np.matrix(list(agent.state.y for agent in swarm.robots))
        Theta = np.matrix(list(agent.state.yaw for agent in swarm.robots))
        if flocking:
            flocking_x = np.sum(G.laplacian_matrix[self.indexSwarm] * X.T)
            flocking_y = np.sum(G.laplacian_matrix[self.indexSwarm] * Y.T)
            flocking_theta = np.sum(G.laplacian_matrix[self.indexSwarm] * Theta.T)
        else:
            flocking_x = 0
            flocking_y = 0
            flocking_theta = 0

        F_flocking = np.array([flocking_x, flocking_y])
        F_potential = np.zeros(2)
        F_giroscopic = np.zeros(2)
        
        if giroscopic:
            gain = 0.7
            interacting = 0
            for i, value in enumerate(swarm.G.adjacency_matrix[self.indexSwarm]):
                if value == 1:
                    if swarm.robots[i].arrived == 1:
                        continue
                    else:
                        F_giroscopic = F_giroscopic + gain * force_giroscopic([self.state.x, self.state.y], 
                                                    [swarm.robots[i].state.x, swarm.robots[i].state.y],
                                                    [self.state.xdot, self.state.ydot],
                                                    [swarm.robots[i].state.xdot, swarm.robots[i].state.ydot],
                                                    clockwise = False)
                        interacting = interacting + 1
            if not interacting == 0:
                F_giroscopic = F_giroscopic/interacting
            
        if potential:
            gain = 0.3
            for i, value in enumerate(swarm.G.adjacency_matrix[self.indexSwarm]):
                if value == 1:
                    if swarm.robots[i].arrived == 1:
                        continue
                    else:
                        F_potential = F_potential + gain * force_potential([self.state.x, self.state.y], 
                                                                [swarm.robots[i].state.x, swarm.robots[i].state.y],
                                                                attractive = False)

        F_sum = F_flocking + F_giroscopic + F_potential

        if map_potential: # and not interacting == 0
            gain = 0.1
            for wall in self.lidar.near_walls:
                F_wall = wall_potential([self.state.x, self.state.y], wall, attractive = False)
                F_sum = F_sum + gain * F_wall
        
        F_sum_magnitude = np.linalg.norm(F_sum)
        saturation = 25
        # print('F_sum_magnitude ', F_sum_magnitude)
        if F_sum_magnitude > saturation:
            F_sum = F_sum * saturation / F_sum_magnitude
        
        # projection of the virtual force on the vehicle reference frame
        vehicle_direction = np.array([math.cos(self.state.yaw), math.sin(self.state.yaw)])
        F_parallel = np.dot(F_sum, vehicle_direction) * vehicle_direction
        F_perpendicular =  F_sum - np.dot(F_sum, vehicle_direction) * vehicle_direction   
        virtual_torque_sign = np.sign(np.cross(vehicle_direction, F_perpendicular))

        # commented to use after the integration, step, according to what are the variables needed
        self.state.xdot = (self.state.v * math.cos(self.state.yaw) + dt * (self.state.vdot * math.cos(self.state.yaw) + F_parallel[0]))
        self.state.ydot = (self.state.v * math.sin(self.state.yaw) + dt * (self.state.vdot * math.sin(self.state.yaw) + F_parallel[1]))
        self.state.omega = (self.state.v + dt * self.state.vdot)/ L * math.tan(delta) + math.sqrt(np.linalg.norm(F_perpendicular)/(robot_size*2)) * virtual_torque_sign # + flocking_theta[2]
        self.state.v = math.sqrt(self.state.xdot**2 + self.state.ydot**2)

    def compute_controls(self):
        self.state.u =  np.array([(2 * self.state.v + self.L * self.state.omega)/(2*r_w),
                                  (2 * self.state.v - self.L * self.state.omega)/(2*r_w)])

    def compute_controls_RVO(self):
        theta2 = math.atan2(self.V_ref[1],self.V_ref[0])
        psi = self.state.yaw
        beta = math.asin(math.sin(theta2 - psi))
        w = (beta)
        v = (np.linalg.norm(self.V_ref) - self.state.v)
        tmp = np.array([v,w])
        # A = np.array([[L,L],[1,-1]]) * r_w/(2*L)
        # Ainv = np.linalg.inv(A)
        Ainv = np.array([[1/r_w,L/r_w],[1/r_w,-L/r_w]])
        thetas_dot = Ainv @ tmp # theta_ddot
        self.state.u =  thetas_dot

    def update_diff_kine(self):
        theta = self.state.yaw
        B = np.array([[math.cos(theta),0],[math.sin(theta),0],[0,1]])
        A = np.array([[L,L],[1,-1]]) * r_w/(2*L)
        ev_matrix = B @ A
        state = np.array([self.state.x,self.state.y,self.state.yaw])
        control = self.state.u
        state = state + ev_matrix @ control * dt
        self.state.x = state[0]
        self.state.y = state[1]
        self.state.yaw = state[2]

        self.state.rear_x = self.state.x - ((L / 2) * math.cos(self.state.yaw))
        self.state.rear_y = self.state.y - ((L / 2) * math.sin(self.state.yaw))

        self.state.xdot = self.state.v * math.cos(self.state.yaw)
        self.state.ydot = self.state.v * math.sin(self.state.yaw)
        self.state.omega = np.array([r_w/(2*L),-r_w/(2*L)]).T @ control

        # if self.state.yaw > math.pi:
        #     self.state.yaw = self.state.yaw - 2*math.pi
        # elif self.state.yaw <  - math.pi:
        #     self.state.yaw = self.state.yaw + 2*math.pi
  
    def save_history(self, time):
        self.history.x.append(self.state.x)
        self.history.y.append(self.state.y)
        self.history.yaw.append(self.state.yaw)
        self.history.v.append(self.state.v)
        self.history.time.append(time)
    
    def nearby_obs(self, robots, adjacency_matrix): 
        # return a list of nearby robots and obstacle 
        near = []
        for _i , item in enumerate(adjacency_matrix[self.indexSwarm,]):
            if item == 1 and robots[_i].arrived == False:
                new_near = obst(robots[_i].state.x,robots[_i].state.y,robots[_i].state.yaw, robots[_i].state.v)
                near.append(new_near)
        for pt in self.lidar.lit_points:
            new_near = obst(pt[0],pt[1],0,0)
            near.append(new_near)
        return near

    def future_collision_detector(self, obstacles):
        if avoidance_algorithm == 'rvo':
            radius = self.lidar.range
        elif avoidance_algorithm == 'potential':
            radius = 10
        f_coll_flag = False
        T_coll1 = math.inf
        T_coll2 = math.inf
        for obs in obstacles:
            T_coll1_new = math.inf
            T_coll2_new = math.inf
            Dx  = obs.x - self.state.x
            Dy  = obs.y - self.state.y
            Dvx = obs.v * math.cos(obs.psi) - self.state.v * math.cos(self.state.yaw)
            Dvy = obs.v * math.sin(obs.psi) - self.state.v * math.sin(self.state.yaw)
            D2  = Dx**2 + Dy**2
            Dv2 = Dvx**2 + Dvy**2
            b = 2*(Dx*Dvx+Dy*Dvy)
            a = Dv2
            c = (D2-(2*radius)**2)
            Dlt = b**2-4*a*c
            if c <= 0 or Dv2 == 0:
                collision_flag = True
                continue
            if Dlt < 0:
                continue 
                # means they will never collide for the current velocities and orientation
            else:
                T_coll1_new = (-b + math.sqrt(Dlt))/(2*a)
                T_coll2_new = (-b - math.sqrt(Dlt))/(2*a)
                minT = np.min((T_coll1_new,T_coll2_new))
                maxT = np.max((T_coll1_new,T_coll2_new))
            if (maxT > 0) and ((minT)*self.state.v < self.lidar.range):
                #print("Colllision will probably occour in {0:.4}".format(np.min((T_coll1,T_coll2)) )) 
                if (minT < np.min((T_coll1,T_coll2))) :
                    T_coll1 = T_coll1_new 
                    T_coll2 = T_coll2_new 
                f_coll_flag =  True
        T_coll_flag = np.min((T_coll1,T_coll2))
        # plot the point of probable collision
        if f_coll_flag == True:
            plt.plot(self.state.x + self.state.v*T_coll_flag*math.cos(self.state.yaw), self.state.y +  self.state.v*T_coll_flag*math.sin(self.state.yaw), "o", color = self.color)  
            # plt.show() 

        return f_coll_flag, T_coll_flag
    
    def pure_pursuit_controlRVO(self, rx, ry, pind):

        ind = calc_target_index_RVO(self, rx, ry, pind)
        state = self.state

        # if pind >= ind:
        #     ind = pind

        if ind < len(rx):
            tx = rx[ind]
            ty = ry[ind]
        else:
            tx = rx[-1]
            ty = ry[-1]
            ind = len(rx) - 1
            

        difference = np.array([tx - state.rear_x, ty - state.rear_y])
        norm = np.linalg.norm(difference)
        V_des = difference * self.target_speed /norm 
        if ind >= len(rx):
            V_des = 0*V_des
        self.V_ref = V_des

    def RVO_update(self, swarm, G): # X,V_des, V_current, ws_model
        ROB_RAD = robot_size ###
        robdirA = np.array([math.cos(self.state.yaw),math.sin(self.state.yaw)])
        vA = self.state.v*robdirA
        pA = np.array([self.state.x,self.state.y])

        RVO_BA_all = []
        for _i , item in enumerate(G[self.indexSwarm,]):
        #for j in range(len(X)):
            if item == 1:
                robdirB = np.array([math.cos(swarm[_i].state.yaw),math.sin(swarm[_i].state.yaw)])
                vB = swarm[_i].state.v * robdirB
                pB = np.array([swarm[_i].state.x,swarm[_i].state.y])
                # use RVO
                transl_vB_vA = pA + 0.5*(vB+vA)
                BA_vec = pB - pA
                dist_BA = np.linalg.norm(BA_vec)
                theta_BA = math.atan2(BA_vec[1], BA_vec[0])
                if 2*ROB_RAD > dist_BA:
                    dist_BA = 2*ROB_RAD
                theta_BAort = math.asin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA+theta_BAort
                bound_left = np.array([math.cos(theta_ort_left), math.sin(theta_ort_left)])
                theta_ort_right = theta_BA-theta_BAort
                bound_right = np.array([math.cos(theta_ort_right), math.sin(theta_ort_right)])
                
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)        
        for wallpt in self.lidar.lit_points:        
            pB = np.array(wallpt)
            transl_vB_vA = pA + 0.5*(vA) ###dubbio
            BA_vec = pB - pA
            dist_BA = np.linalg.norm(BA_vec)
            theta_BA = math.atan2(BA_vec[1], BA_vec[0])
            # over-approximation of square to circular
            OVER_APPROX_C2S = 1.5
            rad = 0#ROB_RAD*OVER_APPROX_C2S
            if (rad+ROB_RAD) > dist_BA:
                dist_BA = rad+ROB_RAD
            theta_BAort = math.asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = np.array([math.cos(theta_ort_left), math.sin(theta_ort_left)])
            theta_ort_right = theta_BA-theta_BAort
            bound_right = np.array([math.cos(theta_ort_right), math.sin(theta_ort_right)])

            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        vA_post = intersect(pA, self.V_ref, RVO_BA_all)
        V_opt = vA_post

        self.V_ref = V_opt #V_des

def PIDControl(agent):
    agent.state.vdot = Kp * (agent.target_speed - agent.state.v) + Kd * (agent.history.v[-1] - agent.state.v) / dt + Ki * (agent.integral_error + dt * (agent.target_speed - agent.state.v))

def pure_pursuit_control(agent, rx, ry, pind):

    ind = calc_target_index(agent, rx, ry)
    state = agent.state
    if pind >= ind:
        ind = pind
    if ind < len(rx):
        tx = rx[ind]
        ty = ry[ind]
    else:
        tx = rx[-1]
        ty = ry[-1]
        ind = len(rx) - 1
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    Lf = k * state.v + Lfc
    delta = math.atan2(2.0 * agent.L * math.sin(alpha), Lf)
    agent.target_index = ind
    return delta

def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersection test'
    norm_v = np.linalg.norm(vA)
    suitable_V = []
    unsuitable_V = []

    new_v = vA
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = new_v+pA-p_0
        theta_dif = math.atan2(dif[1], dif[0])
        theta_right = math.atan2(right[1], right[0])
        theta_left = math.atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    
    if suit == True:
        return vA 

    for theta in np.arange(0, 2*math.pi, 0.1):
        # for rad in np.arange(0.02, norm_v+0.02, norm_v/5.0):
        for rad in np.arange(0, norm_v, norm_v/10.0):
            new_v = np.array([rad*math.cos(theta), rad*math.sin(theta)])
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]

                dif = new_v+pA-p_0
                theta_dif = math.atan2(dif[1], dif[0])
                theta_right = math.atan2(right[1], right[0])
                theta_left = math.atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    #----------------------        
    if suitable_V:
        # print 'Suitable found'
        vA_post = min(suitable_V, key = lambda v: np.linalg.norm(v - vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = new_v+pA-p_0
            theta_dif = math.atan2(dif[1], dif[0])
            theta_right = math.atan2(right[1], right[0])
            theta_left = math.atan2(left[1], left[0])
    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = unsuit_v+pA-p_0
                theta_dif = math.atan2(dif[1], dif[0])
                theta_right = math.atan2(right[1], right[0])
                theta_left = math.atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*math.sin(small_theta)) >= rad:
                        rad = abs(dist*math.sin(small_theta))
                    big_theta = math.asin(abs(dist*math.sin(small_theta))/rad)
                    dist_tg = abs(dist*math.cos(small_theta))-abs(rad*math.cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/np.linalg.norm(dif)
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+np.linalg.norm(v - vA)))
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= math.pi:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*math.pi
            if theta_dif < 0:
                theta_dif += 2*math.pi
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*math.pi
            if theta_dif < 0:
                theta_dif += 2*math.pi
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def is_increasing(difference, first_vec):
    dotprod = np.dot(difference, first_vec) # give the sign of the cosine
    if dotprod > 0:
        flag = +1
    elif dotprod == 0:
        flag = +1
    else:
        flag = -1
    return flag