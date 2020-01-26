'''
 Path planning methods for the roadmap generation
'''
from simulation_parameters import *
from robot import *
from utils import *
from wall import Map, wall
import numpy as np
import scipy.spatial
import random
import math
import cv2

from CentralizedKalman  import centralized_kal###

class Node:
    '''
    Node class for dijkstra search
    '''

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    '''
    Nearest neighbor search class with KDTree
    '''

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        '''
        Search NN

        inp: input data, single frame or multi frame

        '''

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        '''
        find points with in a distance r
        '''

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)
    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)

    if show_animation_roadmap:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)
    
    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for i in range(nstep):
        idxs, dist = okdtree.search(np.array([x, y]).reshape(2, 1))
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    '''
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    '''

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.array([ix, iy]).reshape(2, 1), k=nsample)
        inds = index[0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    '''
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gy: goal y position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    rr: robot radius [m]
    road_map: ??? [m]
    sample_x: ??? [m]
    sample_y: ??? [m]
    
    @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
    '''

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    path_found = True
    
    while True:
        if not openset:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        # show graph
        if show_animation_roadmap and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            #plt.pause(0.0001)

        if c_id == (len(road_map) - 1):
            print("Start and target are connected")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node
    
    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() * (maxx - minx)) + minx
        ty = (random.random() * (maxy - miny)) + miny

        index, dist = obkdtree.search(np.array([tx, ty]).reshape(2, 1))

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y

def planning_from_map():
    # TODO
    return NotImplemented

def polygon(n_robots = n_robots, radius = 50, n_edges = 8, origin = [0, 0]):
    points_A = list([radius * math.cos(2*math.pi*i/n_edges), radius * math.sin(2*math.pi*i/n_edges)] for i in range(n_edges))
    points_B = points_A[1:] + points_A[:1] 
    possible = list(1/math.sqrt(2) * radius * math.cos(2*math.pi*i/n_edges) for i in range(n_edges))

    n_points = 20

    swarm = []
    
    for i in range(n_robots):
        rx = list(0.9 * radius * 2 * (j/n_points - 1/2) * math.cos(2*math.pi*i/n_robots) + 0 * random.random() for j in range(n_points))
        ry = list(0.9 * radius * 2 * (j/n_points - 1/2) * math.sin(2*math.pi*i/n_robots) + 0 * random.random() for j in range(n_points))
        agent = Robots()
        agent.state.x = rx.pop(0)                                                  # initializing initial position of each robot
        agent.state.y = ry.pop(0)
        agent.state.yaw = 2*math.pi*i/n_robots
        agent.path.x = rx
        agent.path.y = ry
        agent.last_index = len(rx) - 1
        agent.indexSwarm = i

        agent.state.rear_x = agent.state.x - ((L / 2) * math.cos(agent.state.yaw))
        agent.state.rear_y = agent.state.y - ((L / 2) * math.sin(agent.state.yaw))
        swarm.append(agent)

        agent.kalman = kal(agent)
        

    poly = [
                list(points_A),
                list(points_B),
                possible
            ]
    # return poly, sx, sy, gx, gy
    return poly, swarm

def random_initialize(possible_points):
    return list(random.choice(possible_points) + random.randint(-1, 1) for _ in range(n_robots))


def contours2wallchain(cont, map_resolution = 0.1):
   ptA = []
   ptB = []
   for polygon in cont:
      first = [0,0]
      for idx, point in enumerate(polygon):
         ptA.append([map_resolution*point[0][0],map_resolution*point[0][1]])
         if idx == 0:
            #first = [map_resolution*point[0][0],map_resolution*point[0][1]]
            first = ptA[-1]
            continue
         #ptB.append([map_resolution*point[0][0],map_resolution*point[0][1]])
         ptB.append(ptA[-1])
      ptB.append(first)
   return ptA, ptB

def img2map(filename, map_resolution = 0.1):
   # pxlsz = map_resolution #[m/pixel]
   # img = cv2.imread("plant.png")
   img = cv2.imread(filename)
   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   ret,thresh1 = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
   contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   wallA, wallB = contours2wallchain(contours, map_resolution)
   return wallA , wallB

def load_from_image(imagename):
    mappa = list(img2map(imagename, 0.3))
    mappa.append([30.0, 150.0, 270.0])

    return Map().from_data(*mappa)


'''
 support for maps with the format
 [[list_points_A], [list_points_B], [possible_points]]
'''

map_1 = [
            [[0, 0], [60,0],  [0, 0], [0, 60], [20, 0], [40,60], [40,30]],
            [[60,0], [60,60], [0,60], [60,60], [20,40], [40,20], [30,20]],
            [10.0, 30.0, 50.0]
        ]

map_2 = [
            [[0,  0], [100,  0], [0,  0], [0,  100], [20, 0], [20,100], [0, 30], [40,30], [40,40], [70,100], [70, 20], [20,80], [40,30]],
            [[100,0], [100,100], [0,100], [100,100], [20,10], [20, 50], [40,30], [40,60], [80,40], [70, 60], [100,20], [50,80], [30,20]],
            [10.0, 50.0, 90.0]
        ]


def probabilistic_roadmap(swarm, plant, possible_points):
    '''
    test random multiple robots
    '''
    sx = random_initialize(possible_points)
    sy = random_initialize(possible_points)
    gx = random_initialize(possible_points)
    gy = random_initialize(possible_points)


    '''
    test two robots
    - to see how the algorithm behaves in a simple case, for map 2
    '''
    # sx = [10., 40.]
    # sy = [10., 10.]
    # gx = [40., 10.]
    # gy = [10., 10.]

    for wall in plant.walls:
        wall.plot()
    plt.show()

    ox = plant.ox
    oy = plant.oy

    print(HEADER + '*'*22 + ' STARTING PATH PLANNING ' + '*'*22 + ENDC)

    
    swarm.plant = plant
    
    i = 0

    if show_animation_roadmap:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    while i < n_robots:
        agent = Robots()
        agent.indexSwarm = i
        print(str('Created a task for' + BOLD + ' Robot {:0}\n' + ENDC + ' it must go from position [{:1.4}, {:2.4}] to [{:3.4}, {:4.4}]').format(
            i, sx[i], sy[i], gx[i], gy[i]
        ))
        rx, ry = PRM_planning(sx[i], sy[i], gx[i], gy[i], ox, oy, safety_distance)    # computing and assigning path to each agent
        rx.reverse()
        ry.reverse()

        if rx == [] or ((sx[i] - gx[i])**2 + (sy[i] - gy[i])**2) < 25:
            print(FAIL + ' but the kinematic of robot {:0} prevents it from reaching goal, retrying'.format(i) + ENDC)
            sx = random_initialize(possible_points)
            sy = random_initialize(possible_points)
            continue
        else:
            print(OKGREEN + ' the path is feasible, routine set' + ENDC)
            print('-'*68)
            agent.path.x = rx                                                    
            agent.path.y = ry
            agent.last_index = len(rx) - 1
            agent.state.x = sx[i]                                                     # initializing initial position of each robot
            agent.state.y = sy[i]
            agent.state.rear_x = agent.state.x - ((L / 2) * math.cos(agent.state.yaw))
            agent.state.rear_y = agent.state.y - ((L / 2) * math.sin(agent.state.yaw))
            i = i + 1
            swarm.robots.append(agent)
            
            agent.kalman = kal(agent)

            if show_animation_roadmap:
                plt.plot(rx, ry, "-r") 

    if show_animation_roadmap:
        plt.show(block = False)
        plt.pause(3)
        plt.close()