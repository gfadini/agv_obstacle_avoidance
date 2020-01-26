import numpy as np
import math
from wall import wall
import matplotlib.pyplot as plt
import time as tim 

class lidar:
    '''
    Defines the model of the lidar sensor
    '''
    def __init__(self, parent, numBeams = 15, sensor_range = 10, color = 'r'):
        self.parent = parent
        self.beams = numBeams
        self.range = sensor_range
        self.angles = np.linspace(-math.pi/2, math.pi/2,self.beams)
        self.distances =  [0] * numBeams
        self.sigma = 0 # error
        self.lit_points = []
        self.near_walls = None  

    def plotscan(self):
        if self.lit_points != None:
            for point in self.lit_points:
                plt.plot(point[0] , point[1], "o", markersize = 2, color = self.parent.color)
                plt.plot([self.parent.state.x, point[0]], [self.parent.state.y, point[1]],color = self.parent.color, linewidth = 0.7, alpha = 0.3)
                
        # Plot all rays
        # for k in range(len(self.angles)):
        #     plt.plot([self.parent.state.x, self.parent.state.x + math.cos(self.angles[k] + self.parent.state.yaw) * self.distances[k]],
        #              [self.parent.state.y, self.parent.state.y + math.sin(self.angles[k] + self.parent.state.yaw) * self.distances[k]],
        #              color = self.parent.color, alpha = 0.4)
    
    def get_near_walls(self, plant):
        '''
        If the robot inside a check set meaning the union of a rectangle [range x wall_lenght] and 2 circles of radius range 
        centered in the extremes of the wall, then than the wall will be added in the close walls for further computations.
        '''
        position = np.array([self.parent.state.x, self.parent.state.y])
        near_walls = []
        for wall in plant.walls:
            ra = np.array([wall.xa, wall.ya])
            rb = np.array([wall.xb, wall.yb])
            if ( abs(np.dot(position - ra, wall.dir)) <= wall.length and np.linalg.norm(np.cross(position - ra, wall.dir)) <= self.range**2) or \
                np.linalg.norm(position - ra) <= self.range or \
                np.linalg.norm(position - rb) <= self.range: 
                near_walls.append(wall)
        self.near_walls = near_walls

    def distance2wall(self, ray, walls, idx):
        first_saved = True
        dist = self.range
        #Mij are entries of a matrix M
        M11 = math.cos(ray+self.parent.state.yaw)
        M21 = math.sin(ray+self.parent.state.yaw)
        for wall in walls:
            # linear system to get the instance
            M12 = - wall.dir[0] * wall.length
            M22 = - wall.dir[1] * wall.length
            DET = M11*M22-M21*M12 # determinant of matrix M
            
            # Bi are entries of vector B
            B1 = wall.xa - self.parent.state.x
            B2 = wall.ya - self.parent.state.y
            # check if ray is parallel with the wall usign determinant(linear dipendency)
            if DET==0:
                continue
            # solve the linear system with explicit inversion (efficient)
            t_dist =  (M22*B1-M12*B2)/DET
            u_dist = (-M21*B1+M11*B2)/DET
            # if statement to deal with various cases
            if any([u_dist > 1, u_dist < 0, t_dist < 0, t_dist > dist]):
                continue

            elif t_dist <= dist:
                dist = t_dist
                if first_saved: # if it is the first it is added
                    self.lit_points.append( [self.parent.state.x+math.cos(ray+self.parent.state.yaw)*dist,
                                             self.parent.state.y+math.sin(ray+self.parent.state.yaw)*dist ] )
                    first_saved = False
                else: # if it is not the first overwrite
                    self.lit_points[-1] = [self.parent.state.x+math.cos(ray+self.parent.state.yaw)*dist,
                                           self.parent.state.y+math.sin(ray+self.parent.state.yaw)*dist ]
            else:
                #dist = self.range
                continue
        return dist
            
    def scan(self, plant):
        '''
        Scanning function, needs a sensor and a set of walls 
        '''
        # clear the points
        self.lit_points = []
        self.get_near_walls(plant)
        # define an array of floats to range
        for ray in self.angles: 
            #idx = self.angles.index(ray)
            idx = np.where(self.angles == ray)[0][0] #DC
            self.distances[idx]  = self.distance2wall(ray, self.near_walls, idx)