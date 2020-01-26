import matplotlib.pyplot as plt
import numpy as np
class wall: 
    '''
    Defines a wall
    [xa, ya] and [xb, yb] are start and ending point
    '''
    def __init__(self, xa, ya, xb, yb): 
        self.xa = xa
        self.ya = ya
        self.xb = xb
        self.yb = yb
        self.length = ((xa-xb)**2 + (ya-yb)**2)**(1/2)
        self.dir = [(xb-xa)/self.length, (yb-ya)/self.length]

    def plot(self, color = 'grey', width = 5.0):
        plt.plot([self.xa, self.xb], [self.ya, self.yb], color = color, linewidth  = width)

class Map:
    '''
    Map is a objects containing the set of wall and obstacles
    '''
    def __init__(self):
        self.walls = []
        self.ox = []
        self.oy = []
    
    def from_data(self, A_points, B_points, possible_points):
        '''
        Function for initializing the walls of a map from a set of points,
        returns also the possible points to reach
        '''
        print('*'*22 + ' MAP LOADED ' + '*'*22)
        plant = Map()
        for A, B in zip(A_points, B_points):
            plant.add_wall(A, B)
        return plant, possible_points

    def add_wall(self, pta, ptb): 
        '''
        Function to create a new wall
        arguments: pta ptb as starting and ending point written as [xa,ya],[xb,yb]
        '''
        new_wall = wall(pta[0], pta[1], ptb[0], ptb[1])
        print('Added new wall from [{0:}, {1:}] to [{2:}, {3:}]'.format(pta[0], pta[1], ptb[0], ptb[1]))
        step = 1
        wall_numpoint = int(new_wall.length/step)
        xwall = np.linspace(pta[0],ptb[0],wall_numpoint)
        ywall = np.linspace(pta[1],ptb[1],wall_numpoint)
        self.ox = np.hstack((self.ox, xwall))
        self.oy = np.hstack((self.oy, ywall))
        self.walls.append(new_wall)
    