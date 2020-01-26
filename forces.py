import numpy as np
import math
from simulation_parameters import *

def get_distance(point_A, point_B):
    dx = point_A[0] - point_B[0]
    dy = point_A[1] - point_B[1]
    return math.sqrt(dx ** 2 + dy ** 2)

def force_potential(point_A, point_B, attractive = True):
    
    distance = abs(get_distance(point_A, point_B) - robot_size)
    if distance < 1e-1:
        distance = 1e-1

    if attractive:
        k = 1
    else:
        k = - 1
    
    force_A = k * (np.array(point_B) - np.array(point_A)) / distance**2

    return force_A

def force_giroscopic(point_A, point_B, velocity_A, velocity_B, clockwise = False):
    
    distance = abs(get_distance(point_A, point_B) - robot_size)
    if distance < 1e-1:
        distance = 1e-1

    velocity_A_norm = np.linalg.norm(np.array(velocity_A))
    if velocity_A_norm < 1e-1:
        velocity_A_norm = 1e-1
    
    velocity_B_norm = np.linalg.norm(np.array(velocity_B))
    if velocity_B_norm < 1e-1:
        velocity_B_norm = 1e-1
    
    velocity_versor_A = np.array(velocity_A)/velocity_A_norm
    velocity_versor_B = np.array(velocity_B)/velocity_B_norm

    if clockwise:
        k = -1
    else:
        k = 1
    
    kf = (1 - np.dot(velocity_versor_A, velocity_versor_B))/2

    force_A = k * kf * np.array([- velocity_versor_A[1], velocity_versor_A[0]]) / distance

    return force_A


def wall_potential(position, wall, attractive = True):
    position = np.array(position)
    ra = np.array([wall.xa, wall.ya])
    rb = np.array([wall.xb, wall.yb])

    projection = np.dot(wall.dir, position - ra)

    if attractive:
        k = - 1
    else:
        k = 1

    if projection <= 0:
        distance = abs(get_distance(position, ra) - robot_size)
        if distance < 1e-1:
            distance = 1e-1
        return k * (position - ra) / distance**2

    elif projection >= wall.length:
        distance = abs(get_distance(position, rb) - robot_size)
        if distance < 1e-1:
            distance = 1e-1
        return k * (position - rb) / distance**2

    else:
        dist_along = np.linalg.norm(np.cross(np.array(position) - ra, wall.dir))
        perpendicular_vector = position - ra - np.dot(position - ra, wall.dir) * np.array(wall.dir)
        return k * perpendicular_vector / (dist_along - robot_size)**2
