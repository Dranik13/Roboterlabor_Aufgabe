'''
The Python script was created as part of the examination for the lecture "Robot Programming" at the University of Applied Sciences Karlsruhe. It serves as a basis for all path smoothing algorithms for the paths determined by the various PRM planners.

Authors: Benjamin Dilly, Dennis MCNab, and Anton Kisel
Date: 01/21/2026

Responsible Professor: Prof. Dr.-Ing. Björn Hein
'''
import random
import numpy as np
import networkx as nx
import copy
from SmootherBase import SmootherBase, Angle 
import time

class SmoothRandom(SmootherBase):
    '''
    This class implements a path smoothing algorithm based on random sampling.
    '''
    def __init__(self, planner, path):
        super().__init__()
        self.smoothed_path = path.copy()
        self.path_planner = copy.deepcopy(planner)
        self.id_counter = 0
        self.limits = planner._collisionChecker.getEnvironmentLimits()

    def smooth_path(self, config):
        '''        
        :param path: Collison free path of path planner
        :param planner: The path planer used for creating the collision free base path
        :param config: Configuration of smoother \n
            "epoches": Max amount of iterations to do / amount of trials to smooth the path\n
            "collision_intervals": Amount of interpolation points to use for the collision checking \n
        :return: smoothed path 
        '''
        self.config:dict = config
        start_time = time.time()
        for i in range(self.config["epoches"]):
            # a could (and should) be replaced by a maximum number of attempts
            a = True
            while a:
                if self.findRandomShortcut():
                    self.path_per_epoche.append(list(self.smoothed_path))
                    break
        self.smoothing_time = time.time() - start_time
        return self.smoothed_path, self.path_planner.graph
    
    def findRandomShortcut(self):
        points = []
        node_positions = nx.get_node_attributes(self.path_planner.graph,'pos')
        
        # Place a random point on each edge.
        # The idea was to generate one point per edge in order to define constraints.
        # For example, the Euclidean distance between consecutive points could be checked.
        # If the distance would be too small, the next point could be selected.
        # Multiple nodes could also be evaluated within a single iteration.
        for i in range(len(self.smoothed_path) - 1):
            node_index1 = self.smoothed_path[i]
            node_index2 = self.smoothed_path[i+1]
            points.append(self.randomPtOnEdge(node_positions[node_index1], node_positions[node_index2]))

        shortcut_collides = True
        tries = 0
        # Select two points and check whether they can be connected collision-free.
        # Try up to 20 times to find a shortcut between the given points.
        while shortcut_collides:
            u = random.randint(0, len(points)-1)
            # Ensure that the second point (v) is not equal to the first point (u)
            v = random.choice([i for i in range(0,len(points)-1) if i != u])

            if(u > v):
                u, v = v, u
            
            shortcut_collides = self.path_planner._collisionChecker.lineInCollision(points[u], points[v], self.config["collision_intervals"])
            
            if not shortcut_collides:
                self.insertAndConnectPointsOnEdges(points, u, v)
                return True
            elif tries >= 20:
                return False 
            tries+=1
    
    def randomPtOnEdge(self, start_node_pt, end_node_pt):
        # random number t [0, 1]
        t = random.random()
        
        # Linearly interpolated point
        x = (1 - t) * start_node_pt[0] + t * end_node_pt[0]
        y = (1 - t) * start_node_pt[1] + t * end_node_pt[1]
        if len(start_node_pt) == 3:
            orientation = self.interpAngle(start_node_pt[2], end_node_pt[2], t)
            return (x, y, orientation)
        else:
            return (x, y)

    def interpAngle(self, theta0, theta1, t):
        # Create angle objects with valid bounds
        angle_0 = Angle(theta0, self.limits[2][0], self.limits[2][1])
        angle_1 = Angle(theta1, self.limits[2][0], self.limits[2][1])
        
        # minimal difference (gibt Wert zwischen -π und π)
        d = angle_1.value - angle_0.value        
        # Interpoliere und normalisiere im gültigen Bereich
        interpolated = angle_0.value + t * d
        # Interpolate and normalize within the valid range
        result = Angle(interpolated, self.limits[2][0], self.limits[2][1])
        return result.value

    def insertAndConnectPointsOnEdges(self, random_edge_pts, u, v):
        # # Labeling of new nodes: S1, S2, S3...
        id_u = f"S{self.id_counter + 1}"
        id_v = f"S{self.id_counter + 2}"

        # Adjust solution path
        if v == u + 1:
            # Directly adjacent: overwrite value at u, insert S2 afterwards,
            # remaining elements stay unchanged
            self.smoothed_path[u+1:u+2] = [id_u, id_v]
        else:
            # Otherwise: replace the range u..v with [S1, S2]
            self.smoothed_path[u+1:v+1] = [id_u, id_v]

        self.path_planner.graph.add_node(id_u, pos=random_edge_pts[u], color="#e28a0e")
        self.path_planner.graph.add_node(id_v, pos=random_edge_pts[v], color="#e28a0e")
        
        self.path_planner.graph.add_edge(self.smoothed_path[u], id_u)
        self.path_planner.graph.add_edge(id_u, id_v)
        
        if v == u+1:
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[v+2])
        else:
            # Find the index of the second shortcut node in the solution list
            i_in_solution = self.smoothed_path.index(id_v)
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[i_in_solution+1])

        self.id_counter += 2

        return True