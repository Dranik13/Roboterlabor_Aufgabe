'''
The Python script was created as part of the examination for the lecture "Robot Programming" at the University of Applied Sciences Karlsruhe. It serves as a basis for all path smoothing algorithms for the paths determined by the various PRM planners.

Authors: Benjamin Dilly, Dennis MCNab, and Anton Kisel
Date: 01/21/2026

Responsible Professor: Prof. Dr.-Ing. Björn Hein
'''

import numpy as np 

from SmootherBase import SmootherBase,Angle

import IPEnvironmentKin
import IPEnvironmentShapeRobot
import time
from math import *
import copy

class SmoothBG(SmootherBase):
    '''
    This class implements the smothing algorithm after Bechthold Glavina
    '''
    def __init__(self):
        super().__init__()
    
    def smooth_path(self, path, planner, config, clean_up = False):
        '''        
        :param path: Collison free path of path planner
        :param planner: The path planer used for creating the collision free base path
        :param config: Configuration of smoother \n
            "corner_threshold": Threshold of interesting corner strength. Can be set to 0. Then the smoothing runs as long, as an corner strength != 0 exists or epoches exceeded\n
            "epoches": Max amount of iterations to do / amount of trials to smooth the path\n
            "collision_intervals": Amount of interpolation points to use for the collision checking \n
            "max_deltree_depth": Number of attempts to connect via intermediate steps if a direct connection is not possible\n
        :return: smoothed path 
        '''
        self.smoothed_path.clear()
        
        if path == []:
            return []
        
        collision_free_path:list = list(path)
        self.path_planner = copy.deepcopy(planner)
        self.config:dict = config
        
        corner_threshold:float = self.config["corner_threshold"]
        epoches = self.config["epoches"]
        epoche_counter = 0

        self.added_nodes = 0
      
        skip_possible = True
        
        priority_list = []

        start_time = time.time()

        # Iteration Loop
        while epoche_counter != epoches and skip_possible == True:
            
            skip_possible = False
            priority_list = []

            self.path_per_epoche.append(list(collision_free_path))
            epoche_counter += 1

            # Filling up the priority list                      
            for i in range(0, len(collision_free_path) - 2):
                start_node_name = collision_free_path[i]
                skip_node_name = collision_free_path[i + 1]
                goal_node_name = collision_free_path[i + 2]
                                
                start_node = np.array(self.path_planner.graph.nodes[start_node_name]["pos"])
                skip_node = np.array(self.path_planner.graph.nodes[skip_node_name]["pos"])
                goal_node = np.array(self.path_planner.graph.nodes[goal_node_name]["pos"])
                
                
                collission_checker_type = type(self.path_planner._collisionChecker)
                limits = self.path_planner._collisionChecker.getEnvironmentLimits()
            
                if collission_checker_type == type(IPEnvironmentShapeRobot.ShapeRobotWithOrientation):
                    start_node[2] = Angle(float(start_node[2]), limits[2][0], limits[2][1])
                    skip_node[2] = Angle(float(skip_node[2]), limits[2][0], limits[2][1])
                    goal_node[2] = Angle(float(goal_node[2]), limits[2][0], limits[2][1])
                    
                    
                if collission_checker_type == type(IPEnvironmentKin.KinChainCollisionChecker):
                    for i in range(len(limits)):
                        start_node[i] = Angle(float(start_node[i]), limits[i][0], limits[i][1])
                        skip_node[i] = Angle(float(skip_node[i]), limits[i][0], limits[i][1])
                        goal_node[i] = Angle(float(goal_node[i]), limits[i][0], limits[i][1])
            
            
                # check if it's a edge which is worth to get skipped
                indirect_connection = np.linalg.norm(skip_node-start_node) + np.linalg.norm(goal_node - skip_node)
                direct_connection = np.linalg.norm(goal_node - start_node)
                
                edge = indirect_connection / direct_connection
                
                if edge > corner_threshold:

                    priority_list.append((
                        edge,
                        i,
                        start_node_name,
                        skip_node_name,
                        goal_node_name,
                        np.copy(start_node),
                        np.copy(skip_node),
                        np.copy(goal_node)
                    ))
                    

            if priority_list != []:
                priority_list.sort(key=lambda s: s[0], reverse=True) 

            # Try to skip the strongest corner and continue the priority list, if it's not possible          
            for _, max_id, max_start_node_name,max_skip_node_name,max_goal_node_name, \
                max_start_node,max_skip_node,max_goal_node in priority_list:

                if self.try_direct_skip(max_start_node, max_goal_node):
                    collision_free_path.remove(max_skip_node_name)
                    skip_possible = True
                    break
                else:                    
                    success, new_start, new_goal = self.try_deltree_skip(max_start_node, max_goal_node, max_skip_node)
                    if success:
                        collision_free_path.insert(max_id + 1, "new_start_node_" + str(self.added_nodes))
                        collision_free_path.insert(max_id + 2, "new_goal_node_" + str(self.added_nodes))
                        # add to graph
                        self.path_planner.graph.add_node("new_start_node_" + str(self.added_nodes), pos = list(new_start))
                        self.path_planner.graph.add_node("new_goal_node_" + str(self.added_nodes), pos = list(new_goal))
                        
                        self.path_planner.graph.add_edge(max_start_node_name, "new_start_node_" + str(self.added_nodes))
                        self.path_planner.graph.add_edge("new_start_node_" + str(self.added_nodes), "new_goal_node_" + str(self.added_nodes))
                        self.path_planner.graph.add_edge("new_goal_node_" + str(self.added_nodes), max_goal_node_name)
                        
                        
                        # remove skipped node
                        collision_free_path.remove(max_skip_node_name)

                        
                        
                        self.added_nodes += 2
                        skip_possible = True
                        break
           
            
        self.smoothing_time = time.time() - start_time    
            
        # Remove all skipped nodes out of the graph
        if clean_up:
            nodes = self.path_planner.graph.nodes()
            to_remove = []
            for node in nodes:
                if not node in collision_free_path and type(node) == type(""):
                    to_remove.append(node)
        
            for node in to_remove:
                self.path_planner.graph.remove_node(node)


        self.smoothed_path = collision_free_path

        return collision_free_path, self.path_planner.graph
                
            
                    
                
            
    
    def try_direct_skip(self, start, goal) -> bool:
        collision_intervals = self.config["collision_intervals"]
        
        unit_vector = (goal - start) / collision_intervals
        
        for i in range(0, collision_intervals + 1):
            if self.path_planner._collisionChecker.pointInCollision(start + unit_vector * i):
                return False
        
        return True
    
    def try_deltree_skip(self, start, goal, skip_node):
        max_deltree_depth:int = self.config["max_deltree_depth"]
        # min_deltree_delta:float = self.config["min_deltree_delta"]
        
        '''
        From step to step the new_start and new_goal node are getting closer to the skipping node
        '''
        for k in range(1, max_deltree_depth + 1):
            new_start_node = start + (skip_node - start) / 2**k
            new_goal_node = skip_node + (goal - skip_node) / 2**k
            
            # delta_1 = np.linalg.norm(skip_node - new_start_node)
            # delta_2 = np.linalg.norm(new_goal_node - skip_node)
            
            # if delta_1 <= min_deltree_delta and delta_2 <= min_deltree_delta:
            #     break
            
            if self.try_direct_skip(new_start_node, new_goal_node):
                return True, new_start_node, new_goal_node
            
            start = new_start_node
            goal = new_goal_node
            
        return False, None, None

   