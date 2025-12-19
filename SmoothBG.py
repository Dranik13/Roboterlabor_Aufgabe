import numpy as np 
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation

from IPython.display import HTML, display

from IPVISLazyPRM import visibilityPRMVisualizeWspace

from SmootherBase import SmootherBase,Angle


import IPEnvironment 
import IPEnvironmentKin
import IPEnvironmentShapeRobot

from math import *

class SmoothBG(SmootherBase):
    def __init__(self):
        super().__init__()
    
    def smooth_path(self, path, planer, config, clean_up = False):
        '''        
        :param path: Collison free path of path planer
        :param planer: Used path planer algorithem for access to the graph and collisionchecker
        :param config: Configuration of smoother
        :return: smoothed path 
        '''
        self.smoothed_path.clear()
        
        if path == []:
            return []
        
        collision_free_path:list = list(path)
        self.path_planer = planer
        self.config:dict = config
        
        corner_threshold:float = self.config["corner_threshold"]
        epoches = self.config["epoches"]
        epoche_counter = 0
        
        '''
        Run the skip process. Starting with first node till the last one
        '''
        self.added_nodes = 0
      
        max_corner = inf
        skip_possible = True
        
        not_smooth_able_pairs = []
        
        while max_corner >= corner_threshold and epoche_counter != epoches and skip_possible == True:
            
            self.path_per_epoche.append(list(collision_free_path))
            epoche_counter += 1
            
            max_start_node_name = None
            max_goal_node_name = None
            max_skip_node_name = None
            
            max_start_node = None
            max_skip_node = None
            max_goal_node = None
            
            max_corner = 0.0
            max_id = 0.0
                        
            for i in range(0, len(collision_free_path) - 2):
                start_node_name = collision_free_path[i]
                skip_node_name = collision_free_path[i + 1]
                goal_node_name = collision_free_path[i + 2]
                                
                start_node = np.array(self.path_planer.graph.nodes[start_node_name]["pos"])
                skip_node = np.array(self.path_planer.graph.nodes[skip_node_name]["pos"])
                goal_node = np.array(self.path_planer.graph.nodes[goal_node_name]["pos"])
                
                
                collission_checker_type = type(self.path_planer._collisionChecker)
                limits = self.path_planer._collisionChecker.getEnvironmentLimits()
            
                if collission_checker_type == type(IPEnvironmentShapeRobot.ShapeRobotWithOrientation):
                    start_node[2] = Angle(float(start_node[2]), limits[2][0], limits[2][1])
                    skip_node[2] = Angle(float(skip_node[2]), limits[2][0], limits[2][1])
                    goal_node[2] = Angle(float(goal_node[2]), limits[2][0], limits[2][1])
                    
                    
                if collission_checker_type == type(IPEnvironmentKin.KinChainCollisionChecker):
                    for i in range(len(limits)):
                        start_node[i] = Angle(float(start_node[i]), limits[i][0], limits[i][1])
                        skip_node[i] = Angle(float(skip_node[i]), limits[i][0], limits[i][1])
                        goal_node[i] = Angle(float(goal_node[i]), limits[i][0], limits[i][1])
            
            
                # check id it's a edge which is worth to get skipped
                indirect_connection = np.linalg.norm(skip_node-start_node) + np.linalg.norm(goal_node - skip_node)
                direct_connection = np.linalg.norm(goal_node - start_node)
                
                edge = abs(1.0 - indirect_connection / direct_connection)
                
                if edge > max_corner and edge > corner_threshold and not i in not_smooth_able_pairs:
                    max_corner = edge
                    
                    max_start_node_name = start_node_name
                    max_skip_node_name = skip_node_name
                    max_goal_node_name = goal_node_name
                    
                    max_start_node = np.copy(start_node)
                    max_skip_node = np.copy(skip_node)
                    max_goal_node = np.copy(goal_node)
                    
                    max_id = i
                    
                        
            if max_skip_node_name != None:

                if self.try_direct_skip(max_start_node, max_goal_node):
                    collision_free_path.remove(max_skip_node_name)
                    not_smooth_able_pairs.clear()
                else:                    
                    success, new_start, new_goal = self.try_deltree_skip(max_start_node, max_goal_node, max_skip_node)
                    if success:
                        collision_free_path.insert(max_id + 1, "new_start_node_" + str(self.added_nodes))
                        collision_free_path.insert(max_id + 2, "new_goal_node_" + str(self.added_nodes))
                        # add to graph
                        self.path_planer.graph.add_node("new_start_node_" + str(self.added_nodes), pos = list(new_start))
                        self.path_planer.graph.add_node("new_goal_node_" + str(self.added_nodes), pos = list(new_goal))
                        
                        self.path_planer.graph.add_edge(max_start_node_name, "new_start_node_" + str(self.added_nodes))
                        self.path_planer.graph.add_edge("new_start_node_" + str(self.added_nodes), "new_goal_node_" + str(self.added_nodes))
                        self.path_planer.graph.add_edge("new_goal_node_" + str(self.added_nodes), max_goal_node_name)
                        
                        
                        # remove skipped node
                        collision_free_path.remove(max_skip_node_name)

                        
                        
                        self.added_nodes += 2
                        not_smooth_able_pairs.clear()
                    else:
                        not_smooth_able_pairs.append(max_id)
            else:
                skip_possible = False
            
            
            
            
        
        if clean_up:
            nodes = self.path_planer.graph.nodes()
            to_remove = []
            for node in nodes:
                if not node in collision_free_path and type(node) == type(""):
                    to_remove.append(node)
        
            for node in to_remove:
                self.path_planer.graph.remove_node(node)


        self.smoothed_path = collision_free_path
        return collision_free_path
                
            
                    
                
            
    
    def try_direct_skip(self, start, goal) -> bool:
        collision_intervals = self.config["collision_intervals"]
        
        unit_vector = (goal - start) / collision_intervals
        
        for i in range(0, collision_intervals + 1):
            if self.path_planer._collisionChecker.pointInCollision(start + unit_vector * i):
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

    # def visualize_smoothing(self, environment):
    #     figure = plt.figure(figsize=(7, 7))
    
    #     ax = figure.add_subplot(1, 1, 1)
        
        
    #     workSpaceLimits = environment.robot.getLimits()
                
    #     def animation(frame):
    #         ## clear taks space figure
    #         ax.cla()
    #         ## fix figure size
    #         ax.set_title("Path smoothing BG", fontsize=14)
    #         ax.set_xlim(workSpaceLimits[0])
    #         ax.set_ylim(workSpaceLimits[1])
    #         ## draw obstacles
    #         environment.drawObstacles(ax)
    #         ## update robot position

    #         graph = nx.Graph()
    #         for node in self.path_per_epoche[frame]:
    #             graph.add_node(node, pos = self.path_planer.graph.nodes[node]["pos"])
            
            
    #         for i in range(len(self.path_per_epoche[frame]) - 1):
    #             graph.add_edge(self.path_per_epoche[frame][i], self.path_per_epoche[frame][i + 1])

    #         pos = nx.get_node_attributes(graph,'pos')
    #         # todo extract from pos the first two dimensions only for drawing in workspace
    #         pos2D = dict()
    #         for key in pos.keys():
    #             pos2D[key] = (pos[key][0], pos[key][1])
                
    #         pos = pos2D

    #         nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=100)
    #         nx.draw_networkx_edges(graph,pos, ax = ax)
        
    #     ani = matplotlib.animation.FuncAnimation(figure, animation, frames=len(self.path_per_epoche))
    #     html = HTML(ani.to_jshtml())
    #     display(html)
    #     plt.close()
                    