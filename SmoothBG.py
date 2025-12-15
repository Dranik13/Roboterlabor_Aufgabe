import numpy as np 

class SmoothBG():
    def __init__(self):
        pass
    
    def smooth_path(self, path, planer, config, clean_up = False):
        '''        
        :param path: Collison free path of path planer
        :param planer: Used path planer algorithem for access to the graph and collisionchecker
        :param config: Configuration of smoother
        :return: smoothed path 
        '''
        
        if path == []:
            return []
        
        collision_free_path:list = list(path)
        self.path_planer = planer
        self.config:dict = config
        
        corner_threshold:float = self.config["corner_threshold"]
        max_new_node_in_cascade = self.config["max_new_nodes"]
        
        '''
        Run the skip process. Starting with first node till the last one
        '''
        id = 0
        self.added_nodes = 0
        new_added_nodes = 0
      
        start_node_name = collision_free_path[id]
        while start_node_name != collision_free_path[-1] and not id + 2 >= len(collision_free_path):
            goal_node_name = collision_free_path[id + 2]
            skip_node_name = collision_free_path[id + 1]
            
            start_node = np.array(self.path_planer.graph.nodes[start_node_name]["pos"])
            skip_node = np.array(self.path_planer.graph.nodes[skip_node_name]["pos"])
            goal_node = np.array(self.path_planer.graph.nodes[goal_node_name]["pos"])
            
            
            # check id it's a edge which is worth to get skipped
            unshorted_length = np.linalg.norm(skip_node-start_node) + np.linalg.norm(goal_node - skip_node)
            shorted_length = np.linalg.norm(goal_node - start_node)
            
            if unshorted_length / shorted_length >= corner_threshold:

                if self.try_direct_skip(start_node, goal_node):
                    collision_free_path.remove(skip_node_name)
                else:                    
                    success, new_start, new_goal = self.try_deltree_skip(start_node, goal_node, skip_node)
                    if success:
                        collision_free_path.insert(id + 1, "new_start_node_" + str(self.added_nodes))
                        collision_free_path.insert(id + 2, "new_goal_node_" + str(self.added_nodes))
                        # add to graph
                        self.path_planer.graph.add_node("new_start_node_" + str(self.added_nodes), pos = list(new_start))
                        self.path_planer.graph.add_node("new_goal_node_" + str(self.added_nodes), pos = list(new_goal))
                        
                        self.path_planer.graph.add_edge(start_node_name, "new_start_node_" + str(self.added_nodes))
                        self.path_planer.graph.add_edge(goal_node_name, "new_goal_node_" + str(self.added_nodes))
                        
                        # remove skipped node
                        collision_free_path.remove(skip_node_name)

                        
                        
                        self.added_nodes += 1  
            
            id += 1
            node_name_cache:str = collision_free_path[id]
            
            if type(node_name_cache) == type(""):
                # Check deltree added nodes. Limitting amount of new nodes per deltree cascade for reducing hypertuning of path and time consumption
                if node_name_cache.find("new_start_node_") != -1 and start_node_name.find("new_start_node_") != -1:
                    new_added_nodes += 1
                    # skip next node in path (what is in that case a "new_start_node") and jump directly to the "new_goal_node"
                    if new_added_nodes == max_new_node_in_cascade:
                        id += 1
                        
                    start_node_name = node_name_cache
                    
                else:
                    new_added_nodes = 0
                    start_node_name = collision_free_path[id]
            
            
        
        if clean_up:
            nodes = self.path_planer.graph.nodes()
            for node in nodes:
                if not node in collision_free_path:
                    self.path_planer.graph.remove_node(node)
        
        return collision_free_path
                
            
                    
                
            
    
    def try_direct_skip(self, start, goal) -> bool:
        collision_intervals = self.config["collision_intervals"]
        
        unit_vector = (goal - start) / np.linalg.norm((goal - start))
        
        for i in range(1, collision_intervals + 1):
            if self.path_planer._collisionChecker.pointInCollision(start + unit_vector * i):
                return False
        
        return True
    
    def try_deltree_skip(self, start, goal, skip_node):
        max_deltree_depth:int = self.config["max_deltree_depth"]
        min_deltree_delta:float = self.config["min_deltree_delta"]
        
        '''
        From step to step the new_start and new_goal node are getting closer to the skipping node
        '''
        for k in range(1, max_deltree_depth + 1):
            new_start_node = start + (skip_node - start) / 2**k
            new_goal_node = skip_node + (goal - skip_node) / 2**k
            
            delta_1 = np.linalg.norm(skip_node - new_start_node)
            delta_2 = np.linalg.norm(new_goal_node - skip_node)
            
            if delta_1 <= min_deltree_delta or delta_2 <= min_deltree_delta:
                return False, None, None
            
            if self.try_direct_skip(new_start_node, new_goal_node):
                return True, new_start_node, new_goal_node
            
            start = new_start_node
            goal = new_goal_node
