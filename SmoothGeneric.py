import random
import numpy as np
import networkx as nx
import copy
from SmootherBase import SmootherBase, Angle 
import time

class SmoothGeneric(SmootherBase):
    def __init__(self, planner, path):
        super().__init__()
        self.smoothed_path = path.copy()
        self.path_planner = copy.deepcopy(planner)
        self.id_counter = 0
        self.limits = planner._collisionChecker.getEnvironmentLimits()

    def wrapToPi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def interpAngle(self, theta0, theta1, t):
        # Erstelle Angle-Objekte mit korrekten Grenzen
        angle_0 = Angle(theta0, self.limits[2][0], self.limits[2][1])
        angle_1 = Angle(theta1, self.limits[2][0], self.limits[2][1])
        
        # Kürzeste Differenz (gibt Wert zwischen -π und π)
        d = angle_1.value - angle_0.value        
        # Interpoliere und normalisiere im gültigen Bereich
        interpolated = angle_0.value + t * d
        # print(f"Interpolation: angle_0={angle_0.value}, angle_1={angle_1.value}, t={t}, interpolated={interpolated}", flush=True)

        result = Angle(interpolated, self.limits[2][0], self.limits[2][1])
        return result.value

    def random_pt_on_edge(self, start_node_pt, end_node_pt):
        # random number t [0, 1]
        t = random.random()
        
        # Linear interpolated point
        x = (1 - t) * start_node_pt[0] + t * end_node_pt[0]
        y = (1 - t) * start_node_pt[1] + t * end_node_pt[1]
        if len(start_node_pt) == 3:
            orientation = self.interpAngle(start_node_pt[2], end_node_pt[2], t)
            return (x, y, orientation)
        else:
            return (x, y)

    def findRandomShortcut(self):
        points = []
        node_positions = nx.get_node_attributes(self.path_planner.graph,'pos')
        for i in range(len(self.smoothed_path) - 1):
            u = self.smoothed_path[i]
            v = self.smoothed_path[i+1]
            
            points.append(self.random_pt_on_edge(node_positions[u], node_positions[v]))

        shortcut_collides = True
        while shortcut_collides:
            u = random.randint(0, len(points)-1)
            # make sure second point (v) != first point (u)
            v = random.choice([i for i in range(0,len(points)-1) if i != u])

            if(u > v):
                u, v = v, u
            
            shortcut_collides = self.path_planner._collisionChecker.lineInCollision(points[u], points[v], self.config["collision_intervals"])
            
            if not shortcut_collides:
                self.insertAndConnectPointsOnEdges(points, u, v)
                return True
            else:
                return False 

    def insertAndConnectPointsOnEdges(self, random_edge_pts, u, v):
        id_u = f"S{self.id_counter + 1}"
        id_v = f"S{self.id_counter + 2}"

        # adjust solution
        if v == u + 1:
            # direkt benachbart: Wert an u überschreiben, S2 danach einfügen,
            # Rest bleibt unverändert
            self.smoothed_path[u+1:u+2] = [id_u, id_v]
        else:
            # sonst: Bereich u..v (inkl.) durch [S1,S2] ersetzen
            self.smoothed_path[u+1:v+1] = [id_u, id_v]

        self.path_planner.graph.add_node(id_u, pos=random_edge_pts[u], color="#e28a0e")
        self.path_planner.graph.add_node(id_v, pos=random_edge_pts[v], color="#e28a0e")
        # print("pos S1: ", random_edge_pts[u], " pos S2: ", random_edge_pts[v], flush=True)
        
        self.path_planner.graph.add_edge(self.smoothed_path[u], id_u)
        self.path_planner.graph.add_edge(id_u, id_v)
        
        if v == u+1:
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[v+2])
        else:
            # find index of second shortcut point in solution list
            i_in_solution = self.smoothed_path.index(id_v)
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[i_in_solution+1])

        self.id_counter += 2

        return True
    def smooth_path(self, config):
        self.config:dict = config
        start_time = time.time()
        for i in range(self.config["epoches"]):
            a = True
            while a:
                if self.findRandomShortcut():
                    self.path_per_epoche.append(list(self.smoothed_path))
                    break
        self.smoothing_time = time.time() - start_time
        return self.smoothed_path, self.path_planner.graph