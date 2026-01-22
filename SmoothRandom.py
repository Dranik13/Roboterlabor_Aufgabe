import random
import numpy as np
import networkx as nx
import copy
from SmootherBase import SmootherBase, Angle 
import time

class SmoothRandom(SmootherBase):
    def __init__(self, planner, path):
        super().__init__()
        self.smoothed_path = path.copy()
        self.path_planner = copy.deepcopy(planner)
        self.id_counter = 0
        self.limits = planner._collisionChecker.getEnvironmentLimits()

    def smooth_path(self, config):
        self.config:dict = config
        start_time = time.time()
        for i in range(self.config["epoches"]):
            # a könnte (und sollte) man durch eine maximale Anz. an Versuchen austauschen
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
        
        # Platziere auf jeder Kante einen zufälligen Punkt.
        # Die Idee dahinter war, direkt pro Kante einen Punkt zu erzeugen, um
        # Randbedingungen formulieren zu können. Beispielsweise könnte
        # die euklidische Distanz zwischen aufeinanderfolgenden Punkten geprüft
        # werden. Ist diese zu klein, kann der nächste Punkt gewählt werden. 
        # Es könnten auch mehrere Knoten in einer Iteration geprüft werden
        for i in range(len(self.smoothed_path) - 1):
            node_index1 = self.smoothed_path[i]
            node_index2 = self.smoothed_path[i+1]
            points.append(self.randomPtOnEdge(node_positions[node_index1], node_positions[node_index2]))

        shortcut_collides = True
        tries = 0
        # Wähle zwei Punkte und prüfe ob sie sich kollisionsfrei verbinden lassen
        # Versuche max. 20 mal zwischen den gegebenen Punkten eine Abkürzung zu finden 
        while shortcut_collides:
            u = random.randint(0, len(points)-1)
            # Stelle sicher, dass der zweite Punkt (v) != erster Punkt (u)
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
        
        # Linear interpolierter Punkt
        x = (1 - t) * start_node_pt[0] + t * end_node_pt[0]
        y = (1 - t) * start_node_pt[1] + t * end_node_pt[1]
        if len(start_node_pt) == 3:
            orientation = self.interpAngle(start_node_pt[2], end_node_pt[2], t)
            return (x, y, orientation)
        else:
            return (x, y)

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

    def insertAndConnectPointsOnEdges(self, random_edge_pts, u, v):
        # Bezeichnung neuer Knoten: S1, S2, S3...
        id_u = f"S{self.id_counter + 1}"
        id_v = f"S{self.id_counter + 2}"

        # Lösungspfad anpassen
        if v == u + 1:
            # direkt benachbart: Wert an u überschreiben, S2 danach einfügen,
            # Rest bleibt unverändert
            self.smoothed_path[u+1:u+2] = [id_u, id_v]
        else:
            # sonst: Bereich u..v durch [S1,S2] ersetzen
            self.smoothed_path[u+1:v+1] = [id_u, id_v]

        self.path_planner.graph.add_node(id_u, pos=random_edge_pts[u], color="#e28a0e")
        self.path_planner.graph.add_node(id_v, pos=random_edge_pts[v], color="#e28a0e")
        
        self.path_planner.graph.add_edge(self.smoothed_path[u], id_u)
        self.path_planner.graph.add_edge(id_u, id_v)
        
        if v == u+1:
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[v+2])
        else:
            # Finde den Index des zweiten Abkürzungsknotens in der Lösungsliste
            i_in_solution = self.smoothed_path.index(id_v)
            self.path_planner.graph.add_edge(id_v, self.smoothed_path[i_in_solution+1])

        self.id_counter += 2

        return True