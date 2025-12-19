import random
import numpy as np
import networkx as nx

class smoothing():
    def __init__(self, Graph, path, CollisionChecker):
        self.path = path.copy()
        self.graph = Graph.copy()
        self.collisionchecker = CollisionChecker
        self.id_counter = 0

    def wrapToPi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def interpAngle(self, theta0, theta1, t):
        d = self.wrapToPi(theta1 - theta0)      # kürzeste Differenz
        return self.wrapToPi(theta0 + t * d)  

    def random_pt_on_edge(self, start_knot_pt, end_knot_pt):
        # random number t [0, 1]
        t = random.random()
        
        # Linear interpolated point
        x = (1 - t) * start_knot_pt[0] + t * end_knot_pt[0]
        y = (1 - t) * start_knot_pt[1] + t * end_knot_pt[1]
        if len(start_knot_pt) == 3:
            orientation = self.interpAngle(start_knot_pt[2], end_knot_pt[2], t)
            return (x, y, orientation)
        else:
            return (x, y)

    def findRandomShortcut(self):
        points = []
        knot_positions = nx.get_node_attributes(self.graph,'pos')
        for i in range(len(self.path) - 1):
            u = self.path[i]
            v = self.path[i+1]
            
            points.append(self.random_pt_on_edge(knot_positions[u], knot_positions[v]))

        shortcut_collides = True
        while shortcut_collides:
            u = random.randint(0, len(points)-1)
            # make sure second point (v) != first point (u)
            v = random.choice([i for i in range(0,len(points)-1) if i != u])

            if(u > v):
                u, v = v, u
            
            shortcut_collides = self.collisionchecker.lineInCollision(points[u], points[v])
            
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
            self.path[u+1:u+2] = [id_u, id_v]
        else:
            # sonst: Bereich u..v (inkl.) durch [S1,S2] ersetzen
            self.path[u+1:v+1] = [id_u, id_v]

        self.graph.add_node(id_u, pos=random_edge_pts[u], color="#e28a0e")
        self.graph.add_node(id_v, pos=random_edge_pts[v], color="#e28a0e")
        # print("pos S1: ", random_edge_pts[u], " pos S2: ", random_edge_pts[v], flush=True)
        
        self.graph.add_edge(self.path[u], id_u)
        self.graph.add_edge(id_u, id_v)
        
        if v == u+1:
            self.graph.add_edge(id_v, self.path[v+2])
        else:
            # find index of second shortcut point in solution list
            i_in_solution = self.path.index(id_v)
            self.graph.add_edge(id_v, self.path[i_in_solution+1])

        self.id_counter += 2

        return True
    def smooth_path(self):
        for i in range(10):
            print(i)
            a = True
            while a:
                if self.findRandomShortcut():
                    break

        return self.path, self.graph