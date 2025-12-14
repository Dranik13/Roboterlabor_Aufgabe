import random
import numpy as np

def wrapToPi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def interpAngle(theta0, theta1, t):
    d = wrapToPi(theta1 - theta0)      # kürzeste Differenz
    return wrapToPi(theta0 + t * d)  

def random_pt_on_edge(start_knot_pt, end_knot_pt):
    # random number t [0, 1]
    t = random.random()
    
    # Linear interpolated point
    x = (1 - t) * start_knot_pt[0] + t * end_knot_pt[0]
    y = (1 - t) * start_knot_pt[1] + t * end_knot_pt[1]
    if len(start_knot_pt) == 3:
        orientation = interpAngle(start_knot_pt[2], end_knot_pt[2], t)
        return (x, y, orientation)
    else:
        return (x, y)

def findRandomShortcut(planner, points):
    
    if len(points) < 2:
        return None
    
    shortcut_collides = True
    while shortcut_collides:
        u = random.randint(0, len(points)-1)
        # make sure second point (v) != first point (u)
        v = random.choice([i for i in range(0,len(points)-1) if i != u])

        if(u > v):
            u, v = v, u

        shortcut_collides = planner._collisionChecker.lineInCollision(points[u], points[v])
        
        if not shortcut_collides:
            return u, v

def insertAndConnectPointsOnEdges(G, solution, random_edge_pts, u, v, id_counter, color):
    
    id_u = f"S{id_counter + 1}"
    id_v = f"S{id_counter + 2}"

    # adjust solution
    if v == u + 1:
        # direkt benachbart: Wert an u überschreiben, S2 danach einfügen,
        # Rest bleibt unverändert
        solution[u+1:u+2] = [id_u, id_v]
    else:
        # sonst: Bereich u..v (inkl.) durch [S1,S2] ersetzen
        solution[u+1:v+1] = [id_u, id_v]

    G.add_node(id_u, pos=random_edge_pts[u], color=color)
    G.add_node(id_v, pos=random_edge_pts[v], color=color)
    # print("pos S1: ", random_edge_pts[u], " pos S2: ", random_edge_pts[v], flush=True)
    
    G.add_edge(solution[u], id_u)
    G.add_edge(id_u, id_v)
    
    if v == u+1:
        G.add_edge(id_v, solution[v+2])
    else:
        # find index of second shortcut point in solution list
        i_in_solution = solution.index(id_v)
        G.add_edge(id_v, solution[i_in_solution+1])

    id_counter += 2

    return True