import random

def random_pt_on_edge(start_knot_pt, end_knot_pt):
    # random number t [0, 1]
    t = random.random()
    
    # Linear interpolated point
    x = (1 - t) * start_knot_pt[0] + t * end_knot_pt[0]
    y = (1 - t) * start_knot_pt[1] + t * end_knot_pt[1]
    
    return (x, y)
