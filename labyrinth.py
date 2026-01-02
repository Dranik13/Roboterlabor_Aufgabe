from shapely.geometry import Polygon
import shapely



def generate(walk_way = 1.5, x_start = 3, y_start = 1, length = 11, height = 7, thickness = 0.3):
    point_list = [(x_start, y_start)]
    
    
    def to_right(start_point, first, last_right=(), last_top =(), last_left=(), last_down=()):
        x = start_point[0]
        y = start_point[1]
        
        if last_top == ():
            new_x = x + length
        else:
            
            new_x = last_top[0]  - (walk_way + thickness)
            if new_x - x < walk_way + thickness:
                point_list.append((x + thickness, y))
                return
        
        point_list.append((new_x,y))
        to_top(point_list[-1], last_right=point_list[-1], last_top=last_top, last_left=last_left, last_down=last_down)
        
        # Recursion
        if not first:
            new_x = x + thickness
            point_list.append((new_x, point_list[-1][1]))
        else:
            point_list.append((x, point_list[-1][1]))
            
        
    def to_top(start_point, last_right=(), last_top =(), last_left=(), last_down=()):
        x = start_point[0]
        y = start_point[1]
        
        if last_left == ():
            new_y = y + height
        else:
            new_y = last_left[1]  - (walk_way + thickness)
            if new_y - y < walk_way + thickness:
                point_list.append((x, y + thickness))
                return
        
        point_list.append((x,new_y))
        to_left(point_list[-1], last_top=point_list[-1], last_down=last_down, last_left=last_left, last_right=last_right)
        
        # Recursion
        new_y = y + thickness
        point_list.append((point_list[-1][0], new_y))
        
    def to_left(start_point, last_right=(), last_top =(), last_left=(), last_down=()):
        x = start_point[0]
        y = start_point[1]
        
        if last_down == ():
            new_x = x - length
        else:
            new_x = (last_down[0] + (walk_way + thickness))
            if x - new_x < walk_way + thickness:
                point_list.append((x - thickness, y))
                return
        
        point_list.append((new_x,y))
        to_down(point_list[-1], last_left=point_list[-1], last_down=last_down, last_right=last_right, last_top=last_top)
        
        # Recursion
        new_x = x - thickness
        point_list.append((new_x, point_list[-1][1]))

    def to_down(start_point, last_right=(), last_top =(), last_left=(), last_down=()):
        x = start_point[0]
        y = start_point[1]
        
        if last_right == ():
            new_y = y - (height - (walk_way + thickness))
        else:
            new_y = (last_right[1] + (walk_way + thickness))
            if y - new_y < walk_way + thickness:
                point_list.append((x, y - thickness))
                return
        
        point_list.append((x,new_y))
        to_right(point_list[-1], False, last_down=point_list[-1], last_left=last_left, last_right=last_right, last_top=last_top)
        
        # Recursion
        new_y = y - thickness
        point_list.append((point_list[-1][0], new_y))
        
    
    to_right(point_list[-1], True, last_right=point_list[-1])
    
    return Polygon(point_list)


print(generate())