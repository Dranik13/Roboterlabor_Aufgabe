'''
The Python script was created as part of the examination for the lecture "Robot Programming" at the University of Applied Sciences Karlsruhe. It serves as a basis for all path smoothing algorithms for the paths determined by the various PRM planners.

Authors: Benjamin Dilly, Dennis MCNab, and Anton Kisel
Date: 01/21/2026

Responsible Professor: Prof. Dr.-Ing. Björn Hein
'''

import numpy as np 
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation

from IPython.display import HTML, display

from IPVISLazyPRM import visibilityPRMVisualizeWspace

import copy

def interpolate_line(startPos, endPos, step_l):
    steps = []
    line = np.array(endPos) - np.array(startPos)
    line_l = np.linalg.norm(line)
    step = line / line_l * step_l
    n_steps = np.floor(line_l / step_l).astype(np.int32)
    c_step = np.array(startPos, dtype=np.float32)
    for i in range(n_steps):
        steps.append(copy.deepcopy(c_step))
        c_step += step
    if not (c_step == np.array(endPos)).all():
        steps.append(np.array(endPos))
    return steps

import math


class Angle:
    '''
    This class is used for the correct mathematical handling of angle calculations.  
    It ensures that the angle difference always returns the shortest rotation angle and that the robot's limitations are taken into account.  
    This guarantees that the robot always turns in the correct direction in later applications.  

    Example: A shaping robot is in the following position at 330°. It should now be checked whether smoothing to a node with an orientation of 15° is possible. With a simple angle difference (15 - 330), the robot would rotate clockwise by 315° to move to the new node. However, this is not collision-free given the current environment.  
    If it were to rotate counterclockwise by 50°, i.e., the shorter angle distance, collision-free smoothing would be possible.
    '''
    def __init__(self, value: float, lower: float = -math.pi, upper: float = math.pi):
        '''
        Docstring for __init__
        
        :param value: initial angle value
        :type value: float
        :param lower: lower angle boundary of movement
        :type lower: float
        :param upper: upper angle boundary of movement
        :type upper: float
        '''
        self.lower = lower
        self.upper = upper
        # zyklisch, wenn der Bereich genau 2π groß ist
        self.is_cyclic = math.isclose(abs(upper - lower), 2*math.pi)
        self.value = self._normalize(value)

    def _normalize(self, val: float) -> float:
        """
        Normalizes the angle to the range [lower, upper] or modulo 2π for cyclic boundaries.
        """
        if self.is_cyclic:
            # modulo 2π normalisieren
            return ((val - self.lower) % (2*math.pi)) + self.lower
        else:
            # normale Begrenzung
            while val < self.lower:
                val += 2*math.pi
            while val > self.upper:
                val -= 2*math.pi
            return val

    def __float__(self):
        return float(self.value)

    def __repr__(self):
        return f"Angle({self.value:.3f} rad, bounds=[{self.lower:.3f}, {self.upper:.3f}], cyclic={self.is_cyclic})"

    # Operatoren
    def __add__(self, other):
        return self._operate(other, "+")

    def __sub__(self, other):
        return self._operate(other, "-")

    def __mul__(self, other):
        return self._operate(other, "*")

    def __truediv__(self, other):
        return self._operate(other, "/")

    def _operate(self, other, op: str):
        if isinstance(other, Angle):
            other_val = other.value
        else:
            other_val = float(other)

        if op == "+":
            new_val = self.value + other_val
        elif op == "-":
            # Kürzeste Winkeldifferenz in Radiant
            diff = (self.value - other_val + math.pi) % (2*math.pi) - math.pi
            new_val = self.value - diff
            # Falls nicht zyklisch und außerhalb → längere Differenz nehmen
            if not self.is_cyclic and not (self.lower <= new_val <= self.upper):
                new_val = self.value - (diff - 2*math.pi if diff > 0 else diff + 2*math.pi)
        elif op == "*":
            new_val = self.value * other_val
        elif op == "/":
            new_val = self.value / other_val
        else:
            raise ValueError("Unbekannter Operator")

        return Angle(new_val, self.lower, self.upper)
    



class SmootherBase():
    '''
    The base class for all smoothing methods. It implements the basic parameters and the animation functions for visualizing the path smoothing and for comparing the robot's movement on the base path and on the smoothed path.

    :param smoothed_path: The smoothed base path
    :param path_planner: The planner used to generate the unpolished path
    :param config: The configuration dictionary for each path smoother
    :param path_per_epoche: The smoothed path at each smoothing iteration. Used for the smoothing animation
    :param smoothing_time: The time the smoother required
    '''
    
    def __init__(self):
        self.smoothed_path = []
        self.path_planner = None
        
        self.config = {}
        self.path_per_epoche = []

        self.smoothing_time = 0
    
    def visualize_smoothing(self, seconds_per_frame=0.1, title="Path smoothing BG"):
        '''
        This functions animates the smoothed path while each iteration of the smoothing process
        '''
        if self.smoothed_path == []:
            print("no Smoothed Path",flush=True)
            return 
        
        figure = plt.figure(figsize=(7, 7))
        ax = figure.add_subplot(1, 1, 1)


        # Get all neccessarily informations
        environment = self.path_planner._collisionChecker
        workSpaceLimits = environment.robot.getLimits()
           
        # Reset the robot to zero position for zero frame
        environment.robot.setTo(self.path_planner.graph.nodes[self.smoothed_path[0]]["pos"])

        # Draws the frame          
        def animation(frame):
            # Setting up of current frame axes
            ax.cla()
            ax.set_title(title, fontsize=14)
            ax.set_xlim(workSpaceLimits[0])
            ax.set_ylim(workSpaceLimits[1])

            ## Draw environement
            environment.drawObstacles(ax)
            
            # Creating of the smoothed graphs nodes at the current epoche/frame
            graph = nx.Graph()
            for node in self.path_per_epoche[frame]:
                graph.add_node(node, pos = self.path_planner.graph.nodes[node]["pos"])
            
            # Creating of edges
            for i in range(len(self.path_per_epoche[frame]) - 1):
                graph.add_edge(self.path_per_epoche[frame][i], self.path_per_epoche[frame][i + 1])

            # Remapping the multidimensional nodes to an two dimensional space for drawing
            pos = nx.get_node_attributes(graph,'pos')
            pos2D = dict()
            for key in pos.keys():
                pos2D[key] = (pos[key][0], pos[key][1])
            
            # Updating of Graph's node positions
            pos = pos2D

            # Final drawing
            nx.draw_networkx_nodes(graph, pos, ax = ax, node_size=100)
            nx.draw_networkx_edges(graph,pos, ax = ax)
        
        # Setting up the animation
        interval_ms = int(round(seconds_per_frame * 1000.0))
        ani = matplotlib.animation.FuncAnimation(figure, animation, frames=len(self.path_per_epoche), interval=interval_ms)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()
        
    
            
            
    def animate_path(self, origin_path, title="Smoothed animation", maintitle = "?",
                    seconds_total=5.0, fps=15):
        '''
        This funtion creates the animation which compares the movement of the robot while using the unpolished path and the 
        smoothed path.
        
        :param origin_path: The unpolished path
        :param seconds_total: Total playtime of the animation
        :param fps: Frames per second of the animation
        '''

        '''''''''''''''''

        Functions starting here
        
        '''''''''''''''''


        def cleaning_of_graphs():
            '''
            Removing of all nodes which aren't part of the final paths
            '''
            to_remove = []
            for node in origin_planer.graph.nodes():
                if node not in origin_path:
                    to_remove.append(node)
            
            origin_planer.graph.remove_nodes_from(to_remove)
                
            to_remove = []  
            for node in smoothed_planer.graph.nodes():
                if node not in self.smoothed_path:
                    to_remove.append(node)
            
            smoothed_planer.graph.remove_nodes_from(to_remove)
            
            # adding edges (filling broken connections)
            for i in range(1, len(self.smoothed_path)):
                smoothed_planer.graph.add_edge(self.smoothed_path[i - 1], self.smoothed_path[i])
        
        # Interpoliert entlang des Pfades nach Bogenlänge auf genau target_frames Punkte
        def resample_by_arclength(positions, target_frames):
            '''
            This function calculates the amount of interpolation steps per path edge.
            The total amount of interpolations (target_frames) is asigned to each edge through it's 
            proportion of the total path length. \n
            The return is the interpolated list of all interpolated positions of the robot to pass by.
            
            :param positions: The list of the position of all nodes which are included in the current path
            :param target_frames: The total amount of frames to render
            '''
            if target_frames <= 0:
                return []
            if len(positions) == 0:
                return []
            
            pos_arr = np.asarray(positions, dtype=float)
            
            # If only one pose is available -> repeat for it for all frames 
            if pos_arr.shape[0] == 1:
                return [positions[0] for _ in range(target_frames)]

            # Calculate the segment (edge) lengths and the total path length
            diffs = pos_arr[1:] - pos_arr[:-1]
            seg_lengths = np.linalg.norm(diffs, axis=1)
            total_length = seg_lengths.sum()

            # If total length is zero -> all positions equal -> repeat start pose for all frames
            if total_length == 0:
                return [positions[0] for _ in range(target_frames)]

            '''
            Interpolation
            '''
            cum = np.concatenate(([0.0], np.cumsum(seg_lengths)))
            cum_norm = cum / cum[-1]  # Länge len = number_of_nodes

            new_t = np.linspace(0.0, 1.0, target_frames)

            # find the corrsponding segment for each t and interpolate the segment
            resampled = []
            for t in new_t:
                if t >= 1.0:
                    resampled.append(pos_arr[-1].tolist())
                    continue
                i = np.searchsorted(cum_norm, t, side='right') - 1
                if i < 0:
                    i = 0
                if i >= len(pos_arr) - 1:
                    resampled.append(pos_arr[-1].tolist())
                    continue
                t0 = cum_norm[i]
                t1 = cum_norm[i + 1]

                local_alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                p = (1.0 - local_alpha) * pos_arr[i] + local_alpha * pos_arr[i + 1]
                resampled.append(p.tolist())

            # Set the start and end position
            resampled[0] = positions[0]
            resampled[-1] = positions[-1]
            return resampled

        def path_positions_from_nodes(path, planer):
            '''Extracts the nodes positions from a given path'''
            return [planer.graph.nodes[node]['pos'] for node in path]

       

        def animation(frame, ax_left, ax_right, origin_positions, smoothed_positions,
                    origin_limits, smoothed_limits, origin_path, smoothed_path):
            '''
            This function draws the final subplots for the comparision frame
            '''
            # left Figur: original
            ax_left.cla()
            ax_left.set_title("Original path", fontsize=14)
            ax_left.set_xlim(origin_limits[0])
            ax_left.set_ylim(origin_limits[1])
            ax_left.set_aspect('equal', adjustable='box')
            ax_left.grid(True)
            
            pos_o = origin_positions[frame]
            origin_robot.setTo(pos_o)
            self.simple_draw(origin_planer, ax=ax_left)

            # right Figur: smoothed
            ax_right.cla()
            ax_right.set_title(title, fontsize=14)
            ax_right.set_xlim(smoothed_limits[0])
            ax_right.set_ylim(smoothed_limits[1])
            ax_right.set_aspect('equal', adjustable='box')
            ax_right.grid(True)
    
            pos_s = smoothed_positions[frame]
            smoothed_robot.setTo(pos_s)
            self.simple_draw(smoothed_planer, ax=ax_right)



        # Animation wrapper function
        def animate_both(frame):
            animation(frame, ax_origin, ax_smoothed,
                    origin_pos, smoothed_pos,
                    origin_limits, smoothed_limits,
                    origin_path, self.smoothed_path)
            
        
        '''''''''

        Code Starts here

        '''''''''

        if not origin_path:
            return
        
        origin_planer = copy.deepcopy(self.path_planner)
        smoothed_planer = copy.deepcopy(self.path_planner)
        
        cleaning_of_graphs()
        
        # Calculate total amount of frames
        total_frames = max(1, int(round(seconds_total * fps)))

        # Extract node positions
        origin_raw = path_positions_from_nodes(origin_path, origin_planer)
        smoothed_raw = path_positions_from_nodes(self.smoothed_path, smoothed_planer)

        # Create list of interpolated robot poses
        origin_pos = resample_by_arclength(origin_raw, total_frames)
        smoothed_pos = resample_by_arclength(smoothed_raw, total_frames)

        # Extract environement metadata
        origin_limits = origin_planer._collisionChecker.robot.getLimits()
        smoothed_limits = smoothed_planer._collisionChecker.robot.getLimits()

        # Extract the robot references
        origin_robot = origin_planer._collisionChecker.robot
        smoothed_robot = smoothed_planer._collisionChecker.robot

        interval_ms = int(round(1000.0 / fps))

        # Figure
        fig_local = plt.figure(figsize=(14, 7))
        fig_local.suptitle(f"{maintitle}")
        ax_origin = fig_local.add_subplot(1, 2, 1)
        ax_smoothed = fig_local.add_subplot(1, 2, 2)

        # Run the animation
        ani = matplotlib.animation.FuncAnimation(fig_local, animate_both, frames=total_frames, interval=interval_ms)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()

        

    def simple_draw(self, planer, ax):
        '''
        Draws the current environement included the robot and graph
        '''
        planer._collisionChecker.drawObstacles(ax)
        
        graph = planer.graph
        pos = nx.get_node_attributes(graph,'pos')
        # todo extract from pos the first two dimensions only for drawing in workspace
        pos2D = dict()
        for key in pos.keys():
            pos2D[key] = (pos[key][0], pos[key][1])
            
        pos = pos2D

        nx.draw_networkx_nodes(graph, pos, ax = ax, node_size=100)
        nx.draw_networkx_edges(graph,pos, ax = ax)