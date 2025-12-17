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

class SmootherBase():
    def __init__(self):
        self.smoothed_path = []
        self.path_planer = None
        
        self.config = {}
        self.path_per_epoche = []
    
    def visualize_smoothing(self, environment):
            figure = plt.figure(figsize=(7, 7))
        
            ax = figure.add_subplot(1, 1, 1)
            
            
            workSpaceLimits = environment.robot.getLimits()
                    
            def animation(frame):
                ## clear taks space figure
                ax.cla()
                ## fix figure size
                ax.set_title("Path smoothing BG", fontsize=14)
                ax.set_xlim(workSpaceLimits[0])
                ax.set_ylim(workSpaceLimits[1])
                ## draw obstacles
                environment.drawObstacles(ax)
                ## update robot position

                graph = nx.Graph()
                for node in self.path_per_epoche[frame]:
                    graph.add_node(node, pos = self.path_planer.graph.nodes[node]["pos"])
                
                
                for i in range(len(self.path_per_epoche[frame]) - 1):
                    graph.add_edge(self.path_per_epoche[frame][i], self.path_per_epoche[frame][i + 1])

                pos = nx.get_node_attributes(graph,'pos')
                # todo extract from pos the first two dimensions only for drawing in workspace
                pos2D = dict()
                for key in pos.keys():
                    pos2D[key] = (pos[key][0], pos[key][1])
                    
                pos = pos2D

                nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=100)
                nx.draw_networkx_edges(graph,pos, ax = ax)
            
            ani = matplotlib.animation.FuncAnimation(figure, animation, frames=len(self.path_per_epoche))
            html = HTML(ani.to_jshtml())
            display(html)
            plt.close()
            
            
    def animate_path(self, environment, origin_path, title = "Smoothed animation"):
        if origin_path == []:
            return
        
        fig_local = plt.figure(figsize=(7, 7))
        ax_origin = fig_local.add_subplot(1, 1, 1)
        
        
        
        def frame_generator(path):
            ## get positions for solution
            solution_pos = [self.path_planer.graph.nodes[node]['pos'] for node in path]
            ## interpolate to obtain a smoother movement
            i_solution_pos = [solution_pos[0]]
            for i in range(1, len(solution_pos)):
                segment_s = solution_pos[i-1]
                segment_e = solution_pos[i]
                i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, 0.5)[1:]
            ## animate
            frames = len(i_solution_pos)

            r = environment.robot
            workSpaceLimits = environment.robot.getLimits()
            
            return i_solution_pos, workSpaceLimits,frames, r
        
        origin_pos, origin_limits, oringin_frames, robot = frame_generator(origin_path)
        smoothed_pos, smoothed_limits, smoothed_frames, robot = frame_generator(self.smoothed_path)
        

        def animation(frame, ax, title, positions, robot, limits, path):
            ## clear taks space figure
            ax.cla()
            ## fix figure size
            ax.set_title(title, fontsize=14)
            
            ax.set_xlim(limits[0])
            ax.set_ylim(limits[1])
            ## draw obstacles
            #environment.drawObstacles(ax)
            ## update robot position
            pos = positions[frame]
            robot.setTo(pos)
            visibilityPRMVisualizeWspace(self.path_planer,path, ax=ax)


        def animate_origin(frame):
            animation(frame, ax_origin, "original_path", origin_pos, robot, origin_limits, origin_path)
            
            

        ani = matplotlib.animation.FuncAnimation(fig_local, animate_origin, frames=oringin_frames)
        html = HTML(ani.to_jshtml())
        display(html)
        
        plt.close()
        
        fig_local2 = plt.figure(figsize=(7, 7))
        ax_smoothed = fig_local2.add_subplot(1, 1, 1)
        def animate_smoothed(frame):
            animation(frame, ax_smoothed, title, smoothed_pos, robot, smoothed_limits, self.smoothed_path)
        
        ani = matplotlib.animation.FuncAnimation(fig_local2, animate_smoothed, frames=smoothed_frames)
        html = HTML(ani.to_jshtml())
        display(html)
        
        plt.close()