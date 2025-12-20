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
    def __init__(self, value: float, lower: float = -math.pi, upper: float = math.pi):
        self.lower = lower
        self.upper = upper
        # zyklisch, wenn der Bereich genau 2π groß ist
        self.is_cyclic = math.isclose(abs(upper - lower), 2*math.pi)
        self.value = self._normalize(value)

    def _normalize(self, val: float) -> float:
        """Normalisiert den Winkel in den Bereich [lower, upper] oder modulo 2π bei zyklischen Grenzen."""
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
    def __init__(self):
        self.smoothed_path = []
        self.path_planner = None
        
        self.config = {}
        self.path_per_epoche = []
    
    def visualize_smoothing(self, seconds_per_frame=0.1, title="Path smoothing BG"):
        if self.smoothed_path == []:
            print("no Smoothed Path",flush=True)
            return 
        figure = plt.figure(figsize=(7, 7))

        ax = figure.add_subplot(1, 1, 1)

        environment = self.path_planner._collisionChecker
        workSpaceLimits = environment.robot.getLimits()
                   
        environment.robot.setTo(self.path_planner.graph.nodes[self.smoothed_path[0]]["pos"])
                    
        def animation(frame):
            ## clear taks space figure
            ax.cla()
            ## fix figure size
            ax.set_title(title, fontsize=14)
            ax.set_xlim(workSpaceLimits[0])
            ax.set_ylim(workSpaceLimits[1])
            ## draw obstacles
            environment.drawObstacles(ax)
            ## update robot position

            graph = nx.Graph()
            for node in self.path_per_epoche[frame]:
                graph.add_node(node, pos = self.path_planner.graph.nodes[node]["pos"])
            
            
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
        
        interval_ms = int(round(seconds_per_frame * 1000.0))
        ani = matplotlib.animation.FuncAnimation(figure, animation, frames=len(self.path_per_epoche), interval=interval_ms)
        html = HTML(ani.to_jshtml())
        display(html)
    
            
            
    def animate_path(self, origin_path, title="Smoothed animation",
                    seconds_total=5.0, fps=15):
        if not origin_path:
            return
        
        origin_planer = copy.deepcopy(self.path_planner)
        smoothed_planer = copy.deepcopy(self.path_planner)
        
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
            # positions: Liste von Posen (iterable von numerischen Arrays/lists)
            if target_frames <= 0:
                return []
            if len(positions) == 0:
                return []
            pos_arr = np.asarray(positions, dtype=float)
            # Falls nur eine Pose vorhanden -> wiederhole sie
            if pos_arr.shape[0] == 1:
                return [positions[0] for _ in range(target_frames)]

            # Berechne Segmentlängen (euclidische Distanz in Pose-Raum)
            diffs = pos_arr[1:] - pos_arr[:-1]
            seg_lengths = np.linalg.norm(diffs, axis=1)
            total_length = seg_lengths.sum()
            # Falls Gesamtlaenge 0 (alle Posen identisch) -> wiederhole Startpose
            if total_length == 0:
                return [positions[0] for _ in range(target_frames)]

            # Kumulative Normierte Längen (0..1)
            cum = np.concatenate(([0.0], np.cumsum(seg_lengths)))
            cum_norm = cum / cum[-1]  # Länge len = number_of_nodes

            # Neue Parameterwerte gleichmäßig über 0..1
            new_t = np.linspace(0.0, 1.0, target_frames)

            # Für jeden neuen t: finde Segment und interpoliere linear
            resampled = []
            for t in new_t:
                # falls t == 1.0, setze auf letztes Element
                if t >= 1.0:
                    resampled.append(pos_arr[-1].tolist())
                    continue
                # finde index i so dass cum_norm[i] <= t < cum_norm[i+1]
                i = np.searchsorted(cum_norm, t, side='right') - 1
                if i < 0:
                    i = 0
                if i >= len(pos_arr) - 1:
                    resampled.append(pos_arr[-1].tolist())
                    continue
                t0 = cum_norm[i]
                t1 = cum_norm[i + 1]
                # lokale Interpolationsparameter (0..1)
                local_alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                p = (1.0 - local_alpha) * pos_arr[i] + local_alpha * pos_arr[i + 1]
                resampled.append(p.tolist())

            # numerische Sicherheit: setze explizit erste und letzte Pose
            resampled[0] = positions[0]
            resampled[-1] = positions[-1]
            return resampled

        # Hilfsfunktion: extrahiere Knotenposen aus einem Pfad
        def path_positions_from_nodes(path, planer):
            return [planer.graph.nodes[node]['pos'] for node in path]

        # Gesamtanzahl Frames (gleich für beide Animationen)
        total_frames = max(1, int(round(seconds_total * fps)))

        # Erzeuge Positionen (ursprünglich mit Knoten) und resample auf total_frames
        origin_raw = path_positions_from_nodes(origin_path, origin_planer)
        smoothed_raw = path_positions_from_nodes(self.smoothed_path, smoothed_planer)

        origin_pos = resample_by_arclength(origin_raw, total_frames)
        smoothed_pos = resample_by_arclength(smoothed_raw, total_frames)

        # Workspace limits (falls unterschiedlich, behalten wir jeweils)
        origin_limits = origin_planer._collisionChecker.robot.getLimits()
        smoothed_limits = smoothed_planer._collisionChecker.robot.getLimits()

        # Robot-Referenz (falls zwei unabhängige Roboter nötig sind, muss environment angepasst werden)
        origin_robot = origin_planer._collisionChecker.robot
        smoothed_robot = smoothed_planer._collisionChecker.robot

        # Gemeinsame Animationsfunktion: aktualisiert beide Subplots synchron
        def animation(frame, ax_left, ax_right, origin_positions, smoothed_positions,
                    origin_limits, smoothed_limits, origin_path, smoothed_path):
            # linke Figur: original
            ax_left.cla()
            ax_left.set_title("original_path", fontsize=14)
            ax_left.set_xlim(origin_limits[0])
            ax_left.set_ylim(origin_limits[1])
            ax_left.set_aspect('equal', adjustable='box')
            ax_left.grid(True)
            
            pos_o = origin_positions[frame]
            origin_robot.setTo(pos_o)
            self.simple_draw(origin_planer, ax=ax_left)

            # rechte Figur: smoothed
            ax_right.cla()
            ax_right.set_title(title, fontsize=14)
            ax_right.set_xlim(smoothed_limits[0])
            ax_right.set_ylim(smoothed_limits[1])
            ax_right.set_aspect('equal', adjustable='box')
            ax_right.grid(True)
    
            pos_s = smoothed_positions[frame]
            smoothed_robot.setTo(pos_s)
            self.simple_draw(smoothed_planer, ax=ax_right)

        interval_ms = int(round(1000.0 / fps))

        # Eine Figur mit zwei Subplots nebeneinander
        fig_local = plt.figure(figsize=(14, 7))
        ax_origin = fig_local.add_subplot(1, 2, 1)
        ax_smoothed = fig_local.add_subplot(1, 2, 2)

        def animate_both(frame):
            animation(frame, ax_origin, ax_smoothed,
                    origin_pos, smoothed_pos,
                    origin_limits, smoothed_limits,
                    origin_path, self.smoothed_path)

        ani = matplotlib.animation.FuncAnimation(fig_local, animate_both, frames=total_frames, interval=interval_ms)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()

    def simple_draw(self, planer, ax):
        planer._collisionChecker.drawObstacles(ax)
        
        graph = planer.graph
        pos = nx.get_node_attributes(graph,'pos')
        # todo extract from pos the first two dimensions only for drawing in workspace
        pos2D = dict()
        for key in pos.keys():
            pos2D[key] = (pos[key][0], pos[key][1])
            
        pos = pos2D

        nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=100)
        nx.draw_networkx_edges(graph,pos, ax = ax)