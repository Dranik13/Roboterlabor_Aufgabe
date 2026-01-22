'''
Description: Creation of diagrams comparing the two smoothing methods.
'''

import numpy as np
import matplotlib.pylab as plt
from shapely.geometry import LineString
from matplotlib.ticker import MaxNLocator

# Erstellung des Diagramms mit den Kriterien Pfadlänge, Anzahl der Knoten und Zeit des Planens bzw. Glättung
def generate_diagramm(testList, resultList):
    for bench in testList:
        title = bench.name
        pathLength = dict()
        planningTime = dict()
        roadmapSize = dict()
        smoothpathLength = dict()
        smoothpathLength_random = dict()
        roadmapSizeBG = dict()
        roadmapSizeRandom = dict()
        smoothingTimeBG = dict()
        smoothingTimeRandom = dict()

        #try:
        for result in resultList:
            if result.benchmark.name == bench.name:
                G = result.graph
                G_BG = result.smooth_graph_bg
                G_Random = result.smooth_graph_random
                path = result.solution
                coords = [(G.nodes[n]['pos'][0], G.nodes[n]['pos'][1]) for n in path]
                line = LineString(coords)
                pathLength[result.plannerFactoryName] = line.length
                smoothpath_bg = result.smoothed_path_bg
                smoothcoords = [(G_BG.nodes[n]['pos'][0], G_BG.nodes[n]['pos'][1]) for n in smoothpath_bg]
                smoothline = LineString(smoothcoords)
                smoothpath_random = result.smoothed_path_random
                smoothcoords_random = [(G_Random.nodes[n]['pos'][0], G_Random.nodes[n]['pos'][1]) for n in smoothpath_random]
                smoothline_random = LineString(smoothcoords_random)
                smoothpathLength_random[result.plannerFactoryName] = smoothline_random.length
                smoothpathLength[result.plannerFactoryName] = smoothline.length
                planningTime[result.plannerFactoryName] = result.perfDataFrame.groupby(["name"]).sum(numeric_only=True)["time"]["planPath"]
                roadmapSize[result.plannerFactoryName] = len(path)
                roadmapSizeBG[result.plannerFactoryName] = len(smoothpath_bg)
                roadmapSizeRandom[result.plannerFactoryName] = len(smoothpath_random)
                smoothingTimeBG[result.plannerFactoryName] = result.bg_smoother.smoothing_time
                smoothingTimeRandom[result.plannerFactoryName] = result.smoothing.smoothing_time


        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
        fig.suptitle(f"{title}")

        labels = list(pathLength.keys())

        # -------- Daten --------
        planned_len   = np.array(list(pathLength.values()))
        bg_len        = np.array(list(smoothpathLength.values()))
        random_len   = np.array(list(smoothpathLength_random.values()))

        planned_rm    = np.array(list(roadmapSize.values()))
        bg_rm         = np.array(list(roadmapSizeBG.values()))
        random_rm    = np.array(list(roadmapSizeRandom.values()))

        planned_time    = np.array(list(planningTime.values()))
        bg_time         = np.array(list(smoothingTimeBG.values()))
        random_time    = np.array(list(smoothingTimeRandom.values()))

        n = len(labels)
        group_size = 6
        gap = 1.5
        width = 0.25

        # -------- X-Positionen mit Abstand nach je 6 --------
        x = []
        pos = 0
        for i in range(n):
            if i > 0 and i % group_size == 0:
                pos += gap
            x.append(pos)
            pos += 1
        x = np.array(x)

        # -------- Plot 1: Path Lengths --------
        ax1.bar(x - width, planned_len,  width, label="Planned path", color="blue")
        ax1.bar(x,         bg_len,       width, label="BG smoothing", color="green")
        ax1.bar(x + width, random_len,  width, label="Random smoothing", color="purple")

        ax1.set_ylabel("Path length")
        ax1.legend()
        ax1.grid(axis="y", linestyle="--", alpha=0.5)

        # -------- Plot 2: Number of Path Points --------
        ax2.bar(x - width, planned_rm,  width, label="Planned roadmap", color="blue")
        ax2.bar(x,         bg_rm,       width, label="BG roadmap", color="green")
        ax2.bar(x + width, random_rm,  width, label="Random roadmap", color="purple")

        ax2.set_ylabel("Roadmap size")
        ax2.legend()
        ax2.grid(axis="y", linestyle="--", alpha=0.5)

        # -------- Plot 3: Planning/Smoothing Time --------
        ax3.bar(x - width, planned_time,  width, label="Planned roadmap", color="blue")
        ax3.bar(x,         bg_time,       width, label="BG roadmap", color="green")
        ax3.bar(x + width, random_time,  width, label="Random roadmap", color="purple")

        ax3.set_ylabel("Time")
        ax3.legend()
        ax3.grid(axis="y", linestyle="--", alpha=0.5)

        # -------- X-Achse --------
        ax2.set_xticks(x)
        ax2.set_xticklabels(labels, rotation=45, ha="right")

        plt.tight_layout()
        plt.show()
        plt.close()

def path_length_from_nodes(path, graph):
    coords = [(graph.nodes[n]['pos'][0], graph.nodes[n]['pos'][1]) for n in path]
    return LineString(coords).length

# Erstellung des Diagramms mit dem Verlauf der Pfadlänge und Anzahl der Knoten
def generate_timeplot(resultList):
    
    for result in resultList:
        if result.solution != []:
            G_BG = result.smooth_graph_bg
            G_Random = result.smooth_graph_random
            fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(12, 4))
            bg_geom_lengths = [
                path_length_from_nodes(p, G_BG)
                for p in result.bg_smoother.path_per_epoche
            ]

            sm_geom_lengths = [
                path_length_from_nodes(p, G_Random)
                for p in result.smoothing.path_per_epoche
            ]

            ax_left.plot(bg_geom_lengths, marker='o', label="BG Smoother")
            ax_left.plot(sm_geom_lengths, marker='x', label="Random Smoother")

            ax_left.set_xlabel("Iteration")
            ax_left.set_ylabel("Pathlength")
            ax_left.set_title("Geometric Pathlength")
            ax_left.grid(True)
            ax_left.legend()

            # --------------------------------
            # RECHTER PLOT: Knotenzahl (bisher)
            # --------------------------------
            bg_lengths = [len(p) for p in result.bg_smoother.path_per_epoche]
            sm_lengths = [len(p) for p in result.smoothing.path_per_epoche]

            ax_right.plot(bg_lengths, marker='o', label="BG Smoother")
            ax_right.plot(sm_lengths, marker='x', label="Random Smoother")

            ax_right.set_xlabel("Iteration")
            ax_right.set_ylabel("Number of nodes")
            ax_right.set_title("Number of nodes")
            ax_right.yaxis.set_major_locator(MaxNLocator(integer=True))
            ax_right.grid(True)
            ax_right.legend()

            # Gesamttitel
            fig.suptitle(
                f"{result.plannerFactoryName} – {result.benchmark.name}",
                fontsize=12
            )

            plt.tight_layout()
            plt.show()
            plt.close()