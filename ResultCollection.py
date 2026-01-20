import SmoothBG
import SmoothRandom

class ResultCollection (object):
    
    def __init__(self, plannerFactoryName, planner, benchmark, solution, perfDataFrame):
        self.bg_smoother = SmoothBG.SmoothBG()
        self.smoothing = SmoothRandom.SmoothRandom(planner, solution)
        configs = {}
        configs["corner_threshold"] = 0
        configs["collision_intervals"] = 200
        configs["max_deltree_depth"] = 10
        configs["epoches"] = 50
        self.plannerFactoryName = plannerFactoryName
        self.planner = planner
        self.benchmark = benchmark
        self.solution = solution
        self.perfDataFrame = perfDataFrame
        self.graph = planner.graph.copy()
        if solution != []:
            self.smoothed_path_random, self.smooth_graph_random = self.smoothing.smooth_path(configs)
            self.smoothed_path_bg, self.smooth_graph_bg = self.bg_smoother.smooth_path(solution, planner, configs)
        else:
            self.smoothed_path_bg = []
            self.smoothed_path_random = []
            self.smooth_graph_random = []
            self.smooth_graph_bg = []