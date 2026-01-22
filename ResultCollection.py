import SmoothBG
import SmoothRandom

class ResultCollection (object):
    """
    Collects all results of a benchmark run, including the original solution,
    smoothed paths, and corresponding graphs for different smoothing strategies.
    """
    def __init__(self, plannerFactoryName, pathplanner, benchmark, solution, perfDataFrame):
        '''
        Initialize a result container for a single benchmark run.\n 
        :param plannerFactoryName: Name of the current benchmark\n
        :param pathplanner: The path planer used for creating the collision free base path\n
        :param benchmark: current benchmark task\n
        :param solution: Collison free path of path planner\n
        :param perfDataFrame: Decorator that keeps track of the number of times a function is called and collects additional information like arguments, return value, and time spent in the function\n
        '''
        self.plannerFactoryName = plannerFactoryName
        self.planner = pathplanner
        self.benchmark = benchmark
        self.solution = solution
        self.perfDataFrame = perfDataFrame

        # Initializing the smoothing classes
        self.bg_smoother = SmoothBG.SmoothBG()
        self.smoothing = SmoothRandom.SmoothRandom(pathplanner, solution)
        self.configs = {}
        self.configs["corner_threshold"] = 0
        self.configs["collision_intervals"] = 200
        self.configs["max_deltree_depth"] = 10
        self.configs["epoches"] = 50
        
        # Smoothed paths and new graphs
        self.smoothed_path_bg = []
        self.smooth_graph_bg = []
        self.smoothed_path_random = []
        self.smooth_graph_random = []
        self.graph = pathplanner.graph.copy() # Work on a copy to avoid modifying the original planner graph
        self.path_smoothing()


    def path_smoothing(self):
        '''        
        :param self: Run smoothing functions
        '''
        if self.solution != []:
            # Apply both smoothing strategies for comparison
            self.smoothed_path_random, self.smooth_graph_random = self.smoothing.smooth_path(self.configs)
            self.smoothed_path_bg, self.smooth_graph_bg = self.bg_smoother.smooth_path(self.solution, self.planner, self.configs)