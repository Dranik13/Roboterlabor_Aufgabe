# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark
from IPPlanarManipulator import PlanarRobot
from IPEnvironmentKin import KinChainCollisionChecker
from shapely.geometry import LineString

benchList = list()
r = PlanarRobot(n_joints=2)

start_joint_pos = [2.0, 0.5]
end_joint_pos = [-2.0, -0.5]

# -----------------------------------------

obst = dict()
obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
obst["obs3"] = LineString([(-0.5, 2), (1, 2)]).buffer(0.3)
obst["obs4"] = LineString([(1, -2), (1, -3)]).buffer(0.3)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Kin_Obst", KinChainCollisionChecker(r, obst,fk_resolution=.2), [start_joint_pos], [end_joint_pos], description, 2))

# ---------- NEW TEST --------------------

start_joint_pos2 = [2.3, -0.0]
end_joint_pos2 = [1, 0]

obst2 = dict()
obst2["obs1"] = LineString([(0, 3), (0, 5)]).buffer(1.5)
obst2["obs2"] = LineString([(-2, -2), (2, -2)]).buffer(1.5)

description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Kin_Obst2", KinChainCollisionChecker(r, obst2,fk_resolution=.2), [start_joint_pos2], [end_joint_pos2], description, 2))

# -----------------------------------------