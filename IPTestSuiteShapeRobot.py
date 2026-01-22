# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironmentShapeRobot import CollisionCheckerShapeRobot
from IPEnvironmentShapeRobot import ShapeRobotWithOrientation
from shapely.geometry import Polygon, LineString

import numpy as np

import labyrinth

#define robot geometry
robot_shape = Polygon(list(np.array([
    (0,0),
    (-3,0),
    (-3,0.5),
    (-0.5,0.5),
    (-0.5, 4.5),
    (2.5, 4.5),
    (2.5, 4),
    (0,4)
])/2))

shape_robot = ShapeRobotWithOrientation(robot_shape, limits=[[0.0, 22.0], [0.0, 22.0], [-np.pi, np.pi]])
benchList = list()

# -----------------------------------------

trapField = dict()
trapField["obs1"] = LineString([(5,10),(5,5),(12.5,5),(12.5,10)]).buffer(0.5)
trapField["obs2"] = LineString([(10,8), (10,15), (18,15), (18,8), (18, 15), (20,15)]).buffer(0.5)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionCheckerShapeRobot(trapField, shape_robot), [[7.5, 7, 0]], [[15, 12, 0]], description, 2))

# -----------------------------------------

myField = dict()
myField["laby"] = labyrinth.generate(walk_way=4.0, height=18, length=18)
description = "The planer needs to find a path into the labyrinth"
benchList.append(Benchmark("Labyrinth", CollisionCheckerShapeRobot(myField, shape_robot), [[1, 2, 0]], [[10, 11, 0]], description, 2))
