# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironmentShapeRobot import CollisionCheckerShapeRobot
from IPEnvironmentShapeRobot import ShapeRobotWithOrientation
from shapely.geometry import Point, LineString

import labyrinth

# define robot geometry
robot_shape = Point([(0,0)]).buffer(0.1)
shape_robot = ShapeRobotWithOrientation(robot_shape, limits=[[0.0, 22.0], [0.0, 22.0], [0,0]])
benchList = list()

# -----------------------------------------

trapField = dict()
trapField["obs1"] = LineString([(5,10),(5,5),(12.5,5),(12.5,10)]).buffer(0.5)
trapField["obs2"] = LineString([(10,8), (10,15), (18,15), (18,8), (18, 15), (20,15)]).buffer(0.5)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionCheckerShapeRobot(trapField, shape_robot), [[7.5, 7, 0]], [[15, 12, 0]], description, 2))

# -----------------------------------------

myField = dict()
myField["laby"] = labyrinth.generate()
description = "The planer needs to find a path into the labyrinth"
benchList.append(Benchmark("Labyrinth", CollisionCheckerShapeRobot(myField, shape_robot), [[3, 2.5, 0]], [[10, 10, 0]], description, 2))
