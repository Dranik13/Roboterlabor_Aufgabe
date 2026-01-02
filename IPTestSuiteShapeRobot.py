# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironment import CollisionChecker
from IPEnvironmentShapeRobot import CollisionCheckerShapeRobot
from IPEnvironmentShapeRobot import ShapeRobot, ShapeRobotWithOrientation
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
import numpy as np

import labyrinth

#define robot geometry
# robot_shape = Polygon([
#     (-0.5, -0.5),  # unten links
#     (3, -0.5),     # unten rechts
#     (3, 0.0),      # Innenknick rechts
#     (0.5, 0.0),    # Innenknick unten horizontal
#     (0.5, 1.0),    # oben links vom L-Schenkel
#     (-0.5, 1.0)    # oben links außen
#     ])

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
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionCheckerShapeRobot(trapField, shape_robot), [[10,15,0]], [[9.5,1,0]], description, 2))

# # -----------------------------------------
# bottleNeckField = dict()
# bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
# bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
# description = "Planer has to find a narrow passage."
# benchList.append(Benchmark("Bottleneck", CollisionCheckerShapeRobot(bottleNeckField, shape_robot), [[4,15,0]], [[18,1,0]], description, 2))

# # -----------------------------------------
# fatBottleNeckField = dict()
# fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
# fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
# description = "Planer has to find a narrow passage with a significant extend."
# benchList.append(Benchmark("Fat bottleneck", CollisionCheckerShapeRobot(fatBottleNeckField, shape_robot), [[4,21,0]], [[18,1,0]], description, 2))

# # -----------------------------------------

# myField = dict()
# myField["L"] = Polygon([(10, 16), (10, 11), (13, 11), (13,12), (11,12), (11,16)])
# myField["T"] = Polygon([(14,16), (14, 15), (15, 15),(15,11), (16,11), (16,15), (17, 15), (17, 16)])
# myField["C"] = Polygon([(19, 16), (19, 11), (22, 11), (22, 12), (20, 12), (20, 15), (22, 15), (22, 16)])

# myField["Antenna_L"] = Polygon([(3, 12), (1, 16), (2, 16), (4, 12)])
# myField["Antenna_Head_L"] = Point(1.5, 16).buffer(1)

# myField["Antenna_R"] = Polygon([(7, 12), (9, 16), (8, 16), (6, 12)])
# myField["Antenna_Head_R"] = Point(8.5, 16).buffer(1)

# myField["Rob_Head"] = Polygon([(2, 13), (2, 8), (8, 8), (8, 13)])
# description = "Planer has to find a passage past a robot head and the print of the LTC."
# benchList.append(Benchmark("MyField", CollisionCheckerShapeRobot(myField, shape_robot), [[4,21,0]], [[18,1,0]], description, 2))

myField = dict()
myField["laby"] = labyrinth.generate(walk_way=3.0, height=15, length=14)
description = "The planer needs to find a path into the labyrinth"
benchList.append(Benchmark("Labyrinth", CollisionCheckerShapeRobot(myField, shape_robot), [[20,3.5,0]], [[9,9,0]], description, 2))
