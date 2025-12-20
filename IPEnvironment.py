# coding: utf-8

"""
This code is part of a series of notebooks regarding  "Introduction to robot path planning".

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPerfMonitor import IPPerfMonitor

import matplotlib.pyplot as plt

from shapely.geometry import Point, Polygon, LineString
from shapely import plotting

import numpy as np
from shapely.affinity import rotate, translate

class CollisionChecker(object):

    def __init__(self, scene, limits=[[0.0, 22.0], [0.0, 22.0]], statistic=None):
        self.scene = scene
        self.limits = limits
        self.dim = len(self.limits)
        self.robot_poly = None

    def getDim(self):
        """ Return dimension of Environment (Shapely should currently always be 2)"""
        return self.dim

    def getEnvironmentLimits(self):
        """ Return limits of Environment"""
        return list(self.limits)

    def transform_polygon_local_to_world(self, poly_local, pos, use_radians=True):
        """
        poly_local bleibt garantiert unverändert (keine Re-Bindings, keine in-place Änderungen).
        pos: (x,y) oder (x,y,theta)
        """
        if len(pos) < 2:
            raise ValueError(f"pos muss mindestens (x,y) enthalten, erhalten: {pos}")

        x = float(pos[0])
        y = float(pos[1])
        theta = float(pos[2]) if len(pos) >= 3 and pos[2] is not None else 0.0

        poly_tmp = poly_local

        if theta != 0.0:
            poly_tmp = rotate(poly_tmp, theta, origin=(0, 0), use_radians=use_radians)

        poly_world = translate(poly_tmp, xoff=x, yoff=y)
        return poly_world

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """ Return whether a configuration is
        inCollision -> True
        Free -> False """
        assert (len(pos) == self.getDim())
        for key, value in self.scene.items():
            if value.intersects(Point(pos[0], pos[1])):
                return True
        return False
    
    @IPPerfMonitor
    def polyInCollision(self, pos):
        """True wenn das (rotierte+verschobene) Polygon mit irgendeinem Hindernis kollidiert."""
        assert len(pos) == self.getDim()  # bei dir 3

        poly_world = self.transform_polygon_local_to_world(self.robot_poly, pos, use_radians=True)

        # Optional: schneller Vorab-Check via Bounding Boxes wäre möglich – aber erstmal simpel:
        for _, obstacle in self.scene.items():
            if obstacle.intersects(poly_world):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ Check whether a line from startPos to endPos is colliding"""
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1
        k = 40
        #print("testing")
        for i in range(k):
            testPoint = p1 + (i+1)/k*p12
            if self.robot_poly != None:
                if self.polyInCollision(testPoint)==True:
                    return True
            else:
                if self.pointInCollision(testPoint) == True:
                    return True
        
        return False
                

#        for key, value in self.scene.items():
#            if value.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])])):
 #               return True
#        return False

    def drawObstacles(self, ax):
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)
            
