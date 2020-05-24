import numpy as np
from HW3.Triangle import *
from HW3.Utils import *
from matplotlib import path


class Delaunay:

    def __init__(self, centerX,centerY,M):
        """
        Delaunay triangulation
        :param points: np.array
        """
        self.__triangles = []

        # create big triangle
        # centerX = np.average(points[:,0])
        # centerY = np.average(points[:,1])
        # width = np.max(points[:,0]) - np.min(points[:,0])
        # height = np.max(points[:,1]) - np.min(points[:,1])
        # M = np.max([width,height])
        p1 = np.array([centerX+3*M,centerY])
        p2 = np.array([centerX,centerY+3*M])
        p3 = np.array([centerX-3*M,centerY-3*M])
        self.__triangles.append(Triangle(p1,p2,p3))

    @property
    def Triangles(self):
        return self.__triangles

    def insertPoint(self, p):
        """
        insert point into triangulation
        :type p: np.array 1X3
        """
        T = self.findTriangle(p)

    def findTriangle(self, p):
        """ find containing triangle"""
        for t in self.Triangles:
            poly = path.Path(t.Points)
            if poly.contains_point(p):
                return t



if __name__ == '__main__':
    p1 = np.array([1,1,1])
    p2 = np.array([2,1,2])
    p3 = np.array([3,3,3])
    d = Delaunay(np.vstack((p1,p2,p3)))
    x=1

