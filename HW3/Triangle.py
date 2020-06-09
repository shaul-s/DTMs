import numpy as np
from HW3.Utils import *

class Triangle:

    def __init__(self,p1,p2,p3,ID=0):
        self.__points = np.vstack((p1,p2,p3))
        self.__neighbors = []
        self.__id = ID

    @property
    def Points(self):
        return self.__points

    @property
    def ID(self):
        return self.__id

    @ID.setter
    def ID(self,val):
        self.__id = val

    @property
    def Neighbors(self):
        return self.__neighbors

    @property
    def Edges(self):
        edges = []
        edges.extend((np.vstack((self.Points[0],self.Points[1])),
                     np.vstack((self.Points[1],self.Points[2])),
                      np.vstack((self.Points[2],self.Points[0]))))
        return edges


    def findNeighbor(self, triangles):
        """find neighbor in list of triangles"""

        for t in triangles:
            if t is not None:
                if np.sum(np.in1d(t.Points[:, 3], self.Points[:, 3])) == 2:
                    return t


    def isInsideCircle(self, p):
        """Check if point p is inside of circumcircle around the triangle tri"""

        points = np.vstack((self.Points, p))
        col1 = points[:, 0]
        col2 = points[:, 1]
        col3 = points[:, 0] ** 2 + points[:, 1] ** 2
        col4 = np.ones((4, 1))[:, 0]
        m = np.vstack((col1, col2, col3, col4)).T
        return np.linalg.det(m) > 0

    def findIntersection(self,l):
        """
        finds and returns intersection points of line with the trianglel edged
        :type line: np.array 2X4
        :return: intersections points and neighbors indices
        """
        for i in range(0,3):
            L1 = line(self.Edges[i][0], self.Edges[i][1])
            L2 = line(l.Edges[i][0], l.Edges[i][1])

            R = intersection(L1, L2)
            if R:
                print
                "Intersection detected:", R
            else:
                print
                "No single intersection point detected"

