import numpy as np
from Point3D import *
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
    # @Neighbors.setter
    # def Neighbors(self,val):

    @property
    def Edges(self):
        edges = []
        edges.extend((np.vstack((self.Points[0],self.Points[1])),
                     np.vstack((self.Points[1],self.Points[2])),
                      np.vstack((self.Points[2],self.Points[0]))))
        return edges
    def circumcenter(self):
        """Compute circumcenter  of a triangle in 2D"""
        
        # self.Points = np.asarray([self.coords[v] for v in tri])
        pts2 = np.dot(self.Points, self.Points.T)
        A = np.bmat([[2 * pts2, [[1],
                                 [1],
                                 [1]]],
                      [[[1, 1, 1, 0]]]])

        b = np.hstack((np.sum(self.Points * self.Points, axis=1), [1]))
        x = np.linalg.solve(A, b)
        bary_coords = x[:-1]
        center = np.dot(bary_coords, self.Points)

        # radius = np.linalg.norm(self.Points[0] - center) # euclidean distance
        radius = np.sum(np.square(self.Points[0] - center))  # squared distance
        return center, radius

    def findNeighbor(self, triangles):
        """find neighbor in list of triangles"""

        for t in triangles:
            if t is not None:
                if np.sum(np.in1d(t.Points[:, 3], self.Points[:, 3])) == 2:
                    return t

    def contains(self, point):
        """
        return whether the point inside or outside the polygon

        :param point: 2D point

        :type point: Point object
        .. warning::

            this function is empty. needs implementation.

        """
        count = 0
        for e in self.Edges:
            if ray_intersect_polygon(point, e):
                count += 1
        if count % 2 == 1:
            return True
        else:
            return False

    def isInsideCircle(self, p):
        """Check if point p is inside of circumcircle around the triangle tri"""

        points = np.vstack((self.Points, p))
        col1 = points[:, 0]
        col2 = points[:, 1]
        col3 = points[:, 0] ** 2 + points[:, 1] ** 2
        col4 = np.ones((4, 1))[:, 0]
        m = np.vstack((col1, col2, col3, col4)).T
        # m = np.vstack((m,np.hstack((p,1))))
        return np.linalg.det(m) > 0