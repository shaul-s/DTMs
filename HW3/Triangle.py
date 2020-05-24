import numpy as np
from Point3D import *


class Triangle:

    def __init__(self,p1,p2,p3):
        self.__points = np.vstack((p1,p2,p3))
        self.__neighbors = []

    @property
    def Points(self):

        return self.__points

    @property
    def Neighbors(self):
        return self.__neighbors
    # @Neighbors.setter
    # def Neighbors(self,val):
    
    def circumcenter(self):
        """Compute bounding circle of a triangle in 2D"""
        
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
        return (center, radius)

