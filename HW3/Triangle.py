from numpy import vstack
from Point3D import *


class Triangle:

    def __init__(self,p1,p2,p3):
        self.__points = vstack((p1,p2,p3))
        self.__neighbors = None

    @property
    def Points(self):

        return self.__points

    @property
    def Edges(self):
        return np.array([[self.Points[1]]])

    def contains(self, p):
        """
        return whether the point inside or outside the polygon

        """
        count = 0
        for i in range(3):
            if ray_intersect_polygon(p, self.Points):
                count += 1
        if count % 2 == 1:
            return True
        else:
            return False