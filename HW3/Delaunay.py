import numpy as np
from HW3.Triangle import *
from HW3.Utils import *
from matplotlib import path
import matplotlib.pyplot as plt


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
        p1 = np.array([centerX+3*M,centerY,0])
        p2 = np.array([centerX,centerY+3*M,0])
        p3 = np.array([centerX-3*M,centerY-3*M,0])
        T = Triangle(p1,p2,p3)
        T.Neighbors.extend([None,None,None])
        self.__triangles.append(T)

    @property
    def Triangles(self):
        return self.__triangles

    def insertPoint(self, p):
        """
        insert point into triangulation
        :type p: np.array 1X3
        """
        # find the triangle that contains the point
        T = self.findTriangle(p)
        # divide to three triangles
        t1 = Triangle(T.Points[0],T.Points[1],p)
        t2 = Triangle(T.Points[1],T.Points[2],p)
        t3 = Triangle(T.Points[2],T.Points[0],p)

        # insert neighbors

        t1.Neighbors.extend([T.Neighbors[0],t2,t3])
        if T.Neighbors[0] is not None:  # case of outer borders
            T.Neighbors[0].Neighbors[T.Neighbors[0].Neighbors.index(T)] = t1
        t2.Neighbors.extend([T.Neighbors[1],t1,t3])
        if T.Neighbors[1] is not None:
            T.Neighbors[1].Neighbors[T.Neighbors[1].Neighbors.index(T)] = t2
        t3.Neighbors.extend([T.Neighbors[2],t1,t2])
        if T.Neighbors[2] is not None:
            T.Neighbors[2].Neighbors[T.Neighbors[2].Neighbors.index(T)] = t3

        # removing old triangle and adding the new ones
        del self.Triangles[self.Triangles.index(T)]
        self.Triangles.extend([t1,t2,t3])

        # check the empty circle property
        for t in [t1,t2,t3]:
            if isInsideCircle(t.Neighbors[0],p):
                # flip


    def findTriangle(self, p):
        """ find containing triangle"""
        for t in self.Triangles:
            poly = path.Path(t.Points[:,:2])
            if poly.contains_point(p):
                return t

    def plotTriangulation(self):
        for t in self.Triangles:
            plt.plot(t.Points[:,0], t.Points[:,1], 'bo-', linewidth=0.4, markersize=1)
        plt.show()

    def flip(self):

if __name__ == '__main__':
    p1 = np.array([1,1,1])
    p2 = np.array([2,1,2])
    p3 = np.array([3,3,3])
    points = np.vstack((p1,p2,p3))
    centerX = np.average(points[:,0])
    centerY = np.average(points[:,1])
    width = np.max(points[:,0]) - np.min(points[:,0])
    height = np.max(points[:,1]) - np.min(points[:,1])
    M = np.max([width,height])
    d = Delaunay(centerX,centerY,M)
    for p in points:
        d.insertPoint(p)
        d.plotTriangulation()

    x=1

