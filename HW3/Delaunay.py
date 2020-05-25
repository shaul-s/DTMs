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
        self.__pointId = 0
        self.__triangleId = 0

        # create big triangle
        # centerX = np.average(points[:,0])
        # centerY = np.average(points[:,1])
        # width = np.max(points[:,0]) - np.min(points[:,0])
        # height = np.max(points[:,1]) - np.min(points[:,1])
        # M = np.max([width,height])
        p1 = np.array([centerX+3*M,centerY,0,-1])
        p2 = np.array([centerX,centerY+3*M,0,-2])
        p3 = np.array([centerX-3*M,centerY-3*M,0,-3])
        T = Triangle(p1,p2,p3)
        T.Neighbors.extend([None,None,None])
        self.__triangles.append(T)

    @property
    def Triangles(self):
        return self.__triangles

    @property
    def PointId(self):
        return self.__pointId

    @PointId.setter
    def PointId(self,val):
        self.__pointId = val

    @property
    def TriangleId(self):
        return self.__triangleId

    @TriangleId.setter
    def TriangleId(self,val):
        self.__triangleId = val



    def insertPoint(self, p):
        """
        insert point into triangulation
        :type p: np.array 1X3
        """
        # adding point id in the triangulation
        p = np.hstack((p,self.PointId))
        self.PointId += 1

        # find the triangle that contains the point
        T = self.findTriangle(p)
        # divide to three triangles
        t1 = Triangle(T.Points[0],T.Points[1],p,self.TriangleId)
        self.TriangleId += 1
        t2 = Triangle(T.Points[1],T.Points[2],p,self.TriangleId)
        self.TriangleId += 1
        t3 = Triangle(T.Points[2],T.Points[0],p,self.TriangleId)
        self.TriangleId += 1

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

        self.plotTriangulation()
        # check the empty circle property
        for i,t in enumerate([t1,t2,t3]):
            if t.Neighbors[0] is not None:
                self.legalizeEdge(p,t,t.Neighbors[0])




    def findTriangle(self, p):
        """ find containing triangle"""
        for t in self.Triangles:
            poly = path.Path(t.Points[:,:2])
            if poly.contains_point(p):
                return t

    def plotTriangulation(self):
        for t in self.Triangles:
            plt.plot(t.Points[:,0], t.Points[:,1], 'bo-', linewidth=0.4, markersize=1)
            plt.annotate(t.ID, (np.average(t.Points[:,0]), np.average(t.Points[:,1])))
            for p in t.Points:
                plt.annotate(p[3], (p[0], p[1]))
        plt.show()

    def legalizeEdge(self, p, tri, opositTriangle):
        """flipping triangulation of 4 edge polygon  """

        if opositTriangle is not None:
            self.plotTriangulation()
            if isInsideCircle(opositTriangle, p):
                # building the two triangles the flip made
                farPointInd = np.argmin(np.in1d(opositTriangle.Points[:,3], tri.Points[:,3]))
                newTriangle1 = Triangle(opositTriangle.Points[farPointInd],
                                        opositTriangle.Points[(farPointInd+1) % 3],p,self.TriangleId)
                self.TriangleId += 1
                newTriangle2 = Triangle(opositTriangle.Points[(farPointInd - 1) % 3],
                                        opositTriangle.Points[farPointInd],p,self.TriangleId)
                self.TriangleId += 1
                # populate neighbors
                del tri.Neighbors[tri.Neighbors.index(opositTriangle)]
                neighbor1 = newTriangle1.findNeighbor(opositTriangle.Neighbors)
                if neighbor1 is not None:
                    neighbor1.Neighbors[neighbor1.Neighbors.index(opositTriangle)] = newTriangle1
                neighbor2 = newTriangle1.findNeighbor(tri.Neighbors)
                if neighbor2 is not None:
                    neighbor2.Neighbors[neighbor2.Neighbors.index(tri)] = newTriangle1
                neighbor3 = newTriangle2
                newTriangle1.Neighbors.extend([neighbor1,neighbor2,neighbor3])
                del opositTriangle.Neighbors[opositTriangle.Neighbors.index(tri)]
                neighbor1 = newTriangle2.findNeighbor(opositTriangle.Neighbors)
                if neighbor1 is not None:
                    neighbor1.Neighbors[neighbor1.Neighbors.index(opositTriangle)] = newTriangle2
                neighbor2 = newTriangle1
                neighbor3 = newTriangle2.findNeighbor(tri.Neighbors)
                if neighbor3 is not None:
                    neighbor3.Neighbors[neighbor3.Neighbors.index(tri)] = newTriangle2
                newTriangle2.Neighbors.extend([neighbor1, neighbor2, neighbor3])

                self.Triangles.extend((newTriangle1,newTriangle2))
                # delete changed triangles
                del self.Triangles[self.Triangles.index(tri)]
                del self.Triangles[self.Triangles.index(opositTriangle)]

                # continue until all edge are legal
                self.legalizeEdge(p, newTriangle1, newTriangle1.Neighbors[0])
                self.legalizeEdge(p, newTriangle2, newTriangle2.Neighbors[0])




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

