import numpy as np
from HW3.Triangle import *
from HW3.Utils import *
from matplotlib import path
import matplotlib.pyplot as plt


class Delaunay:

    def __init__(self, points):
        """
        Delaunay triangulation
        :param points: np.array
        """
        self.__triangles = []
        self.__pointId = 1
        self.__triangleId = 0

        # create big triangle
        centerX = np.average(points[:, 0])
        centerY = np.average(points[:, 1])
        width = np.max(points[:, 0]) - np.min(points[:, 0])
        height = np.max(points[:, 1]) - np.min(points[:, 1])
        M = np.max([width, height])
        p1 = np.array([centerX + 3 * M, centerY, 0, -1])
        p2 = np.array([centerX, centerY + 3 * M, 0, -2])
        p3 = np.array([centerX - 3 * M, centerY - 3 * M, 0, -3])
        T = Triangle(p1, p2, p3)
        T.Neighbors.extend([None, None, None])  # set external face to None
        self.__triangles.append(T)
        # insert points to the triangulation
        for i, p in enumerate(points):
            self.insertPoint(p)
            if i % 100 == 0:
                print(i, 'points inserted')  # show progress
            # self.plotTriangulation()
        self.deleteOuterTriangles()

    @property
    def Triangles(self):
        return self.__triangles

    @Triangles.setter
    def Triangles(self, val):
        self.__triangles = val

    @property
    def PointId(self):
        return self.__pointId

    @PointId.setter
    def PointId(self, val):
        self.__pointId = val

    @property
    def TriangleId(self):
        return self.__triangleId

    @TriangleId.setter
    def TriangleId(self, val):
        self.__triangleId = val

    def insertPoint(self, p):
        """
        insert point into triangulation
        :type p: np.array 1X3
        """
        # adding point id in the triangulation
        if p.size < 4:
            p = np.hstack((p, self.PointId))
            self.PointId += 1
        elif p[3] is None:
            p[3] = self.PointId
            self.PointId += 1

        # find the triangle that contains the point
        T = self.findTriangle(p)
        # divide to three triangles
        # keeps points order counterclockwise
        # keeps neighbors order correspond to points order
        t1 = Triangle(T.Points[0], T.Points[1], p, self.TriangleId)
        self.TriangleId += 1
        t2 = Triangle(T.Points[1], T.Points[2], p, self.TriangleId)
        self.TriangleId += 1
        t3 = Triangle(T.Points[2], T.Points[0], p, self.TriangleId)
        self.TriangleId += 1

        # insert neighbors
        # self.plotTriangulation()
        t1.Neighbors.extend([T.Neighbors[0], t2, t3])
        if T.Neighbors[0] is not None:  # case of outer borders
            T.Neighbors[0].Neighbors[T.Neighbors[0].Neighbors.index(T)] = t1
        t2.Neighbors.extend([T.Neighbors[1], t3, t1])
        if T.Neighbors[1] is not None:
            T.Neighbors[1].Neighbors[T.Neighbors[1].Neighbors.index(T)] = t2
        t3.Neighbors.extend([T.Neighbors[2], t1, t2])
        if T.Neighbors[2] is not None:
            T.Neighbors[2].Neighbors[T.Neighbors[2].Neighbors.index(T)] = t3

        # removing old triangle and adding the new ones
        del self.Triangles[self.Triangles.index(T)]
        self.Triangles.extend([t1, t2, t3])

        # check the empty circle property
        for i, t in enumerate([t1, t2, t3]):
            if t.Neighbors[0] is not None:
                # self.legalizeEdge(p, t, t.Neighbors[0])
                self.legalizeEdge(t, 0)

    def findTriangle(self, p):
        """ find containing triangle"""
        for t in self.Triangles:
            poly = path.Path(t.Points[:, :2])
            if poly.contains_point(p):
                return t

    def plotTriangulation(self):
        for t in self.Triangles:
            tempPoints = np.vstack((t.Points, t.Points[0]))
            plt.plot(tempPoints[:, 0], tempPoints[:, 1], 'bo-', linewidth=0.4, markersize=1)
            # plt.annotate(t.ID, (np.average(t.Points[:, 0]), np.average(t.Points[:, 1])))
            # for p in t.Points:
            #     plt.annotate(p[3], (p[0], p[1]))
        plt.show()

    def plotTriangulationWithConstrain(self,constrain):
        for i in range(0,len(constrain),2):
            plt.plot(constrain[i:i+2,0],constrain[i:i+2,1],'r-', linewidth=1.5, markersize=1)
        # plt.plot(pIntersect[0],pIntersect[1],'ro', linewidth=0.6, markersize=1)
        for t in self.Triangles:
            tempPoints = np.vstack((t.Points, t.Points[0]))
            plt.plot(tempPoints[:, 0], tempPoints[:, 1], 'bo-', linewidth=0.4, markersize=1)
            # plt.annotate(t.ID, (np.average(t.Points[:, 0]), np.average(t.Points[:, 1])))
            # for p in t.Points:
            #     plt.annotate(p[3], (p[0], p[1]))
        plt.show()

    # def legalizeEdge(self, p, tri, opositTriangle):
    def legalizeEdge(self,tri, edge):
        """flipping triangulation of 4 edge polygon  """
        opositTriangle = tri.Neighbors[edge]
        p = tri.Points[(edge + 2) % 3]
        if opositTriangle is not None:

            if opositTriangle.isInsideCircle(p):
                # swap diagonal
                newTriangle1, newTriangle2 = self.flip(tri,edge)

                # keep going until all triangles are legal
                self.legalizeEdge(newTriangle1, edge)
                self.legalizeEdge(newTriangle2, edge)

    def deleteOuterTriangles(self):
        """ deleting the triangles connected to the "big triangle" """
        outerTriangles = []
        for i, t in enumerate(self.Triangles):
            if np.sum(t.Points[:, 3] < 0) > 0:  # means the points belong to the "big triangle"
                outerTriangles.append(i)
                # remove redundant neighbors from triangulation
                self.removeTriangle(t)

        self.Triangles = [i for j, i in enumerate(self.Triangles) if j not in outerTriangles]

    def addConstrains(self,constrains):
        """
        adding constrains to the triangulation
        :param constrains: lines that can't be crossed .represented by two points each
        :type constrains: np.array 2nX4
        :return: none
        """
        for i in range(0,constrains.shape[0],2):
            self.plotTriangulationWithConstrain(constrains)
            # making list of  the edges intersected by the constrain
            intersectedTriangles, intersectingEdges = self.findIntersectingEdges(constrains[i:i+2])
            j=0
            while j <= len(intersectingEdges)-1:
                # building the 4 edges polygon to check
                polyPoints = intersectedTriangles[j].Points
                for p in intersectedTriangles[j].Neighbors[intersectingEdges[j]].Points:
                    if (p == polyPoints).any():
                        continue
                    farPoint = p

                # arrange the polyPoints anticlockwise()
                polyPoints = np.vstack((intersectedTriangles[j].Points[intersectingEdges[j]],farPoint,
                                        intersectedTriangles[j].Points[(intersectingEdges[j]+1)%3],
                                        intersectedTriangles[j].Points[(intersectingEdges[j]+2)%3]))
                # self.plotTriangulationWithConstrain(constrains[i:i+2])

                # if the polygon is convex we want to flip the diagonal and remove from the intersected edges list
                if isConvex(polyPoints):
                    newTriangle1, newTriangle2 = self.flip(intersectedTriangles[j], intersectingEdges[j])
                    # self.plotTriangulationWithConstrain(constrains[i:i + 2])
                    # if j+1 == len(intersectedTriangles)-1:
                    #     break
                    if j+2 <= len(intersectedTriangles)-1:
                        if newTriangle1 in intersectedTriangles[j+1].Neighbors[intersectingEdges[j+1]].Neighbors:
                            intersectedTriangles[j+1] = newTriangle1
                            intersectingEdges[j+1] = 0
                        elif newTriangle2 in intersectedTriangles[j+1].Neighbors[intersectingEdges[j+1]].Neighbors:
                            intersectedTriangles[j + 1] = newTriangle2
                            intersectingEdges[j + 1] = 0
                        # intersectionPoint = intersect(self.Edges[i], l, self.Edges[i][0, 2])
                        # if type(intersectionPoint) != int:
                        #
                        # self.plotTriangulationWithConstrain(constrains[i:i+2])
                else: # if not convex we put the edge back to the list
                    intersectedTriangles.append(intersectedTriangles[j])
                    intersectingEdges.append(intersectingEdges[j])

                j += 1

    def findIntersectingEdges(self,constrain):
        intersectedTriangles = []
        intersectedEdges = []

        if constrain[0, 0] < constrain[1, 0]:
            epsilon = 0.05
        else:
            epsilon = -0.05
        constrainTemp = np.copy(constrain)
        # moving the end points of the constrainTemp into the triangles
        constrainTemp[0, 0] = constrain[0, 0] + epsilon
        constrainTemp[0, 1] = constrain[0, 1] + epsilon * (constrain[1, 1] - constrain[0, 1]) / (
                    constrain[1, 0] - constrain[0, 0])
        constrainTemp[1, 0] = constrain[1, 0] - epsilon
        constrainTemp[1, 1] = constrain[1, 1] - epsilon * (constrain[1, 1] - constrain[0, 1]) / (
                    constrain[1, 0] - constrain[0, 0])

        # init hole triangles and points
        intersectedTriangles.append(self.findTriangle(constrainTemp[0]))
        # holePoints = intersectedTriangles[-1].Points
        pIntersect = constrainTemp[0]
        
        while type(pIntersect) != int:
            constrainTemp[0, 0] = pIntersect[0] + epsilon
            constrainTemp[0, 1] = pIntersect[1] + epsilon * (constrainTemp[1, 1] - pIntersect[1]) / (
                        constrainTemp[1, 0] - pIntersect[0])

            # finding intersection point and the adjacent neighbor
            # self.plotTriangulationWithConstrain(constrainTemp)
            pIntersect, intersectedEdge = intersectedTriangles[-1].findIntersection(constrainTemp)
            if type(pIntersect) != int:
                intersectedEdges.append(intersectedEdge)
                intersectedTriangles.append(intersectedTriangles[-1].Neighbors[intersectedEdge])
        intersectedTriangles.pop(-1)
        return intersectedTriangles, intersectedEdges


    def removeTriangle(self,t):
        """removing from triangulation"""
        for tri in t.Neighbors:
            if tri is not None:
                for j, tri_neighbor in enumerate(tri.Neighbors):
                    if tri_neighbor is not None:
                        if tri_neighbor.ID == t.ID:
                            tri.Neighbors[j] = None
        # del self.Triangles[self.Triangles.index(t)]

    # def mergeHoleTriangulation(self, holeTriangulation):

    def flip(self,tri,edge):
        opositTriangle = tri.Neighbors[edge]
        p = tri.Points[(edge + 2) % 3]
        # building the two triangles the flip made
        farPointInd = np.argmin(np.in1d(opositTriangle.Points[:, 3], tri.Points[:, 3]))
        newTriangle1 = Triangle(opositTriangle.Points[farPointInd],
                                opositTriangle.Points[(farPointInd + 1) % 3], p, self.TriangleId)
        self.TriangleId += 1
        newTriangle2 = Triangle(opositTriangle.Points[(farPointInd - 1) % 3],
                                opositTriangle.Points[farPointInd], p, self.TriangleId)
        self.TriangleId += 1

        # populate neighbors and counter neighbors
        # newTriangle1
        neighbor1 = opositTriangle.Neighbors[farPointInd]
        if neighbor1 is not None:  # case of outer borders
            neighbor1.Neighbors[
                neighbor1.Neighbors.index(opositTriangle)] = newTriangle1  # populate oposit neighbor
        neighbor2 = tri.Neighbors[(np.argmax(tri.Points[:, 3] == p[3]) - 1) % 3]
        if neighbor2 is not None:
            neighbor2.Neighbors[neighbor2.Neighbors.index(tri)] = newTriangle1
        neighbor3 = newTriangle2
        newTriangle1.Neighbors.extend([neighbor1, neighbor2, neighbor3])

        # newTriangle2
        neighbor1 = opositTriangle.Neighbors[(farPointInd - 1) % 3]
        if neighbor1 is not None:  # case of outer borders
            neighbor1.Neighbors[neighbor1.Neighbors.index(opositTriangle)] = newTriangle2
        neighbor2 = newTriangle1
        neighbor3 = tri.Neighbors[np.argmax(tri.Points[:, 3] == p[3])]
        if neighbor3 is not None:
            neighbor3.Neighbors[neighbor3.Neighbors.index(tri)] = newTriangle2
        newTriangle2.Neighbors.extend([neighbor1, neighbor2, neighbor3])

        # adding the new triangles
        self.Triangles.extend((newTriangle1, newTriangle2))

        # delete changed triangles
        del self.Triangles[self.Triangles.index(tri)]
        del self.Triangles[self.Triangles.index(opositTriangle)]

        return newTriangle1, newTriangle2

if __name__ == '__main__':
    points = initializeData()
    d = Delaunay(points)
    d.plotTriangulation()
    d.addConstrains(np.vstack((points[0],points[9],points[2],points[7],points[1],points[4])))


    # temp_points = []
    # filename = 'data.xyz'
    # try:
    #     with open(filename) as file:
    #         lines = file.readlines()
    #         for line in lines:
    #             line = line.split()
    #             if len(line) < 3:
    #                 continue
    #             else:
    #                 temp_points.append(np.array(line[0:3]).astype(float))
    # except:
    #     print('Oops! your file is not supported')
    # points = np.vstack(temp_points)