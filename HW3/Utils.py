import numpy as np
from tkinter.filedialog import askopenfilenames
# from __future__ import division

def RightLeftCollinear(p1, p2, p3):
    """
    checking kind of turn between 3 points
    :return: 1 for right turn , 2 for left turn, 3 for collinear
    """
    delta = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])
    if delta < 0:
        return 1
    if delta > 0:
        return 2
    if delta == 0:
        return 3



def initializeData():
    """
    getting user input, obtaining cloud point data and storing in a PointCloud object
    """
    point3Dfiles = askopenfilenames(title='Select Input File')

    temp_points = []

    for filename in point3Dfiles:
        try:
            with open(filename) as file:
                lines = file.readlines()
                for line in lines:
                    line = line.split()
                    if len(line) < 3:
                        continue
                    else:
                        temp_points.append(np.array(line[0:3]).astype(float))
        except:
            print('Oops! your file is not supported')

    return np.vstack(temp_points)

def ray_intersect_polygon(p, edge):
    """
    check for intersection between ray from a point and edge of polygon

    :param p: (point) starting point of the ray
    :param edge: edge of polygon
    :return: True or False if the ray intersect the edge
    """
    epsilon = 0.001

    # sort the points [1][1] value
    if edge[0][1] >= edge[1][1]:
        pA, pB = edge[1], edge[0]
    elif edge[0][1] < edge[1][1]:
        pB, pA = edge[1], edge[0]

    # checking if inside borders of edge
    if p[1] > pB[1]:
        return False
    if p[1] < pA[1]:
        return False
    if p[0] > max(pA[0], pB[0]):
        return False

    # handling spacial cases
    if p[1] == edge[0][1] or p[1] == edge[1][1]:  #[1] of point equal to[1] of an edge point
        p[1] += epsilon
        return ray_intersect_polygon(p, edge)
    if p[0] == edge[0][0] and p[0] == edge[1][0]:  # the point is in a vertical edge
        p[0] += epsilon
        return ray_intersect_polygon(p, edge)

    # checking position compare to the edge
    if RightLeftCollinear(pB, pA, p) == 1:
        return True
    elif RightLeftCollinear(pB, pA, p) == 2:
        return False
    else:  # collinear
        p[1] += epsilon
        return ray_intersect_polygon(p, edge)


def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C


def intersection(L1, L2, z):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return np.array([x, y, z, None])
    else:
        return 0

def intersect(l1, l2, z):
    """
    finding point of intersection between two segments
    :param l2: (LineSegment) the other segment
    :return: if their is intersection return point of intersection' if not return 0
    """

    epsilon = pow(10, -5)
    a1 = (l1[1,1] - l1[0,1]) / (l1[1,0] - l1[0,0])
    b1 = l1[0,1] - l1[0,0] * a1
    a2 = (l2[1, 1] - l2[0, 1]) / (l2[1, 0] - l2[0, 0])
    b2 = l2[0, 1] - l2[0, 0] * a2
    if a1 - a2 == 0:
        return 0
    else:
        x_intersection = -(b1 - b2) / (a1 - a2)
        y_intersection = b1 + a1 * x_intersection
        if ((l1[0,0] - x_intersection) * (x_intersection - l1[1,0]) >= 0 - epsilon and
                (l2[0,0] - x_intersection) * (x_intersection - l2[1,0]) >= 0 - epsilon and
                (l1[0,1] - y_intersection) * (y_intersection - l1[1,1]) >= 0 - epsilon and
                (l2[0,1] - y_intersection) * (y_intersection - l2[1,1]) >= 0 - epsilon):
            return np.array([x_intersection, y_intersection, z, None])
        else:
            return 0


def isConvex(points):
    """
    :type points: List[List[int]]
    :rtype: bool
    """
    n = len(points)
    zcrossproduct = None

    for i in range(-2, n - 2):
        x = [points[i, 0], points[i + 1, 0], points[i + 2, 0]]
        y = [points[i, 1], points[i + 1, 1], points[i + 2, 1]]

        dx1 = x[1] - x[0]
        dy1 = y[1] - y[0]

        dx2 = x[2] - x[1]
        dy2 = y[2] - y[1]

        if not zcrossproduct:
            zcrossproduct = dx1 * dy2 - dy1 * dx2
        elif (dx1 * dy2 - dy1 * dx2) * zcrossproduct < 0:
            return False
    return True


def rightTurn (p1, p2, p3):
    """
    defines if their is a right turn between 3 points
    Args:
        p1: np.array 1X3
        p2: np.array 1X3
        p3: np.array 1X3

    Returns: (boolean) true if right turn, false otherwise

    """
    if (p2[0] - p1[0])*(p3[1] - p1[1]) - (p3[0] - p1[0])*(p2[1]-p1[1]) < 0:
        return True
    else:
        return False

def anticlockwise(points):
    """
    arrange the points of polygons anticlockwise

    """

    x_min = points[0,0]
    i_min = 0
    for i in range(len(points)):
        if points[i,0] < x_min:
            x_min = points[i,0]
            i_min = i
    if i_min < len(points)-1:
        if i_min == 0:
            if rightTurn(points[i_min], points[i_min + 1], points[-1]):
                points = list(reversed(points))
        elif rightTurn(points[i_min], points[i_min + 1], points[i_min - 1]):
            points = list(reversed(points))
    elif rightTurn(points[i_min], points[0], points[i_min - 1]):
            points = list(reversed(points))
