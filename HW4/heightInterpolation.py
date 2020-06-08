import numpy as np
from HW3.Delaunay import *


def isInsideTriangle(t, p):
    """
    True or False: if point 'p' is inside triangle 't'
    """

    def area(x1, y1, x2, y2, x3, y3):
        """
        compute area of triangle given 3 planimetric points
        """
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                    + x3 * (y1 - y2)) / 2.0)

    x, y = p[0:2]
    t_points = t.Points
    x1, y1 = t_points[0, 0:2]
    x2, y2 = t_points[1, 0:2]
    x3, y3 = t_points[2, 0:2]

    # Calculate area of triangle ABC
    A = area(x1, y1, x2, y2, x3, y3)

    # Calculate area of triangle PBC
    A1 = area(x, y, x2, y2, x3, y3)

    # Calculate area of triangle PAC
    A2 = area(x1, y1, x, y, x3, y3)

    # Calculate area of triangle PAB
    A3 = area(x1, y1, x2, y2, x, y)

    # Check if sum of A1, A2 and A3
    # is same as A
    if A == A1 + A2 + A3:
        return True
    else:
        return False


def computeBiQuadraticSurface(points):
    """
    adjusting the bi-quadratic surface 6 parameters - [z=a+bx+cy+dxx+exy+fyy]
    """
    A = np.zeros((len(points), 6))
    A[:, 0] = np.ones(A[:, 0].shape)
    A[:, 1] = points[:, 0]
    A[:, 2] = points[:, 1]
    A[:, 3] = points[:, 0] ** 2
    A[:, 4] = points[:, 0] * points[:, 1]
    A[:, 5] = points[:, 1] ** 2

    b = points[:, 2]

    return np.linalg.solve(np.dot(A.T, A), np.dot(A.T, b))


def computeRegSurface(points):
    """
    adjusting the regular surface 3 parameters - [z=a+bx+cy]
    """
    A = np.zeros((len(points), 3))
    A[:, 0] = np.ones(A[:, 0].shape)
    A[:, 1] = points[:, 0]
    A[:, 2] = points[:, 1]

    b = points[:, 2]

    return np.linalg.solve(np.dot(A.T, A), np.dot(A.T, b))


def interpolateHeight(t, p):
    """
    interpolating points height using heights of neighbor points
    if point is in or on triangle with 3 or more neighbors (more if point is located on one of the triangle edges)
    than we will compute a bi-quadratic surface and get height
    else we will compute a regular 3 parameters surface and get height
    """

    # getting all points relevant to the interpolation
    points = []
    for tri in t:
        for neighbor in tri.Neighbors:
            points.append(neighbor.Points)

    # sorting and arranging points, also subtracting the centroid to avoid numeric problems
    points = np.unique(np.vstack(points), axis=0)[:, 0:3]
    centroid = np.array([np.average(points[:, 0]), np.average(points[:, 1]), np.average(points[:, 2])])
    points = points - centroid

    # check how many points we have for interpolation
    if len(points) >= 6:
        # computing the bi-quadratic surface
        params = computeBiQuadraticSurface(points)
        p = p - centroid[0:2]
        xp = np.array([p[0], p[1], p[0] * p[0], p[0] * p[1], p[1] * p[1]])
        point_height = np.dot(params[1:len(xp) + 1], xp) + params[0] + centroid[-1]
    else:
        params = computeRegSurface(points)
        p = p - centroid[0:2]
        # xp = np.array([p[0], p[1], p[0] * p[0], p[0] * p[1], p[1] * p[1]])
        point_height = np.dot(params[1:len(p) + 1], p) + params[0] + centroid[-1]

    return point_height


if __name__ == '__main__':
    points = initializeData()
    d = Delaunay(points)
    # define point 'p' for height interpolation
    p = np.array([1., 121.5])
    # check if any triangle in the triangulation contains point 'p'
    isIn = []
    for t in d.Triangles:
        if isInsideTriangle(t, p):
            isIn.append(t)
    # if the returned list has no members - program cannot interpolate height
    if not len(isIn):
        print('we cannot interpolate height, the point is not in the triangulation')
    # if the returned list has more than 2 members -
    # the point is a triangle point and therefore already has a known height
    elif len(isIn) > 2:
        print('we cannot interpolate height, the point already has a known height')

    # if the returned list has 1 or 2 members - we interpolate it's height
    else:
        point_height = interpolateHeight(isIn, p)
        print(point_height)


