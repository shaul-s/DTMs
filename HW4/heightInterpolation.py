import numpy as np
from matplotlib import pyplot as plt
from HW3.Delaunay import *
from HW4.elevationCrossSection import *
from HW4.vtkTriangulation import *

def convexHull(d):
    """
    return convex hull of a given 'd', delaunay triangulation
    """
    convex = []
    for t in d.Triangles:
        if None in t.Neighbors:
            convex.append(t.Edges[t.Neighbors.index(None)])

    return np.vstack(convex)


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


def isInterp(isIn, p):
    """
    t - list of triangles that our point was found in (list could be empty)
    this method checks if height interpolation is possible
    """
    # if the returned list has no members - program cannot interpolate height
    if not len(isIn):
        print('we cannot interpolate height, the point is not in the triangulation')
    # if the returned list has more than 2 members -
    # the point is a triangle point and therefore already has a known height
    elif len(isIn) > 2:
        print('we cannot interpolate height, the point already has a known height')
    # if the returned list has 1 or 2 members - we interpolate it's height
    else:
        p_height = interpolateHeight(isIn, p)
        print('point', p, 'height is:', np.round(p_height, 4), '[m]')
        return np.hstack((p, p_height))


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
            if neighbor is not None:
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
    p = np.array([3, 116.5])
    p2 = np.array([8.5, 120.5])
    # check if any triangle in the triangulation contains point 'p'
    isIn = []
    isIn2 = []
    for t in d.Triangles:
        if isInsideTriangle(t, p):
            isIn.append(t)
        if isInsideTriangle(t, p2):
            isIn2.append(t)

    p_new = isInterp(isIn, p)
    p2_new = isInterp(isIn2, p2)

    # computing elevation cross section given 2 points
    intersecting_tri = []
    inter_points = []
    for t in d.Triangles:
        for edge in t.Edges:
            lines = np.vstack((edge[:, 0:2], p, p2))
            if doIntersect(lines):
                intersecting_tri.append(t)
                inter_points.append(interPoint(lines))

    inter_points = np.vstack(inter_points)
    inter_points = inter_points[inter_points[:, 0].argsort()]
    elevation_points = [p_new, p2_new]
    for i, p in enumerate(inter_points):
        if i == len(inter_points) - 1:
            break
        if p[0] == inter_points[i + 1, :][0]:
            elevation_points.append(isInterp([intersecting_tri[i], intersecting_tri[i + 1]], p))
        # else:
        #     elevation_points.append(isInterp([intersecting_tri[i]], p))

    elevation_points = np.vstack([x for x in elevation_points if x is not None])
    elevation_points = elevation_points[elevation_points[:, 1].argsort()]

    fig, ax = plt.subplots()

    start_pnt_height = p_new[-1]
    for i, point in enumerate(elevation_points):
        if point[-1] > start_pnt_height:
            ax.scatter(point[1], point[2], color='r')
        else:
            ax.scatter(point[1], point[2], color='g')

    ax.plot(elevation_points[:, 1], elevation_points[:, 2])
    for i, point in enumerate(elevation_points):
        ax.annotate("({},{},{})".format(np.round(point[0], 2), np.round(point[1], 2), np.round(point[2], 2)),
                    (point[1], point[2]))
    ax.set_title('Elevation Cross Section')
    ax.set_xlabel('Y[m]')
    ax.set_ylabel('Z[m]')
    plt.show()

    # visualizeTriangulation(points, d)

    # compute convex of triangulation
    # ch = convexHull(d)
    # plt.scatter(ch[:, 0], ch[:, 1], color='r')
    # d.plotTriangulation()
    # plt.show()

    print('hi')
