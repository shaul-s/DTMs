import numpy as np


def isObject(p, points, threshold):
    """
    checking if a point is an object point or ground point according to slope
    :param p: tested point
    :param points: neighboring points
    :param threshold: maximum slope for a ground point [deg]
    :return: object point or not(ground point)

    :type p: array 1X3:(x,y,z)
    :type points: array nX3
    :type threshold: float

    """
    threshold = np.radians(threshold)
    slope = computeSlope(p, points)
    return any(-slope >= threshold)



def computeSlope(p, points):
    """
    compute slope between two points
    :param p: point3D
    :param q: point3D
    :return: slope
    """
    return (points[:,2] - p[2]) / distanceVector(p, points)


def distanceVector(p, points):
    """
    horizontal distance between two points
    """
    diffx = points[:, 0] - p[0]
    diffy = points[:, 1] - p[1]
    return np.sqrt(diffx ** 2 + diffy ** 2)


def distance(p, q):
    """
    horizontal distance between two points
    """
    return np.linalg.norm(p[0:2]-q[0:2])


def sqDist(p, cloud):
    """
    :return: float squared distance between two points
    """
    if cloud.size <= 3:
        diffx = cloud[0] - p[0]
        diffy = cloud[1] - p[1]
        return diffx ** 2 + diffy ** 2
    diffx = cloud[:,0] - p[0]
    diffy = cloud[:,1] - p[1]
    return diffx**2 + diffy**2


if __name__ == '__main__':
    p1 = np.array((0, 0, 0))
    p2 = np.array((1, 1, 1))
    p3 = np.array((1, 0, 1))
    points = np.vstack((p1, p2, p3))
    print(isObject(p1, points, 2))
    # print(computeSlope(p1,p2))
