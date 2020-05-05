import numpy as np


def isObject(p,points, threshold):
    """
    checking if a point is an object point or ground point according to slope
    :param p: tested point
    :param points: neighboring points
    :param threshold: maximum slope for a ground point
    :return: object point or not(ground point)

    :type p: array 1X3:(x,y,z)
    :type points: array nX3
    :type threshold: float

    """
    for q in points:
        slope = computeSlope(p,q)
        if slope >= threshold:
            return True
    return False


def computeSlope(p,q):
    """
    compute slope between two points
    :param p: point3D
    :param q: point3D
    :return: slope
    """
    return np.abs(p[2]-q[2])/distance(p,q)

def distance(p,q):
    """
    horizontal distance between two points
    """
    return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

if __name__ == '__main__':
    p1 = np.array((0,0,0))
    p2 = np.array((1,1,1))
    p3 = np.array((1,0,1))
    points = np.vstack((p1,p2,p3))
    print(isObject(p1,points,2))
    # print(computeSlope(p1,p2))
