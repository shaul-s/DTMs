import numpy as np
from HW3.Triangle import Triangle


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

def isInsideCircle(tri, p):
    """Check if point p is inside of circumcircle around the triangle tri"""

    col1 = tri.Points[:,0]
    col2 = tri.Points[:,1]
    col3 = tri.Points[:,0]**2 + tri.Points[:,1]**2
    col4 = np.ones((4,1))
    m = np.hstack((col1,col2,col3,col4))
    return np.linalg.det(m) > 0
