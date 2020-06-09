import numpy as np


def orientation(p1, p2, p3):
    """
    method for finding orientation of 3 points
    p = np.array 1x2
    """
    o = (float(p2[1] - p1[1]) * (p3[0] - p2[0])) - (float(p2[0] - p1[0]) * (p3[1] - p2[1]))
    if o > 0:  # delta > 0 - ccw (2)
        return 2  # delta < 0 - cw   (1)
    if o < 0:  # delta = 0 - collinear (0)
        return 1
    return 0


def doIntersect(lines):
    """
    method for checking if two linesegments l1 and l2 are intersecting:
    let p1,p2 and q1,q2 define two line segments
    if the orientation of 3 points (2 from one line and 1 from the other line)
    is different, than the two lines intersect !
    this is without checking any special cases like collinearity or congruence
    """
    p1, p2, q1, q2 = lines[0, :], lines[1, :], lines[2, :], lines[3, :]
    o1 = orientation(p1, p2, q1)
    o2 = orientation(p1, p2, q2)
    o3 = orientation(q1, q2, p1)
    o4 = orientation(q1, q2, p2)

    if o1 != o2 and o3 != o4:
        return True
    return False


def interPoint(lines):
    """
    method that return intersection points between two intersecting lines
    line1 - p1,p2; line2 - q1, q2
    """
    p1, p2, q1, q2 = lines[0, :], lines[1, :], lines[2, :], lines[3, :]
    a1 = (p2[1] - p1[1]) / (p2[0] - p1[0])
    a2 = (q2[1] - q1[1]) / (q2[0] - q1[0])
    b1 = p1[1] - a1 * p1[0]
    b2 = q1[1] - a2 * q1[0]

    x = -(b1 - b2) / (a1 - a2)
    y = b1 + a1 * x
    return np.array([x, y])


if __name__ == '__main__':
    pass
