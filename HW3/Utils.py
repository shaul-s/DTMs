from Point3D import Point3D




def ray_intersect_polygon(p, edge):
    """
    check for intersection between ray from a point and edge of polygon

    :param p: (point) starting point of the ray
    :param edge: edge of polygon
    :return: True or False if the ray intersect the edge
    """
    epsilon = 0.001
    # pB = Point3D
    # pA = Point3D

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
        return ray_intersect_polygon(Point3D(p[0], p[1] + epsilon), edge)
    if p[0] == edge[0][0] and p[0] == edge[1][0]:  # the point is in a vertical edge
        return ray_intersect_polygon(Point3D(p[0] + epsilon, p[1]), edge)

    # checking position compare to the edge
    if RightLeftCollinear(pB, pA, p) == 1:
        return True
    elif RightLeftCollinear(pB, pA, p) == 2:
        return False
    else:  # collinear
        return ray_intersect_polygon(Point3D(p[0], p[1] + epsilon), edge)


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
