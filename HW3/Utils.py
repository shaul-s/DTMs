import numpy as np
from tkinter.filedialog import askopenfilenames


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
