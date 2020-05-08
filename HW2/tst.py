from KDTree import *
from PointCloud import *
from HW2.utils import *
from numpy import array, argsort, deg2rad, arctan
import time


def sqDist(p, cloud):
    """
    float squared distance between point and cloud of points
    :param p: point
    :param cloud: cloud of points
    :return: distance

    :type p: array 1X3
    :type cloud: array nX3
    """
    diffx = cloud[:, 0] - p[0]
    diffy = cloud[:, 1] - p[1]
    return diffx * diffx + diffy * diffy


def naiveSearch(radius, threshold, cloud):
    """
    performs a naive linear search across cloud of points to apply the morphological filter
    :param radius: search radius
    :param threshold: threshold slope between 2 points
    :param cloud: cloud of points
    :return:

    :type radius: float
    :type: threshold: deg
    :type: cloud: array nX3
    """
    start = time.time()
    terrain = []
    objects = []
    for p in cloud:
        dist = sqDist(p, cloud)
        pointsInRadius = cloud[dist < radius * radius, :]
        if isObject(p, pointsInRadius, threshold):
            objects.append(p)
        else:
            terrain.append(p)
            # if radius * radius >= dist > 0 and p[2] > q[2]:
            #     slope = computeSlope(p, q)
            #     if slope <= threshold:
            #         terrain.append(p)
            #     else:
            #         objects.append(p)
            #     break
    end = time.time()
    print('Naive search took ', end - start, 'seconds')
    return terrain, objects


def KDTreeSearch(radius, threshold, cloud, kdtree):
    start = time.time()
    terrain = []
    objects = []
    for p in cloud:
        pointsInRadius = kdtree.nnsInRadius(p, cloud.shape[1] - 1, radius)
        # print(pointsInRadius.shape[0])
        if isObject(p, pointsInRadius, threshold):
            objects.append(p)
        else:
            terrain.append(p)

    end = time.time()
    print('KDTree search took ', end - start, 'seconds')
    return terrain, objects


if __name__ == '__main__':
    cloud = PointCloud()
    cloud.initializeData()
    # cloud.drawPointCloud()

    cloud.pts = cloud.pts[argsort(cloud.pts[:, 0])]

    kdtree = KDTree()
    kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1] - 1)
    #
    # p1 = array([171962.554, 433217.960, 6.609])
    # res = kdtree.nnsInRadius(p1, p1.shape[0], 45.)

    # terrain, objects = naiveSearch(5, 65, cloud.pts)

    terrain, objects = KDTreeSearch(5, 65, cloud.pts, kdtree)

    cloud.drawFilteredPointCloud(objects, terrain)

    pnt = array([171930.554, 433217.960, 6.609])

    print('hi')
