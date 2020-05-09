from KDTree import *
from PointCloud import *
from HW2.utils import *
from numpy import array, argsort, deg2rad, arctan
from HW2.GeoEqualCells import *
import time


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
        # distance from p to every point in the cloud
        dist = sqDist(p, cloud)
        # finding the points inside the radius
        pointsInRadius = cloud[dist < radius * radius, :]
        # checking if a point is object or terrain
        if isObject(p, pointsInRadius, threshold):
            objects.append(p)
        else:
            terrain.append(p)

    end = time.time()
    print('naive search took ', end - start, 'seconds')
    return terrain, objects


def KDTreeSearch(radius, threshold, cloud, kdtree):
    start = time.time()
    terrain = []
    objects = []
    for p in cloud:
        pointsInRadius = kdtree.nnsInRadius(p, cloud.shape[1] - 1, radius)
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

    g = GeoEqualCells()
    g.initializeGeoEqualCells(cloud.pts, 10, [1, 1])
    # kdtree = KDTree()
    # kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1] - 1)
    #
    # p1 = array([171962.554, 433217.960, 6.609])
    # res = kdtree.nnsInRadius(p1, p1.shape[0], 45.)

    start = time.time()
    # terrain, objects = naiveSearch(5, 65, cloud.pts)
    # terrain, objects = KDTreeSearch(10, 15, cloud.pts, kdtree)
    terrain, objects = g.classifyPoints(5, 65)
    end = time.time()
    print(end - start)
    cloud.drawFilteredPointCloud(objects, terrain)

    pnt = array([171930.554, 433217.960, 6.609])

    print('hi')
