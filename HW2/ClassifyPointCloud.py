from KDTree import *
from PointCloud import *
from GeoEqualCells import *
from utils import *
from numpy import array, argsort, deg2rad, arctan
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

    return terrain, objects, end - start


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

    return terrain, objects, end - start


def classifyPointCloud(radius, threshold, method):
    """

    :param radius: search radius
    :param threshold: max slope between points
    :param method: the data structure the user would like to use

    :type radius: float
    :type threshold: deg
    :type method: string

    :return:
    """
    cloud = PointCloud()
    cloud.initializeData()

    if method == 'KDTree':
        kdtree = KDTree()
        kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1] - 1)
        return KDTreeSearch(radius, threshold, cloud.pts, kdtree)

    elif method == 'Naive':
        return naiveSearch(radius, threshold, cloud.pts)

    elif method == 'EqualCells':
        g = GeoEqualCells()
        g.initializeGeoEqualCells(cloud.pts, 100, [1, 1])
        # terrain, objects, rtime = g.classifyPoints(radius, threshold)
        return g.classifyPoints(radius, threshold)

    else:
        return print('method needs to be of the following: Naive, EqualCells or KDTree')


def drawClassification(terrain, objects, rtime, flag):
    """

    :param terrain: terrain cloud of points
    :param objects: objects cloud of points
    :param rtime: run time of the search in seconds
    :param flag: what does the user want to be drawn - 'terrain', 'objects' or 'all'

    :type terrain: array nX3
    :type objects: array nX3
    :type rtime: float
    :type flag: string

    :return: void
    """
    c = PointCloud()
    c.drawFilteredPointCloud(objects, terrain, flag)
    print('Classification took ', rtime, 'seconds')


if __name__ == '__main__':
    terrain, objects, rtime = classifyPointCloud(5, 65, 'KDTree')
    drawClassification(terrain, objects, rtime, 'terrain')
