from KDTree import *
from PointCloud import *
from HW2.utils import *
from numpy import array, argsort, deg2rad, arctan
from HW2.GeoEqualCells import *
import time



def naiveSearch(radius, threshold, cloud):
    """

    :param radius:
    :param threshold:
    :param cloud:
    :return:
    """
    terrain = []
    objects = []

    for i, p in enumerate(cloud):
        # distance from p to every point in the cloud
        dist = sqDist(p, cloud)
        # finding the points inside the radius
        pointsInRadius = cloud[dist<radius**2,:]
        # checking if a point is object or terrain
        if isObject(p, pointsInRadius, threshold):
            objects.append(p)
        else:
            terrain.append(p)

    return terrain, objects


if __name__ == '__main__':
    cloud = PointCloud()
    cloud.initializeData()
    # cloud.drawPointCloud()

    cloud.pts = cloud.pts[argsort(cloud.pts[:, 0])]

    g = GeoEqualCells()
    g.initializeGeoEqualCells(cloud.pts,100,[1,1])
    # kdtree = KDTree()
    # kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1])
    #
    # p1 = array([171962.554, 433217.960, 6.609])
    # res = kdtree.nnsInRadius(p1, p1.shape[0], 45.)

    start = time.time()
    # terrain, objects = naiveSearch(5, 65, cloud.pts)
    terrain, objects = g.classifyPoints(5, 65)
    end = time.time()
    print(end)
    cloud.drawFilteredPointCloud(objects, terrain)

    print('hi')
