from KDTree import *
from PointCloud import *
from HW2.utils import *
from numpy import array, argsort, deg2rad, arctan


def sqDist(p, cloud):
    """
    :return: float squared distance between two points
    """
    diffx = cloud[:,0] - p[0]
    diffy = cloud[:,1] - p[1]
    return diffx**2 + diffy**2


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

    # kdtree = KDTree()
    # kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1])
    #
    # p1 = array([171962.554, 433217.960, 6.609])
    # res = kdtree.nnsInRadius(p1, p1.shape[0], 45.)

    terrain, objects = naiveSearch(5, 65, cloud.pts)

    cloud.drawFilteredPointCloud(objects, terrain)

    print('hi')
