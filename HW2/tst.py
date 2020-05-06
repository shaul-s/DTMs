from KDTree import *
from PointCloud import *
from HW2.utils import *
from numpy import array, argsort, deg2rad, arctan


def sqDist(p1, p2):
    """
    :return: float squared distance between two points
    """
    diff = p1[0:2] - p2[0:2]
    return sum(diff * diff)


def naiveSearch(radius, threshold, cloud):
    terrain = []
    objects = []
    for i, p in enumerate(cloud):
        for q in cloud:
            dist = sqDist(p, q)
            if radius * radius >= dist > 0 and p[2] > q[2]:
                slope = computeSlope(p, q)
                if slope <= threshold:
                    terrain.append(p)
                else:
                    objects.append(p)
                break

    return terrain, objects


if __name__ == '__main__':
    cloud = PointCloud()
    cloud.initializeData()
    # cloud.drawPointCloud()

    cloud.pts = cloud.pts[argsort(cloud.pts[:, 0])]

    kdtree = KDTree()
    kdtree.initializeKDTree(cloud.pts, cloud.pts.shape[1])

    p1 = array([171962.554, 433217.960, 6.609])
    # res = kdtree.nnsInRadius(p1, p1.shape[0], 45.)

    terrain, objects = naiveSearch(5, deg2rad(arctan(65)), cloud.pts)

    cloud.drawFilteredPointCloud(objects, terrain)

    print('hi')
