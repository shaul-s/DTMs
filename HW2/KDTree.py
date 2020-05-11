from numpy import argsort, array, reshape, vstack
from utils import *


class KDTree:
    def __init__(self):
        self.data = None

    def initializeKDTree(self, pts, dim):
        """
        Constructing KDTree from the point cloud and storing it as 'data' inside the tree
        :param pts: the point cloud nd.array n*(dim)
        :param dim: K of the tree
        :return KDTree Node where KDTree.data[0] is the left sub-tree, KDTree.data[1] is the right sub-tree,
                KDTree.data[2] is the points stored in the leaf and KDTree[3] is the axis split value
        """
        self.data = self.__constructKDTree(pts, dim)

    def nnsInRadius(self, pnt, dim, radius):
        """
        performing a recursive search in a KDTree and returning points within radius current point
        :param pnt: 3d point nX3
        :param dim: dim of tree
        :param radius: search radius
        :return: array of points withing radius of the search point nX3
        """
        # list to hold neighbors
        neighbors = []
        # going in searching the tree recursively to obtain points that are suspected of being within radius
        self.__nnsInRadius(self.data, pnt, dim, radius, neighbors)
        # vstacking the neighbors that we found and computing distances
        neighbors = vstack(neighbors)
        dist = sqDist(pnt, neighbors)
        # returning only neighbors that are within radius of the search point
        return neighbors[dist < radius * radius, :]

    # ---------------------- Private methods ----------------------
    def __constructKDTree(self, pts, dim, depth=0):
        # compute how many points are in the current iteration
        n = len(pts)
        # getting axis by which we divide the points (0 - x, 1 - y)
        axis = depth % dim
        # sorting point according to selected axis
        sorted_pts = pts[argsort(pts[:, axis])]

        # making sure there is a maximum of 10 points in a leaf
        if n <= 10:
            return [None,
                    None,
                    sorted_pts, None]
        else:
            # going in recursively to get - left branch, right branch, points if leaf or None if branch, axis split value
            return [self.__constructKDTree(sorted_pts[:n // 2], dim, depth + 1),
                    self.__constructKDTree(sorted_pts[n // 2:], dim, depth + 1),
                    None, sorted_pts[n // 2][axis]]

    def __nnsInRadius(self, data, pnt, dim, radius, neighbors, depth=0):
        """
        searching recursively in KDTree data to obtain list of points that
        are suspected of being in radius of the search point
        :param data: KDTree data in nested lists
        :param pnt: the point we are currently searching around
        :param dim: dim of tree
        :param radius: search radius
        :param neighbors: list to hold suspected points of being in radius
        :param depth: depth of current iteration for axis computing
        """
        # computing current axis (0 or 1)
        axis = depth % dim
        # break condition - if leaf than return all points in leaf
        if data[0] is None:
            neighbors.append(data[2])
            return
        # if the distance between split axis and axis of point is greater than radius and positive than go into left branch
        if data[3] - pnt[axis] >= radius:
            return self.__nnsInRadius(data[0], pnt, dim, radius, neighbors, depth + 1)
        # if the distance is negative and greater than radius than go into the right branch
        elif -(data[3] - pnt[axis]) > radius:
            return self.__nnsInRadius(data[1], pnt, dim, radius, neighbors, depth + 1)
        # if the pnt is range of both sides than go into both left and right branches
        else:
            return self.__nnsInRadius(data[0], pnt, dim, radius, neighbors, depth + 1), self.__nnsInRadius(
                data[1], pnt, dim, radius, neighbors, depth + 1)


if __name__ == '__main__':
    p1 = array([1, 4, 6])
    p2 = array([2, 1, 3])

    kdtree = KDTree()
