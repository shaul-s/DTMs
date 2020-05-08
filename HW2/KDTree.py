from numpy import argsort, array, reshape


class KDTree:
    def __init__(self):
        self.data = None

    def initializeKDTree(self, pts, dim):
        """
        Constructing KDTree from the point cloud and storing it as 'data' inside the tree
        :param pts: the point cloud nd.array n*(dim)
        :param dim: K of the tree
        :return KDTree Node where KDTree.data[0] is the left sub-tree, KDTree.data[1] is the right sub-tree
                and KDTree.data[2] is the point within the cell
        """
        self.data = self.__constructKDTree(pts, dim)

    def nnsInRadius(self, pnt, dim, radius):

        neighbors = []
        neighbors = self.__nnsInRadiusNaive(self.data, pnt, dim, radius, neighbors)

        return reshape(array(neighbors), (len(neighbors), dim + 1))

    # ---------------------- Private methods ----------------------
    def __constructKDTree(self, pts, dim, depth=0):

        n = len(pts)
        if n <= 1:
            return [None, None, pts]

        else:
            axis = depth % dim  # getting axis by which we divide the points (0 - x, 1 - y, 2 - z)
            sorted_pts = pts[argsort(pts[:, axis])]

            return [self.__constructKDTree(sorted_pts[:n // 2], dim, depth + 1),
                    self.__constructKDTree(sorted_pts[n // 2 + 1:], dim, depth + 1),
                    sorted_pts[n // 2]]

    def __sqDist(self, p1, p2):
        """
        :return: float squared distance between two points
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return dx * dx + dy * dy

    def __closerDist(self, pivot, p1, p2):
        """
        :return: point that is closest to the pivot
        """
        if p1 is None:
            return p2

        if p2 is None:
            return p1

        d1 = self.__sqDist(pivot, reshape(array(p1), (len(pivot))))
        d2 = self.__sqDist(pivot, reshape(array(p2), (len(pivot))))

        if d1 < d2:
            return p1
        else:
            return p2

    def __nnsInRadiusNaive(self, data, pnt, dim, radius, neighbors, depth=0):

        if data is None or not data[2].size:  # check to see if child is empty
            return neighbors

        axis = depth % dim

        next_branch = None

        data[2] = reshape(data[2], (dim + 1,))

        if self.__sqDist(pnt, data[2]) < radius * radius:
            neighbors.append(data[2])

        if pnt[axis] < data[2][axis]:
            next_branch = data[0]
        else:
            next_branch = data[1]

        return self.__nnsInRadiusNaive(next_branch, pnt, dim, radius, neighbors, depth + 1)

    def __nnsInRadius(self, data, pnt, dim, radius, neighbors, depth=0):
        """
        still under work

        """
        if data is None or not data[2].size:  # check to see if child is empty
            return None

        axis = depth % dim

        next_branch = None
        opposite_branch = None

        data[2] = reshape(data[2], (dim + 1,))

        if pnt[axis] < data[2][axis]:
            next_branch = data[0]
            opposite_branch = data[1]
        else:
            next_branch = data[1]
            opposite_branch = data[0]

        best = self.__closerDist(pnt, self.__nnsInRadius(next_branch, pnt, dim, radius, neighbors, depth + 1), data[2])

        if self.__sqDist(pnt, best) > (pnt[axis] - data[2][axis]) ** 2:
            if self.__sqDist(pnt, best) < radius * radius:
                neighbors.append(data[2])
            best = self.__closerDist(pnt, self.__nnsInRadius(opposite_branch, pnt, dim, radius, neighbors, depth + 1),
                                     best)

        return neighbors


if __name__ == '__main__':
    p1 = array([1, 4, 6])
    p2 = array([2, 1, 3])

    kdtree = KDTree()
