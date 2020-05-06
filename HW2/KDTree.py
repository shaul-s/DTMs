from numpy import argsort, array, inf


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

    def nnsInRadius(self, pnt, dim, radius, i=0):

        neighbors = []
        self.__nnsInRadius(self.data, pnt, dim, radius, neighbors)

        return neighbors


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
        if p1.shape[0] == p2.shape[0]:
            diff = p1 - p2
            return sum(diff * diff)
        else:
            return inf

    def __nnsInRadius(self, data, pnt, dim, radius, neighbors, i=0):

        if data is not None:
            dist = self.__sqDist(pnt, data[2])
            dx = data[2][i] - pnt[i]
            if dist < radius * radius:
                neighbors.append(data[2])
            i = (i + 1) % dim
            """
            if pnt[i] <= data[2][i] + dist:
                if data[0] is not None:
                    self.__nnsInRadius(data[0], pnt, dim, radius, neighbors, i)
            if pnt[i] >= data[2][i] - dist:
                if data[1] is not None:
                    self.__nnsInRadius(data[1], pnt, dim, radius, neighbors, i)
            """
            for b in [dx < 0] + [dx >= 0] * (dx * dx < radius * radius):
                self.__nnsInRadius(data[b], pnt, dim, neighbors, i)
        else:
            print('Oops! your tree does not have data')


if __name__ == '__main__':
    p1 = array([1, 4, 6])
    p2 = array([2, 1, 3])

    kdtree = KDTree()
