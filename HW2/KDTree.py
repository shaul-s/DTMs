import vtk
from numpy import argsort


class KDTree:
    def __init__(self):
        self.data = None

    def initializeKDTree(self, pts, dim):

        self.data = self.__constructKDTree(pts, dim)

    # ---------------------- Private methods ----------------------
    def __constructKDTree(self, pts, dim, depth=0):
        n = len(pts)
        if n <= 1:
            return [None, None, pts]

        else:
            col = depth % dim  # getting axis by which we divide the points (0 - x, 1 - y, 2 - z)
            sorted_pts = pts[argsort(pts[:, col])]

            return [self.__constructKDTree(sorted_pts[:n // 2], dim, depth + 1),
                    self.__constructKDTree(sorted_pts[n // 2 + 1:], dim, depth + 1),
                    sorted_pts[n // 2]]




if __name__ == '__main__':
    pass
