import numpy as np
from tkinter.filedialog import askopenfilenames
from scipy import spatial as spat
from matplotlib import pyplot as plt


def initializeData():
    """
    getting user input, obtaining cloud point data and storing in a PointCloud object
    """
    point3Dfiles = askopenfilenames(title='Select Input File')

    temp_points = []

    for filename in point3Dfiles:
        try:
            with open(filename) as file:
                lines = file.readlines()
                for line in lines:
                    line = line.split()
                    if len(line) < 3:
                        continue
                    else:
                        temp_points.append(np.array(line[0:3]).astype(float))
        except:
            print('Oops! your file is not supported')

    return np.vstack(temp_points)


def computeTriArea(del_tri):
    def triArea(tri_pts):
        """
        return triangle 2d area given 3 points
        :param tri_pts: array 3X3 with points
        :return: area of the triangle
        """
        x1, y1, x2, y2, x3, y3 = tri_pts[0, 0], tri_pts[0, 1], tri_pts[1, 0], tri_pts[1, 1], tri_pts[2, 0], tri_pts[
            2, 1]
        return abs(0.5 * (((x2 - x1) * (y3 - y1)) - ((x3 - x1) * (y2 - y1))))

    areas = []
    for tri in del_tri.simplices:
        tri_pts = np.vstack((del_tri.points[tri[0]], del_tri.points[tri[1]], del_tri.points[tri[2]]))
        areas.append(triArea(tri_pts))

    return np.vstack(areas)


def vertexRemoval(del_tri, vertex_idx):
    """
    detects and removes a vertex from a given set of points. returns an updated delaunay triangulation
    :param del_tri: delaunay triangulation object
    :param vertex_idx: points index in the triangulation
    :return: del triangulation with the vertex removed
    """
    if 0 <= vertex_idx < len(del_tri.points):
        temp_points = np.delete(del_tri.points, vertex_idx, axis=0)
        return spat.Delaunay(temp_points)
    else:
        print('the vertex index is invalid')
        return


def triangleRemoval(del_tri, tris):
    """
    removes triangles from the delaunay triangulation
    :param del_tri: delaunay triangulation object (As given by scipy)
    :param tri: array nX3 with the point indexes of the triangle to be removed
    :return: del triangulation with the triangles removed
    """
    tris = np.unique(tris)  # make sure we are not deleting wrong indexes
    temp_points = np.delete(del_tri.points, [*tris], axis=0)
    return spat.Delaunay(temp_points)


def getKminimalIndexes(array, K):
    # Smallest K elements indices
    # using sorted() + lambda + list slicing
    return sorted(range(len(area)), key=lambda sub: area[sub])[:K]


def getCircumcenter(tri):
    center = np.zeros(2)
    #
    #  Circumradius.
    #
    tri = tri.T
    a = np.sqrt((tri[0, 0] - tri[0, 1]) ** 2 + (tri[1, 0] - tri[1, 1]) ** 2)
    b = np.sqrt((tri[0, 1] - tri[0, 2]) ** 2 + (tri[1, 1] - tri[1, 2]) ** 2)
    c = np.sqrt((tri[0, 2] - tri[0, 0]) ** 2 + (tri[1, 2] - tri[1, 0]) ** 2)

    bot = (a + b + c) * (- a + b + c) * (a - b + c) * (a + b - c)

    if bot <= 0.0:
        r = - 1.0
        return center

    r = a * b * c / np.sqrt(bot)
    #
    #  Circumcenter.
    #
    f = np.zeros(2)

    f[0] = (tri[0, 1] - tri[0, 0]) ** 2 + (tri[1, 1] - tri[1, 0]) ** 2
    f[1] = (tri[0, 2] - tri[0, 0]) ** 2 + (tri[1, 2] - tri[1, 0]) ** 2

    top = np.zeros(2)

    top[0] = (tri[1, 2] - tri[1, 0]) * f[0] - (tri[1, 1] - tri[1, 0]) * f[1]
    top[1] = - (tri[0, 2] - tri[0, 0]) * f[0] + (tri[0, 1] - tri[0, 0]) * f[1]

    det = (tri[1, 2] - tri[1, 0]) * (tri[0, 1] - tri[0, 0]) \
          - (tri[1, 1] - tri[1, 0]) * (tri[0, 2] - tri[0, 0])

    center[0] = tri[0, 0] + 0.5 * top[0] / det
    center[1] = tri[1, 0] + 0.5 * top[1] / det

    return center


def triangleContraction(del_tri, tris):
    circumcenters = []
    for tri in tris:
        tri_pts = np.vstack((del_tri.points[tri[0]], del_tri.points[tri[1]], del_tri.points[tri[2]]))
        circumcenters.append(getCircumcenter(tri_pts))
    tris = np.unique(tris)  # make sure we are not deleting wrong indexes
    temp_points = np.delete(del_tri.points, [*tris], axis=0)
    temp_points = np.vstack((temp_points, np.vstack(circumcenters)))

    return spat.Delaunay(temp_points)


if __name__ == '__main__':
    points = initializeData()
    del_tri = spat.Delaunay(points[:, 0: 2])

    vertex = del_tri.points[10, 0:2]
    vertex_idx = int(10)
    tri = del_tri.simplices[10, :]

    # del_tri = vertexRemoval(del_tri, vertex_idx)
    # compute area of every triangle in the triangulation
    area = computeTriArea(del_tri)

    # try to delete 20% of lowest area triangles
    K = int(0.3 * len(area))
    idx_for_delete = getKminimalIndexes(area, K)
    triangles_for_deletion = del_tri.simplices[idx_for_delete]
    # del_tri = triangleRemoval(del_tri, triangles_for_deletion)

    # try to delete 20% of lowest area using triangle contraction
    # we basically delete all vertexes of the triangle but add another point - the circumcenter of triangle
    del_tri = triangleContraction(del_tri, triangles_for_deletion)

    fig, ax = plt.subplots()
    ax.triplot(del_tri.points[:, 0], del_tri.points[:, 1], del_tri.simplices)
    ax.plot(del_tri.points[:, 0], del_tri.points[:, 1], 'o', markersize=5)

    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

    plt.show()
