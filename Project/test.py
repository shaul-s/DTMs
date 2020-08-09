import numpy as np
from tkinter.filedialog import askopenfilenames
from scipy import spatial as spat
from scipy import interpolate as interp
from matplotlib import pyplot as plt
from vtkTriangulation import *


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


def computeTriArea(triangulation):
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
    for tri in triangulation.simplices:
        tri_pts = np.vstack((triangulation.points[tri[0]], triangulation.points[tri[1]], triangulation.points[tri[2]]))
        areas.append(triArea(tri_pts))

    return np.vstack(areas)


def vertexRemoval(triangulation, vertex_idx):
    """
    detects and removes a vertex from a given set of points. returns an updated delaunay triangulation
    :param triangulation: delaunay triangulation object
    :param vertex_idx: points index in the triangulation
    :return: del triangulation with the vertex removed
    """
    if 0 <= vertex_idx < len(triangulation.points):
        temp_points = np.delete(triangulation.points, vertex_idx, axis=0)
        return spat.Delaunay(temp_points)
    else:
        print('the vertex index is invalid')
        return


def triangleRemoval(triangulation, tris):
    """
    removes triangulation from the delaunay triangulation
    :param triangulation: delaunay triangulation object (As given by scipy)
    :param tri: array nX3 with the point indexes of the triangle to be removed
    :return: del triangulation with the triangulation removed
    """
    tris = np.unique(tris)  # make sure we are not deleting wrong indexes
    temp_points = np.delete(triangulation.points, [*tris], axis=0)
    return spat.Delaunay(temp_points)


def getKminimalIndexes(area, K):
    # Smallest K elements indices
    # using sorted() + lambda + list slicing
    return sorted(range(len(area)), key=lambda sub: area[sub])[:K]


def getCircumcenter(tri):
    """
    circumcenter solution taken from the web
    """
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


def getCentroid(tri):
    """
    get centroid of tri given 3 triangle points
    """
    centroid = np.array([np.average(tri[:, 0]), np.average(tri[:, 1])])
    return centroid


def triangleContraction(triangulation, triangles_to_delete, heights):
    """
    replace chosen triangles with their centroid
    :param triangulation: Scipy triangulation
    :param triangles_to_delete: list of triangles to remove from triangulation
    :param heights: array of the heights of the triangulation points
    :return: Scipy triangulation simplified, hights of the simplify triangulation points
    """

    # computing hight value of the new point by linear interpulation
    interpulation = interp.LinearNDInterpolator(triangulation.points[:,:2], heights)

    # collecting all the points of the triangles that need to be deleted
    # for each of these triangles computing the centroid to replace with
    centroids = []
    new_hights = []
    for tri in triangles_to_delete:
        tri_pts = np.vstack((triangulation.points[tri[0]], triangulation.points[tri[1]], triangulation.points[tri[2]]))
        centroid = getCentroid(tri_pts)
        new_hights.append(interpulation(centroid))
        centroids.append(centroid)

    points_to_delete = np.unique(triangles_to_delete)  # make sure we are not deleting wrong indexes
    
    # deleting the chosen points and their heights
    temp_points = np.delete(triangulation.points, [*points_to_delete], axis=0)
    heights = np.delete(heights, [*points_to_delete], axis=0)

    # adding the centroids of the deleted triangles to the points and heights lists
    temp_points = np.vstack((temp_points, np.vstack(centroids)))
    heights = np.vstack((heights, np.squeeze(np.asarray(new_hights), axis=2)))

    return spat.Delaunay(temp_points), heights

def edgeContraction(triangulation, triangles_to_delete, heights):
    """
    replace edge of chosen triangles with their middle point
    :param triangulation: Scipy triangulation
    :param triangles_to_delete: list of triangles to remove from triangulation
    :param heights: array of the heights of the triangulation points
    :return: Scipy triangulation simplified, hights of the simplify triangulation points
    """
    # collecting all the points of the edges that need to be deleted
    # we deleting the first edge of each triangle that need to be deleted
    # for each of these edges computing the centroid to replace with
    centroids = []
    new_heights = []
    for tri in triangles_to_delete:
        edge_pts = np.vstack((triangulation.points[tri[0]], triangulation.points[tri[1]]))
        centroid = getCentroid(edge_pts)
        new_heights.append((heights[tri[0]] + heights[tri[1]])/2)
        centroids.append(centroid)

    points_to_delete = np.unique(triangles_to_delete[:,:2])  # make sure we are not deleting wrong indexes

    # deleting the chosen points and their heights
    temp_points = np.delete(triangulation.points, [*points_to_delete], axis=0)
    heights = np.delete(heights, [*points_to_delete], axis=0)

    # adding the centroids of the deleted triangles to the points and heights lists
    temp_points = np.vstack((temp_points, np.vstack(centroids)))
    heights = np.vstack((heights, np.asarray(new_heights)))

    return spat.Delaunay(temp_points), heights


if __name__ == '__main__':
    points = initializeData()
    triangulation = spat.Delaunay(points[:, 0: 2])
    heights = points[:,2,None]
    # triangulation = spat.Delaunay(points)
    simplify_precent = 0.5

    vertex = triangulation.points[10, 0:2]
    vertex_idx = int(10)
    tri = triangulation.simplices[10, :]

    # triangulation = vertexRemoval(triangulation, vertex_idx)
    # compute area of every triangle in the triangulation
    area = computeTriArea(triangulation)

    # try to delete 20% of lowest area triangles
    K = int(simplify_precent * len(area))
    idx_for_delete = getKminimalIndexes(area, K)
    triangles_for_deletion = triangulation.simplices[idx_for_delete]

    # triangle removal
    # delete all vertices of the triangles
    triangulation_simplify_20_Tremoval = triangleRemoval(triangulation, triangles_for_deletion)

    # triangle contraction
    # we basically delete all vertices of the triangle but add another point - the centroid of triangle
    triangulation_simplify_20_Tcontraction, heights_simplify_20_Tcontraction = triangleContraction(triangulation, triangles_for_deletion, heights)

    # edge contraction
    # we basically delete one edge of the triangle and replace with it's center
    triangulation_simplify_20_Econtraction, heights_simplify_20_Econtraction = edgeContraction(triangulation,
                                                                                      triangles_for_deletion, heights)

    # plotting

    # full triangulation
    fig1, ax1 = plt.subplots()
    ax1.triplot(triangulation.points[:, 0], triangulation.points[:, 1], triangulation.simplices)
    ax1.plot(triangulation.points[:, 0], triangulation.points[:, 1], 'o', markersize=3)

    # simplified 20% triangulation

    # triangle contraction
    fig2, ax2 = plt.subplots()
    ax2.triplot(triangulation_simplify_20_Tcontraction.points[:, 0], triangulation_simplify_20_Tcontraction.points[:, 1], triangulation_simplify_20_Tcontraction.simplices)
    ax2.plot(triangulation_simplify_20_Tcontraction.points[:, 0], triangulation_simplify_20_Tcontraction.points[:, 1], 'o', markersize=3)

    # edge contraction
    fig3, ax3 = plt.subplots()
    ax3.triplot(triangulation_simplify_20_Econtraction.points[:, 0],
                triangulation_simplify_20_Econtraction.points[:, 1], triangulation_simplify_20_Econtraction.simplices)
    ax3.plot(triangulation_simplify_20_Econtraction.points[:, 0], triangulation_simplify_20_Econtraction.points[:, 1],
             'o', markersize=3)



    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

    plt.show()

    # 3d visualization
    visualizeScipyTriangulation(triangulation,heights)
    visualizeScipyTriangulation(triangulation_simplify_20_Tcontraction,heights_simplify_20_Tcontraction)
    visualizeScipyTriangulation(triangulation_simplify_20_Econtraction,heights_simplify_20_Econtraction)
