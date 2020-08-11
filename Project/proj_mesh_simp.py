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


def getKminimalIndexes(area, K):
    # Smallest K elements indices
    # using sorted() + lambda + list slicing
    return sorted(range(len(area)), key=lambda sub: area[sub])[:K]


def getCircumcenter(tri):
    """
    circumcenter solution taken from the web
    """
    center = np.zeros(2)

    tri = tri.T
    a = np.sqrt((tri[0, 0] - tri[0, 1]) ** 2 + (tri[1, 0] - tri[1, 1]) ** 2)
    b = np.sqrt((tri[0, 1] - tri[0, 2]) ** 2 + (tri[1, 1] - tri[1, 2]) ** 2)
    c = np.sqrt((tri[0, 2] - tri[0, 0]) ** 2 + (tri[1, 2] - tri[1, 0]) ** 2)

    bot = (a + b + c) * (- a + b + c) * (a - b + c) * (a + b - c)

    if bot <= 0.0:
        r = - 1.0
        return center

    r = a * b * c / np.sqrt(bot)

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


def interpHeight(triangulation, heights, deleted_point):
    """
    given a simplified triangulation, return the heights of the deleted points
    :param triangulation: delaunay triangulation object
    :param heights: the heights of points in the triangulation
    :param deleted_point: points that we want to interpolate height
    :return: new heights for the deleted points
    """
    interpolation = interp.LinearNDInterpolator(triangulation.points[:, :2], heights)
    return interpolation(deleted_point[:, :2])


def vertexRemoval(triangulation, heights, K):
    """
    detects and removes a vertex from a given set of points. returns an updated delaunay triangulation
    :param triangulation: delaunay triangulation object
    :param vertex_idx: points index in the triangulation
    :return: del triangulation with the vertex removed
    """
    deleted_points = []
    while triangulation.simplices.shape[0] > K:
        area = computeTriArea(triangulation)
        tri_to_delete = triangulation.simplices[np.argmin(area)]

        points_to_delete = tri_to_delete[0]  # indices of the points of the triangle we deleting
        deleted_points.append(np.hstack((triangulation.points[points_to_delete], heights[points_to_delete])))

        # deleting the chosen points and their heights
        temp_points = np.delete(triangulation.points, points_to_delete, axis=0)
        heights = np.delete(heights, points_to_delete, axis=0)

        triangulation = spat.Delaunay(temp_points)
        print("number of triangles is ", triangulation.simplices.shape[0])  # progress indication

    return triangulation, heights, np.vstack(deleted_points)


def triangleRemoval(triangulation, heights, K):
    """
    removes triangulation from the delaunay triangulation
    :param triangulation: delaunay triangulation object (As given by scipy)
    :param tri: array nX3 with the point indexes of the triangle to be removed
    :return: del triangulation with the triangulation removed
    """
    deleted_points = []
    while triangulation.simplices.shape[0] > K:
        area = computeTriArea(triangulation)
        tri_to_delete = triangulation.simplices[np.argmin(area)]

        points_to_delete = tri_to_delete  # indices of the points of the triangle we deleting
        deleted_points.append(
            np.hstack((np.take(triangulation.points, tri_to_delete, axis=0), np.take(heights, tri_to_delete, axis=0))))

        # deleting the chosen points and their heights
        temp_points = np.delete(triangulation.points, [*points_to_delete], axis=0)
        heights = np.delete(heights, [*points_to_delete], axis=0)

        triangulation = spat.Delaunay(temp_points)
        print("number of triangles is ", triangulation.simplices.shape[0])  # progress indication

    return triangulation, heights, np.vstack(deleted_points)


def triangleContraction(triangulation, heights, K):
    """
    replace chosen triangles with their centroid
    :param triangulation: Scipy triangulation
    :param triangles_to_delete: list of triangles to remove from triangulation
    :param heights: array of the heights of the triangulation points
    :return: Scipy triangulation simplified, heights of the simplify triangulation points
    """
    deleted_points = []
    # computing height value of the new point by linear interpolation
    interpulation = interp.LinearNDInterpolator(triangulation.points[:, :2], heights)

    # collecting all the points of the triangles that need to be deleted
    # for each of these triangles computing the centroid to replace with

    while triangulation.simplices.shape[0] > K:
        area = computeTriArea(triangulation)
        tri_to_delete = triangulation.simplices[np.argmin(area)]
        tri_pts = np.vstack((triangulation.points[tri_to_delete[0]], triangulation.points[tri_to_delete[1]],
                             triangulation.points[tri_to_delete[2]]))
        centroid = getCentroid(tri_pts)
        new_height = interpulation(centroid)

        points_to_delete = tri_to_delete  # indices of the points of the triangle we deleting
        deleted_points.append(
            np.hstack((np.take(triangulation.points, tri_to_delete, axis=0), np.take(heights, tri_to_delete, axis=0))))
        # deleting the chosen points and their heights
        temp_points = np.delete(triangulation.points, [*points_to_delete], axis=0)
        heights = np.delete(heights, [*points_to_delete], axis=0)

        # adding the centroid of the deleted triangle to the points and heights lists
        temp_points = np.vstack((temp_points, centroid))
        heights = np.vstack((heights, new_height))

        # temp_points = triangulation.points[1:,:]
        triangulation = spat.Delaunay(temp_points)
        print("number of triangles is ", triangulation.simplices.shape[0])  # progress indication



    return triangulation, heights, np.vstack(deleted_points)




def edgeContraction(triangulation, heights, K):
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
    deleted_points = []
    while triangulation.simplices.shape[0] > K:
        area = computeTriArea(triangulation)
        tri_to_delete = triangulation.simplices[np.argmin(area)]
        edge_pts = np.vstack((triangulation.points[tri_to_delete[0]], triangulation.points[tri_to_delete[1]]))
        centroid = getCentroid(edge_pts)
        new_height = (heights[tri_to_delete[0]] + heights[tri_to_delete[1]]) / 2

        points_to_delete = tri_to_delete[:2]  # indices of the points of the edge we deleting
        deleted_points.append(
            np.hstack((np.take(triangulation.points, tri_to_delete, axis=0), np.take(heights, tri_to_delete, axis=0))))
        # deleting the chosen points and their heights
        temp_points = np.delete(triangulation.points, [*points_to_delete], axis=0)
        heights = np.delete(heights, [*points_to_delete], axis=0)

        # adding the centroid of the deleted triangle to the points and heights lists
        temp_points = np.vstack((temp_points, centroid))
        heights = np.vstack((heights, new_height))

        triangulation = spat.Delaunay(temp_points)

        print("number of triangles is ", triangulation.simplices.shape[0])  # progress indication

    return triangulation, heights, np.vstack(deleted_points)


def mesh_simp_wrapper(triangulation, heights, simp_percent=0.2, method='vertex removal'):
    """

    :param triangulation: scipy delaunay triangulation object
    :param heights: heights of the 2d simplices of the triangulation
    :param method: the method of simplification in one of the strings:
                    "vertex removal", "triangle removal", "triangle contraction", "edge contraction"
                    default method is vertex removal
    :param simp_percent: percent of triangles to simplify. by default it is 20%
    :return: simplified triangulation, hights and deleted points
    """
    K = int((1 - simp_percent) * len(triangulation.simplices))

    if method == 'vertex removal':
        # vertex removal
        triangulation_simplify, heights_simplify, deleted_points = vertexRemoval(
            triangulation, heights, K)
    elif method == 'triangle removal':
        # triangle removal
        triangulation_simplify, heights_simplify, deleted_points = triangleRemoval(
            triangulation, heights, K)
    elif method == 'triangle contraction':
        # triangle contraction
        triangulation_simplify, heights_simplify, deleted_points = triangleContraction(
            triangulation,
            heights, K)
    elif method == 'edge contraction':
        # edge contraction
        triangulation_simplify, heights_simplify, deleted_points = edgeContraction(
            triangulation, heights, K)
    else:
        print('methods are: vertex removal, triangle removal, triangle contraction, edge contraction ')
        return

    return triangulation_simplify, heights_simplify, deleted_points

if __name__ == '__main__':
    points = initializeData()
    triangulation = spat.Delaunay(points[:, 0: 2])
    heights = points[:, 2, None]
    # triangulation = spat.Delaunay(points)
    

    vertex = triangulation.points[10, 0:2]
    vertex_idx = int(10)
    tri = triangulation.simplices[10, :]

    # triangulation = vertexRemoval(triangulation, vertex_idx)
    # compute area of every triangle in the triangulation
    area = computeTriArea(triangulation)



    simplify_precent = 0.1

    # vertex removal
    # delete vertex of each triangle
    triangulation_simplify_Vremoval, heights_simplify_Vremoval, deleted_points_Vremoval = mesh_simp_wrapper(
        triangulation, heights, simplify_precent, method='vertex removal')

    # test
    # del_points_heights_Vremoval = interpHeight(triangulation_simplify_Vremoval, heights_simplify_Vremoval,
    #                                            deleted_points_Vremoval)
    # indices = ~np.isnan(del_points_heights_Vremoval)
    # del_points_heights_Vremoval = del_points_heights_Vremoval[indices]  # removing nan values
    # deleted_points_Vremoval = deleted_points_Vremoval[:,2,None][indices]


    # triangle removal
    # delete all vertices of the triangles
    triangulation_simplify_Tremoval, heights_simplify_Tremoval, deleted_points_Tremoval = mesh_simp_wrapper(
        triangulation, heights, simplify_precent, method='triangle removal' )

    # triangle contraction
    # we basically delete all vertices of the triangle but add another point - the centroid of triangle
    triangulation_simplify_Tcontraction, heights_simplify_Tcontraction, deleted_points_Tcontraction = mesh_simp_wrapper(
        triangulation,
        heights, simplify_precent, method='triangle contraction')

    # edge contraction
    # we basically delete one edge of the triangle and replace with it's center
    triangulation_simplify_Econtraction, heights_simplify_Econtraction, deleted_points_Econtraction = mesh_simp_wrapper(
        triangulation, heights, simplify_precent, method='edge contraction')


    ### Analysis ###

    # height interpolation
    # vertex removal
    del_points_heights_Vremoval = interpHeight(triangulation_simplify_Vremoval, heights_simplify_Vremoval,
                                               deleted_points_Vremoval)
    indices = ~np.isnan(del_points_heights_Vremoval)
    del_points_heights_Vremoval = del_points_heights_Vremoval[indices]  # removing nan values
    deleted_points_Vremoval = deleted_points_Vremoval[:, 2, None][indices]

    # triangle removal
    del_points_heights_Tremoval = interpHeight(triangulation_simplify_Tremoval, heights_simplify_Tremoval,
                                               deleted_points_Tremoval)
    indices = ~np.isnan(del_points_heights_Tremoval)
    del_points_heights_Tremoval = del_points_heights_Tremoval[indices]  # removing nan values
    deleted_points_Tremoval = deleted_points_Tremoval[:, 2, None][indices]

    # triangle contraction
    del_points_heights_Tcontraction = interpHeight(triangulation_simplify_Tcontraction,
                                                   heights_simplify_Tcontraction,
                                                   deleted_points_Tcontraction)
    indices = ~np.isnan(del_points_heights_Tcontraction)
    del_points_heights_Tcontraction = del_points_heights_Tcontraction[indices]  # removing nan values
    deleted_points_Tcontraction = deleted_points_Tcontraction[:, 2, None][indices]

    # edge contraction
    del_points_heights_Econtraction = interpHeight(triangulation_simplify_Econtraction,
                                                   heights_simplify_Econtraction,
                                                   deleted_points_Econtraction)
    indices = ~np.isnan(del_points_heights_Econtraction)
    del_points_heights_Econtraction = del_points_heights_Econtraction[indices]  # removing nan values
    deleted_points_Econtraction = deleted_points_Econtraction[:, 2, None][indices]



    # heights differences
    heights_diff_Vremoval = np.abs(deleted_points_Vremoval-del_points_heights_Vremoval)
    heights_diff_Tremoval = np.abs(deleted_points_Tremoval-del_points_heights_Tremoval)
    heights_diff_Tcontraction = np.abs(deleted_points_Tcontraction-del_points_heights_Tcontraction)
    heights_diff_Econtraction = np.abs(deleted_points_Econtraction-del_points_heights_Econtraction)


    std_Vremoval = np.std(heights_diff_Vremoval)
    std_Tremoval = np.std(heights_diff_Tremoval)
    std_Tcontraction = np.std(heights_diff_Tcontraction)
    std_Econtraction = np.std(heights_diff_Econtraction)

    average_Vremoval = np.average(heights_diff_Vremoval)
    average_Tremoval = np.average(heights_diff_Tremoval)
    average_Tcontraction = np.average(heights_diff_Tcontraction)
    average_Econtraction = np.average(heights_diff_Econtraction)

    median_Vremoval = np.median(heights_diff_Vremoval)
    median_Tremoval = np.median(heights_diff_Tremoval)
    median_Tcontraction = np.median(heights_diff_Tcontraction)
    median_Econtraction = np.median(heights_diff_Econtraction)

    maxDiff_Vremoval = np.max(heights_diff_Vremoval)
    maxDiff_Tremoval = np.max(heights_diff_Tremoval)
    maxDiff_Tcontraction = np.max(heights_diff_Tcontraction)
    maxDiff_Econtraction = np.max(heights_diff_Econtraction)

    print('Vremoval')
    print('average = ',average_Vremoval)
    print('std = ',std_Vremoval)
    print('median = ',median_Vremoval)
    print('max diff = ',maxDiff_Vremoval)
    print()

    print('Tremoval')
    print('average = ', average_Tremoval)
    print('std = ', std_Tremoval)
    print('median = ', median_Tremoval)
    print('max diff = ', maxDiff_Tremoval)
    print()

    print('Tcontraction')
    print('average = ', average_Tcontraction)
    print('std = ', std_Tcontraction)
    print('median = ', median_Tcontraction)
    print('max diff = ', maxDiff_Tcontraction)
    print()

    print('Econtraction')
    print('average = ', average_Econtraction)
    print('std = ', std_Econtraction)
    print('median = ', median_Econtraction)
    print('max diff = ', maxDiff_Econtraction)

    # 2d plotting

    # full triangulation
    fig1, ax1 = plt.subplots()
    ax1.triplot(triangulation.points[:, 0], triangulation.points[:, 1], triangulation.simplices)
    ax1.plot(triangulation.points[:, 0], triangulation.points[:, 1], 'o', markersize=3)
    plt.title('Original')
    plt.axis('off')
    plt.savefig("Original.png", bbox_inches='tight')

    # simplified triangulation

    # vertex removal
    fig2, ax2 = plt.subplots()
    ax2.triplot(triangulation_simplify_Vremoval.points[:, 0],
                triangulation_simplify_Vremoval.points[:, 1], triangulation_simplify_Vremoval.simplices)
    ax2.plot(triangulation_simplify_Vremoval.points[:, 0], triangulation_simplify_Vremoval.points[:, 1],
             'o', markersize=3)
    plt.title('Vertex Removal 10%')
    plt.axis('off')
    plt.savefig("triangulation_simplify_Vremoval.png", bbox_inches='tight')

    # triangle removal
    fig3, ax3 = plt.subplots()
    ax3.triplot(triangulation_simplify_Tremoval.points[:, 0],
                triangulation_simplify_Tremoval.points[:, 1], triangulation_simplify_Tremoval.simplices)
    ax3.plot(triangulation_simplify_Tremoval.points[:, 0], triangulation_simplify_Tremoval.points[:, 1],
             'o', markersize=3)
    plt.title('Triangle Removal 10%')
    plt.axis('off')
    plt.savefig("triangulation_simplify_Tremoval.png", bbox_inches='tight')

    # triangle contraction
    fig4, ax4 = plt.subplots()
    ax4.triplot(triangulation_simplify_Tcontraction.points[:, 0],
                triangulation_simplify_Tcontraction.points[:, 1], triangulation_simplify_Tcontraction.simplices)
    ax4.plot(triangulation_simplify_Tcontraction.points[:, 0], triangulation_simplify_Tcontraction.points[:, 1],
             'o', markersize=3)
    plt.title('Triangle Contraction 10%')
    plt.axis('off')
    plt.savefig("triangulation_simplify_Tcontraction.png", bbox_inches='tight')

    # edge contraction
    fig5, ax5 = plt.subplots()
    ax5.triplot(triangulation_simplify_Econtraction.points[:, 0],
                triangulation_simplify_Econtraction.points[:, 1], triangulation_simplify_Econtraction.simplices)
    ax5.plot(triangulation_simplify_Econtraction.points[:, 0], triangulation_simplify_Econtraction.points[:, 1],
             'o', markersize=3)
    plt.title('Edge Contraction 10%')
    plt.axis('off')
    plt.savefig("triangulation_simplify_Econtraction.png", bbox_inches='tight')



    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

    plt.show()

    # 3d visualization
    visualizeScipyTriangulation(triangulation, heights,'Original')
    visualizeScipyTriangulation(triangulation_simplify_Vremoval, heights_simplify_Vremoval, 'VertexRemoval')
    visualizeScipyTriangulation(triangulation_simplify_Tremoval, heights_simplify_Tremoval, 'TriangleRemoval')
    visualizeScipyTriangulation(triangulation_simplify_Tcontraction, heights_simplify_Tcontraction, 'TriangleContraction')
    visualizeScipyTriangulation(triangulation_simplify_Econtraction, heights_simplify_Econtraction, 'EdgeContraction')
