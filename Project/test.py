from scipy import spatial as spat
from matplotlib import pyplot as plt
from HW3.Delaunay import *


if __name__ == '__main__':
    points = initializeData()

    tri = spat.Delaunay(points[:, 0: 2])

    fig, ax = plt.subplots()
    ax.triplot(points[:, 0], points[:, 1], tri.simplices)
    ax.plot(points[:, 0], points[:, 1], 'o', markersize=5)

    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

    plt.show()

    print('hi')
