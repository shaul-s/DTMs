from scipy import spatial as spat
from scipy import ndimage
from matplotlib import pyplot as plt
from HW3.Delaunay import *
from matplotlib.colors import LightSource

def hillShade(map, azdeg, altdeg):
    """
    create hillShade map
    :param grid: grid of heights
    :param azdeg: azimuth of light source
    :param altdeg: declination of light source
    :return: hillShade map
    """
    azdeg = np.radians(azdeg)
    altdeg = np.radians(altdeg)

    p0 = np.sin(azdeg)
    q0 = np.cos(azdeg)
    b = np.cos(altdeg)

    Zx = ndimage.sobel(map, axis=0)
    Zy = ndimage.sobel(map, axis=1)

    pTag = (Zx*p0+Zy*q0)/np.sqrt(Zx**2+Zy**2)

    shadeValue = 255*0.5*(1+(pTag/b))
    return shadeValue


if __name__ == '__main__':
    # load data
    points,nrows, ncol, cellSize = loadHeightsGrid()
    HeightsGrid = np.reshape(points[:,2],(nrows,ncol))
    # create triangulation
    tri = spat.Delaunay(points[:, 0: 2])

    # fig, axs = plt.subplots(ncols=2,nrows=1)
    fig, (ax1, ax2) = plt.subplots(1,2)

    # draw triangulation
    # ax.triplot(points[:, 0], points[:, 1], tri.simplices)

    minHeight = np.min(points[:,2])
    maxHeight = np.max(points[:,2])
    grayLevel = (points[:,2] - minHeight)/(maxHeight-minHeight)*255
    points = np.hstack((points,np.reshape(grayLevel.T,(points.shape[0],1))))

    # draw vertices in gray scale by height
    # axs[0, 0].scatter(points[:, 0], points[:, 1], c=points[:, 2], s=5, cmap='gray')
    ax1.scatter(points[:, 0], points[:, 1], c=points[:, 2], s=5, cmap='gray')

    shadeValues = hillShade(HeightsGrid,135,45)
    shadeValues = shadeValues.flatten()
    ax2.scatter(points[:, 0], points[:, 1], c=shadeValues, s=5, cmap='gray')

    # ls = LightSource(azdeg=135, altdeg=45)
    # ax2.imshow(ls.hillshade(HeightsGrid, vert_exag=1), cmap='gray')
    # ax2.set(xlabel='Illumination Intensity')
    plt.show()

    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

