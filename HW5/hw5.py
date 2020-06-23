from scipy import spatial as spat
from scipy import ndimage
from matplotlib import pyplot as plt
from HW3.Delaunay import *
from matplotlib.colors import LightSource
import matplotlib.colors
import cv2
from matplotlib.colors import hsv_to_rgb

def hillShadeMap(map, azdeg, altdeg, cellSize):
    """
    create hillShade map
    :param grid: grid of heights
    :param azdeg: azimuth of light source in degrees
    :param altdeg: declination of light source in degrees
    :return: hillShade map slope map and aspect map

    :type grid: np.array nXm
    :type azdeg: float
    :type altdeg: float
    :rtype : np.array nXm,np.array nXm,np.array nXm
    """
    azdeg = np.radians(azdeg)
    altdeg = np.radians(altdeg)

    # calculate zanital degree
    Zl = np.pi/2 - altdeg


    # calculate derivatives
    # filter_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float)/8
    # filter_y = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]], dtype=np.float)/8
    # Zx = signal.convolve2d(map, filter_x, mode="same", boundary="symm", fillvalue=0)
    # Zy = signal.convolve2d(map, filter_y, mode="same", boundary="symm", fillvalue=0)
    Zx = ndimage.sobel(map, axis=0)/(8*cellSize)
    Zy = ndimage.sobel(map, axis=1)/(8*cellSize)

    # calculate slope and aspect values
    slope = np.arctan(Zx**2+Zy**2)
    aspect = np.arctan2(Zy,Zx)

    # if aspect is negative, add 360 deg
    mask = (aspect < 0)*1
    aspect = aspect + 2*np.pi*mask

    # calculate shade value
    shadeValue = 255*(np.cos(Zl)*np.cos(slope)+ np.sin(Zl)*np.sin(slope)*np.cos(azdeg-aspect))
    return shadeValue, slope, aspect

def grayscale_height_normalization(grid, normalize_by):
    """
    normalize heigits by normalization value
    :param grid: grid
    :param normalize_by: normalization value
    :return: normalized [0,normalize_by] heights
    """
    min = grid.min()
    max = grid.max()
    range = max - min
    # normalize
    grid = grid - min
    grid = grid / range
    # scale
    grid = grid * normalize_by

    return grid

if __name__ == '__main__':
    # load data
    points,nrows, ncol, cellSize = loadHeightsGrid()
    HeightsGrid = np.reshape(points[:,2],(nrows,ncol))
    # create triangulation
    tri = spat.Delaunay(points[:, 0: 2])

    # fig, axs = plt.subplots(ncols=2,nrows=1)
    fig1, (heights, shades) = plt.subplots(1,2)

    # draw triangulation
    # ax.triplot(points[:, 0], points[:, 1], tri.simplices)

    minHeight = np.min(points[:,2])
    maxHeight = np.max(points[:,2])
    grayLevel = (points[:,2] - minHeight)/(maxHeight-minHeight)*255
    points = np.hstack((points,np.reshape(grayLevel.T,(points.shape[0],1))))

    # draw vertices in gray scale by height
    heights.scatter(points[:, 0], points[:, 1], c=points[:, 2], s=5, cmap='gray')

    # create hillshade map
    shadeValues, slope, aspect = hillShadeMap(HeightsGrid,135,45,cellSize)
    shades.imshow(shadeValues,cmap='gray')

    # ls = LightSource(azdeg=135, altdeg=45)
    # shades.imshow(ls.hillshade(HeightsGrid, vert_exag=1), cmap='gray')
    shades.set(xlabel='Illumination Intensity')

    # slopes and aspects maps
    fig2, (slopes, aspects) = plt.subplots(1, 2)

    slopes.imshow(slope,cmap='gray')
    slopes.set(xlabel='slopes map')
    # V = np.ones(aspect.shape)
    # S = np.ones(aspect.shape)
    # aspect = grayscale_height_normalization(aspect, 179)
    # H = aspect
    # HSV = np.dstack((H, S, V)).astype('uint8')
    # RGB = hsv_to_rgb(HSV).astype('uint8')
    # aspects.imshow(RGB)
    aspects.imshow(aspect, cmap='hsv')
    aspects.set(xlabel='aspects map')
    plt.show()



    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

