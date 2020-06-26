from scipy import spatial as spat
from scipy import ndimage
from scipy import linalg as la
import pandas as pd
from matplotlib import pyplot as plt
from HW3.Delaunay import *
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
    azdeg = 360 - (azdeg + 90)
    azdeg = np.radians(azdeg)
    altdeg = np.radians(altdeg)

    # calculate zanital degree
    Zl = np.pi / 2 - altdeg

    # calculate derivatives
    Zx = ndimage.sobel(map, axis=0) / (cellSize)
    Zy = ndimage.sobel(map, axis=1) / (cellSize)

    # calculate slope and aspect values
    slope = np.arctan(Zx ** 2 + Zy ** 2)
    aspect = np.arctan2(Zy, Zx)

    # if aspect is negative, add 360 deg
    mask = (aspect < 0) * 1
    aspect = aspect + 2 * np.pi * mask

    # calculate shade value
    shadeValue = 255 * (np.cos(Zl) * np.cos(slope) + np.sin(Zl) * np.sin(slope) * np.cos(azdeg - aspect))
    return shadeValue, slope, aspect


def normalize(map):
    """normalize values to 0-1 range"""
    return (map - np.min(map)) / (np.max(map) - np.min(map))


# def bcubic(DEM, nrows, ncol, cellSize, points):
#     """ bcubic interpolation on DEM """
#     # calculate derivatives
#     ZxTemp = (DEM.T[2:] - DEM.T[0:DEM.shape[1] - 2])
#     Zx = ZxTemp.T / (2 * cellSize)
#     Zx = Zx[1:Zx.shape[0] - 1]
#     Zy = DEM[2:] - DEM[0:DEM.shape[0] - 2] / (2 * cellSize)
#     Zy = Zy[:, 1:Zy.shape[1] - 1]
#     Zxy = ZxTemp.T[2:] - ZxTemp.T[0:ZxTemp.T.shape[0] - 2] / (4 * cellSize * cellSize)
#
#     # find points's cells indices
#
#     Xinv = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [-3, -2, 3, -1], [2, 1, -2, 1]])
#     XinvT = Xinv.T

def bcubic(oldDEM, point2d, idx, jdx):
    def computeH(oldDEM, i, j):
        h00 = oldDEM["DEM"][i, j]
        h01 = (oldDEM["DEM"][i + 1, j] - oldDEM["DEM"][i - 1, j]) / (2 * oldDEM["cell_size"])
        h10 = (oldDEM["DEM"][i, j + 1] - oldDEM["DEM"][i, j - 1]) / (2 * oldDEM["cell_size"])
        h11 = (oldDEM["DEM"][i + 1, j + 1] - oldDEM["DEM"][i - 1, j + 1] - oldDEM["DEM"][i + 1, j - 1] +
               oldDEM["DEM"][i - 1, j - 1]) / (4 * oldDEM["cell_size"] ** 2)
        return np.array([[h00, h01], [h10, h11]])

    # get x and y values of the cell upper right point in the old DEM
    x = oldDEM["cell_size"] * (point2d[0] - oldDEM["xll"]) % oldDEM["cell_size"]
    y = oldDEM["cell_size"] * (point2d[1] - oldDEM["yll"]) % oldDEM["cell_size"]

    # computing matrices for interpolation
    H_matrices = []
    for i in range(idx, idx + 2):
        for j in range(jdx, jdx + 2):
            H_matrices.append(computeH(oldDEM, i, j))

    H = np.vstack((np.hstack((H_matrices[0], H_matrices[1])), np.hstack((H_matrices[2], H_matrices[-1]))))
    X = Y = np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [1, oldDEM["cell_size"], oldDEM["cell_size"] ** 2, oldDEM["cell_size"] ** 3],
         [0, 1, 2 * oldDEM["cell_size"], 3 * oldDEM["cell_size"] ** 2]])

    # compute bi-cubic surface parameteres
    A = np.dot(np.dot(la.inv(X), H), la.inv(Y.T))

    # interpolate and return height
    x_vec = np.array([1, x, x * x, x * x * x])
    y_vec = np.array([1, y, y * y, y * y * y])
    return np.dot(np.dot(x_vec, A), y_vec.T)


def blinear(oldDEM, point2d, idx, jdx):
    """

    :param oldDEM:
    :param point2d:
    :param idx:
    :param jdx:
    :return:
    """
    # get x and y values of the cell upper right point in the old DEM
    x = oldDEM["cell_size"] * (point2d[0] - oldDEM["xll"]) % oldDEM["cell_size"]
    y = oldDEM["cell_size"] * (point2d[1] - oldDEM["yll"]) % oldDEM["cell_size"]

    # compute bi-linear surface parameters
    a = oldDEM["DEM"][jdx, idx]
    b = (oldDEM["DEM"][jdx + 1, idx] - a) / oldDEM["cell_size"]
    c = (oldDEM["DEM"][jdx, idx + 1] - a) / oldDEM["cell_size"]
    d = (a - oldDEM["DEM"][jdx, idx + 1] - oldDEM["DEM"][jdx + 1, idx] + oldDEM["DEM"][jdx + 1, idx + 1]) / (
            oldDEM["cell_size"] ** 2)

    # compute interpolated height
    return a + b * x + c * y + d * x * y


def computeNewDEM(oldDEM, nrows, ncol, xll, yll, cellsize):
    """
    computing a new DEM based on a known DEM (oldDEM)
    :param oldDEM: the known DEM has to be dict at shown
    :param nrows: number of rows
    :param ncol: number of columns
    :param xll: lower left x
    :param yll: lower left y
    :param cellsize: size of cell
    :return: new DEM
    """
    # check if user entered vars are legit
    if (cellsize * ncol + xll > oldDEM["cell_size"] * oldDEM["ncol"] + oldDEM["xll"]) or \
            (cellsize * nrows + yll > oldDEM["cell_size"] * oldDEM["nrows"] + oldDEM["yll"]):
        print('the data you entered is invalid')
        return

    heightsgrid_xy = []
    heightsgrid_z = []

    for i in range(nrows):
        for j in range(ncol):
            #  create the XY grid
            point2d = np.array([xll + j * cellsize, yll + (i) * cellsize])
            heightsgrid_xy.append(point2d)
            # find point's cell indexes in the old DEM so we can find them
            idx = int((point2d[0] - oldDEM["xll"]) / oldDEM["cell_size"])
            jdx = int((point2d[1] - oldDEM["yll"]) / oldDEM["cell_size"])
            cell = np.array([idx, jdx])
            # check if cell is on the edges, if it is use bi-linear, if it isn't use bi-cubic
            if ((np.all((0, 0) < cell) and cell[1] < oldDEM["nrows"]) or \
                    (np.all((0, 0) < cell) and cell[0] < oldDEM["ncol"]) or
                    (np.all((0, oldDEM["nrows"]) < cell) and np.all(cell) < (oldDEM["ncol"], oldDEM["nrows"])) or
                    (np.all((oldDEM["ncol"], 0) < cell) and np.all(cell) < (oldDEM["ncol"], oldDEM["nrows"]))):

                heightsgrid_z.append(bcubic(oldDEM, point2d, idx, jdx))

            else:
                heightsgrid_z.append(blinear(oldDEM, point2d, idx, jdx))

    heights_grid = np.hstack((np.vstack(heightsgrid_xy), np.vstack(heightsgrid_z)))
    return heights_grid


if __name__ == '__main__':
    # load data
    points, nrows, ncol, cellSize = loadHeightsGrid()
    HeightsGrid = np.reshape(points[:, 2], (nrows, ncol))
    # create triangulation
    tri = spat.Delaunay(points[:, 0: 2])

    # fig, axs = plt.subplots(ncols=2,nrows=1)
    fig1, (heights, shades) = plt.subplots(1, 2)

    # draw triangulation
    # ax.triplot(points[:, 0], points[:, 1], tri.simplices)

    minHeight = np.min(points[:, 2])
    maxHeight = np.max(points[:, 2])
    grayLevel = (points[:, 2] - minHeight) / (maxHeight - minHeight) * 255
    points = np.hstack((points, np.reshape(grayLevel.T, (points.shape[0], 1))))

    # draw vertices in gray scale by height
    heights.scatter(points[:, 0], points[:, 1], c=points[:, 2], s=5, cmap='gray')

    # create hillshade map
    shadeValues, slope, aspect = hillShadeMap(HeightsGrid, 135, 45, cellSize)
    shades.imshow(shadeValues, cmap='gray')
    shades.set(xlabel='Illumination Intensity')

    # slopes and aspects maps
    fig2, (slopes, aspects, slopeAspect) = plt.subplots(1, 3)

    slopes.imshow(slope, cmap='gray')
    slopes.set(xlabel='slopes map')

    # aspect to HSV
    V = np.ones(aspect.shape)
    S = np.ones(aspect.shape)
    H = np.rad2deg(aspect) / 360
    aspectHSV = np.dstack((H, S, V))
    aspectRGB = hsv_to_rgb(aspectHSV)
    aspects.imshow(aspectRGB)
    aspects.set(xlabel='aspects map')

    # slope and aspect combine
    S = normalize(slope)
    slopeAspectHSV = np.dstack((H, S, V))
    slopeAspectRGB = hsv_to_rgb(slopeAspectHSV)
    slopeAspect.imshow(slopeAspectRGB)
    slopeAspect.set(xlabel='slopeAspects map')
    plt.show()

    oldDEM = {"DEM": HeightsGrid, "nrows": nrows, "ncol": ncol, "xll": 1000, "yll": 1000, "cell_size": cellSize}

    nrows = 200
    ncol = 200

    new_points = computeNewDEM(oldDEM, nrows, ncol, 1025, 1025, 35)

    new_HeightsGrid = np.reshape(new_points[:, 2], (nrows, ncol))

    minHeight = np.min(new_points[:, 2])
    maxHeight = np.max(new_points[:, 2])
    grayLevel = (new_points[:, 2] - minHeight) / (maxHeight - minHeight) * 255
    new_points = np.hstack((new_points, np.reshape(grayLevel.T, (new_points.shape[0], 1))))

    # draw vertices in gray scale by height
    plt.scatter(new_points[:, 0], new_points[:, 1], c=new_points[:, 2], s=5, cmap='gray')
    plt.show()

    # print(pd.DataFrame(new_HeightsGrid))

