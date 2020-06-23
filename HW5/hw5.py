from scipy import spatial as spat
from scipy import ndimage
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
    azdeg = 360-(azdeg+90)
    azdeg = np.radians(azdeg)
    altdeg = np.radians(altdeg)

    # calculate zanital degree
    Zl = np.pi/2 - altdeg


    # calculate derivatives
    Zx = ndimage.sobel(map, axis=0)/(cellSize)
    Zy = ndimage.sobel(map, axis=1)/(cellSize)

    # calculate slope and aspect values
    slope = np.arctan(Zx**2+Zy**2)
    aspect = np.arctan2(Zy,Zx)

    # if aspect is negative, add 360 deg
    mask = (aspect < 0)*1
    aspect = aspect + 2*np.pi*mask

    # calculate shade value
    shadeValue = 255*(np.cos(Zl)*np.cos(slope)+ np.sin(Zl)*np.sin(slope)*np.cos(azdeg-aspect))
    return shadeValue, slope, aspect

def normalize(map):
    """normalize values to 0-1 range"""
    return (map-np.min(map))/(np.max(map)-np.min(map))


def bcubic(DEM, nrows, ncol, cellSize, points):
    """ bcubic interpolation on DEM """
    # calculate derivatives
    ZxTemp = (DEM.T[2:]-DEM.T[0:DEM.shape[1]-2])
    Zx = ZxTemp.T/(2*cellSize)
    Zx = Zx[1:Zx.shape[0]-1]
    Zy = DEM[2:]-DEM[0:DEM.shape[0]-2]/(2*cellSize)
    Zy = Zy[:,1:Zy.shape[1]-1]
    Zxy = ZxTemp.T[2:]-ZxTemp.T[0:ZxTemp.T.shape[0]-2]/(4*cellSize*cellSize)

    # find points's cells indices


    Xinv = np.array([[1,0,0,0],[0,1,0,0],[-3,-2,3,-1],[2,1,-2,1]])
    XinvT = Xinv.T


if __name__ == '__main__':
    # load data
    points,nrows, ncol, cellSize = loadHeightsGrid()
    HeightsGrid = np.reshape(points[:,2],(nrows,ncol))
    # create triangulation
    tri = spat.Delaunay(points[:, 0: 2])

    bcubic(HeightsGrid, nrows, ncol, cellSize, points)

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
    shades.set(xlabel='Illumination Intensity')

    # slopes and aspects maps
    fig2, (slopes, aspects, slopeAspect) = plt.subplots(1, 3)

    slopes.imshow(slope,cmap='gray')
    slopes.set(xlabel='slopes map')

    # aspect to HSV
    V = np.ones(aspect.shape)
    S = np.ones(aspect.shape)
    H = np.rad2deg(aspect)/360
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



    # for i, triangle in enumerate(tri.simplices):
    #     tri_points = np.vstack((points[triangle[0]], points[triangle[1]], points[triangle[2]]))
    #     centroid = np.average(tri_points, axis=0)
    #     ax.annotate("({})".format(i), (centroid[0], centroid[1]))

