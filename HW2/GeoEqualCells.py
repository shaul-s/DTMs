from GeoGridCell import GeoGridCell
from HW2.utils import *

class GeoEqualCells:
    def __init__(self):
        self.__avgN = None
        self.__ratio = None
        self.__cells = None
        self.__borders = None
        self.__area = None

    @property
    def Cells(self):
        return self.__cells

    @property
    def Ratio(self):
        return self.__ratio

    @property
    def Borders(self):
        return self.__borders

    @property
    def Area(self):
        return self.__area

    def initializeGeoEqualCells(self, points, avgN, ratio):
        self.__borders = self.findBorders(points)
        cellsN = points.shape(0)/avgN
        gridArea = distance(self.Borders[0,0],self.Borders[0,1])*distance(self.Borders[0,0],self.Borders[1,0])
        cellArea = gridArea/cellsN

    def findBorders(self,points):
        xmax = np.max(points[:,0])
        ymax = np.max(points[:,1])
        xmin = np.min(points[:,0])
        ymin = np.max(points[:,1])

        return np.array([[xmin,ymax],[xmax,ymax],[xmin,ymin],[xmax,ymin]])
