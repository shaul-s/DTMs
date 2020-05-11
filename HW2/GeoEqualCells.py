from HW2.GeoGridCell import GeoGridCell
from HW2.utils import *
import time


class GeoEqualCells:
    def __init__(self):
        self.__avgN = None  # [int]
        self.__ratio = None  # [dx:dy]
        self.__cells = None  # list of GeoGridCell kX1
        self.__borders = None  # array 1X4
        self.__area = None  # float
        self.__pointsN = None  # [int]
        self.__points = None  # array nX3

    @property
    def Cells(self):
        return self.__cells

    @property
    def Ratio(self):
        return self.__ratio

    @property
    def Borders(self):
        """bounding rectangle [xmax,ymax,xmin,ymin]"""
        return self.__borders

    @property
    def Area(self):
        return self.Lx * self.Ly

    @property
    def Lx(self):
        return np.abs(self.Borders[0] - self.Borders[2])

    @property
    def Ly(self):
        return np.abs(self.Borders[1] - self.Borders[3])

    @property
    def Points(self):
        return self.__points

    @property
    def CellsN(self):
        """number of cells"""
        return self.Points.shape[0] // self.__avgN

    @property
    def CellLy(self):
        return np.sqrt((self.Area / self.CellsN) * (self.Ratio[0] / self.Ratio[1]))

    @property
    def CellLx(self):
        return (self.Area / self.CellsN) / self.CellLy

    @property
    def CellsNx(self):
        """number of cells along x axis"""
        return self.Lx // self.CellLx + 1

    @property
    def CellsNy(self):
        """number of cells along y axis"""
        return self.Ly // self.CellLy + 1

    def initializeGeoEqualCells(self, points, avgN, ratio):
        self.__avgN = avgN
        self.__points = points
        self.__ratio = ratio
        # finding bounding rectangle
        self.__borders = self.findBorders()
        # create cells
        self.__cells = self.createGrid()
        # populates the cells with points according to their coordinates
        self.populateCells()




    def findBorders(self):
        """finds bounding rectangle [xmax,ymax,xmin,ymin] """
        xmax = np.max(self.Points[:, 0])
        ymax = np.max(self.Points[:, 1])
        xmin = np.min(self.Points[:, 0])
        ymin = np.min(self.Points[:, 1])

        return np.array([xmax, ymax, xmin, ymin])

    def findBordersbyRadius(self, p, radius):
        """finds bounding rectangle"""
        xmax = p[0] + radius
        ymax = p[1] + radius
        xmin = p[0] - radius
        ymin = p[1] - radius

        return np.array([xmax, ymax, xmin, ymin])

    def createGrid(self):
        """creates geographic grid of cells"""
        geoGrid = []
        for i in range(int(self.CellsNy)):
            for j in range(int(self.CellsNx)):
                geoGrid.append(GeoGridCell(i * self.CellsNx + j))
        return geoGrid

    def populateCells(self):
        """inserting points to the GeoGridCells by their coordinates"""
        for p in self.Points:
            self.Cells[self.findInd(p)].append(p)

    def findPointsInRadius(self, p, radius):
        """ finding all the points inside the given radius from the given point"""
        # finding coordinates of radius bounding box
        borders = self.findBordersbyRadius(p, radius)
        # in case radius exceeds grid boundary
        if borders[0] > self.Borders[0]: borders[0] = self.Borders[0]
        if borders[1] > self.Borders[1]: borders[1] = self.Borders[1]
        if borders[2] < self.Borders[2]: borders[2] = self.Borders[2]
        if borders[3] < self.Borders[3]: borders[3] = self.Borders[3]
        # finding cells in the bounding box
        minCell = self.findInd(np.array([borders[2], borders[3]]))
        maxCell = self.findInd(np.array([borders[0], borders[1]]))
        minRow = minCell // self.CellsNx
        minCol = minCell % self.CellsNx
        maxRow = maxCell // self.CellsNx
        maxCol = maxCell % self.CellsNx

        # searching points inside the radius only in relevant cells
        indices = np.arange(len(self.Cells))
        indices = indices[(indices // self.CellsNx <= maxRow) * (indices // self.CellsNx >= minRow)
                          * (indices % self.CellsNx <= maxCol) * (indices % self.CellsNx >= minCol)]

        # extracting all the points from the cells in the radius
        pointsToCheck = np.vstack([x for x in map(self.addPoints, indices) if x is not None])
        # searching for points in the radius
        distances = sqDist(p, pointsToCheck)
        pointsInRadius = pointsToCheck[distances < radius ** 2, :]

        return pointsInRadius

    def addPoints(self,ind):
        """extracting points from cell"""
        if self.Cells[ind].Points.size == 0:
            return None
        return self.Cells[ind].Points

    def findInd(self, p):
        """calculate index of cell in grid"""
        return int((p[0] - self.Borders[2]) // self.CellLx + ((p[1] - self.Borders[3]) // self.CellLy) * self.CellsNx)

    def classifyPoints(self, radius, threshold):
        """
        classify points claude to object and terrain points by slope filter
        :param radius: radius from point to check
        :param threshold: maximal slope value between terrain points [deg]
        :return: object points and terrain points

        :type radius: float
        :type threshold: float
        :rtype: list[mX3],list[kX3]
        """
        start = time.time()
        terrain = []
        objects = []
        for i, p in enumerate(self.Points):
            pointsInRadius = self.findPointsInRadius(p, radius)
            if pointsInRadius.size <= 3:
                terrain.append(p)
                continue
            # checking if a point is object or terrain
            if isObject(p, pointsInRadius, threshold):
                objects.append(p)
            else:
                terrain.append(p)
        end = time.time()
        return terrain, objects, end-start


if __name__ == '__main__':
    pass
