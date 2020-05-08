from HW2.GeoGridCell import GeoGridCell
from HW2.utils import *


class GeoEqualCells:
    def __init__(self):
        self.__avgN = None
        self.__ratio = None
        self.__cells = None
        self.__borders = None
        self.__area = None
        self.__pointsN = None
        self.__points = None

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
        return self.Points.shape[0] // self.__avgN

    @property
    def CellLy(self):
        return np.sqrt((self.Area / self.CellsN) * (self.Ratio[0] / self.Ratio[1]))

    @property
    def CellLx(self):
        return (self.Area / self.CellsN) / self.CellLy

    @property
    def CellsNx(self):
        return self.Lx // self.CellLx + 1

    @property
    def CellsNy(self):
        return self.Ly // self.CellLy + 1

    def initializeGeoEqualCells(self, points, avgN, ratio):
        self.__avgN = avgN
        self.__points = points
        # finding bounding rectangle
        self.__borders = self.findBorders()
        # calculating number of cells
        cellsN = self.CellsN
        # calculating area of cell
        cellArea = self.Area / cellsN
        # calculate edges of cell
        self.__ratio = ratio
        cellLy = self.CellLy
        cellLx = self.CellLx
        # number of cells in each edge of the grid
        cellsNx = self.CellsNx
        cellsNy = self.CellsNy
        # create cells
        self.__cells = self.createGrid(cellsNx, cellsNy)
        # populates the cells with points according to their coordinates
        self.populateCells()

    def findBorders(self):
        """finds bounding rectangle"""
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

    def createGrid(self, cellsNx, cellsNy):
        """creates geographic grid of cells"""
        geoGrid = []
        for i in range(int(cellsNy)):
            for j in range(int(cellsNx)):
                geoGrid.append(GeoGridCell(i * cellsNx + j))
        return geoGrid

    def populateCells(self):
        """inserting points to the GeoGridCells by their coordinates"""
        for p in self.Points:
            self.Cells[self.findInd(p)].append(p)

    def findPointsInRadius(self, p, radius):
        borders = self.findBordersbyRadius(p, radius)
        # in case radius exceeds grid boundary
        if borders[0] > self.Borders[0]: borders[0] = self.Borders[0]
        if borders[1] > self.Borders[1]: borders[1] = self.Borders[1]
        if borders[2] < self.Borders[2]: borders[2] = self.Borders[2]
        if borders[3] < self.Borders[3]: borders[3] = self.Borders[3]
        minCell = self.findInd(np.array([borders[2], borders[3]]))
        maxCell = self.findInd(np.array([borders[0], borders[1]]))
        minRow = minCell // self.CellsNx
        minCol = minCell % self.CellsNx
        maxRow = maxCell // self.CellsNx
        maxCol = maxCell % self.CellsNx

        # searching points inside the radius only in relevant cells
        indices = np.arange(len(self.Cells))
        indices = indices[(indices // self.CellsNx <= maxRow) * (indices // self.CellsNx > minRow)
                          * (indices % self.CellsNx <= maxCol) * (indices % self.CellsNx > minCol)]
        pointsToCheck = p   # initialize np array, in the end we remove the first row
        for i in indices:
            if self.Cells[i].Points.size == 0:
                continue
            pointsToCheck = np.vstack((pointsToCheck,self.Cells[i].Points))
        pointsToCheck = pointsToCheck[1:, :]
        distances = sqDist(p, pointsToCheck)
        pointsInRadius = pointsToCheck[distances < radius ** 2, :]

        pointsInRadius = pointsInRadius[1:, :]

        return pointsInRadius

    def findInd(self, p):
        """calculate index of cell in grid"""
        return int((p[0] - self.Borders[2]) // self.CellLx + ((p[1] - self.Borders[3]) // self.CellLy) * self.CellsNx)

    def classifyPoints(self, radius, threshold):
        terrain = []
        objects = []
        for i, p in enumerate(self.Points):
            pointsInRadius = self.findPointsInRadius(p, radius)
            # checking if a point is object or terrain
            if isObject(p, pointsInRadius, threshold):
                objects.append(p)
            else:
                terrain.append(p)

        return terrain, objects


if __name__ == '__main__':
    pass
