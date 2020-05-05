from GeoGridCell import GeoGridCell


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
