import numpy as np


class GeoGridCell:
    def __init__(self,ind):
        """
        data structure of a geographic cell for GeoEqualCells
        """
        self.__ind = ind
        self.__points = np.array([])
        # self.__lx = lx
        # self.__ly = ly

    # @property
    # def Lx(self):
    #     return self.__lx
    #
    # @property
    # def Ly(self):
    #     return self.__ly

    @property
    def Ind(self):
        return self.__ind

    @property
    def Points(self):
        return self.__points

    def append(self,p):
        if self.Points.size == 0:
            self.__points = p
        else:
            self.__points = np.vstack((self.Points, p))
    # @property
    # def Area(self):
    #     return self.__lx*self.__ly