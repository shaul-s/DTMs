class GeoGridCell:
    def __init__(self,ind,lx,ly,bottomLeft):
        """
        data structure of a geographic cell for GeoEqualCells
        """
        self.__ind = None
        self.__points = None
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

    # @property
    # def Area(self):
    #     return self.__lx*self.__ly