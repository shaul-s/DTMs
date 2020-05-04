import vtk
from tkinter.filedialog import askopenfilenames
from numpy import hstack, array, concatenate, vstack


class point3D:

    def __init__(self, x, y, z):
        """
        Initialize a 3D point with x, y and z ordinates
        :type x: float
        :type y: float
        :type z: float
        """
        # defining parameters
        self.x = x  # x ordinate
        self.y = y  # y ordinate
        self.z = z  # z ordinate


class PointCloud:

    def __init__(self):
        self.pts = None

    def initializeData(self):
        try:
            point3Dfiles = askopenfilenames(title='Select Input File')
        except:
            print('Oops! something went wrong :(')

        temp_points = []

        for filename in point3Dfiles:
            with open(filename) as file:
                lines = file.readlines()
                for line in lines:
                    line = line.split()
                    if len(line) < 3:
                        continue
                    else:
                        temp_points.append(array(line[0:3]).astype(float))

        self.pts = vstack(temp_points)


if __name__ == '__main__':
    cloud = PointCloud()
    cloud.initializeData()
    print(cloud.pts.shape)
