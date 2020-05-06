import vtk
from tkinter.filedialog import askopenfilenames
from numpy import array, vstack


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

        point3Dfiles = askopenfilenames(title='Select Input File')

        temp_points = []

        for filename in point3Dfiles:
            try:
                with open(filename) as file:
                    lines = file.readlines()
                    for line in lines:
                        line = line.split()
                        if len(line) < 3:
                            continue
                        else:
                            temp_points.append(array(line[0:3]).astype(float))
            except:
                print('Oops! your file is not supported')

        self.pts = vstack(temp_points)

    def drawPointCloud(self):

        # Initialize VTK points object
        vtkPnt = vtk.vtkPoints()
        # Initialize color scalars
        pnt_rgb = vtk.vtkUnsignedCharArray()
        # R, G, B
        pnt_rgb.SetNumberOfComponents(3)
        # Colors??
        pnt_rgb.SetName("Colors")

        # Initialize VTK PolyData object for vertices
        vtkVertex = vtk.vtkPolyData()
        # Initialize VTK vertices object for points
        vtkVertex_ind = vtk.vtkCellArray()

        # Setting up the vtkPoints and scalars
        for pnt in self.pts:
            # Inserting the i-th point to the vtkPoints object
            rgb = array([255, 255, 255])
            id = vtkPnt.InsertNextPoint(pnt[0], pnt[1], pnt[2])
            # Adding color for the i-th point
            pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
            # Adding the index of i-th point to vertex vtk index array
            vtkVertex_ind.InsertNextCell(1)
            vtkVertex_ind.InsertCellPoint(id)

        # Set vtkpoint in vertexes poly data object
        vtkVertex.SetPoints(vtkPnt)
        vtkVertex.SetVerts(vtkVertex_ind)
        # Add color to the vtkVertex object
        vtkVertex.GetPointData().SetScalars(pnt_rgb)

        # Finishing up VTK pipeline
        # Initialize a VTK mapper
        vtkMapper = vtk.vtkPolyDataMapper()
        vtkMapper.SetInputData(vtkVertex)
        # Initialize a VTK actor
        vtkActor = vtk.vtkActor()
        vtkActor.SetMapper(vtkMapper)
        # Initialize a VTK render window
        vtkRenderWindow = vtk.vtkRenderWindow()

        # Initialize a VTK renderer
        # Contains the actors to render
        vtkRenderer = vtk.vtkRenderer()
        # Add the VTK renderer to the VTK render window
        vtkRenderWindow.AddRenderer(vtkRenderer)
        # define the renderer
        vtkRenderer.AddActor(vtkActor)
        vtkActor.GetProperty().LightingOn()
        # vtkActor.GetProperty().SetRepresentationToWireframe()
        # vtkActor.GetProperty().SetRepresentationToPoints()
        #

        # Set camera and background data
        vtkRenderer.ResetCamera()
        vtkRenderWindow.Render()
        # Enable user interface interactor
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(vtkRenderWindow)
        vtkRenderWindow.Render()
        interactor.Start()

    def drawFilteredPointCloud(self, object, terrain):

        # Initialize VTK points object
        vtkPnt = vtk.vtkPoints()
        # Initialize color scalars
        pnt_rgb = vtk.vtkUnsignedCharArray()
        # R, G, B
        pnt_rgb.SetNumberOfComponents(3)
        # Colors??
        pnt_rgb.SetName("Colors")

        # Initialize VTK PolyData object for vertices
        vtkVertex = vtk.vtkPolyData()
        # Initialize VTK vertices object for points
        vtkVertex_ind = vtk.vtkCellArray()

        # Setting up the vtkPoints and scalars
        for pnt in object:
            # Inserting the i-th point to the vtkPoints object
            rgb = array([255, 0, 0])
            id = vtkPnt.InsertNextPoint(pnt[0], pnt[1], pnt[2])
            # Adding color for the i-th point
            pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
            # Adding the index of i-th point to vertex vtk index array
            vtkVertex_ind.InsertNextCell(1)
            vtkVertex_ind.InsertCellPoint(id)

        # Setting up the vtkPoints and scalars
        for pnt in terrain:
            # Inserting the i-th point to the vtkPoints object
            rgb = array([0, 255, 0])
            id = vtkPnt.InsertNextPoint(pnt[0], pnt[1], pnt[2])
            # Adding color for the i-th point
            pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
            # Adding the index of i-th point to vertex vtk index array
            vtkVertex_ind.InsertNextCell(1)
            vtkVertex_ind.InsertCellPoint(id)

        # Set vtkpoint in vertexes poly data object
        vtkVertex.SetPoints(vtkPnt)
        vtkVertex.SetVerts(vtkVertex_ind)
        # Add color to the vtkVertex object
        vtkVertex.GetPointData().SetScalars(pnt_rgb)

        # Finishing up VTK pipeline
        # Initialize a VTK mapper
        vtkMapper = vtk.vtkPolyDataMapper()
        vtkMapper.SetInputData(vtkVertex)
        # Initialize a VTK actor
        vtkActor = vtk.vtkActor()
        vtkActor.SetMapper(vtkMapper)
        # Initialize a VTK render window
        vtkRenderWindow = vtk.vtkRenderWindow()

        # Initialize a VTK renderer
        # Contains the actors to render
        vtkRenderer = vtk.vtkRenderer()
        # Add the VTK renderer to the VTK render window
        vtkRenderWindow.AddRenderer(vtkRenderer)
        # define the renderer
        vtkRenderer.AddActor(vtkActor)
        vtkActor.GetProperty().LightingOn()
        # vtkActor.GetProperty().SetRepresentationToWireframe()
        # vtkActor.GetProperty().SetRepresentationToPoints()
        #

        # Set camera and background data
        vtkRenderer.ResetCamera()
        vtkRenderWindow.Render()
        # Enable user interface interactor
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(vtkRenderWindow)
        vtkRenderWindow.Render()
        interactor.Start()

if __name__ == '__main__':
    cloud = PointCloud()

    cloud.initializeData()

    cloud.drawPointCloud()
