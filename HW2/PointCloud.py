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
        """
        getting user input, obtaining cloud point data and storing in a PointCloud object
        """
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
        """
        drawing the cloud of points in white with vtk
        """
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

    def drawFilteredPointCloud(self, objects, terrain, flag='all',runNumber=1):
        """
        drawing cloud of points after classification to terrain and objects
        :param objects: points classified as objects
        :param terrain: points classified as terrain
        :param flag: draw 'all', 'object' only or 'terrain' only
        """
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

        if flag == 'terrain':
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

        elif flag == 'objects':
            for pnt in objects:
                # Inserting the i-th point to the vtkPoints object
                rgb = array([255, 0, 0])
                id = vtkPnt.InsertNextPoint(pnt[0], pnt[1], pnt[2])
                # Adding color for the i-th point
                pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
                # Adding the index of i-th point to vertex vtk index array
                vtkVertex_ind.InsertNextCell(1)
                vtkVertex_ind.InsertCellPoint(id)

        elif flag == 'all':
            # Setting up the vtkPoints and scalars
            for pnt in objects:
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

        else:
            return print('flag needs to be of the following: terrain, objects or all')

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
        WriteImage('output'+str(runNumber), vtkRenderWindow,rgba=False )
        interactor.Start()


        # writer = vtk.vtkUnstructuredGridWriter()
        # writer.SetInputData(vtkVertex)
        # writer.SetFileName("Output.vtk")
        # writer.Write()

def WriteImage(fileName, renWin, rgba=True):
    """
    Write the render window view to an image file.

    Image types supported are:
     BMP, JPEG, PNM, PNG, PostScript, TIFF.
    The default parameters are used for all writers, change as needed.

    :param fileName: The file name, if no extension then PNG is assumed.
    :param renWin: The render window.
    :param rgba: Used to set the buffer type.
    :return:
    """

    import os

    if fileName:
        # Select the writer to use.
        path, ext = os.path.splitext(fileName)
        ext = ext.lower()
        if not ext:
            ext = '.png'
            fileName = fileName + ext
        if ext == '.bmp':
            writer = vtk.vtkBMPWriter()
        elif ext == '.jpg':
            writer = vtk.vtkJPEGWriter()
        elif ext == '.pnm':
            writer = vtk.vtkPNMWriter()
        elif ext == '.ps':
            if rgba:
                rgba = False
            writer = vtk.vtkPostScriptWriter()
        elif ext == '.tiff':
            writer = vtk.vtkTIFFWriter()
        else:
            writer = vtk.vtkPNGWriter()

        windowto_image_filter = vtk.vtkWindowToImageFilter()
        windowto_image_filter.SetInput(renWin)
        windowto_image_filter.SetScale(1)  # image quality
        if rgba:
            windowto_image_filter.SetInputBufferTypeToRGBA()
        else:
            windowto_image_filter.SetInputBufferTypeToRGB()
            # Read from the front buffer.
            windowto_image_filter.ReadFrontBufferOff()
            windowto_image_filter.Update()

        writer.SetFileName(fileName)
        writer.SetInputConnection(windowto_image_filter.GetOutputPort())
        writer.Write()
    else:
        raise RuntimeError('Need a filename.')
if __name__ == '__main__':
    cloud = PointCloud()

    cloud.initializeData()

    cloud.drawPointCloud()
