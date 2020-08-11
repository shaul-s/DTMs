import vtk
from numpy import hstack, array


def visualizeTriangulation(points, d):
    # # importing point data from csv file
    vertices = []
    # with open(r'vertices.csv') as file:
    #     lines = file.readlines()
    #     lines.pop(0)
    #     for i, line in enumerate(lines):
    #         line = line.split(',')
    #         vertices.append(hstack((i, line)))
    for i, p in enumerate(points):
        vertices.append(hstack((i, p, array([51, 153, 255]))))
    vertices = array(vertices)


    # importing triangle data from csv file
    triangles = []
    # with open(r'triangles.csv') as file:
    #     lines = file.readlines()
    #     for i, line in enumerate(lines):
    #         line = line.split(',')
    #         triangles.append(hstack((i, line)))

    for i, tri in enumerate(d.Triangles):
        triangles.append(hstack((i, tri.Points[:, -1])))
    triangles = array(triangles)

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
    # Initialize VTK PolyDataobject for triangulation
    vtkTri = vtk.vtkPolyData()
    # Initialize VTK vertices object for points
    vtkVertex_ind = vtk.vtkCellArray()
    # Initialize VTK vertices object for triangles
    vtkTri_ind = vtk.vtkCellArray()

    # Setting up the vtkPoints and scalars
    for pnt in vertices:
        # Inserting the i-th point to the vtkPoints object
        rgb = pnt[4:7].astype(int)
        pnt = pnt.astype(float)
        id = vtkPnt.InsertNextPoint(pnt[1], pnt[2], pnt[3])
        # Adding color for the i-th point
        pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
        # Adding the index of i-th point to vertex vtk index array
        vtkVertex_ind.InsertNextCell(1)
        vtkVertex_ind.InsertCellPoint(id)

    # Set vtkpoint in triangle poly data object
    vtkTri.SetPoints(vtkPnt)
    # Add color to the vtkTri object
    vtkTri.GetPointData().SetScalars(pnt_rgb)
    # Set vtkpoint in vertexes poly data object
    vtkVertex.SetPoints(vtkPnt)
    vtkVertex.SetVerts(vtkVertex_ind)
    # Add color to the vtkVertex object
    vtkVertex.GetPointData().SetScalars(pnt_rgb)

    # Setting up the vtkPolyData and scalars
    for tri in triangles:
        # Set triangle's 3 vertices by ID
        ith_tri = vtk.vtkTriangle()
        ith_tri.GetPointIds().SetId(0, int(tri[1]))
        ith_tri.GetPointIds().SetId(1, int(tri[2]))
        ith_tri.GetPointIds().SetId(2, int(tri[3]))
        # Insert the i-th triangle data index
        vtkTri_ind.InsertNextCell(ith_tri)

    # Finishing up VTK pipeline
    # Initialize a VTK mapper
    vtkMapper = vtk.vtkPolyDataMapper()
    vtkMapper.SetInputData(vtkVertex)
    vtkTri.SetPolys(vtkTri_ind)
    vtkMapper.SetInputData(vtkTri)
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
    #vtkActor.GetProperty().SetRepresentationToWireframe()
    #vtkActor.GetProperty().SetRepresentationToPoints()
    #

    # Set camera and background data
    vtkRenderer.ResetCamera()
    vtkRenderWindow.Render()
    # Enable user interface interactor
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(vtkRenderWindow)
    vtkRenderWindow.Render()
    interactor.Start()

def visualizeScipyTriangulation(triangulation, heights, name):
    # importing vertices data
    vertices = []
    for i, p in enumerate(triangulation.points):
        vertices.append(hstack((i, p, heights[i], array([51, 153, 255]))))
    vertices = array(vertices)


    # importing triangle data
    triangles = triangulation.simplices
    # for i, tri in enumerate(triangulation.simplices):
    #     triangles.append(hstack((i, tri.Points[:, -1])))
    # triangles = array(triangles)

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
    # Initialize VTK PolyDataobject for triangulation
    vtkTri = vtk.vtkPolyData()
    # Initialize VTK vertices object for points
    vtkVertex_ind = vtk.vtkCellArray()
    # Initialize VTK vertices object for triangles
    vtkTri_ind = vtk.vtkCellArray()

    # Setting up the vtkPoints and scalars
    for pnt in vertices:
        # Inserting the i-th point to the vtkPoints object
        rgb = pnt[4:7].astype(int)
        pnt = pnt.astype(float)
        id = vtkPnt.InsertNextPoint(pnt[1], pnt[2], pnt[3])
        # Adding color for the i-th point
        pnt_rgb.InsertNextTuple3(rgb[0], rgb[1], rgb[2])
        # Adding the index of i-th point to vertex vtk index array
        vtkVertex_ind.InsertNextCell(1)
        vtkVertex_ind.InsertCellPoint(id)

    # Set vtkpoint in triangle poly data object
    vtkTri.SetPoints(vtkPnt)
    # Add color to the vtkTri object
    vtkTri.GetPointData().SetScalars(pnt_rgb)
    # Set vtkpoint in vertexes poly data object
    vtkVertex.SetPoints(vtkPnt)
    vtkVertex.SetVerts(vtkVertex_ind)
    # Add color to the vtkVertex object
    vtkVertex.GetPointData().SetScalars(pnt_rgb)

    # Setting up the vtkPolyData and scalars
    for tri in triangles:
        # Set triangle's 3 vertices by ID
        ith_tri = vtk.vtkTriangle()
        ith_tri.GetPointIds().SetId(0, int(tri[0]))
        ith_tri.GetPointIds().SetId(1, int(tri[1]))
        ith_tri.GetPointIds().SetId(2, int(tri[2]))
        # Insert the i-th triangle data index
        vtkTri_ind.InsertNextCell(ith_tri)

    # Finishing up VTK pipeline
    # Initialize a VTK mapper
    vtkMapper = vtk.vtkPolyDataMapper()
    vtkMapper.SetInputData(vtkVertex)
    vtkTri.SetPolys(vtkTri_ind)
    vtkMapper.SetInputData(vtkTri)
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
    #vtkActor.GetProperty().SetRepresentationToWireframe()
    #vtkActor.GetProperty().SetRepresentationToPoints()
    #

    # Set camera and background data
    vtkRenderer.ResetCamera()
    vtkRenderWindow.Render()
    # Enable user interface interactor
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(vtkRenderWindow)
    vtkRenderWindow.Render()
    WriteImage(name, vtkRenderWindow, rgba=False)
    interactor.Start()

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

if __name__ == "__main__":
    pass
