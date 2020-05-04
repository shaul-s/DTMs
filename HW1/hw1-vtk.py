import vtk
from numpy import hstack, array

# importing point data from csv file
vertices = []
with open(r'vertices.csv') as file:
    lines = file.readlines()
    lines.pop(0)
    for i, line in enumerate(lines):
        line = line.split(',')
        vertices.append(hstack((i, line)))

vertices = array(vertices)

# importing triangle data from csv file
triangles = []
with open(r'triangles.csv') as file:
    lines = file.readlines()
    for i, line in enumerate(lines):
        line = line.split(',')
        triangles.append(hstack((i, line)))

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

# print('hi')
