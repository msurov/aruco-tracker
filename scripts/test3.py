from vtk import *
import numpy as np
import os
import json


def imwrite(fileName, renWin, rgba=True):
    if not fileName:
        raise RuntimeError('Need a filename.')

    _, ext = os.path.splitext(fileName)
    ext = ext.lower()

    if not ext:
        ext = '.png'
        fileName = fileName + ext
    if ext == '.bmp':
        writer = vtkBMPWriter()
    elif ext == '.png':
        writer = vtkPNGWriter()
    elif ext == '.jpg':
        writer = vtkBMPWriter()
    else:
        raise Exception('unsupported extension ' + ext)

    windowto_image_filter = vtkWindowToImageFilter()
    windowto_image_filter.SetInput(renWin)
    # windowto_image_filter.SetScale(1)  # image quality

    if rgba:
        windowto_image_filter.SetInputBufferTypeToRGBA()
    else:
        windowto_image_filter.SetInputBufferTypeToRGB()
        windowto_image_filter.ReadFrontBufferOff()
        windowto_image_filter.Update()

    writer.SetFileName(fileName)
    writer.SetInputConnection(windowto_image_filter.GetOutputPort())
    writer.Write()


def set_cam_intrinsics(camera, intrinsics):
    nx, ny = intrinsics['resolution']
    f = intrinsics['focus']
    cx,cy = intrinsics['principal']

    camera.SetPosition(0, 0, 0)
    camera.SetFocalPoint(0, 0, 1)
    camera.SetViewUp(0, -1, 0)
    camera.SetClippingRange(0.1, 1e+2)
    wcx = -2. * (cx - nx/2.) / nx
    wcy =  2. * (cy - ny/2.) / ny

    angle = 180 * 2.0 * np.arctan2(ny/2.0, f) / np.pi
    camera.SetWindowCenter(wcx, wcy)
    camera.SetViewAngle(angle)


def createCube(dims, center):
    dx,dy,dz = dims
    x,y,z = center
    cubeSource = vtkCubeSource()
    cubeSource.SetCenter(x,y,z)
    cubeSource.SetXLength(dx)
    cubeSource.SetYLength(dy)
    cubeSource.SetZLength(dz)

    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(cubeSource.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(255, 255, 255)

    return actor


def createTexturedBox():
    dims = 110e-3, 140e-3, 0.1e-3
    center = 40e-3, 25e-3, 0.
    impath = 'dataset/vtk/aruco_1.svg.png'

    # Generate a cube 
    dx,dy,dz = dims
    x,y,z = center
    cubeSource = vtkCubeSource()
    cubeSource.SetCenter(x,y,z)
    cubeSource.SetXLength(dx)
    cubeSource.SetYLength(dy)
    cubeSource.SetZLength(dz)

    # Read the image data from a file
    reader = vtkPNGReader()
    reader.SetFileName(impath)

    # Create texture object
    texture = vtkTexture()
    texture.SetInputConnection(reader.GetOutputPort())

    #Map texture coordinates
    map_to_plane = vtkTextureMapToPlane()
    map_to_plane.SetInputConnection(cubeSource.GetOutputPort())

    # Create mapper and set the mapped texture as input
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(map_to_plane.GetOutputPort())

    # Create actor and set the mapper and the texture
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.SetTexture(texture)

    T = vtkTransform()
    T.Translate(0.1, -0., 2.0)
    T.RotateX(160)
    T.RotateZ(15)
    print T.GetPosition()
    print T.GetOrientationWXYZ()
    actor.SetUserTransform(T)

    # dx,dy,dz = 0.1, 0.1, 1e-4
    # x,y,z = 0.2, 0., 2.

    # cubeSource = vtkCubeSource()
    # cubeSource.SetCenter(x,y,z)
    # cubeSource.SetXLength(dx)
    # cubeSource.SetYLength(dy)
    # cubeSource.SetZLength(dz)

    # mapper = vtkPolyDataMapper()
    # mapper.SetInputConnection(cubeSource.GetOutputPort())
    # actor = vtkActor()
    # actor.SetMapper(mapper)
    # actor.GetProperty().SetColor(255, 255, 255)

    return actor


def createMarker():
    assembly = vtkAssembly()
    cube1 = createCube([14e-2,14e-2,1e-4], [5e-2, 5e-2, 0])
    cube1.GetProperty().SetColor(255, 255, 255)

    cube2 = createCube([10e-2,10e-2,2e-4], [5e-2, 5e-2, 0])
    cube2.GetProperty().SetColor(0, 0, 0)

    assembly.AddPart(cube1)
    assembly.AddPart(cube2)
    return assembly


def main():
    camera_cfg = {
        'focus': 1000,
        'resolution': [1000,1000],
        'principal': [500.5,500.5]
    }
    camera = vtkCamera()
    camera.SetPosition(0, 0, 0)
    set_cam_intrinsics(camera, camera_cfg)

    renderer = vtkRenderer()
    renderer.SetActiveCamera(camera)

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(*camera_cfg['resolution'])
    renderWindow.SetAAFrames(12)

    # intercator
    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # actor
    marker = createTexturedBox()
    renderer.AddActor(marker)
    renderer.SetBackground(0.5, 0.5, 1.)

    marker_cfg = {
        "side": 80e-3,
        "type": "5x5x1000"
    }
    pose_cfg = {
        "rvec": [0,0,0],
        "tvec": [0,0,1]
    }
    cfg = {
        'camera': camera_cfg,
        'marker': marker_cfg,
        'pose': pose_cfg
    }
    s = json.dumps(cfg, indent=2)
    f = open('./dataset/vtk/1.json', 'w')
    f.write(s)

    # launch
    renderWindow.Render()
    imwrite('./dataset/vtk/1.png', renderWindow)
    renderWindowInteractor.Start()

main()

