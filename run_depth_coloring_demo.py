import glob
import os
import random
import time
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better
from realsense_handler import RealsenseHandler

'''
Reads from RGBD camera and spits out (colorized) depth to the projector.
Reloads projector intr + extr calibrations from their respective
files frequently so they can be hand-edited for fine tuning.

Significant reference to
https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/pyglet_pointcloud_viewer.py
'''

# Subsample on depth image size
sN = 1

def get_extrinsics():
    return np.loadtxt("extrinsics.csv")
    
def get_intrinsics():
    intr_3x3 = np.loadtxt("projector_intrinsics.csv")
    intr_4x4 = np.eye(4)
    intr_4x4[:3, :3] = intr_3x3
    return intr_4x4

def get_fov():
    fov = np.loadtxt("projector_fov.csv")
    return fov
    


def convert_fmt(fmt):
    """rs.format to pyglet format string"""
    return {
        rs.format.rgb8: 'RGB',
        rs.format.bgr8: 'BGR',
        rs.format.rgba8: 'RGBA',
        rs.format.bgra8: 'BGRA',
        rs.format.y8: 'L',
    }[fmt]


realsense_manager = RealsenseHandler()

# pyglet
window = pyglet.window.Window(
    config=gl.Config(
        double_buffer=True,
        #samples=8  # MSAA,
    ),
    fullscreen=True, vsync=True)
keys = pyglet.window.key.KeyStateHandler()
window.push_handlers(keys)

# Create a VertexList to hold pointcloud data
# Will pre-allocates memory according to the attributes below
w = int(realsense_manager.w / sN)
h = int(realsense_manager.h / sN)
vertex_list = pyglet.graphics.vertex_list(
    w * h, 'v3f/stream', 't2f/stream', 'n3f/stream')
# Create and allocate memory for our color data
image_data = pyglet.image.ImageData(w, h, 'RGB', (gl.GLubyte * (w * h * 3))())

# FPS display
fps_display = pyglet.window.FPSDisplay(window)

def on_key_press(symbol, modifiers):
    if symbol == pyglet.window.key.Q:
        window.close()            
window.push_handlers(on_key_press)

@window.event
def on_draw():
    window.clear()
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_LINE_SMOOTH)

    width, height = window.get_size()
    print("On draw %dx%d" % (width, height))
    gl.glViewport(0, 0, width, height)

    # Set view intrinsics to exactly projector intrinsics
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    gl.gluPerspective(get_fov(), width / float(height), 0.01, 20.)
    #gl_better.glMultMatrixd(get_intrinsics())


    gl.glMatrixMode(gl.GL_TEXTURE)
    gl.glLoadIdentity()
    # texcoords are [0..1] and relative to top-left pixel corner, add 0.5 to center
    gl.glTranslatef(0.5 / image_data.width, 0.5 / image_data.height, 0)
    image_texture = image_data.get_texture()
    # texture size may be increased by pyglet to a power of 2
    tw, th = image_texture.owner.width, image_texture.owner.height
    gl.glScalef(image_data.width / float(tw),
                image_data.height / float(th), 1)

    # Set view extrinsics to projector offset in RGBD frame
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    gl_better.glMultMatrixd(get_extrinsics())
    #gl.glTranslated(0., 0., -20.)
    gl.glRotated(180., 0., 1., 0.)
    gl.glRotated(180., 0., 0., 1.)
    '''
    try:
        x, y, z, r, p, y = get_extrinsics()
        gl_better.glRotated(y, 0., 0., 1.)
        gl_better.glRotated(p, 0., 1., 0.)
        gl_better.glRotated(r, 1., 0., 0.)
        gl_better.glTranslated(x, y, z)
    except ValueError as e:
        print("skipping extrinsics this time")
    '''

    gl.glPointSize(10.)
    distance = (1., 0, 0.)
    gl.glPointParameterfv(gl.GL_POINT_DISTANCE_ATTENUATION,
                        (gl.GLfloat * 3)(*distance))

    gl.glColor3f(1, 1, 1)
    texture = image_data.get_texture()
    gl.glEnable(texture.target)
    gl.glBindTexture(texture.target, texture.id)
    gl.glTexParameteri(
        gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

    # comment this to get round points with MSAA on
    gl.glEnable(gl.GL_POINT_SPRITE)

    print("Drawing vertex list")
    vertex_list.draw(gl.GL_POINTS)
    print("Done drawing vertex list")
    gl.glDisable(texture.target)
    gl.glDisable(gl.GL_LIGHTING)

    # Draw fps display in a corner
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    gl.glOrtho(0, width, 0, height, -1, 1)
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    gl.glMatrixMode(gl.GL_TEXTURE)
    gl.glLoadIdentity()
    gl.glDisable(gl.GL_DEPTH_TEST)

    fps_display.draw()
    print("Done with draw")


def run(dt):
    global w, h, sN
    window.set_caption("RealSense (%dx%d) %dFPS (%.2fms)" %
                       (w, h, 0 if dt == 0 else 1.0 / dt, dt * 1000))

    print("Waiting for frame")
    color_image, depth_image, points = realsense_manager.get_frame(include_pointcloud=True)
    #plt.imsave("curr.png", color_image[::-1, ::-1, ::-1])
    depth_image = depth_image[::sN, ::sN]
    print("Max/mean depth: ", np.max(depth_image)/1000., np.mean(depth_image)/1000.)
    min_depth = 1.0*1000
    max_depth = 2.0*1000
    color_source = plt.get_cmap("hsv")((depth_image - min_depth)/(max_depth-min_depth))[:, :, :3]
    color_source = np.uint8(color_source * 255)

    print("Color range: ", np.min(color_source), np.max(color_source))
    global image_data
    
    # copy image data to pyglet
    if (image_data.format, image_data.pitch) != ("RGB", color_source.strides[0]):
        image_w, image_h = w, h

        empty = (gl.GLubyte * (image_w * image_h * 3))()
        image_data = pyglet.image.ImageData(image_w, image_h, "RGB", empty)
        print("Changing format to ", image_w, image_h, "RGB", empty)
        print("Image data pitch: ", image_data.pitch, " vs", color_source.strides[0])
    image_data.set_data("RGB", color_source.strides[0], color_source.ctypes.data)

    verts = np.asarray(points.get_vertices(2)).reshape(h*sN, w*sN, 3)
    verts = verts[::sN, ::sN, :]
    texcoords = np.asarray(points.get_texture_coordinates(2)).reshape(h*sN, w*sN, 2)
    texcoords = texcoords[::sN, ::sN, :]

    if len(vertex_list.vertices) != verts.size:
        vertex_list.resize(verts.size // 3)
        # need to reassign after resizing
        vertex_list.vertices = verts.ravel()
        vertex_list.tex_coords = texcoords.ravel()
    # copy our data to pre-allocated buffers, this is faster than assigning...
    # pyglet will take care of uploading to GPU
    def copy(dst, src):
        """copy numpy array to pyglet array"""
        np.array(dst, copy=False)[:] = src.ravel()

    copy(vertex_list.vertices, verts)
    copy(vertex_list.tex_coords, texcoords)

pyglet.clock.schedule(run)
pyglet.app.run()

        