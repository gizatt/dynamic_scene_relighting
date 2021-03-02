import glob
import os
import random
import time
import sys

import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from realsense_handler import RealsenseHandler
from window_manager import *


'''
Reads from RGBD camera and spits out (colorized) depth to the projector.
Reloads projector intr + extr calibrations from their respective
files frequently so they can be hand-edited for fine tuning.

Significant reference to
https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/pyglet_pointcloud_viewer.py
'''

if __name__ == "__main__":    
    # Subsample on depth image size
    sN = 4
    realsense_manager = RealsenseHandler(
        resolution=(640, 480),
        framerate=30,
        decimation_magnitude=sN
    )

    def update_geometry(cls):
        print("Waiting for frame")
        color_image, depth_image, points = realsense_manager.get_frame(include_pointcloud=True, do_alignment=False)
        color_image = color_image[::sN, ::sN, :]
        #plt.imsave("out/curr_color_%03d.png" % iteration, color_image[::-1, ::-1, :])
        h, w = color_image.shape[:2]
        verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)
        #verts = verts[::sN, ::sN, :]
        print(verts.shape)

        print("Max/mean depth: ", np.max(depth_image)/1000., np.mean(depth_image)/1000.)
        min_depth = 0.5*1000
        max_depth = 1.2*1000
        
        depth_color_source = plt.get_cmap("hsv")((depth_image - min_depth)/(max_depth-min_depth))
        #depth_color_source = np.uint8(depth_color_source * 255)
        
        #plt.imsave("out/curr_depth.png", depth_color_source[::-1, ::-1, ::-1])
        #color_source =  depth_color_source

        # copy image data to pyglet
        color_source = depth_color_source
        cls.update_geometry(verts, color_source)

    wm = WindowManager(callback=update_geometry)
    wm.start()