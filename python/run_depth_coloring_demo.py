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

    k = 0
    def update_geometry(cls):
        global k
        k += 1
        print("Waiting for frame")
        color_full, depth_image, points = realsense_manager.get_frame(include_pointcloud=True, do_alignment=False)
        color_image = color_full[::sN, ::sN, ::-1]
        h, w = color_image.shape[:2]
        verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)
        #verts = verts[::sN, ::sN, :]
        print(verts.shape)


        #if k % 30 == 0:
            #plt.imsave("../out/curr_color_%03d.png" % k, color_full[::-1, ::-1, ::-1])
            #cv2.imwrite("../out/curr_depth_%03d.png" % k, depth_image[::-1, ::-1].astype(np.uint16))
        print("Max/mean depth: ", np.max(depth_image)/1000., np.mean(depth_image)/1000.)
        min_depth = 0.90*1000
        max_depth = 1.1*1000
        
        depth_color_source = plt.get_cmap("hsv")((depth_image - min_depth)/(max_depth-min_depth))
        #depth_color_source = np.uint8(depth_color_source * 255)
        color_source = depth_color_source

        # Very inefficient normal estimation
        if True:
            dy, dx = np.gradient(verts, axis=(0, 1))
            n = np.cross(dx, dy)
            norm = np.sqrt((n*n).sum(axis=2, keepdims=True))
            normals = np.divide(n, norm, out=n, where=norm != 0)
            normals = np.ascontiguousarray(normals)
            #normal_image = np.uint8(normals * 127 + 127)
            plt.imsave("../out/curr_normals.png", normals[::1, ::-1, :]/2. + 0.5)

            # Simulate a light orbiting around the z axis by brighting
            # base on dot product of a time-varying normal and the
            # normal image.
            t = k * np.pi/15
            print("T: ", t)
            light_direction = np.array([2.*np.cos(t), 2.*np.sin(t), 1.0])
            light_direction /= np.linalg.norm(light_direction)
            normal_alignment = (light_direction * normals).sum(axis=-1)
            brightness = np.tanh(normal_alignment*2. - 1.)*0.5 + 0.5
            brightness[depth_image < min_depth] = 0.
            brightness[depth_image > max_depth] = 0.

            #brightness = np.repeat(brightness[:, :, np.newaxis], 4, axis=-1)
            #brightness_color_source = np.uint8(brightness * 255)

        #plt.imsave("out/curr_depth.png", depth_color_source[::-1, ::-1, ::-1])
        color_source =  depth_color_source
        #color_source = np.ones(color_image.shape[:2] + (4,))
        color_source[:, :, :-1] = normals*0.5 + 0.5
        color_source[:, :, -1] = brightness
        #color_source = depth_color_source
        # copy image data to pyglet
        cls.update_geometry(verts, color_source)

    wm = WindowManager(callback=update_geometry)
    wm.start()