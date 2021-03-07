import glob
import os
import random
import time
import sys
import logging

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better
import mediapipe as mp
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

from realsense_handler import RealsenseHandler
from window_manager import *
from interface import InterfaceManager
from calibration_utils import get_extrinsics, get_projector_intrinsics


'''
Reads from RGBD camera and spits out (colorized) depth to the projector.
Reloads projector intr + extr calibrations from their respective
files frequently so they can be hand-edited for fine tuning.

Significant reference to
https://github.com/IntelRealSense/librealsense/blob/development/wrappers/python/examples/pyglet_pointcloud_viewer.py
'''

def meshcat_draw_frustrum(vis, TF, K, near_distance, far_distance, w, h):
    # TODO(gizatt): This obviously isn't right -- the projected
    # light doesn't match the drawn view frustrum.
    # Not dealing with, for now; I think the issue is a combination
    # of bad intrinsics and bugs related to flipped image coordinates
    # somewhere along the pipeline.
    image_bbox_verts = np.array([
        [0., w, w, 0.],
        [0., 0., h, h]
    ])
    TF_inv = np.eye(4)
    TF_inv[:3, :3] = TF[:3, :3].T
    TF_inv[:3, 3] = -TF_inv[:3, :3].dot(TF[:3, 3])
    TF = TF_inv
            
    N = image_bbox_verts.shape[1]
    Kinv = np.linalg.inv(K)
    def project_bbox_verts(dist):
        homog = np.concatenate(
            [image_bbox_verts*dist, dist*np.ones((1, N))],
            axis=0
        )
        pts = np.dot(Kinv, homog)
        return ((TF[:3, :3].dot(pts)).T + TF[:3, 3]).T
    near_pts = project_bbox_verts(near_distance)
    far_pts= project_bbox_verts(far_distance)
    near_colors = np.zeros((3, N))
    near_colors[1, :] = 1.
    far_colors = np.zeros((3, N))
    far_colors[2, :] = 1.
    
    vis['frustrum']['near'].set_object(g.LineLoop(
        g.PointsGeometry(near_pts, color=near_colors),
        g.MeshBasicMaterial(vertexColors=True, linewidth=0.1)))
    vis['frustrum']['far'].set_object(g.LineLoop(
        g.PointsGeometry(far_pts, color=far_colors),
        g.MeshBasicMaterial(vertexColors=True, linewidth=0.1)))
    connecting = np.zeros((3, N*2))
    connecting[:, ::2] = near_pts
    connecting[:, 1::2] = far_pts
    connecting_colors = np.zeros((3, N*2))
    connecting_colors[:, ::2] = near_colors
    connecting_colors[:, 1::2] = far_colors
    vis['frustrum']['connecting'].set_object(g.LineSegments(
        g.PointsGeometry(connecting, color=connecting_colors),
        g.MeshBasicMaterial(vertexColors=True, linewidth=1.)
    ))

def meshcat_draw_pointcloud(vis, verts, colors):
    color_vis = colors[:, :, ::-1].astype(np.float32) / 255.
    verts_vis = verts.reshape(-1, 3).T
    color_vis = color_vis.reshape(-1, 3).T
    vis.set_object(g.Points(
        g.PointsGeometry(verts_vis, color=color_vis),
        g.PointsMaterial(size=0.01)
    ))

def meshcat_draw_lights(vis, light_locations):
    N = light_locations.shape[1]
    vis.set_object(g.Points(
        g.PointsGeometry(
            light_locations,
            color=np.stack([np.array([1., 0.8, 0.7])]*N, axis=-1))
    ))

if __name__ == "__main__":    
    logging.basicConfig(filename='../out/info.log', filemode='w', level=logging.DEBUG)

    # First connect to meshcat: this'll block if there's no server + display a helpful
    # reminder for me to start a server.
    print("Opening meshcat server at default url: make sure meshcat-server is running.")
    vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    camera_frame_vis = vis["camera_frame"]
    # Makes point cloud look "upright" in meshcat vis
    camera_frame_vis.set_transform(tf.rotation_matrix(np.pi/2, [1., 0., 0]))

    meshcat_draw_frustrum(camera_frame_vis,
        TF=get_extrinsics(),
        K=get_projector_intrinsics(),
        near_distance=0.025,
        far_distance=1.,
        w=1280,
        h=768
    )

    interface = InterfaceManager()

    # Subsample on depth image size
    sN = 4
    realsense_manager = RealsenseHandler(
        resolution=(640, 480),
        framerate=30,
        decimation_magnitude=sN
    )
    face_mesh_detector = mp_face_mesh.FaceMesh(
        min_detection_confidence=0.3,
        min_tracking_confidence=0.3)

    k = 0
    def on_idle(cls):
        global k, camera_frame_vis
        k += 1

        if not interface.is_alive():
            raise StopIteration

        realsense_manager.set_exposure(interface.get_exposure())

        logging.info("Waiting for frame")
        color_full, depth_image, points = realsense_manager.get_frame(include_pointcloud=True, do_alignment=False)
        color_image = color_full[::sN, ::sN, ::-1]
        h, w = color_image.shape[:2]
        verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)

        if interface.get_meshcat_vis_active():
            meshcat_draw_pointcloud(camera_frame_vis, verts, color_image)
        
        divide_rate = interface.get_image_save_rate()
        if divide_rate > 0 and k % divide_rate == 0:
            logging.info("Sending downsampled pts to meshcat and saving color image")
            plt.imsave("../out/curr_color.png", color_full[::-1, ::-1, ::-1])
            #cv2.imwrite("../out/curr_depth.png" % k, depth_image[::-1, ::-1].astype(np.uint16))
        logging.info("Depth range:[%f, %f]", np.max(depth_image)/1000., np.mean(depth_image)/1000.)
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
            logging.info("T: %f", t)
            light_direction = np.array([2.*np.cos(t), 2.*np.sin(t), 1.0])
            light_direction /= np.linalg.norm(light_direction)
            normal_alignment = (light_direction * normals).sum(axis=-1)
            brightness = np.tanh(normal_alignment*2. - 1.)*0.5 + 0.5
            brightness[depth_image < min_depth] = 0.
            brightness[depth_image > max_depth] = 0.

            #brightness = np.repeat(brightness[:, :, np.newaxis], 4, axis=-1)
            #brightness_color_source = np.uint8(brightness * 255)

        results = face_mesh_detector.process(color_image)
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                logging.info("Had a face!")
                landmark_pixel_locations = []
                for idx, landmark in enumerate(face_landmarks.landmark):
                    if ((landmark.HasField('visibility') and
                        landmark.visibility < VISIBILITY_THRESHOLD) or
                        (landmark.HasField('presence') and
                        landmark.presence < PRESENCE_THRESHOLD)):
                        continue
                    landmark_px = mp_drawing._normalized_to_pixel_coordinates(
                        landmark.x, landmark.y, h, w)
                    landmark_pixel_locations.append(landmark_px)
                logging.info("Pixel locations: %s", str(landmark_pixel_locations))

        #plt.imsave("out/curr_depth.png", depth_color_source[::-1, ::-1, ::-1])
        color_source =  depth_color_source
        #color_source = np.ones(color_image.shape[:2] + (4,))
        color_source[:, :, :-1] = normals*0.5 + 0.5
        color_source[:, :, -1] = brightness
        #color_source = depth_color_source
        # copy image data to pyglet
        cls.update_geometry(verts, color_source)

    wm = WindowManager(callback=on_idle)
    wm.start()