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
Reads from RGBD camera and spits out images to the projector.
Reloads projector intr + extr calibrations from their respective
files frequently so they can be hand-edited for fine tuning.

Support demo modes:
- Colorize by depth.
- Colorize by normal.
- Relight according to a rotating light.
- Relight according to a fixed set of lights.

Visualizes point cloud and lights (when present) to meshcat.

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

    # Draw a little box for the projector :)
    vis['projector'].set_object(
        g.Box([0.1, 0.1, 0.1]),
        g.MeshLambertMaterial(
            color=0xaaffaa))

def meshcat_draw_pointcloud(vis, verts, colors):
    color_vis = colors[:, :, ::-1].astype(np.float32) / 255.
    verts_vis = verts.reshape(-1, 3).T
    color_vis = color_vis.reshape(-1, 3).T
    vis.set_object(g.Points(
        g.PointsGeometry(verts_vis, color=color_vis),
        g.PointsMaterial(size=0.01)
    ))

def meshcat_draw_lights(vis, light_locations, light_attenuations):
    N = light_locations.shape[1]
    colors = np.zeros((3, N))
    colors[2, :] = light_attenuations
    for k in range(N):
        vis["lights"]["%d" % k].set_object(
            g.Sphere(radius=0.05),
            g.MeshLambertMaterial(
                             color=0xffffff,
                             reflectivity=0.8))
        vis["lights"]["%d" % k].set_transform(
            tf.translation_matrix(light_locations[:, k]))


def colorize_depth(depth_image, min_depth, max_depth, cmap_name="hsv"):
    return plt.get_cmap("hsv")((depth_image - min_depth)/(max_depth-min_depth))
        
def calc_normals(verts):
    dy, dx = np.gradient(verts, axis=(0, 1))
    n = np.cross(dx, dy)
    norm = np.sqrt((n*n).sum(axis=2, keepdims=True))
    normals = np.divide(n, norm, out=n, where=norm != 0)
    normals = np.ascontiguousarray(normals)
    return normals

def calc_light_brightness(verts, light_locations, light_attenuations):
    # Light locations should be a 3xN list of light locations.
    # This should *really* be done in a shader, not by hand like this.
    normals = calc_normals(verts)

    N = light_locations.shape[1]
    assert light_locations.shape[0] == 3, light_locations.shape
    assert len(light_attenuations) == N

    total_brightness = np.zeros(normals.shape[:2])

    for k in range(N):
        displacements = verts - light_locations[:, k]
        distances = np.square(displacements).sum(axis=-1)
        # Normalize displacements
        norm_displacements = displacements / np.dstack([np.sqrt(distances)]*3)
        alignments = np.clip((norm_displacements * normals).sum(axis=-1), 0., 1.)
        brightness_diffuse = alignments/(1. + light_attenuations[k]*distances)
        total_brightness += brightness_diffuse
        if light_attenuations[k] > 0.:
            # "Ambient" being much more generous about allowing light at 
            # slanted normals
            brightness_ambient = np.tanh(alignments*10.)/(light_attenuations[k]*distances)
            total_brightness += brightness_ambient

    return np.clip(total_brightness, 0., 1.)

def generate_light_info(pattern_name, **kwargs):
    if pattern_name == "orbiting single light":
        xyz_center = kwargs["xyz_center"]
        xyz_amplitude = kwargs["xyz_amplitude"]
        a = kwargs["attenuation"]
        light_location = xyz_center + np.array([
            np.cos(time.time()),
            np.sin(time.time()),
            np.sin(time.time())]) * xyz_amplitude
        light_locations = light_location.reshape(3, 1)
        light_attenuations = [a]
    elif pattern_name == "soft face lights":
        light_locations = np.array([
            [-0.3, 0.3, 0.8],
            [-0.3, -0.3, 0.8],
            [0.3, -0.3, 0.8],
            [0.3, 0.3, 0.8],
        ]).T
        light_attenuations = [40.0, 40.0, 40.0, 40.0]
    else:
        raise NotImplementedError("Rotating light pattern_name %s" % pattern_name)
    return light_locations, light_attenuations

if __name__ == "__main__":    
    logging.basicConfig(filename='../out/info.log', filemode='w', level=logging.DEBUG)

    # First connect to meshcat: this'll block if there's no server + display a helpful
    # reminder for me to start a server.
    print("Opening meshcat server at default url: make sure meshcat-server is running.")
    vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    vis.delete()
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

    modes = [
        "Off",
        "Depth recoloring",
        "Normal recoloring",
        "Depth-colored rotating light",
        "Orbiting hard light",
        "Orbiting soft light",
        "Moving light",
        "soft face lights"
    ]
    interface = InterfaceManager(modes=modes)

    # Subsample on depth image size
    sN = 4
    realsense_manager = RealsenseHandler(
        resolution=(640, 480),
        framerate=30,
        decimation_magnitude=sN,
        spatial_smooth=False
    )
    face_mesh_detector = mp_face_mesh.FaceMesh(
        min_detection_confidence=0.3,
        min_tracking_confidence=0.3)

    k = 0
    last_time = time.time()
    fps_est = 0.
    def on_idle(cls):
        global k, camera_frame_vis, last_time, fps_est
        k += 1
        now_time = time.time()
        dt = now_time - last_time
        last_time = now_time
        fps_est = fps_est * 0.9 + (1. / dt)*0.1

        if not interface.is_alive():
            raise StopIteration

        # Update camera config based on interface changes.
        realsense_manager.set_exposure(interface.get_exposure())

        # Grab a new frame and process it.
        color_full, depth_image, points = realsense_manager.get_frame(include_pointcloud=True, do_alignment=False)
        color_image = color_full[::sN, ::sN, :]
        h, w = color_image.shape[:2]
        verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)

        # Do drawing and saving as requested.
        if interface.get_meshcat_vis_active():
            meshcat_draw_pointcloud(camera_frame_vis, verts, color_image)
        
        divide_rate = interface.get_image_save_rate()
        if divide_rate > 0 and k % divide_rate == 0:
            logging.info("Sending downsampled pts to meshcat and saving color image")
            plt.imsave("../out/curr_color.png", color_full[::-1, ::-1, ::-1])
            #cv2.imwrite("../out/curr_depth.png" % k, depth_image[::-1, ::-1].astype(np.uint16))
        
        interface.report_camera_stats(
            fps=fps_est,
            min=np.min(depth_image)/1000.,
            max=np.max(depth_image)/1000.)

        depth_range = interface.get_depth_range()
        min_depth = depth_range[0]*1000
        max_depth = depth_range[1]*1000

        # Finally, branch by demo mode.
        mode_name = interface.get_demo_mode()
        light_locations = None
        if mode_name == "Off":
            color_source = np.zeros(depth_image.shape[:2] + (4,))
        elif mode_name == "Depth recoloring":
            color_source = colorize_depth(depth_image, min_depth, max_depth)
            color_source[:, :, 3] = np.logical_and(
                depth_image >= min_depth, depth_image <= max_depth)
        elif mode_name == "Normal recoloring":
            normals = calc_normals(verts)
            color_source = np.ones(normals.shape[:2] + (4,))
            color_source[:, :, :3] = normals*0.5 + 0.5
        elif mode_name == "Depth-colored rotating light":
            color_source = colorize_depth(depth_image, min_depth, max_depth)
            light_locations, light_attenuations = generate_light_info(
                pattern_name="orbiting single light",
                xyz_center=np.array([0., 0., 0.5]),
                xyz_amplitude=np.array([0.8, 0.8, 0.8]),
                attenuation=0.0)
            brightness = calc_light_brightness(
                verts, light_locations, light_attenuations)
            brightness *= np.logical_and(
                depth_image >= min_depth, depth_image <= max_depth)
            color_source[:, :, -1] = brightness
        elif mode_name == "Orbiting hard light":
            light_locations, light_attenuations = generate_light_info(
                pattern_name="orbiting single light",
                xyz_center=np.array([0., 0., 0.7]),
                xyz_amplitude=np.array([0.3, 0.3, 0.]),
                attenuation=0.0)
            brightness = calc_light_brightness(verts, light_locations, light_attenuations)
            color_source = np.dstack([brightness]*4)
        elif mode_name == "Orbiting soft light":
            light_locations, light_attenuations = generate_light_info(
                pattern_name="orbiting single light",
                xyz_center=np.array([0., 0., 0.7]),
                xyz_amplitude=np.array([0.3, 0.3, 0.]),
                attenuation=10.0)
            brightness = calc_light_brightness(verts, light_locations, light_attenuations)
            color_source = np.dstack([brightness]*4)
        elif mode_name == "Moving light":
            light_locations, light_attenuations = generate_light_info(
                pattern_name="orbiting single light",
                xyz_center=np.array([0., 0., 0.9]),
                xyz_amplitude=np.array([0.0, 0.0, 0.3]),
                attenuation=25.0)
            brightness = calc_light_brightness(verts, light_locations, light_attenuations)
            color_source = np.dstack([brightness]*4)
        elif mode_name == "soft face lights":
            light_locations, light_attenuations = generate_light_info(
                pattern_name="soft face lights")
            brightness = calc_light_brightness(verts, light_locations, light_attenuations)
            brightness *= np.logical_and(
                depth_image >= min_depth, depth_image <= max_depth)
            color_source = np.dstack([brightness]*4)
        else:
            logging.error("Bad mode name: %s" % mode_name)
            return

        if light_locations is not None:
            meshcat_draw_lights(camera_frame_vis, light_locations, light_attenuations)
        else:
            camera_frame_vis["lights"].delete()

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

        #color_source = depth_color_source
        # copy image data to pyglet
        cls.update_geometry(verts, color_source)

    wm = WindowManager(callback=on_idle)
    wm.start()