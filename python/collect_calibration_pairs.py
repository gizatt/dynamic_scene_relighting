import glob
import os
import random
import time
import sys
import asyncio 

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from apriltag import apriltag

from .realsense_handler import RealsenseHandler

'''
Opens a fullscreen window and repeatedly:
- Shows an apriltag at a random translation / rotation, recording the corner locations in pixels
- Reads from the realsense camera, loading in an RGBD frame
- Detects apriltags in the detected frame, recording the detected pixel locations
- Appends the [sent, observed] pixel location pairs to a file.
'''

realsense_manager = RealsenseHandler()

# Make a fullscreen pyglet window
window = pyglet.window.Window(
    config=gl.Config(
        double_buffer=True,
        #samples=8  # MSAA,
    ),
    fullscreen=True, vsync=True)

# Create and allocate memory for the apriltag
tag_path = "../images/tag36_11_%05d.png" % 0
base_tag = pyglet.image.load(tag_path)
tag_w, tag_h = base_tag.width, base_tag.height


# Get ready to do detections
detector = apriltag(family="tag36h11", debug=True)


apriltag_x = 0
apriltag_y = 0
apriltag_size = 0
@window.event
def on_draw():
    # Draw the apriltag image on a quad with vert edges
    # from the vert buffer.
    
    gl.glClearColor(1.0, 0.0, 1.0, 1.0)
    window.clear()

    width, height = window.get_size()
    print("On draw %dx%d" % (width, height))
    gl.glViewport(0, 0, width, height)

    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    gl.glOrtho(0, width, 0, height, -1, 1)
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    
    texture = base_tag.get_texture()
    gl.glTexParameteri(texture.target, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
    gl.glTexParameteri(texture.target, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

    base_tag.blit(apriltag_x, apriltag_y, width=apriltag_size, height=apriltag_size)
    print("Done with draw")

last_sent_corners_time = time.time() - 1000.
projected_corners = None
def send_new_corners():
    global window, projected_corners, apriltag_x, apriltag_y, apriltag_size
    width, height = window.get_size()
    # Randomly choose a size and location for the apriltag
    size_log_2 = random.randrange(7, 9)
    size = 2**(size_log_2)
    assert size < width, size < height
    x = random.randrange(0, width-size)
    y = random.randrange(0, height-size)
    
    apriltag_x = x
    apriltag_y = y
    apriltag_size = size
    print("X (%d), Y (%d), size (%d)" % (x, y, size))
    projected_corners = np.array([[x, y],
                            [x+size, y],
                            [x+size, y+size],
                            [x, y+size]])
    last_sent_corners_time = time.time()

def log_detections():
    global projected_corners, window
    # Try to detect it from the realsense.
    color_image, depth_image, points = realsense_manager.get_frame(include_pointcloud=True)
    verts = np.asarray(points.get_vertices(2)).reshape(depth_image.shape[0], depth_image.shape[1], 3)
    plt.imsave("depth_image.png", depth_image)
    pyglet.image.get_buffer_manager().get_color_buffer().save("buffer_dump.png")
    image_window = cv2.cvtColor(cv2.imread("buffer_dump.png"), cv2.COLOR_BGR2GRAY)
    sanity_detections = detector.detect(image_window)
    print("Sanity check detections: ", sanity_detections)

    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    plt.imsave("color_image.png", color_image_rgb)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray_image)
    print("REALSENSE DETECTIONS:", detections)
    if len(detections) == 1 and detections[0]["id"] == 0 and len(sanity_detections) == 1 and sanity_detections[0]["id"] == 0:
        # Passed sanity check, so record this calibration pair.
        sanity_detection = sanity_detections[0]
        detection = detections[0]
        with open("../data/calibration_pairs.csv", "a") as f:
            for k in range(4):
                #u1, v1 = projected_corners[k, :]
                u1, v1 = sanity_detection['lb-rb-rt-lt'][k, :]

                # Collect detection in realsense frame and project
                # out to a camera-frame XYZ triplet using inverse
                # RGBD intrinsics.
                u2, v2 = detection['lb-rb-rt-lt'][k, :]
                x2, y2, z2 = verts[int(v2), int(u2)]
                f.write("%f, %f, %f, %f, %f, %f, %f\n" % (u1, v1, u2, v2, x2, y2, z2))
        print("GOOD")

def run(dt):
    global projected_corners
    if (time.time() - last_sent_corners_time) > 0.5:
        if projected_corners is not None:
            log_detections()
        send_new_corners()

pyglet.clock.schedule(run)
pyglet.app.run()
