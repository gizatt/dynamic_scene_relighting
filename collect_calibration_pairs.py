import glob
import os
import random
import time
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk

from apriltag import apriltag
import pyrealsense2.pyrealsense2 as rs


'''
Opens a fullscreen window and repeatedly:
- Shows an apriltag at a random translation / rotation, recording the corner locations in pixels
- Reads from the realsense camera, loading in an RGBD frame
- Detects apriltags in the detected frame, recording the detected pixel locations
- Appends the [sent, observed] pixel location pairs to a file.
'''

"""
Returns a camera matrix K from librealsense intrinsics
From librealsense/wrappers/python/examples/t265_stereo.py
"""
def to_camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

class RealsenseHandler():
    '''
        Handles core realsense functionality: reading in images
        and doing rectification.
        Based on
        https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
    '''
    def __init__(self):
        self.pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Detected depth Scale is: " , self.depth_scale)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # We'll want to store the intrinsics once we start getting images.
        self.aligned_depth_K = None
    
    def get_frame(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            print("Invalid aligned or color frame.")
            return

        if self.aligned_depth_K is None:
            depth_intrinsics = rs.video_stream_profile(aligned_depth_frame.profile).get_intrinsics()
            self.aligned_depth_K = to_camera_matrix(depth_intrinsics)
            self.aligned_depth_inv = np.linalg.inv(self.aligned_depth_K)
            np.savetxt("d415_intrinsics.csv", self.aligned_depth_K)

        # Extract aligned depth frame intrinsics
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image
        

if __name__ == "__main__":
    realsense_manager = RealsenseHandler()

    # Make a fullscreen window
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    root.bind('<Escape>',lambda e: root.destroy())
    w, h = root.winfo_screenwidth(), root.winfo_screenheight()
    im_size = min(w, h)

    # Show an apriltag image
    canvas = tk.Canvas(root, width=w, height=h)      
    canvas.pack()      
    canvas.configure(background='black')

    tag_paths = ["images/tag36_11_%05d.png" % k for k in range(3)]
    base_tags = [Image.open(path) for path in tag_paths]


    detector = apriltag(family="tag36h11", debug=True)
    while (1):
        # Put a random apriltag at a random location.
        canvas.delete("all")
        base_tag_k = random.randrange(3)
        base_tag = base_tags[base_tag_k]
        size = random.randrange(128, 1024)
        tag_img = base_tag.resize((size, size), Image.NEAREST)
        img = ImageTk.PhotoImage(tag_img)
        x = random.randrange(0, w-size)
        y = random.randrange(0, h-size)
        canvas.create_image(x,y, anchor=tk.NW, image=img)
        
        # Get projected image as an image
        canvas.postscript(file = "projected.eps")
        projected_image = Image.open("projected.eps").convert('L').resize((w, h))
        root.update()
        
        projected_corners = np.array([[x, y+size],
                              [x+size, y+size],
                              [x+size, y],
                              [x, y]])
        projected_detections = detector.detect(projected_image)
        print("Detections from projected: ", projected_detections)
        print("Sanity-check: expected corners: ", projected_corners)
        if len(projected_detections) != 1 or projected_detections[0]["id"] != base_tag_k:
            print("Bad detection of projected image, skipping")
            continue
        projected_detection = projected_detections[0]
        # There's a shift between those two. Huh? That'll be a problem
        # in calibration... but I probably won't be using TKinter for much
        # longer, so I'll sleep on this and get the pipeline working.

        # Sleep to be sure the projector is doing its thing.
        time.sleep(0.5)

        # Try to detect it from the realsense.
        color_image, depth_image = realsense_manager.get_frame()
        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        plt.imsave("color_image.png", color_image_rgb)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray_image)
        print("REALSENSE DETECTIONS:", detections)
        if len(detections) == 1 and detections[0]["id"] == base_tag_k:
            # Passed sanity check, so record this calibration pair.
            detection = detections[0]
            with open("calibration_pairs.csv", "a") as f:
                for k in range(4):
                    u1, v1 = projected_detection['lb-rb-rt-lt'][k, :]

                    # Collect detection in realsense frame and project
                    # out to a camera-frame XYZ triplet using inverse
                    # RGBD intrinsics.
                    u2, v2 = detection['lb-rb-rt-lt'][k, :]
                    depth = depth_image[int(v2), int(u2)]
                    x2, y2, z2 = np.dot(realsense_manager.aligned_depth_inv,
                                        np.array([u2, v2, depth]))
                    f.write("%f, %f, %f, %f, %f, %f, %f\n" % (u1, v1, u2, v2, x2, y2, z2))
            print("GOOD")
            

