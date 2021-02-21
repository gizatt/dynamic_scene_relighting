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

from realsense_handler import RealsenseHandler

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
        color_image, depth_image, _ = realsense_manager.get_frame()
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
            

