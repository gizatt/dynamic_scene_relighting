import tkinter as tk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
import os
import cv2
import numpy as np
from apriltag import apriltag
import pyrealsense2.pyrealsense2 as rs



'''
Opens a fullscreen window and repeatedly:
- Shows an apriltag at a random translation / rotation, recording the corner locations in pixels
- Reads from the realsense camera, loading in an RGBD frame
- Detects apriltags in the detected frame, recording the detected pixel locations
- Appends the [sent, observed] pixel location pairs to a file.
'''

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

    img_path = "images/tag36_11_%05d.png" % 0
    assert os.path.isfile(img_path), img_path
    img = Image.open(img_path)
    img = img.resize((int(im_size/2), int(im_size/2)), Image.NEAREST)
    img = ImageTk.PhotoImage(img)
    canvas.create_image(0,0, anchor=tk.NW, image=img)


    detector = apriltag(family="tag36h11", blur=0.8, debug=1)
    while (1):
        # Try to detect it.
        color_image, depth_image = realsense_manager.get_frame()
        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        plt.imsave("color_image.png", color_image_rgb)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray_image)
        print("DETECTIONS:", detections)
        root.update()
