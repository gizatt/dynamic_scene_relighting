import numpy as np
import pyrealsense2.pyrealsense2 as rs
import logging

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
    def __init__(self, resolution=(640, 480), framerate=30, decimation_magnitude=1,
                 spatial_smooth=True, temporal_smooth=True):
        self.pipeline = rs.pipeline()

        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()
        w, h = resolution
        self.config.enable_stream(rs.stream.depth, w, h, rs.format.z16, framerate)
        self.config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, framerate)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.color_sensor = self.profile.get_device().first_color_sensor()
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.depth
        self.align = rs.align(align_to)

        # We'll want to store the intrinsics once we start getting images.
        self.aligned_depth_K = None

        self.filters = []
        # Decimation filter
        if decimation_magnitude > 1:
            decimate_filter = rs.decimation_filter()
            decimate_filter.set_option(rs.option.filter_magnitude, decimation_magnitude)
            self.filters.append(decimate_filter)
        
        if spatial_smooth or temporal_smooth:
            self.filters.append(rs.disparity_transform())
        if spatial_smooth:
            self.filters.append(rs.spatial_filter())
        if temporal_smooth:
            self.filters.append(rs.temporal_filter())
        if spatial_smooth or temporal_smooth:
            self.filters.append(rs.disparity_transform(False))

        # Pointcloud conversion utility.
        self.pc = rs.pointcloud()
        
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height

    def set_exposure(self, exposure):
        # Set exposure to 0 to set auto exposure
        # Exposure ranges 1-10000.
        if exposure <= 0:
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        else:
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 0)
            self.color_sensor.set_option(rs.option.exposure, exposure)

    def get_frame(self, include_pointcloud=False, do_alignment=True):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        if do_alignment:
            frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = frames.get_color_frame()

        # Apply filters
        for filter in self.filters:
            aligned_depth_frame = filter.process(aligned_depth_frame)

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            logging.error("Invalid aligned or color frame.")
            return

        if self.aligned_depth_K is None:
            depth_intrinsics = rs.video_stream_profile(aligned_depth_frame.profile).get_intrinsics()
            self.aligned_depth_K = to_camera_matrix(depth_intrinsics)
            self.aligned_depth_inv = np.linalg.inv(self.aligned_depth_K)
            np.savetxt("d415_intrinsics.csv", self.aligned_depth_K)

        # Extract aligned depth frame intrinsics
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if include_pointcloud:
            points = self.pc.calculate(aligned_depth_frame)
            self.pc.map_to(aligned_depth_frame)
        else:
            points = None

        return color_image, depth_image, points
        