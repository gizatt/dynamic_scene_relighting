Reactive Scene Relighting with Projection Mapping
-------------------------------------

# Dependencies

Mobile unit is a Rbpi 4b running Ubuntu 20.04, with an HDMI project
as the monitor and a Realsense D415 as the RGBD sensor.

From apt: `git python3 python3-pip python3-tk ros-base-dev ros-base-python-dev`
(This should be ROS Melodic, at least as of time of writing.)

Librealsense, for the camera driver:
1) Install `librealsense` packages from [https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages]
2) Install realsense ROS package following from-source instructions [https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md]




# Commands to run

Via SSH:
1) SSH in
2) `export DISPLAY=:0.0`
3) Invoke script with python3.

Natively:
1) Just invoke with python3.

