Reactive Scene Relighting with Projection Mapping
-------------------------------------

# Dependencies

Mobile unit is a Rbpi 4b running Ubuntu 20.04, with an HDMI project
as the monitor and a Realsense D415 as the RGBD sensor.

Install from apt `build-essential`

Install apriltag core library from `https://github.com/AprilRobotics/apriltag/tree/3.1.1`, following directions.

Install `librealsense` packages from source following  [https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md]. When doing CMake config, make sure the apriltag library is found, enable any examples you want (recommend `BUILD_EXAMPLES` and `BUILD_GRAPHICAL_EXAMPLES` so you at least get `realsense-viewer`), and `BUILD_PYTHON_BINDINGS` is on.

(Maybe follow [ROS Noetic install-from-apt instructions]. I use ` ros-noetic-desktop` for dev, but I suspect `ros-noetic-ros-base` will have everything you need to run this. But I don't require this yet, I'm seeing if I can avoid having ROS in there...)


For Python, need Python3, `tkinter`, `pillow`.

# Commands to run

Via SSH (default 192.168.0.142 on home net))
1) SSH in
2) `export DISPLAY=:0.0`
3) Invoke script with python3.

Natively:
1) Just invoke with python3.

