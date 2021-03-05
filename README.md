Reactive Scene Relighting with Projection Mapping
-------------------------------------

# Dependencies

Mobile unit is a Rbpi 4b running Ubuntu 20.04, with an HDMI project
as the monitor and a Realsense D415 as the RGBD sensor.

Use a common venv for all of these steps -- mediapipe wants to install into a venv,
and I haven't figured out how to trick it into producing libraries you can sudo-install
without also building opencv from scratch.

Follow [https://www.dedoimedo.com/computers/rpi4-ubuntu-mate-hw-video-acceleration.html] to get hardware acceleration working.

Install from apt `build-essential libopencv-dev python python3 pil.imagetk python3-opencv libgl1-dev freeglut3 freeglut3-dev libglew-dev mesa-utils`.

Install apriltag core library from `https://github.com/AprilRobotics/apriltag/` (on ~master, or recent enough to have python bindings), following directions. Make sure the python bindings get installed: you may need to `mkdir -p ~/.local/lib/<python version>/site-packages`.

Install `librealsense` packages from source following  [https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md]. When doing CMake config, make sure the apriltag library is found, enable any examples you want (recommend `BUILD_EXAMPLES` and `BUILD_GRAPHICAL_EXAMPLES` so you at least get `realsense-viewer`), and `BUILD_PYTHON_BINDINGS` is on.

(Maybe follow [ROS Noetic install-from-apt instructions]. I use ` ros-noetic-desktop` for dev, but I suspect `ros-noetic-ros-base` will have everything you need to run this. But I don't require this yet, I'm seeing if I can avoid having ROS in there...)

For Python, need Python3, `pillow scipy numpy pyglet pyopengl`.

For the face detector, I'm trying:
    - [tflite](https://www.tensorflow.org/lite/guide/python) for installing TFlite.
    - [MediaPipe](https://google.github.io/mediapipe/getting_started/install.html) [and then python install](https://google.github.io/mediapipe/getting_started/python.html) -- latter requires at least the bazel install from the first one. If you can get bazel binaries of the right version instead, that'd be way faster... bazel builds very slowly. I think adding `build --local_ram_resources=HOST_RAM*.5 --local_cpu_resources=HOST_CPUS-1` to `~/.bazelrc` reduces the odds of bazel crashing the rbpi. I needed to edit `thirdparty/opencv_linux.BUILD` and edit the header directories to point to the `opencv4` locations instead.

# Commands to run

Via SSH (default 192.168.0.142 on home net))
1) SSH in
2) `export DISPLAY=:0.0`
3) `source ~/envs/mp_env/bin/activate`
4) Invoke script with python3.

