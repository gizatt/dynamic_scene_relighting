import glob
import os
import random
import time
import sys

import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from .projector_utils import *

if __name__ == "__main__":
    width = 1280
    height = 720
    U, V = np.meshgrid(range(width), range(height))
    fake_colors = np.stack([
        np.cos(U / 100.)/2. + 0.5,
        np.sin(V / 100.)/2. + 0.5,
        np.cos(U / 200.)/2. + 0.5
    ], axis=-1)
    fake_depths = np.cos(U / 100.) + np.sin(V / 100.) + 2.
    fake_points = np.stack([
        fake_depths
    ], axis=-1)

    window = pyglet.window.Window(
    config=gl.Config(
        double_buffer=True,
        samples=4  # MSAA,
    ),
    fullscreen=False, vsync=True)

    pyglet.app.run()
