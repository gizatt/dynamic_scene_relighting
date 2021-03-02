import os
import random
import time
import sys
from functools import partial

import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from pyshaders import from_string
from calibration_utils import get_extrinsics, get_projector_gl_intrinsics

class WindowManager(pyglet.window.Window):
    def __init__(self, callback=None):
        super().__init__(
            config=gl.Config(
                double_buffer=True,
                samples=4  # MSAA,
            ),
            fullscreen=True, vsync=True)
        self.prep_program_and_buffers()
        self.fps_display = pyglet.window.FPSDisplay(self)
        self.callback = callback

    def start(self):
        self.iteration = 0
        pyglet.clock.schedule(self.on_idle)
        pyglet.app.run()

    def prep_program_and_buffers(self):
        self.prog = from_string(
            verts='''
                #version 330
                uniform mat4 Mvp;
                layout(location = 0) in vec3 in_position;
                layout(location = 1) in vec4 in_color;
                out vec4 color;
                void main() {
                    gl_Position = Mvp * vec4(in_position, 1.0);
                    color = in_color;
                }
            ''',
            frags='''
                #version 330
                uniform mat4 Mvp;
                in vec4 color;
                out vec4 f_color;
                void main() {
                    f_color = color;
                }
            '''
        )

        self.num_pts = 1000
        verts = np.random.random((self.num_pts, 3))
        colors = np.random.random((self.num_pts, 4))
        # MgNf = layout location M, N floats.
        self.vertex_info = pyglet.graphics.vertex_list(
            self.num_pts, ('0g3f', verts.ravel()), ('1g4f', colors.ravel())
        )

    def on_draw(self):
        self.clear()
        width, height = self.get_size()
        gl.glViewport(0, 0, width, height)
    
        gl.glPointSize(5.)
        
        K_gl = get_projector_gl_intrinsics()
        TF = get_extrinsics()
        R = np.array([[-1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., -1., 0.],
                    [0., 0., 0., 1.]])
        full_mat = np.ascontiguousarray((K_gl.dot(TF.dot(R))).astype('f4'))
        with self.prog.using():
            self.prog.uniforms.Mvp = full_mat.tolist()
            
            self.vertex_info.draw(gl.GL_POINTS)

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(0, width, 0, height, -1, 1)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glLoadIdentity()
        gl.glDisable(gl.GL_DEPTH_TEST)

        self.fps_display.draw()
        print("Done with draw")

    def on_idle(self, dt):
        self.iteration += 1
        self.set_caption("RealSense %d FPS (%.2fms)" %
            (0 if dt == 0 else 1.0 / dt, dt * 1000))
        if self.callback:
            self.callback(self)

    def update_geometry(self, points, colors):
        assert len(points.shape) == 3 and points.shape[-1] == 3
        assert len(colors.shape) == 3 and colors.shape[-1] == 4
        def write_to_vbo(vbo, array):
            array = np.ascontiguousarray(array.astype('f4'))
            data = array.tobytes()
            if len(data) != vbo.size:
                vbo.orphan(len(data))
            vbo.write(array)
        num_pts = points.shape[0] * points.shape[1]
        if self.vertex_info.get_size() != num_pts:
            self.vertex_info.resize(num_pts)
        def copy(dst, src):
            np.array(dst, copy=False)[:] = src.ravel()
        self.vertex_info._set_attribute_data(0, points.ravel())
        self.vertex_info._set_attribute_data(1, colors.ravel())
        self.num_pts = num_pts