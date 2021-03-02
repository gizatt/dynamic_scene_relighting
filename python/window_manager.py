import os
import random
import time
import sys
from functools import partial

import numpy as np
import pyglet
import pyglet.gl as gl
import OpenGL.GL as gl_better

from calibration_utils import get_extrinsics, get_projector_gl_intrinsics

class WindowManager(pyglet.window.Window):
    def __init__(self):
        super().__init__(
            config=gl.Config(
                double_buffer=True,
                samples=4  # MSAA,
            ),
            fullscreen=True, vsync=True)
        #self.prep_program_and_buffers()
        self.fps_display = pyglet.window.FPSDisplay(self)

    def start(self):
        self.iteration = 0
        pyglet.clock.schedule(self.on_idle)
        pyglet.app.run()

    def prep_program_and_buffers(self):
        self.prog = self.ctx.program(
            vertex_shader='''
                #version 330
                uniform mat4 Mvp;
                in vec3 in_position;
                in vec4 in_color;
                out vec4 color;
                void main() {
                    gl_Position = Mvp * vec4(in_position, 1.0);
                    color = in_color;
                }
            ''',
            fragment_shader='''
                #version 330
                uniform mat4 Mvp;
                in vec4 color;
                out vec4 f_color;
                void main() {
                    f_color = color;
                }
            ''',
        )
        
        self.geom_width, self.geom_height = 10, 10
        #width = self.geom_width
        #height = self.geom_height
        dummy_verts = np.array([[-0.5, 0.5, -0.5, 0.5, 0.],
                               [-0.5, -0.5, 0.5, 0.5, 0.],
                               [1., 1.2, 1.5, 1.8, 2.0]], dtype='f4').T
        self.num_points = dummy_verts.shape[0]
        self.vert_vbo = self.ctx.buffer(
            data=np.ascontiguousarray(dummy_verts), #, reserve=width*height*3*4, # 3 floats each
            dynamic=True
        )
        dummy_colors = np.array([
            [1., 1., 1., 1., 1.],
            [1., 1., 1., 1., 0.],
            [1., 1., 1., 1., 0.],
            [1., 1., 1., 1., 1.],
        ], dtype='f4').T
        self.color_vbo = self.ctx.buffer(
            data=np.ascontiguousarray(dummy_colors), #, reserve=width*height*4*4,  # 4 floats each
            dynamic=True
        )
        vao = self.ctx.vertex_array(self.prog,
            [
                (self.vert_vbo, "3f", 'in_position'),
                (self.color_vbo, "4f", 'in_color')
            ])
        self.vaos = [vao]

    def on_draw(self):
        self.clear()
        width, height = self.get_size()
        gl.glViewport(0, 0, width, height)
    
        K_gl = get_projector_gl_intrinsics()
        TF = get_extrinsics()
        R = np.array([[-1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., -1., 0.],
                    [0., 0., 0., 1.]])
        #full_mat = np.ascontiguousarray((K_gl.dot(TF.dot(R))).T.astype('f4'))
        ##full_mat = np.eye(4, dtype='f4')
        #self.prog['Mvp'].write(full_mat)
        #gl.glPointSize(3.)
        #for vao in self.vaos:
        #    vao.render(mode=moderngl.POINTS, vertices=self.num_points)

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

    def update_geometry(self, points, colors):
        assert len(points.shape) == 3 and points.shape[-1] == 3
        assert len(colors.shape) == 3 and colors.shape[-1] == 4
        def write_to_vbo(vbo, array):
            array = np.ascontiguousarray(array.astype('f4'))
            data = array.tobytes()
            if len(data) != vbo.size:
                vbo.orphan(len(data))
            vbo.write(array)
        write_to_vbo(self.vert_vbo, points)
        write_to_vbo(self.color_vbo, colors)
        self.num_points = points.shape[0] * points.shape[1]
