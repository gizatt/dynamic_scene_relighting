import glob
import os
import random
import time
import sys

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pyglet
import pyglet.gl as gl
import moderngl
from pyrr import Matrix44
import moderngl_window as mglw
from moderngl_window.utils.scheduler import Scheduler

from calibration_utils import get_extrinsics, get_projector_gl_intrinsics
 
class WindowManager(mglw.WindowConfig):
    gl_version = (3, 3)
    window_size = (1280, 768)

    @classmethod
    def run(cls):
        mglw.run_window_config(cls)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.prep_program_and_buffers()
        self.scheduler = Scheduler(self.timer)
        self.update_event = self.scheduler.run_every(self.update, 1.)

    def start(self):
        self.run()

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

    def render(self, time, frametime):
        self.scheduler.execute()
        self.ctx.clear()

        K_gl = get_projector_gl_intrinsics()
        TF = get_extrinsics()
        R = np.array([[-1., 0., 0., 0.],
                    [0., 1., 0., 0.],
                    [0., 0., -1., 0.],
                    [0., 0., 0., 1.]])
        full_mat = np.ascontiguousarray((K_gl.dot(TF.dot(R))).T.astype('f4'))
        #full_mat = np.eye(4, dtype='f4')
        self.prog['Mvp'].write(full_mat)
        gl.glPointSize(3.)
        for vao in self.vaos:
            vao.render(mode=moderngl.POINTS, vertices=self.num_points)

    def update(self):
        global fake_points
        global fake_colors
        #print("Update to ", points_ndc)
        #self.update_geometry(points_ndc[:, :, :3], fake_colors)
        self.update_geometry(fake_points, fake_colors)

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


if __name__ == "__main__":
    width = 1280
    height = 720
    U, V = np.meshgrid(range(width), range(height))
    period = width / 8
    fake_colors = np.stack([
        np.cos(U / period)/2. + 0.5,
        np.sin(V / period)/2. + 0.5,
        np.cos(U / (period / 2.))/2. + 0.5,
        U*0 + 1.
    ], axis=-1)
    fake_depths = np.cos(U / period) + np.sin(V / period) + 3.
    cx = cy = 200
    K = np.array([
        [cx, 0., width / 2.],
        [0., cy, height / 2.],
        [0., 0., 1.]
    ])
    K_inv = np.linalg.inv(K)

    fake_points = np.stack([
        U*fake_depths,
        V*fake_depths,
        fake_depths
    ], axis=-1)
    fake_points = K_inv.dot(fake_points.transpose(0, 2, 1)).transpose(1, 2, 0)
    
    '''
    fig = plt.figure()
    ax = fig.add_subplot(311, projection='3d')
    ax.scatter(fake_points[:, :, 0].flatten(),
               fake_points[:, :, 1].flatten(),
               fake_points[:, :, 2].flatten(),
               c=fake_colors.reshape(-1, 4))
    plt.subplot(3, 1, 2)
    plt.imshow(fake_colors, vmin=0., vmax=1.)
    K_gl = get_projector_gl_intrinsics()
    TF = get_extrinsics()
    R = np.array([[-1., 0., 0., 0.],
                  [0., 1., 0., 0.],
                  [0., 0., -1., 0.],
                  [0., 0., 0., 1.]])
    full_mat = K_gl.dot(TF.dot(R)).astype(np.float32)
    points_homog = np.concatenate([fake_points, np.ones(U.shape)[:, :, np.newaxis]], axis=-1)
    points_ndc = np.dot(full_mat, points_homog.transpose(0, 2, 1)).transpose(1, 2, 0)
    for k in range(4):
        points_ndc[:, :, k] /= points_ndc[:, :, -1]
    
    ax = fig.add_subplot(313, projection='3d')
    ax.scatter(points_ndc[:, :, 0].flatten(),
               points_ndc[:, :, 1].flatten(),
               points_ndc[:, :, 2].flatten(),
               c=fake_colors.reshape(-1, 4))
    plt.show()
    '''
    WindowManager.run()