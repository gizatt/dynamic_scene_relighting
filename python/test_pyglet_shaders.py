import glob
import os
import random
import time
import sys
from functools import partial

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

from window_manager import *

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
    def update_geometry(cls):
        cls.update_geometry(fake_points, fake_colors)

    wm = WindowManager(callback=update_geometry)
    wm.start()