import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector

      From https://github.com/ClayFlannigan/icp/blob/master/icp.py
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

if __name__ == "__main__":
    with open("calibration_pairs.csv", "r") as f:
        data = np.loadtxt(f, delimiter=",", skiprows=0)
        assert data.shape[1] == (2 + 2 + 3)
        print("Loaded %d example pairs" % data.shape[0])
    projected_uv = data[:, :2].T
    observed_uv = data[:, 2:4].T
    observed_xyz = data[:, 4:].T

    print("Projected uv: ", projected_uv)
    print("Observed XYZ: ", observed_xyz)

    # observed_xyz = * T_rel^[-1] * K_projector^[-1] * projector_xy
    # (observed_xyz = K_cam^[-1] * observed_uv, calculated
    # with factory intrinsics.)
    # T_rel and K_projector we'd like to reconstruct
    # K_projector * T_rel * observed_xyz = projector_xy

    # Both K_projector and T_rel have structure -- K_projector
    # is sparse, and T_rel is a rigid transform. I'll tackle with
    # an alternation approach: fix K, update transform estimate
    # with closed-form solution based on SVD; and then fix transform
    # estimate and update K with a linear solve.

    # We know the resolution, just need to nail the focal length
    proj_width = 1280
    proj_height = 1024
    def build_K_est(f_x, f_y):
        return np.array([[f_x, 0, proj_width/2.],
                      [0, f_y, proj_height/2.],
                      [0., 0., 1.]])
    def f_from_fov(fov, c):
        fov = fov * np.pi / 180.
        return c / np.tan(fov/2)
    def fov(f, c):
        return 2.*np.arctan2(c*2, 2.*f)*180/np.pi
    # At 26" image was 20x13.5"
    fov_x_deg = 2*np.arctan2(20./2., 26)*180/np.pi
    fov_y_deg = 2*np.arctan2(13.5/2., 26)*180/np.pi
    print("%f x %f FOV" % (fov_x_deg, fov_y_deg))
    K_est = build_K_est(f_from_fov(fov_x_deg, proj_width/2.), f_from_fov(fov_y_deg, proj_width/2.))
    R_est = Rotation.random().as_matrix() #from_rotvec(np.pi * np.array([0., 0., 1.])).as_matrix()
    t_est = np.zeros(3)

    print("Init k est: ", K_est)
    print("FOVx = %f, FOVy = %f" % (fov(K_est[0, 0], K_est[0, 2]), fov(K_est[1, 1], K_est[1, 2])))
    print("Init tf ", R_est, t_est)
    for k in range(10):
        # Update focal length estimate
        projector_frame_xyz = (R_est.dot(observed_xyz).T + t_est).T
        centered_projected_uv = (projected_uv[:2, :].T - K_est[:2, 2]).T
        fx_obs = centered_projected_uv[0, :] / (projector_frame_xyz[0, :] / projector_frame_xyz[2, :])
        fy_obs = centered_projected_uv[1, :] / (projector_frame_xyz[1, :] / projector_frame_xyz[2, :])
        #K_est[0, 0] = fx_obs[np.isfinite(fx_obs)].mean()
        #K_est[1, 1] = fy_obs[np.isfinite(fy_obs)].mean()
        print("Kest", K_est)
        print("FOVx = %f, FOVy = %f" % (fov(K_est[0, 0], K_est[0, 2]), fov(K_est[1, 1], K_est[1, 2])))
        # Update transform -- given fixed focal length, and
        # ASSUME SAME Z AS OUTPUT -- not great, but probably
        # approximately true...
        Kinv = np.linalg.inv(K_est)
        # Borrow depth from the RGBD sensor
        projected_cam_pts = Kinv.dot(np.vstack([projected_uv, observed_xyz[2, :]]))
        # Update transform estimate with SVD trick
        _, R_est_new, t_est_new = best_fit_transform(observed_xyz.T, projected_cam_pts.T)
        diff_t = t_est_new - t_est
        diff_R = R_est_new - R_est
        R_est = R_est_new
        t_est = t_est_new
        print("R_est, t_est", R_est, t_est)


