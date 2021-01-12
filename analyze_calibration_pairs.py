import os

import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    with open("calibration_pairs.csv", "r") as f:
        data = np.loadtxt(f, delimiter=",", skiprows=0)
        assert data.shape[1] == 4
        print("Loaded %d example pairs" % data.shape[0])
    projector_xy = data[:, :2]
    observed_xy = data[:, -2:]

    # observed_pts = K_cam * T_rel^[-1] * K_projector^[-1] * projector_xy
    # K_cam we have from factory
    # T_rel and K_projector we'd like to reconstruct
    # K_cam^-1 * observed_pts = rgbd frame 3d pts
    # K_projector * T_rel * K_cam^[-1] * observed_pts = projector_xy

    # Both K_projector and T_rel have structure -- K_projector
    # is sparse, and T_rel is a rigid transform. So some hope if we
    # write this symbolically and solve out as an NLP.
