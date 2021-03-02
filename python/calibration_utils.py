import numpy as np

def get_extrinsics(inv=False):
    extr = np.loadtxt("../data/extrinsics.csv")
    if inv:
        extr[:3, :3] = extr[:3, :3].T
        extr[:3, 3] = -extr[:3, :3].dot(extr[:3, 3])
    return extr

def convert_hz_intrinsic_to_opengl_projection(K,x0,y0,width,height,znear,zfar, window_coords='y up'):
    # https://gist.github.com/astraw/1341472#file_calib_test_numpy.py
    znear = float(znear)
    zfar = float(zfar)
    depth = zfar - znear
    q = -(zfar + znear) / depth
    qn = -2 * (zfar * znear) / depth

    if window_coords=='y up':
        proj = np.array([[ -2*K[0,0]/width, -2*K[0,1]/width, (-2*K[0,2]+width+2*x0)/width, 0 ],
                         [  0,             -2*K[1,1]/height,(-2*K[1,2]+height+2*y0)/height, 0],
                         [0,0,q,qn],  # This row is standard glPerspective and sets near and far planes.
                         [0,0,-1,0]]) # This row is also standard glPerspective.
    else:
        assert window_coords=='y down'
        proj = np.array([[ -2*K[0,0]/width, -2*K[0,1]/width, (-2*K[0,2]+width+2*x0)/width, 0 ],
                         [  0,              2*K[1,1]/height,( 2*K[1,2]-height+2*y0)/height, 0],
                         [0,0,q,qn],  # This row is standard glPerspective and sets near and far planes.
                         [0,0,-1,0]]) # This row is also standard glPerspective.
    return proj
    
def get_projector_intrinsics():
    intr_3x3 = np.loadtxt("../data/projector_intrinsics.csv")
    return intr_3x3

def get_projector_gl_intrinsics():
    # REf https://amytabb.com/ts/2019_06_28/
    intr_3x3 = get_projector_intrinsics()
    cx = intr_3x3[0, 2]
    cy = intr_3x3[1, 2]
    width = cx * 2
    height = cy * 2
    far = 20.0
    near = 0.01
    intr_mat_1 = convert_hz_intrinsic_to_opengl_projection(intr_3x3, 0., 0., width, height, near, far)
    
    fx = intr_3x3[0, 0]
    fy = intr_3x3[1, 1]
    cx = intr_3x3[0, 2]
    cy = intr_3x3[1, 2]
    width = cx * 2
    height = cy * 2
    K_gl = np.array([
        [-fx, 0., -cx, 0.],
        [0., -fy, -cy, 0.],
        [0., 0., (near + far), (near * far)],
        [0., 0., -1., 0.]
    ])
    NDC = np.array([
        [2 / width, 0., 0., -1.],
        [0., 2 / height, 0., -1.],
        [0., 0., -2. / (far - near), -(far + near)/(far - near)],
        [0., 0., 0., 1.]
    ])

    #print("total projection: ", NDC.dot(K_gl))
    #print("Test project a point: ", NDC.dot(K_gl).dot(np.array([0.7, 0.5, 1.33, 1.])))
    #print("Total projection, second method: ", intr_mat_1)
    #print("Test project a point: ", intr_mat_1.dot(np.array([0.7, 0.5, 1.33, 1.])))
    return np.dot(NDC, K_gl)
