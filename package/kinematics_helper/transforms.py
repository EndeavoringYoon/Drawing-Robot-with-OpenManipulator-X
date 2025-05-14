import numpy as np

def t2pr(T):
    """
    Convert a 4x4 transformation matrix to its position vector and rotation matrix.
    
    Parameters:
        T (np.array): 4x4 transformation matrix.
        
    Returns:
        p (np.array): 3-element position vector.
        R (np.array): 3x3 rotation matrix.
    """ 
    p = T[:3,3]
    R = T[:3,:3]
    return p,R

def t2p(T):
    """
    Extract the position vector from a 4x4 transformation matrix.
    
    Parameters:
        T (np.array): 4x4 transformation matrix.
        
    Returns:
        p (np.array): 3-element position vector.
    """
    p = T[:3,3]
    return p

def t2r(T):
    """
    Extract the rotation matrix from a 4x4 transformation matrix.
    
    Parameters:
        T (np.array): 4x4 transformation matrix.
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """
    R = T[:3,:3]
    return R

def rpy2r(rpy_rad):
    """
    Convert roll, pitch, and yaw angles (in radians) to a 3x3 rotation matrix.
    
    Parameters:
        rpy_rad (np.array): Array of [roll, pitch, yaw] in radians.
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """
    roll  = rpy_rad[0]
    pitch = rpy_rad[1]
    yaw   = rpy_rad[2]
    Cphi  = np.cos(roll)
    Sphi  = np.sin(roll)
    Cthe  = np.cos(pitch)
    Sthe  = np.sin(pitch)
    Cpsi  = np.cos(yaw)
    Spsi  = np.sin(yaw)
    R     = np.array([
        [Cpsi * Cthe, -Spsi * Cphi + Cpsi * Sthe * Sphi, Spsi * Sphi + Cpsi * Sthe * Cphi],
        [Spsi * Cthe, Cpsi * Cphi + Spsi * Sthe * Sphi, -Cpsi * Sphi + Spsi * Sthe * Cphi],
        [-Sthe, Cthe * Sphi, Cthe * Cphi]
    ])
    assert R.shape == (3, 3)
    return R

def rpy2r_order(r0, order=[0,1,2]):
    """
    Convert roll, pitch, and yaw angles (in radians) to a 3x3 rotation matrix with a specified rotation order.
    
    Parameters:
        r0 (np.array): Array of [roll, pitch, yaw] in radians.
        order (list): List specifying the order of rotations (default is [0, 1, 2]).
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """
    c1 = np.cos(r0[0]); c2 = np.cos(r0[1]); c3 = np.cos(r0[2])
    s1 = np.sin(r0[0]); s2 = np.sin(r0[1]); s3 = np.sin(r0[2])
    a1 = np.array([[1,0,0],[0,c1,-s1],[0,s1,c1]])
    a2 = np.array([[c2,0,s2],[0,1,0],[-s2,0,c2]])
    a3 = np.array([[c3,-s3,0],[s3,c3,0],[0,0,1]])
    a_list = [a1,a2,a3]
    a = np.matmul(np.matmul(a_list[order[0]],a_list[order[1]]),a_list[order[2]])
    assert a.shape == (3,3)
    return a

def r2rpy(R,unit='rad'):
    """
    Convert a 3x3 rotation matrix to roll, pitch, and yaw angles.
    
    Parameters:
        R (np.array): 3x3 rotation matrix.
        unit (str): Unit for output angles ('rad' for radians or 'deg' for degrees).
        
    Returns:
        np.array: Array of [roll, pitch, yaw] angles.
    """
    roll  = np.math.atan2(R[2, 1], R[2, 2])
    pitch = np.math.atan2(-R[2, 0], (np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
    yaw   = np.math.atan2(R[1, 0], R[0, 0])
    if unit == 'rad':
        out = np.array([roll, pitch, yaw])
    elif unit == 'deg':
        out = np.array([roll, pitch, yaw])*180/np.pi
    else:
        out = None
        raise Exception("[r2rpy] Unknown unit:[%s]"%(unit))
    return out

def r2quat(R):
    """
    Convert a 3x3 rotation matrix to a quaternion.
    
    Parameters:
        R (np.array): 3x3 rotation matrix.
        
    Returns:
        q (np.array): Quaternion in the form [w, x, y, z].
    """
    R = np.asarray(R, dtype=np.float64)
    Qxx, Qyx, Qzx = R[..., 0, 0], R[..., 0, 1], R[..., 0, 2]
    Qxy, Qyy, Qzy = R[..., 1, 0], R[..., 1, 1], R[..., 1, 2]
    Qxz, Qyz, Qzz = R[..., 2, 0], R[..., 2, 1], R[..., 2, 2]
    # Fill only lower half of symmetric matrix
    K = np.zeros(R.shape[:-2] + (4, 4), dtype=np.float64)
    K[..., 0, 0] = Qxx - Qyy - Qzz
    K[..., 1, 0] = Qyx + Qxy
    K[..., 1, 1] = Qyy - Qxx - Qzz
    K[..., 2, 0] = Qzx + Qxz
    K[..., 2, 1] = Qzy + Qyz
    K[..., 2, 2] = Qzz - Qxx - Qyy
    K[..., 3, 0] = Qyz - Qzy
    K[..., 3, 1] = Qzx - Qxz
    K[..., 3, 2] = Qxy - Qyx
    K[..., 3, 3] = Qxx + Qyy + Qzz
    K /= 3.0
    # TODO: vectorize this -- probably could be made faster
    q = np.empty(K.shape[:-2] + (4,))
    it = np.nditer(q[..., 0], flags=['multi_index'])
    while not it.finished:
        # Use Hermitian eigenvectors, values for speed
        vals, vecs = np.linalg.eigh(K[it.multi_index])
        # Select largest eigenvector, reorder to w,x,y,z quaternion
        q[it.multi_index] = vecs[[3, 0, 1, 2], np.argmax(vals)]
        # Prefer quaternion with positive w
        # (q * -1 corresponds to same rotation as q)
        if q[it.multi_index][0] < 0:
            q[it.multi_index] *= -1
        it.iternext()
    return q

def pr2t(p,R):
    """
    Convert a position vector and rotation matrix into a 4x4 transformation matrix.
    
    Parameters:
        p (np.array): 3-element position vector.
        R (np.array): 3x3 rotation matrix.
        
    Returns:
        T (np.array): 4x4 transformation matrix.
    """
    p0 = p.ravel() # flatten
    T = np.block([
        [R, p0[:, np.newaxis]],
        [np.zeros(3), 1]
    ])
    return T

def r2w(R):
    """
    Compute the angular velocity vector (axis-angle representation) from a 3x3 rotation matrix.
    
    Parameters:
        R (np.array): 3x3 rotation matrix.
        
    Returns:
        w (np.array): Angular velocity vector.
    """
    el = np.array([
            [R[2,1] - R[1,2]],
            [R[0,2] - R[2,0]], 
            [R[1,0] - R[0,1]]
        ])
    norm_el = np.linalg.norm(el)
    if norm_el > 1e-10:
        w = np.arctan2(norm_el, np.trace(R)-1) / norm_el * el
    elif R[0,0] > 0 and R[1,1] > 0 and R[2,2] > 0:
        w = np.array([[0, 0, 0]]).T
    else:
        w = np.pi/2 * np.array([[R[0,0]+1], [R[1,1]+1], [R[2,2]+1]])
    return w.flatten()

def meters2xyz(depth_img,cam_matrix):
    """
    Convert a scaled depth image into a 3D point cloud.
    
    Parameters:
        depth_img (np.array): Depth image with depth values.
        cam_matrix (np.array): Camera intrinsic matrix.
        
    Returns:
        xyz_img (np.array): 3D point cloud of shape [H, W, 3].
    """
    fx = cam_matrix[0][0]
    cx = cam_matrix[0][2]
    fy = cam_matrix[1][1]
    cy = cam_matrix[1][2]
    
    height = depth_img.shape[0]
    width = depth_img.shape[1]
    indices = np.indices((height, width),dtype=np.float32).transpose(1,2,0)
    
    z_e = depth_img
    x_e = (indices[..., 1] - cx) * z_e / fx
    y_e = (indices[..., 0] - cy) * z_e / fy
    
    # Order of y_ e is reversed !
    xyz_img = np.stack([z_e, -x_e, -y_e], axis=-1) # [H x W x 3] 
    return xyz_img # [H x W x 3]

def get_rotation_matrix_from_two_points(p_fr,p_to):
    """
    Compute a rotation matrix that aligns a reference direction with the direction from one point to another.
    
    Parameters:
        p_fr (np.array): Starting point.
        p_to (np.array): Ending point.
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """
    p_a  = np.copy(np.array([1e-10,-1e-10,1.0]))
    if np.linalg.norm(p_to-p_fr) < 1e-8: # if two points are too close
        return np.eye(3)
    p_b  = (p_to-p_fr)/np.linalg.norm(p_to-p_fr)
    v    = np.cross(p_a,p_b)
    S = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    if np.linalg.norm(v) == 0:
        R = np.eye(3,3)
    else:
        R = np.eye(3,3) + S + S@S*(1-np.dot(p_a,p_b))/(np.linalg.norm(v)*np.linalg.norm(v))
    return R

def skew(x):
    """
    Compute the skew-symmetric matrix of a 3-element vector.
    
    Parameters:
        x (np.array): 3-element vector.
        
    Returns:
        x_hat (np.array): 3x3 skew-symmetric matrix.
    """
    x_hat = np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
    return x_hat

def rodrigues(a=np.array([1,0,0]),q_rad=0.0):
    """
    Compute the rotation matrix from an axis-angle representation using Rodrigues' formula.
    
    Parameters:
        a (np.array): 3-element unit vector representing the rotation axis.
        q_rad (float): Rotation angle in radians.
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """
    a_norm = np.linalg.norm(a)
    if abs(a_norm-1) > 1e-6:
        print ("[rodrigues] norm of a should be 1.0 not [%.2e]."%(a_norm))
        return np.eye(3)
    
    a = a / a_norm
    q_rad = q_rad * a_norm
    a_hat = skew(a)
    
    R = np.eye(3) + a_hat*np.sin(q_rad) + a_hat@a_hat*(1-np.cos(q_rad))
    return R


def rodrigues_batch(a=np.array([1, 0, 0]), q_rad=np.array([0.0])):
    """
    Compute rotation matrices for a batch of axis-angle representations using Rodrigues' formula.
    
    Parameters:
        a (np.array): Array of shape [B, 3] representing unit rotation axes.
        q_rad (np.array): Array of shape [B] representing rotation angles in radians.
        
    Returns:
        R (np.array): Array of shape [B, 3, 3] containing rotation matrices.
    """
    a = np.asarray(a)
    q_rad = np.asarray(q_rad)
    B = a.shape[0]

    a_norm = np.linalg.norm(a, axis=1, keepdims=True)
    if np.any(np.abs(a_norm - 1) > 1e-6):
        print("[rodrigues] Norm of each vector in a should be 1.0.")
        return np.tile(np.eye(3), (B, 1, 1))

    # Vectorized skew-symmetric matrix computation
    a_hat = np.zeros((B, 3, 3))  # shape [B, 3, 3]
    a_hat[:, 0, 1] = -a[:, 2]
    a_hat[:, 0, 2] = a[:, 1]
    a_hat[:, 1, 0] = a[:, 2]
    a_hat[:, 1, 2] = -a[:, 0]
    a_hat[:, 2, 0] = -a[:, 1]
    a_hat[:, 2, 1] = a[:, 0]

    sin_q = np.sin(q_rad).reshape(-1, 1, 1)  # shape [B, 1, 1]
    cos_q = np.cos(q_rad).reshape(-1, 1, 1)  # shape [B, 1, 1]

    R = (
        np.eye(3).reshape(1, 3, 3) +
        a_hat * sin_q +
        np.matmul(a_hat, a_hat) * (1 - cos_q)
    )  # shape [B, 3, 3]
    return R

def R_yuzf2zuxf(R):
    """
    Convert a rotation matrix from a Y-up Z-front coordinate system to a Z-up X-front coordinate system.
    
    Parameters:
        R (np.array): 3x3 rotation matrix in Y-up Z-front format.
        
    Returns:
        R_converted (np.array): 3x3 rotation matrix in Z-up X-front format.
    """
    R_offset = rpy2r(np.radians([-90,0,-90]))
    return R_offset@R

def T_yuzf2zuxf(T):
    """
    Convert a transformation matrix from a Y-up Z-front coordinate system to a Z-up X-front coordinate system.
    
    Parameters:
        T (np.array): 4x4 transformation matrix in Y-up Z-front format.
        
    Returns:
        T_converted (np.array): 4x4 transformation matrix in Z-up X-front format.
    """
    p,R = t2pr(T)
    T = pr2t(p=p,R=R_yuzf2zuxf(R))
    return T

def quat2r(q):
    """
    Convert a quaternion to a 3x3 rotation matrix.
    
    Parameters:
        q (np.array): Quaternion in the form [w, x, y, z].
        
    Returns:
        R (np.array): 3x3 rotation matrix.
    """    
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    
def align_z_axis(R):
    """
    Align the z-axis of a 3x3 rotation matrix with the global z-axis.
    
    Parameters:
        R (np.array): 3x3 rotation matrix.
        
    Returns:
        R_aligned (np.array): 3x3 rotation matrix with its z-axis aligned to [0, 0, 1].
    """
    q = r2quat(R)
    z_axis = R[:, 2]
    
    # Compute the rotation axis and angle
    rotation_axis = np.cross(z_axis, [0, 0, 1])
    rotation_axis_norm = np.linalg.norm(rotation_axis)
    
    if rotation_axis_norm < 1e-15:  # z_axis is already [0,0,1] or [0,0,-1]
        if z_axis[2] < 0:  # [0,0,-1] case
            return R @ quat2r([0, 1, 0, 0])  # 180 degree rotation around x-axis
        else:
            return R
    
    rotation_axis /= rotation_axis_norm
    cos_theta = np.dot(z_axis, [0, 0, 1])
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    
    # Compute the rotation quaternion
    q_rot = np.array([np.cos(theta/2)] + list(np.sin(theta/2) * rotation_axis))
    
    # Apply the rotation
    q_result = np.array([
        q_rot[0]*q[0] - q_rot[1]*q[1] - q_rot[2]*q[2] - q_rot[3]*q[3],
        q_rot[0]*q[1] + q_rot[1]*q[0] + q_rot[2]*q[3] - q_rot[3]*q[2],
        q_rot[0]*q[2] - q_rot[1]*q[3] + q_rot[2]*q[0] + q_rot[3]*q[1],
        q_rot[0]*q[3] + q_rot[1]*q[2] - q_rot[2]*q[1] + q_rot[3]*q[0]
    ])
    
    return quat2r(q_result)