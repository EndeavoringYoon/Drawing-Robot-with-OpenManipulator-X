import numpy as np
import cvxpy as cp
from ik import solve_ik
from utils import get_idxs

def d2r(deg):
    return np.radians(deg)

def finite_difference_matrix(n, dt, order):
    """
    Compute a finite difference matrix for numerical differentiation.
    
    Parameters:
        n (int): Number of points.
        dt (float): Time interval between points.
        order (int): Derivative order (1: velocity, 2: acceleration, 3: jerk).
    
    Returns:
        np.array: Finite difference matrix scaled by dt^order.
    """
    # Order
    if order == 1:  # velocity
        coeffs = np.array([-1, 1])
    elif order == 2:  # acceleration
        coeffs = np.array([1, -2, 1])
    elif order == 3:  # jerk
        coeffs = np.array([-1, 3, -3, 1])
    else:
        raise ValueError("Order must be 1, 2, or 3.")

    # Fill-in matrix
    mat = np.zeros((n, n))
    for i in range(n - order):
        for j, c in enumerate(coeffs):
            mat[i, i + j] = c

    # (optional) Handling boundary conditions with backward differences
    if order == 1:  # velocity
        mat[-1, -2:] = np.array([-1, 1])  # backward difference
    elif order == 2:  # acceleration
        mat[-1, -3:] = np.array([1, -2, 1])  # backward difference
        mat[-2, -3:] = np.array([1, -2, 1])  # backward difference
    elif order == 3:  # jerk
        mat[-1, -4:] = np.array([-1, 3, -3, 1])  # backward difference
        mat[-2, -4:] = np.array([-1, 3, -3, 1])  # backward difference
        mat[-3, -4:] = np.array([-1, 3, -3, 1])  # backward difference

    # Return 
    return mat / (dt ** order)

def get_A_vel_acc_jerk(n=100,dt=1e-2):
    """
    Generate finite difference matrices for velocity, acceleration, and jerk.
    
    Parameters:
        n (int): Number of points.
        dt (float): Time interval.
    
    Returns:
        tuple: (A_vel, A_acc, A_jerk) matrices.
    """
    A_vel  = finite_difference_matrix(n,dt,order=1)
    A_acc  = finite_difference_matrix(n,dt,order=2)
    A_jerk = finite_difference_matrix(n,dt,order=3)
    return A_vel,A_acc,A_jerk

def solve_ik_and_interpolate(
        env,
        joint_names_for_ik = None,
        body_name_trgt     = None,
        p_trgt             = None,
        R_trgt             = None,
        max_ik_tick        = 500,
        ik_err_th          = 1e-4,
        restore_state      = True,
        jerk_limit         = d2r(360),
        vel_interp_max     = d2r(90),
        vel_interp_min     = d2r(10),
    ):
    """ 
        Solve IK and interpolate
    """
    # Start joint position
    q_start = env.get_qpos_joints(joint_names=joint_names_for_ik)

    # Solve IK
    q_final,ik_err_stack,ik_info = solve_ik(
        env=env,joint_names_for_ik=joint_names_for_ik,
        body_name_trgt = body_name_trgt,
        p_trgt         = p_trgt,
        R_trgt         = R_trgt,
        max_ik_tick    = max_ik_tick,
        ik_err_th      = ik_err_th,
        restore_state  = restore_state,
    )

    # Interpolate
    q_anchors = np.vstack((q_start,q_final))
    times,traj_interp,traj_smt,times_anchor = interpolate_and_smooth_nd(
        anchors        = q_anchors,
        HZ             = env.HZ,
        x_lowers       = env.joint_ranges[get_idxs(env.joint_names,joint_names_for_ik),0], 
        x_uppers       = env.joint_ranges[get_idxs(env.joint_names,joint_names_for_ik),1],
        jerk_limit     = jerk_limit,
        vel_interp_max = vel_interp_max,
        vel_interp_min = vel_interp_min,
    )

    # Return
    return times,traj_smt

def smooth_optm_1d(
        traj,
        dt          = 0.1,
        x_init      = None,
        x_final     = None,
        vel_init    = None,
        vel_final   = None,
        x_lower     = None,
        x_upper     = None,
        vel_limit   = None,
        acc_limit   = None,
        jerk_limit  = None,
        idxs_remain = None,
        vals_remain = None,
        p_norm      = 2,
        verbose     = True,
    ):
    """
    Perform 1-D smoothing of a trajectory using optimization.
    
    Parameters:
        traj (array): Original trajectory.
        dt (float): Time interval.
        x_init (float, optional): Initial position.
        x_final (float, optional): Final position.
        vel_init (float, optional): Initial velocity.
        vel_final (float, optional): Final velocity.
        x_lower (float, optional): Lower position bound.
        x_upper (float, optional): Upper position bound.
        vel_limit (float, optional): Maximum velocity.
        acc_limit (float, optional): Maximum acceleration.
        jerk_limit (float, optional): Maximum jerk.
        idxs_remain (array, optional): Fixed indices.
        vals_remain (array, optional): Values at fixed indices.
        p_norm (int): Norm degree for objective.
        verbose (bool): Verbosity flag.
    
    Returns:
        np.array: Smoothed trajectory.
    """
    n = len(traj)
    A_pos = np.eye(n,n)
    A_vel,A_acc,A_jerk = get_A_vel_acc_jerk(n=n,dt=dt)
    
    # Objective 
    x = cp.Variable(n)
    objective = cp.Minimize(cp.norm(x-traj,p_norm))
    
    # Equality constraints
    A_list,b_list = [],[]
    if x_init is not None:
        A_list.append(A_pos[0,:])
        b_list.append(x_init)
    if x_final is not None:
        A_list.append(A_pos[-1,:])
        b_list.append(x_final)
    if vel_init is not None:
        A_list.append(A_vel[0,:])
        b_list.append(vel_init)
    if vel_final is not None:
        A_list.append(A_vel[-1,:])
        b_list.append(vel_final)
    if idxs_remain is not None:
        A_list.append(A_pos[idxs_remain,:])
        if vals_remain is not None:
            b_list.append(vals_remain)
        else:
            b_list.append(traj[idxs_remain])

    # Inequality constraints
    C_list,d_list = [],[]
    if x_lower is not None:
        C_list.append(-A_pos)
        d_list.append(-x_lower*np.ones(n))
    if x_upper is not None:
        C_list.append(A_pos)
        d_list.append(x_upper*np.ones(n))
    if vel_limit is not None:
        C_list.append(A_vel)
        C_list.append(-A_vel)
        d_list.append(vel_limit*np.ones(n))
        d_list.append(vel_limit*np.ones(n))
    if acc_limit is not None:
        C_list.append(A_acc)
        C_list.append(-A_acc)
        d_list.append(acc_limit*np.ones(n))
        d_list.append(acc_limit*np.ones(n))
    if jerk_limit is not None:
        C_list.append(A_jerk)
        C_list.append(-A_jerk)
        d_list.append(jerk_limit*np.ones(n))
        d_list.append(jerk_limit*np.ones(n))
    constraints = []
    if A_list:
        A = np.vstack(A_list)
        b = np.hstack(b_list).squeeze()
        constraints.append(A @ x == b) 
    if C_list:
        C = np.vstack(C_list)
        d = np.hstack(d_list).squeeze()
        constraints.append(C @ x <= d)
    
    # Solve
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.CLARABEL)

    # Return
    traj_smt = x.value

    # Null check
    if traj_smt is None and verbose:
        print ("[smooth_optm_1d] Optimization failed.")
    return traj_smt

def get_idxs_closest_ndarray(ndarray_query,ndarray_domain):
    """
    Get indices of closest elements in ndarray_domain for each query element.
    
    Parameters:
        ndarray_query (array): Query array.
        ndarray_domain (array): Domain array.
    
    Returns:
        list: Indices of closest matches.
    """    
    return [np.argmin(np.abs(ndarray_query-x)) for x in ndarray_domain]

def get_interp_const_vel_traj_nd(
        anchors, # [L x D]
        vel = 1.0,
        HZ  = 100,
        ord = np.inf,
    ):
    """
    Generate a linearly interpolated (with constant velocity) trajectory.
    
    Parameters:
        anchors (array): Anchor points [L x D].
        vel (float): Constant velocity.
        HZ (int): Sampling frequency.
        ord (float): Norm order for distance.
    
    Returns:
        tuple: (times_interp, anchors_interp, times_anchor, idxs_anchor)
    """
    L = anchors.shape[0]
    D = anchors.shape[1]
    dists = np.zeros(L)
    for tick in range(L):
        if tick > 0:
            p_prev,p_curr = anchors[tick-1,:],anchors[tick,:]
            dists[tick] = np.linalg.norm(p_prev-p_curr,ord=ord)
    times_anchor = np.cumsum(dists/vel) # [L]
    L_interp     = int(times_anchor[-1]*HZ)
    times_interp = np.linspace(0,times_anchor[-1],L_interp) # [L_interp]
    anchors_interp  = np.zeros((L_interp,D)) # [L_interp x D]
    for d_idx in range(D): # for each dim
        anchors_interp[:,d_idx] = np.interp(times_interp,times_anchor,anchors[:,d_idx])
    idxs_anchor = get_idxs_closest_ndarray(times_interp,times_anchor)
    return times_interp,anchors_interp,times_anchor,idxs_anchor

def interpolate_and_smooth_nd(
        anchors, # List or [N x D]
        HZ             = 50,
        vel_init       = 0.0,
        vel_final      = 0.0,
        x_lowers       = None, # [D]
        x_uppers       = None, # [D]
        vel_limit      = None, # [1]
        acc_limit      = None, # [1]
        jerk_limit     = None, # [1]
        vel_interp_max = d2r(180),
        vel_interp_min = d2r(10),
        n_interp       = 10,
        verbose        = False,
    ):
    """ 
        Interpolate anchors and smooth [N x D] anchors
    """
    if isinstance(anchors, list):
        # If 'anchors' is given as a list, make it an ndarray
        anchors = np.vstack(anchors)
    
    D = anchors.shape[1]
    vels = np.linspace(start=vel_interp_max,stop=vel_interp_min,num=n_interp)
    for v_idx,vel_interp in enumerate(vels):
        # First, interploate
        times,traj_interp,times_anchor,idxs_anchor = get_interp_const_vel_traj_nd(
            anchors = anchors,
            vel     = vel_interp,
            HZ      = HZ,
        )
        dt = times[1] - times[0]
        # Second, smooth
        traj_smt = np.zeros_like(traj_interp)
        is_success = True
        for d_idx in range(D):
            traj_d = traj_interp[:,d_idx]
            if x_lowers is not None: x_lower_d = x_lowers[d_idx]
            else: x_lower_d = None
            if x_uppers is not None: x_upper_d = x_uppers[d_idx]
            else: x_upper_d = None
            traj_smt_d = smooth_optm_1d(
                traj        = traj_d,
                dt          = dt,
                idxs_remain = idxs_anchor,
                vals_remain = anchors[:,d_idx],
                vel_init    = vel_init,
                vel_final   = vel_final,
                x_lower     = x_lower_d,
                x_upper     = x_upper_d,
                vel_limit   = vel_limit,
                acc_limit   = acc_limit,
                jerk_limit  = jerk_limit,
                p_norm      = 2,
                verbose     = False,
            )
            if traj_smt_d is None:
                is_success = False
                break
            # Append
            traj_smt[:,d_idx] = traj_smt_d

        # Check success
        if is_success:
            if verbose:
                print ("Optimization succeeded. vel_interp:[%.3f]"%(vel_interp))
            return times,traj_interp,traj_smt,times_anchor
        else:
            if verbose:
                print (" v_idx:[%d/%d] vel_interp:[%.2f] failed."%(v_idx,n_interp,vel_interp))
    
    # Optimization failed
    if verbose:
        print ("Optimization failed.")
    return times,traj_interp,traj_smt,times_anchor