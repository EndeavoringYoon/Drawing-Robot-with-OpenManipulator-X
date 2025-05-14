import sys
import numpy as np

"""
sys.path.append('../../package/utility/') # for 'utils.py'
"""
from utils import (
    get_colors,
    get_idxs,
)

# Inverse kinematics helper
def init_ik_info():
    """
    Initialize inverse kinematics (IK) information.
    
    This function creates and returns an empty dictionary to store IK target
    information. The returned dictionary contains lists for body names, geometry names,
    target positions, and target orientations, as well as a counter for the number of targets.
    
    Usage example:
        ik_info = init_ik_info()
        add_ik_info(ik_info, body_name='BODY_NAME', p_trgt=P_TRGT, R_trgt=R_TRGT)
        ...
        for ik_tick in range(max_ik_tick):
            dq, ik_err_stack = get_dq_from_ik_info(
                env=env,
                ik_info=ik_info,
                stepsize=1,
                eps=1e-2,
                th=np.radians(10.0),
                joint_idxs_jac=joint_idxs_jac,
            )
            qpos = env.get_qpos()
            mujoco.mj_integratePos(env.model, qpos, dq, 1)
            env.forward(q=qpos)
            if np.linalg.norm(ik_err_stack) < 0.05: break
    
    Returns:
        dict: A dictionary with the following keys:
            - 'body_names': list of body names (str)
            - 'geom_names': list of geometry names (str)
            - 'p_trgts': list of target positions (np.array, expected shape: (3,))
            - 'R_trgts': list of target orientations (np.array, expected shape: (3, 3))
            - 'n_trgt': int, the number of IK targets added.
    """
    ik_info = {
        'body_names':[],
        'geom_names':[],
        'p_trgts':[],
        'R_trgts':[],
        'n_trgt':0,
    }
    return ik_info

def add_ik_info(
        ik_info,
        body_name = None,
        geom_name = None,
        p_trgt    = None,
        R_trgt    = None,
    ):
    """
    Add inverse kinematics (IK) target information to an existing IK info dictionary.
    
    Parameters:
        ik_info (dict): Dictionary storing IK information, as initialized by init_ik_info().
        body_name (str, optional): Name of the body for which the IK target is defined.
        geom_name (str, optional): Name of the geometry for which the IK target is defined.
        p_trgt (np.array, optional): Target position for IK. Expected shape: (3,).
        R_trgt (np.array, optional): Target orientation (rotation matrix) for IK. Expected shape: (3, 3).
        
    Side Effects:
        Appends the provided information to the respective lists in ik_info and increments 'n_trgt'.
    """
    ik_info['body_names'].append(body_name)
    ik_info['geom_names'].append(geom_name)
    ik_info['p_trgts'].append(p_trgt)
    ik_info['R_trgts'].append(R_trgt)
    ik_info['n_trgt'] = ik_info['n_trgt'] + 1
    
def get_dq_from_ik_info(
        env,
        ik_info,
        stepsize       = 1,
        eps            = 1e-2,
        th             = np.radians(1.0),
        joint_idxs_jac = None,
        joint_names    = None,
    ):
    """
    Compute the change in joint configuration (delta q) using the augmented Jacobian method.
    
    This function gathers the IK ingredients for each target (Jacobian and IK error)
    from the environment via env.get_ik_ingredients, stacks the results, optionally selects
    a subset of joint columns, and then computes dq using a damped least-squares method.
    
    Parameters:
        env: The simulation environment object that provides the methods:
             - get_ik_ingredients(body_name, geom_name, p_trgt, R_trgt, IK_P, IK_R)
             - damped_ls(J, ik_err_stack, stepsize, eps, th)
             - get_idxs_jac(joint_names=...) if joint_names is provided.
        ik_info (dict): IK information dictionary containing lists for 'body_names', 'geom_names',
                        'p_trgts', and 'R_trgts'.
        stepsize (float): Scaling factor for the computed dq.
        eps (float): Small damping term for the least-squares computation.
        th (float): Threshold (in radians) for damped least squares.
        joint_idxs_jac (list or np.array, optional): Indices of joints to use in the Jacobian.
        joint_names (list, optional): List of joint names; if provided, joint_idxs_jac is obtained via env.get_idxs_jac.
    
    Returns:
        tuple:
            - dq (np.array): Change in joint configuration. Shape depends on the number of joints.
            - ik_err_stack (np.array): Stacked IK error vector from all targets.
    """
    J_list,ik_err_list = [],[]
    for ik_idx,(ik_body_name,ik_geom_name) in enumerate(zip(ik_info['body_names'],ik_info['geom_names'])):
        ik_p_trgt = ik_info['p_trgts'][ik_idx]
        ik_R_trgt = ik_info['R_trgts'][ik_idx]
        IK_P = ik_p_trgt is not None
        IK_R = ik_R_trgt is not None
        J,ik_err = env.get_ik_ingredients(
            body_name = ik_body_name,
            geom_name = ik_geom_name,
            p_trgt    = ik_p_trgt,
            R_trgt    = ik_R_trgt,
            IK_P      = IK_P,
            IK_R      = IK_R,
        )
        J_list.append(J)
        ik_err_list.append(ik_err)

    J_stack      = np.vstack(J_list)
    ik_err_stack = np.hstack(ik_err_list)

    # Select Jacobian columns that are within the joints to use
    if joint_names is not None:
        joint_idxs_jac = env.get_idxs_jac(joint_names=joint_names)
    if joint_idxs_jac is not None:
        J_stack_backup = J_stack.copy()
        J_stack = np.zeros_like(J_stack)
        J_stack[:,joint_idxs_jac] = J_stack_backup[:,joint_idxs_jac]
        

    # Compute dq from damped least square
    dq = env.damped_ls(J_stack,ik_err_stack,stepsize=stepsize,eps=eps,th=th)
    return dq,ik_err_stack

def plot_ik_info(
        env,
        ik_info,
        axis_len   = 0.05,
        axis_width = 0.005,
        sphere_r   = 0.01,
        ):
    """
    Plot inverse kinematics (IK) information on the environment.
    
    This function visualizes both the current and target IK information for each target defined
    in ik_info. For each target, it plots the body and geometry using the environment's plotting
    methods, drawing spheres, axes, and lines to indicate current and target positions and orientations.
    
    Parameters:
        env: Environment object with plotting methods including:
             - plot_body_T(body_name, plot_axis, axis_len, axis_width, plot_sphere, sphere_r, sphere_rgba, label)
             - plot_geom_T(geom_name, plot_axis, axis_len, axis_width, plot_sphere, sphere_r, sphere_rgba, label)
             - plot_sphere(p, r, rgba, label)
             - plot_line_fr2to(p_fr, p_to, rgba)
             - plot_T(p, R, plot_axis, axis_len, axis_width)
             - get_p_body(body_name) and get_p_geom(geom_name)
        ik_info (dict): Dictionary containing IK target information.
        axis_len (float): Length of the axes to plot (m).
        axis_width (float): Width of the plotted axes.
        sphere_r (float): Radius for the plotted spheres (m).
    """
    colors = get_colors(cmap_name='gist_rainbow',n_color=ik_info['n_trgt'])
    for ik_idx,(ik_body_name,ik_geom_name) in enumerate(zip(ik_info['body_names'],ik_info['geom_names'])):
        color = colors[ik_idx]
        ik_p_trgt = ik_info['p_trgts'][ik_idx]
        ik_R_trgt = ik_info['R_trgts'][ik_idx]
        IK_P = ik_p_trgt is not None
        IK_R = ik_R_trgt is not None

        if ik_body_name is not None:
            # Plot current 
            env.plot_body_T(
                body_name   = ik_body_name,
                plot_axis   = IK_R,
                axis_len    = axis_len,
                axis_width  = axis_width,
                plot_sphere = IK_P,
                sphere_r    = sphere_r,
                sphere_rgba = color,
                label       = '' # ''/ik_body_name
            )
            # Plot target
            if IK_P:
                env.plot_sphere(p=ik_p_trgt,r=sphere_r,rgba=color,label='') 
                env.plot_line_fr2to(p_fr=env.get_p_body(body_name=ik_body_name),p_to=ik_p_trgt,rgba=color)
            if IK_P and IK_R:
                env.plot_T(p=ik_p_trgt,R=ik_R_trgt,plot_axis=True,axis_len=axis_len,axis_width=axis_width)
            if not IK_P and IK_R: # rotation only
                p_curr = env.get_p_body(body_name=ik_body_name)
                env.plot_T(p=p_curr,R=ik_R_trgt,plot_axis=True,axis_len=axis_len,axis_width=axis_width)
            
        if ik_geom_name is not None:
            # Plot current 
            env.plot_geom_T(
                geom_name   = ik_geom_name,
                plot_axis   = IK_R,
                axis_len    = axis_len,
                axis_width  = axis_width,
                plot_sphere = IK_P,
                sphere_r    = sphere_r,
                sphere_rgba = color,
                label       = '' # ''/ik_geom_name
            )
            # Plot target
            if IK_P:
                env.plot_sphere(p=ik_p_trgt,r=sphere_r,rgba=color,label='') 
                env.plot_line_fr2to(p_fr=env.get_p_geom(geom_name=ik_geom_name),p_to=ik_p_trgt,rgba=color)
            if IK_P and IK_R:
                env.plot_T(p=ik_p_trgt,R=ik_R_trgt,plot_axis=True,axis_len=axis_len,axis_width=axis_width)
            if not IK_P and IK_R: # rotation only
                p_curr = env.get_p_geom(geom_name=ik_geom_name)
                env.plot_T(p=p_curr,R=ik_R_trgt,plot_axis=True,axis_len=axis_len,axis_width=axis_width)

def solve_ik(
        env,
        joint_names_for_ik,
        body_name_trgt,
        q_init          = None, # IK start from the initial pose
        p_trgt          = None,
        R_trgt          = None,
        max_ik_tick     = 1000,
        ik_err_th       = 1e-2,
        restore_state   = True,
        ik_stepsize     = 1.0,
        ik_eps          = 1e-2,
        ik_th           = np.radians(1.0),
        verbose         = False,
        verbose_warning = True,
        reset_env       = False,
        render          = False,
        render_every    = 1,
    ):
    """
    Solve inverse kinematics (IK) for a given target using an iterative augmented Jacobian method.
    
    This function attempts to solve the IK problem for the specified body by iteratively computing
    the change in joint configuration (dq) until the IK error falls below a threshold or a maximum
    number of iterations is reached. Optionally, the environment state can be reset and rendered.
    
    Parameters:
        env: Simulation environment object providing methods for forward kinematics, state storage,
             and IK computations.
        joint_names_for_ik (list): List of joint names to be used for the IK computation.
        body_name_trgt (str): Name of the target body for which IK is being solved.
        q_init (np.array, optional): Initial joint configuration to start IK. Shape: (num_joints,).
        p_trgt (np.array, optional): Target position. Expected shape: (3,).
        R_trgt (np.array, optional): Target orientation (rotation matrix). Expected shape: (3, 3).
        max_ik_tick (int): Maximum number of IK iterations.
        ik_err_th (float): Error threshold for termination.
        restore_state (bool): If True, store and later restore the environment state.
        ik_stepsize (float): Stepsize scaling factor for IK updates.
        ik_eps (float): Damping term for the least-squares computation.
        ik_th (float): Threshold (in radians) for damped least squares.
        verbose (bool): If True, print progress information.
        verbose_warning (bool): If True, print a warning if the final IK error is above the threshold.
        reset_env (bool): If True, reset the environment at the beginning.
        render (bool): If True, render the environment during IK iterations.
        render_every (int): Frequency (in iterations) at which to render the environment.
    
    Returns:
        tuple:
            - q_curr (np.array): Final joint configuration after IK. Shape: (num_joints,).
            - ik_err_stack (np.array): Final stacked IK error vector.
            - ik_info (dict): Dictionary containing the IK target information.
    """
    # Reset
    if reset_env:
        env.reset()
    if render:
        env.init_viewer()
    # Joint indices
    joint_idxs_jac = env.get_idxs_jac(joint_names=joint_names_for_ik)
    joint_idxs_fwd = env.get_idxs_fwd(joint_names=joint_names_for_ik)
    # Joint range
    q_mins = env.joint_ranges[get_idxs(env.joint_names,joint_names_for_ik),0]
    q_maxs = env.joint_ranges[get_idxs(env.joint_names,joint_names_for_ik),1]
    # Store MuJoCo state
    if restore_state:
        env.store_state()
    # Initial IK pose
    if q_init is not None:
        env.forward(q=q_init,joint_idxs=joint_idxs_fwd,increase_tick=False)
    # Initialize IK information
    ik_info = init_ik_info()
    add_ik_info(
        ik_info  = ik_info,
        body_name= body_name_trgt,
        p_trgt   = p_trgt,
        R_trgt   = R_trgt, 
    )
    # Loop
    q_curr = env.get_qpos_joints(joint_names=joint_names_for_ik)
    for ik_tick in range(max_ik_tick):
        dq,ik_err_stack = get_dq_from_ik_info(
            env            = env,
            ik_info        = ik_info,
            stepsize       = ik_stepsize,
            eps            = ik_eps,
            th             = ik_th,
            joint_idxs_jac = joint_idxs_jac,
        )
        q_curr = q_curr + dq[joint_idxs_jac] # update
        q_curr = np.clip(q_curr,q_mins,q_maxs) # clip
        env.forward(q=q_curr,joint_idxs=joint_idxs_fwd,increase_tick=False) # fk
        ik_err = np.linalg.norm(ik_err_stack) # IK error
        if ik_err < ik_err_th: break # terminate condition
        if verbose:
            print ("[%d/%d] ik_err:[%.3f]"%(ik_tick,max_ik_tick,ik_err))
        if render:
            if ik_tick%render_every==0:
                plot_ik_info(env,ik_info)
                env.render()
    # Print if IK error is too high
    if verbose_warning and ik_err > ik_err_th:
        print ("ik_err:[%.4f] is higher than ik_err_th:[%.4f]."%
               (ik_err,ik_err_th))
        print ("You may want to increase max_ik_tick:[%d]"%
               (max_ik_tick))
    # Restore backuped state
    if restore_state:
        env.restore_state()
    # Close viewer
    if render:
        env.close_viewer()
    # Return
    return q_curr,ik_err_stack,ik_info
