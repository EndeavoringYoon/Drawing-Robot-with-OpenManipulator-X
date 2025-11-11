def get_id_dict_of_omx():
    """
    Get the body name -> ID mapping for OMX(OpenManipulator-X).
    """
    id_dict = {
        'joint1':11,
        'joint2':12,
        'joint3':13,
        'joint4':14,
        'gripper_crank_joint':15,
    }
    return id_dict


def get_id_list_of_omx(body_names=None):
    """
    Return a list of IDs ordered by names.

    - If body_names is None: use env.body_names order, include only names present in the ID dict.
    - If body_names is a list: use that order, include only names present in the ID dict.

    Unknown names (not in the ID dict) are silently skipped.
    """
    id_dict = get_id_dict_of_omx()

    # decide the source order
    if body_names is None:
        try:
            source_names = id_dict.keys()
        except AttributeError:
            raise ValueError("env must have an iterable attribute 'body_names' when body_names=None.")
    else:
        source_names = list(body_names)

    # keep only names we know, preserving order
    id_list = [id_dict[name] for name in source_names if name in id_dict]
    return id_list