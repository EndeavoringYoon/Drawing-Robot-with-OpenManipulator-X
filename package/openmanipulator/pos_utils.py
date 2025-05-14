import numpy as np

def deg_to_byte(deg, range=[-180, 180]):
    """
    Convert a degree value into an integer (0 to 4095) based on a specified range.
    
    Parameters:
        deg (float): Degree value within the specified range.
        range (list): Optional range for the degree value [min, max].
        
    Returns:
        int: Corresponding integer value in the range of 0 to 4095.
    """
    if deg < range[0] or deg > range[1]:
        raise ValueError(f"Degree value must be in the range [{range[0]}, {range[1]}].")
    return int((deg - range[0]) * (4095 / (range[1] - range[0])))

def rad_to_byte(rad, range=[-np.pi, np.pi]):
    """
    Convert a radian value (-π to π) into an integer (0 to 4095).
    
    Parameters:
        rad (float): Radian value in the range of -π to π.
        range (list): Optional range for the radian value [min, max].
        
    Returns:
        int: Corresponding integer value in the range of 0 to 4095.
    """
    if rad < range[0] or rad > range[1]:
        raise ValueError("Radian value must be in the range [-π, π].")
    return int(rad * (4095 / (2 * np.pi)) + 2048)

def degs_to_bytes(degs, ranges=np.array([[-180, 180]])):
    """
    Convert an array of degree values into integers (0 to 4095) based on a fixed scale of [-180, 180].
    Each degree value is validated against its corresponding range.
    
    Parameters:
        degs (numpy.ndarray): Array of degree values.
        ranges (numpy.ndarray): Array of ranges for validation, where each range is [min, max].
                                If only one range is provided, it is applied to all values.
        
    Returns:
        numpy.ndarray: Array of corresponding integer values in the range of 0 to 4095.
    """
    degs = np.asarray(degs)  # Ensure input is a NumPy array
    ranges = np.asarray(ranges)  # Ensure ranges is a NumPy array

    # If only one range is provided, apply it to all values
    if len(ranges) == 1:
        ranges = np.tile(ranges, (len(degs), 1))

    # Validate the input values against their corresponding ranges
    for i, (deg, r) in enumerate(zip(degs, ranges)):
        if deg < r[0] or deg > r[1]:
            raise ValueError(f"Degree value at index {i} must be in the range [{r[0]}, {r[1]}].")

    # Convert using the fixed scale of [-180, 180]
    return ((degs + 180) * (4095 / 360)).astype(int)

def rads_to_bytes(rads, ranges=np.array([[-np.pi, np.pi]])):
    """
    Convert an array of radian values into integers (0 to 4095) based on a fixed scale of [-π, π].
    Each radian value is validated against its corresponding range.
    
    Parameters:
        rads (numpy.ndarray): Array of radian values.
        ranges (numpy.ndarray): Array of ranges for validation, where each range is [min, max].
                                If only one range is provided, it is applied to all values.
        
    Returns:
        numpy.ndarray: Array of corresponding integer values in the range of 0 to 4095.
    """
    rads = np.asarray(rads)  # Ensure input is a NumPy array
    ranges = np.asarray(ranges)  # Ensure ranges is a NumPy array

    # If only one range is provided, apply it to all values
    if len(ranges) == 1:
        ranges = np.tile(ranges, (len(rads), 1))

    # Validate the input values against their corresponding ranges
    for i, (rad, r) in enumerate(zip(rads, ranges)):
        if rad < r[0] or rad > r[1]:
            raise ValueError(f"Radian value at index {i} must be in the range [{r[0]}, {r[1]}].")

    # Convert using the fixed scale of [-π, π]
    return ((rads + np.pi) * (4095 / (2 * np.pi))).astype(int)

def bytes_to_rads(bytes):
    """
    Convert an array of byte values (0 to 4095) into radians based on specified ranges.
    
    Parameters:
        bytes (numpy.ndarray): Array of byte values.
        
    Returns:
        numpy.ndarray: Array of corresponding radian values.
    """
    bytes = np.asarray(bytes)  # Ensure input is a NumPy array

    return ((bytes - 2048) * (2 * np.pi / 4095))