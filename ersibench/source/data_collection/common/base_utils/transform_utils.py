# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""
Utility functions of matrix and vector transformations.
Adapted from OmniGibson
NOTE: convention for quaternions is (x, y, z, w)
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

PI = np.pi
EPS = np.finfo(float).eps * 4.0


def quat_xyzw_to_wxyz(quat_xyzw):
    """
    Convert quaternion from (x,y,z,w) to (w,x,y,z) format.

    Args:
        quat_xyzw (np.array): Quaternion in (x,y,z,w) format

    Returns:
        np.array: Quaternion in (w,x,y,z) format
    """
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def quat_wxyz_to_xyzw(quat_wxyz):
    """
    Convert quaternion from (w,x,y,z) to (x,y,z,w) format.

    Args:
        quat_wxyz (np.array): Quaternion in (w,x,y,z) format

    Returns:
        np.array: Quaternion in (x,y,z,w) format
    """
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])


def quat2mat_wxyz(quaternion_wxyz):
    """
    Converts given quaternion in (w,x,y,z) format to rotation matrix.

    Args:
        quaternion_wxyz (np.array): (..., 4) (w,x,y,z) float quaternion angles

    Returns:
        np.array: (..., 3, 3) rotation matrix
    """
    w, x, y, z = quaternion_wxyz
    rot = np.array(
        [
            [2 * (w**2 + x**2) - 1, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 2 * (w**2 + y**2) - 1, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 2 * (w**2 + z**2) - 1],
        ]
    )
    return rot


def mat2quat_wxyz(rmat):
    """
    Converts given rotation matrix to quaternion in (w,x,y,z) format.

    Args:
        rmat (np.array): (..., 3, 3) rotation matrix

    Returns:
        np.array: (..., 4) (w,x,y,z) float quaternion angles
    """
    quat_xyzw = R.from_matrix(rmat).as_quat()
    if quat_xyzw.ndim == 1:
        return quat_xyzw[[3, 0, 1, 2]]
    else:
        return quat_xyzw[:, [3, 0, 1, 2]]


def euler2quat_wxyz(euler, order="xyz"):
    """
    Convert euler angles to quaternion in wxyz format.
    """
    quat_xyzw = R.from_euler(order, euler).as_quat()
    return quat_xyzw_to_wxyz(quat_xyzw)


def quat_multiply(quaternion1, quaternion0):
    """
    Return multiplication of two quaternions (q1 * q0).

    E.g.:
    >>> q = quat_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> np.allclose(q, [-44, -14, 48, 28])
    True

    Args:
        quaternion1 (np.array): (x,y,z,w) quaternion
        quaternion0 (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) multiplied quaternion
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array(
        (
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        ),
        dtype=quaternion0.dtype,
    )


def quat_conjugate(quaternion):
    """
    Return conjugate of quaternion.

    E.g.:
    >>> q0 = random_quaternion()
    >>> q1 = quat_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True

    Args:
        quaternion (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) quaternion conjugate
    """
    return np.array(
        (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]),
        dtype=quaternion.dtype,
    )


def quat_inverse(quaternion):
    """
    Return inverse of quaternion.

    E.g.:
    >>> q0 = random_quaternion()
    >>> q1 = quat_inverse(q0)
    >>> np.allclose(quat_multiply(q0, q1), [0, 0, 0, 1])
    True

    Args:
        quaternion (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) quaternion inverse
    """
    return quat_conjugate(quaternion) / np.dot(quaternion, quaternion)


def mat2quat(rmat):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat (np.array): (..., 3, 3) rotation matrix

    Returns:
        np.array: (..., 4) (x,y,z,w) float quaternion angles
    """
    return R.from_matrix(rmat).as_quat()


def euler2mat(euler, order="xyz"):
    """
    Converts euler angles into rotation matrix form

    Args:
        euler (np.array): (r,p,y) angles in radians
        order (str): Euler angle order, e.g., "xyz", "zyx", "yxz", etc. Defaults to "xyz".

    Returns:
        np.array: 3x3 rotation matrix

    Raises:
        AssertionError: [Invalid input shape]
    """

    euler = np.asarray(euler, dtype=np.float64)
    assert euler.shape[-1] == 3, "Invalid shaped euler {}".format(euler)

    return R.from_euler(order, euler).as_matrix()


def mat2euler(rmat, order="xyz"):
    """
    Converts given rotation matrix to euler angles in radian.

    Args:
        rmat (np.array): 3x3 rotation matrix
        order (str): Euler angle order, e.g., "xyz", "zyx", "yxz", etc. Defaults to "xyz".

    Returns:
        np.array: (r,p,y) converted euler angles in radian vec3 float
    """
    M = np.array(rmat, dtype=rmat.dtype, copy=False)[:3, :3]
    return R.from_matrix(M).as_euler(order)


def pose2mat(pose):
    """
    Converts pose to homogeneous matrix.

    Args:
        pose (2-tuple): a (pos, orn) tuple where pos is vec3 float cartesian, and
            orn is vec4 float quaternion.

    Returns:
        np.array: 4x4 homogeneous matrix
    """
    homo_pose_mat = np.zeros((4, 4), dtype=pose[0].dtype)
    homo_pose_mat[:3, :3] = quat2mat(pose[1])
    homo_pose_mat[:3, 3] = np.array(pose[0], dtype=pose[0].dtype)
    homo_pose_mat[3, 3] = 1.0
    return homo_pose_mat


def quat2mat(quaternion):
    """
    Converts given quaternion to matrix.

    Args:
        quaternion (np.array): (..., 4) (x,y,z,w) float quaternion angles

    Returns:
        np.array: (..., 3, 3) rotation matrix
    """
    return R.from_quat(quaternion).as_matrix()


def euler2quat(euler, order="xyz"):
    """
    Converts euler angles into quaternion form

    Args:
        euler (np.array): (r,p,y) angles in radians
        order (str): Euler angle order, e.g., "xyz", "zyx", "yxz", etc. Defaults to "xyz".

    Returns:
        np.array: (x,y,z,w) float quaternion angles

    Raises:
        AssertionError: [Invalid input shape]
    """
    return R.from_euler(order, euler).as_quat()


def pose_inv(pose_mat):
    """
    Computes the inverse of a homogeneous matrix corresponding to the pose of some
    frame B in frame A. The inverse is the pose of frame A in frame B.

    Args:
        pose_mat (np.array): 4x4 matrix for the pose to inverse

    Returns:
        np.array: 4x4 matrix for the inverse pose
    """

    # Note, the inverse of a pose matrix is the following
    # [R t; 0 1]^-1 = [R.T -R.T*t; 0 1]

    # Intuitively, this makes sense.
    # The original pose matrix translates by t, then rotates by R.
    # We just invert the rotation by applying R-1 = R.T, and also translate back.
    # Since we apply translation first before rotation, we need to translate by
    # -t in the original frame, which is -R-1*t in the new frame, and then rotate back by
    # R-1 to align the axis again.

    pose_inv = np.zeros((4, 4))
    pose_inv[:3, :3] = pose_mat[:3, :3].T
    pose_inv[:3, 3] = -pose_inv[:3, :3].dot(pose_mat[:3, 3])
    pose_inv[3, 3] = 1.0
    return pose_inv


def rotation_matrix(angle, direction, point=None):
    """
    Returns matrix to rotate about axis defined by point and direction.

    E.g.:
        >>> angle = (random.random() - 0.5) * (2*math.pi)
        >>> direc = numpy.random.random(3) - 0.5
        >>> point = numpy.random.random(3) - 0.5
        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> R0 = rotation_matrix(angle, direc, point)
        >>> R1 = rotation_matrix(-angle, -direc, point)
        >>> is_same_transform(R0, R1)
        True

        >>> I = numpy.identity(4, numpy.float32)
        >>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
        True

        >>> numpy.allclose(2., numpy.trace(rotation_matrix(math.pi/2,
        ...                                                direc, point)))
        True

    Args:
        angle (float): Magnitude of rotation
        direction (np.array): (ax,ay,az) axis about which to rotate
        point (None or np.array): If specified, is the (x,y,z) point about which the rotation will occur

    Returns:
        np.array: 4x4 homogeneous matrix that includes the desired rotation
    """
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0, 0.0, cosa)), dtype=direction.dtype)
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0], 0.0),
        ),
        dtype=direction.dtype,
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=direction.dtype, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


def unit_vector(data, axis=None, out=None):
    """
    Returns ndarray normalized by length, i.e. eucledian norm, along axis.

    E.g.:
        >>> v0 = numpy.random.random(3)
        >>> v1 = unit_vector(v0)
        >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
        True

        >>> v0 = numpy.random.rand(5, 4, 3)
        >>> v1 = unit_vector(v0, axis=-1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = unit_vector(v0, axis=1)
        >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
        >>> numpy.allclose(v1, v2)
        True

        >>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float32)
        >>> unit_vector(v0, axis=1, out=v1)
        >>> numpy.allclose(v1, v2)
        True

        >>> list(unit_vector([]))
        []

        >>> list(unit_vector([1.0]))
        [1.0]

    Args:
        data (np.array): data to normalize
        axis (None or int): If specified, determines specific axis along data to normalize
        out (None or np.array): If specified, will store computation in this variable

    Returns:
        None or np.array: If @out is not specified, will return normalized vector. Otherwise, stores the output in @out
    """
    if out is None:
        data = np.array(data, dtype=data.dtype, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


def frustum(left, right, bottom, top, znear, zfar):
    """Create view frustum matrix."""
    assert right != left
    assert bottom != top
    assert znear != zfar

    M = np.zeros((4, 4), dtype=np.float32)
    M[0, 0] = +2.0 * znear / (right - left)
    M[2, 0] = (right + left) / (right - left)
    M[1, 1] = +2.0 * znear / (top - bottom)
    M[2, 1] = (top + bottom) / (top - bottom)
    M[2, 2] = -(zfar + znear) / (zfar - znear)
    M[3, 2] = -2.0 * znear * zfar / (zfar - znear)
    M[2, 3] = -1.0
    return M


def perspective(fovy, aspect, znear, zfar):
    """Create perspective projection matrix."""
    # fovy is in degree
    assert znear != zfar
    h = np.tan(fovy / 360.0 * np.pi) * znear
    w = h * aspect
    return frustum(-w, w, -h, h, znear, zfar)


def anorm(x, axis=None, keepdims=False):
    """Compute L2 norms alogn specified axes."""
    return np.linalg.norm(x, axis=axis, keepdims=keepdims)


def normalize(v, axis=None, eps=1e-10):
    """L2 Normalize along specified axes."""
    norm = anorm(v, axis=axis, keepdims=True)
    return v / np.where(norm < eps, eps, norm)


def deg2rad(deg):
    return deg * np.pi / 180.0


def rad2deg(rad):
    return rad * 180.0 / np.pi


def pose_difference(pose1, pose2):
    """
    Calculate the position and rotation difference between two poses.

    Args:
        pose1 (np.array): 4x4 homogeneous transformation matrix
        pose2 (np.array): 4x4 homogeneous transformation matrix

    Returns:
        tuple: (position_distance, angle_difference_in_degrees)
    """
    # Extract positions
    position1 = pose1[:3, 3]
    position2 = pose2[:3, 3]

    # Calculate Euclidean distance of positions
    position_distance = np.linalg.norm(position1 - position2)

    # Extract rotation matrices
    rotation1 = pose1[:3, :3]
    rotation2 = pose2[:3, :3]

    # Calculate angle difference of rotation matrices
    r1 = R.from_matrix(rotation1)
    r2 = R.from_matrix(rotation2)

    # Calculate rotation difference
    relative_rotation = r1.inv() * r2
    angle_difference = relative_rotation.magnitude()

    return position_distance, np.degrees(angle_difference)


def rotate_along_axis(target_affine, angle_degrees, rot_axis="z", use_local=True):
    """
    Rotate target_affine according to specified angle and rotation axis.

    Args:
        target_affine (np.array): 4x4 affine transformation matrix
        angle_degrees (float): Rotation angle (in degrees)
        rot_axis (str): Rotation axis, 'x', 'y', or 'z'
        use_local (bool): If True, rotate in local frame; if False, rotate in world frame

    Returns:
        np.array: 4x4 rotated affine transformation matrix
    """
    # Convert angle to radians
    angle_radians = np.deg2rad(angle_degrees)

    # Create rotation object
    if rot_axis == "z":
        rotation_vector = np.array([0, 0, angle_radians])
    elif rot_axis == "y":
        rotation_vector = np.array([0, angle_radians, 0])
    elif rot_axis == "x":
        rotation_vector = np.array([angle_radians, 0, 0])
    else:
        raise ValueError("Invalid rotation axis. Please choose from 'x', 'y', 'z'.")

    # Generate rotation matrix
    R_angle = R.from_rotvec(rotation_vector).as_matrix()

    # Extract rotation part (3x3 matrix)
    target_rotation = target_affine[:3, :3]

    # Rotate target_rotation around specified axis by specified angle
    if use_local:
        target_rotation_2 = np.dot(target_rotation, R_angle)
    else:
        target_rotation_2 = np.dot(R_angle, target_rotation)

    # Recombine rotation matrix and original translation part
    target_affine_2 = np.eye(4)
    target_affine_2[:3, :3] = target_rotation_2
    target_affine_2[:3, 3] = target_affine[:3, 3]

    return target_affine_2


def quaternion_rotate(quaternion, axis, angle):
    """
    Rotate a quaternion around a specified axis by a given angle.

    Args:
        quaternion (np.array): The input quaternion [w, x, y, z].
        axis (str): The axis to rotate around ('x', 'y', or 'z').
        angle (float): The rotation angle in degrees.

    Returns:
        np.array: The rotated quaternion.
    """
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle)

    # Calculate the rotation quaternion based on the specified axis
    cos_half_angle = np.cos(angle_rad / 2)
    sin_half_angle = np.sin(angle_rad / 2)

    if axis == "x":
        q_axis = np.array([cos_half_angle, sin_half_angle, 0, 0])
    elif axis == "y":
        q_axis = np.array([cos_half_angle, 0, sin_half_angle, 0])
    elif axis == "z":
        q_axis = np.array([cos_half_angle, 0, 0, sin_half_angle])
    else:
        raise ValueError("Unsupported axis. Use 'x', 'y', or 'z'.")

    # Extract components of the input quaternion
    w1, x1, y1, z1 = quaternion
    w2, x2, y2, z2 = q_axis

    # Quaternion multiplication
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def axis_to_quaternion(axis, target_axis="y", up_side_down=False):
    """
    Calculate the quaternion that rotates a given axis to the target axis.

    Args:
        axis (str): The axis in the object's local coordinate system ('x', 'y', or 'z').
        target_axis (str): The target axis in the world coordinate system ('x', 'y', or 'z').
        up_side_down (bool): If True, flip the axis direction.

    Returns:
        np.array: The quaternion representing the rotation [w, x, y, z].
    """
    # Define unit vectors for each axis
    unit_vectors = {
        "x": np.array([1, 0, 0]),
        "y": np.array([0, 1, 0]),
        "z": np.array([0, 0, 1]),
    }

    if axis not in unit_vectors or target_axis not in unit_vectors:
        raise ValueError("Unsupported axis. Use 'x', 'y', or 'z'.")

    if axis == "z" and up_side_down:
        # Special case: 180 degree rotation around x or y axis
        return np.array([0, 1, 0, 0])  # 180 degree rotation around x-axis

    v1 = unit_vectors[axis] * (-1 if up_side_down else 1)
    v2 = unit_vectors[target_axis]

    # Calculate the cross product and dot product
    cross_prod = np.cross(v1, v2)
    dot_prod = np.dot(v1, v2)

    # Calculate the quaternion
    w = np.sqrt((np.linalg.norm(v1) ** 2) * (np.linalg.norm(v2) ** 2)) + dot_prod
    x, y, z = cross_prod

    # Normalize the quaternion
    q = np.array([w, x, y, z])
    q = q / np.linalg.norm(q)

    return q


def skew(vector: np.ndarray) -> np.ndarray:
    """
    Convert vector to skew symmetric matrix.

    This function returns a skew-symmetric matrix to perform cross-product
    as a matrix multiplication operation, i.e.: np.cross(a, b) = np.dot(skew(a), b)

    Args:
        vector (np.ndarray): A 3x1 vector.

    Returns:
        np.ndarray: The resulting skew-symmetric matrix.
    """
    mat = np.array(
        [
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0],
        ]
    )
    return mat


def world_to_camera(points_world: list, t_camera: np.ndarray) -> list:
    """
    Convert multiple points from world coordinate system to camera coordinate system.

    Args:
        points_world: List of points in world coordinate system, each point is np.ndarray or list [x, y, z]
        t_camera: World transformation matrix of camera (4x4)

    Returns:
        list: List of points in camera coordinate system
    """
    t_camera_inv = np.linalg.inv(t_camera)

    camera_points = []
    for point in points_world:
        point_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        point_camera_homogeneous = np.dot(t_camera_inv, point_homogeneous)
        point_camera = point_camera_homogeneous[:3]
        camera_points.append(point_camera)

    return camera_points


def world_to_robot_base(points_world: list, t_robot: np.ndarray) -> list:
    """
    Convert multiple points from world coordinate system to robot coordinate system.

    Args:
        points_world: List of points in world coordinate system, each point is np.ndarray or list [x, y, z]
        t_robot: World transformation matrix of robot (4x4)

    Returns:
        list: List of points in robot coordinate system
    """
    t_robot_inv = np.linalg.inv(t_robot)

    robot_points = []
    for point in points_world:
        point_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        point_robot_homogeneous = np.dot(t_robot_inv, point_homogeneous)
        point_robot = point_robot_homogeneous[:3]
        robot_points.append(point_robot)

    return robot_points


def get_pose_wxyz(xyz: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Construct a 4x4 pose matrix from position and quaternion (w,x,y,z format).

    Args:
        xyz (np.ndarray): Position [x, y, z]
        quat_wxyz (np.ndarray): Quaternion in (w, x, y, z) format

    Returns:
        np.ndarray: 4x4 pose matrix
    """
    pose = np.eye(4)
    pose[:3, :3] = quat2mat_wxyz(quat_wxyz)
    pose[:3, 3] = xyz
    return pose


def transform_world_axis_to_robot_axis(world_pose_matrix, robot_position, robot_rotation):
    """
    Transform a pose from world coordinates to robot base coordinates.

    Args:
        world_pose_matrix (np.ndarray): 4x4 pose matrix in world coordinates
        robot_position (np.ndarray): Robot base position [x, y, z]
        robot_rotation (np.ndarray): Robot base rotation quaternion (w, x, y, z)

    Returns:
        np.ndarray: 4x4 pose matrix in robot base coordinates
    """
    robot_pose = get_pose_wxyz(robot_position, robot_rotation)
    robot_pose_inv = np.linalg.inv(robot_pose)
    return robot_pose_inv @ world_pose_matrix


def calculate_y_axis_projection(transform_matrix: np.ndarray, size: tuple) -> list:
    """
    Calculate four vertices of object's Y-axis projection in its own coordinate system
    (coordinates in world coordinate system).

    Args:
        transform_matrix (np.ndarray): 4x4 transformation matrix
        size (tuple): Object size (dx, dy, dz)

    Returns:
        list: List of four vertex coordinates
    """
    dx, dy, dz = size
    local_vertices_y = np.array(
        [
            [dx / 2, dy / 2, dz / 2, 1],
            [-dx / 2, dy / 2, dz / 2, 1],
            [dx / 2, dy / 2, -dz / 2, 1],
            [-dx / 2, dy / 2, -dz / 2, 1],
        ]
    )
    world_vertices = []
    for v in local_vertices_y:
        v_world = np.dot(transform_matrix, v.T)[:3]
        world_vertices.append(v_world)

    return world_vertices


# ===============================================
# Functions merged from transforms.py
# ===============================================


def calculate_rotation_matrix2(v1, v2):
    """
    Calculate the rotation matrix that aligns v1 to v2.

    Args:
        v1 (np.ndarray): Source vector
        v2 (np.ndarray): Target vector

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # 1. Check if already aligned (parallel and same direction)
    if np.allclose(v1, v2, atol=1e-10):
        return np.eye(3)  # No rotation needed

    # 2. Check if anti-parallel
    if np.allclose(v1, -v2, atol=1e-10):
        # Need 180 degree rotation
        # Find a vector perpendicular to v1 as rotation axis
        if abs(v1[0]) < 0.9:  # Avoid selecting axis parallel to v1
            rot_axis = np.cross(v1, [1, 0, 0])
        else:
            rot_axis = np.cross(v1, [0, 1, 0])
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_angle = np.pi
    else:
        # 3. General case
        rot_axis = np.cross(v1, v2)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    # Use scipy's Rotation to create rotation matrix
    return R.from_rotvec(rot_axis * rot_angle).as_matrix()


def get_cross_prod_mat(pVec_Arr):
    """
    Get cross product matrix (skew-symmetric matrix) for a vector.

    Args:
        pVec_Arr (np.ndarray): 3D vector

    Returns:
        np.ndarray: 3x3 skew-symmetric matrix
    """
    qCross_prod_mat = np.array(
        [
            [0, -pVec_Arr[2], pVec_Arr[1]],
            [pVec_Arr[2], 0, -pVec_Arr[0]],
            [-pVec_Arr[1], pVec_Arr[0], 0],
        ]
    )
    return qCross_prod_mat


def calculate_rotation_matrix(v1, v2):
    """
    Calculate rotation matrix that aligns v1 to v2 (alternative implementation).

    Args:
        v1 (np.ndarray): Source vector
        v2 (np.ndarray): Target vector

    Returns:
        np.ndarray: 3x3 rotation matrix (scaled by norm of v2)
    """
    scale = np.linalg.norm(v2)
    v2 = v2 / scale
    # must ensure pVec_Arr is also a unit vec.
    z_mat = get_cross_prod_mat(v1)

    z_c_vec = np.matmul(z_mat, v2)
    z_c_vec_mat = get_cross_prod_mat(z_c_vec)

    if np.dot(v1, v2) == -1:
        qTrans_Mat = -np.eye(3, 3)
    elif np.dot(v1, v2) == 1:
        qTrans_Mat = np.eye(3, 3)
    else:
        qTrans_Mat = np.eye(3, 3) + z_c_vec_mat + np.matmul(z_c_vec_mat, z_c_vec_mat) / (1 + np.dot(v1, v2))

    qTrans_Mat *= scale
    return qTrans_Mat


def calculate_rotation_from_two_axes(v_dir1, v_axis1, v_dir2, v_axis2, tol=1e-6):
    """
    Compute a rotation matrix R that maps two orthogonal directions from
    one frame to another. The function builds orthonormal bases (x,y,z) for
    each object where z is the direction (v_dir) and x is the constraint axis
    projected to be orthogonal to z. Returns R such that R @ [x1 y1 z1] = [x2 y2 z2].

    If the constraint axis is degenerate (nearly parallel to direction), the
    function falls back to a minimal rotation aligning v_dir1->v_dir2.

    Args:
        v_dir1 (np.ndarray): First direction vector
        v_axis1 (np.ndarray): First constraint axis
        v_dir2 (np.ndarray): Second direction vector
        v_axis2 (np.ndarray): Second constraint axis
        tol (float): Tolerance for degenerate cases

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    v_dir1 = np.array(v_dir1, dtype=float)
    v_dir2 = np.array(v_dir2, dtype=float)
    v_axis1 = np.array(v_axis1, dtype=float)
    v_axis2 = np.array(v_axis2, dtype=float)

    # normalize directions
    if np.linalg.norm(v_dir1) < tol or np.linalg.norm(v_dir2) < tol:
        raise ValueError("direction vectors must be non-zero")
    z1 = v_dir1 / np.linalg.norm(v_dir1)
    z2 = v_dir2 / np.linalg.norm(v_dir2)

    # project constraint axes to plane orthogonal to respective z and normalize
    a1 = v_axis1 - np.dot(v_axis1, z1) * z1
    a2 = v_axis2 - np.dot(v_axis2, z2) * z2
    n1 = np.linalg.norm(a1)
    n2 = np.linalg.norm(a2)

    if n1 < tol or n2 < tol:
        # degenerate: constraint axis nearly parallel to direction; fall back
        # to minimal single-axis rotation (keep existing behavior)
        dot = np.clip(np.dot(z1, z2), -1.0, 1.0)
        if np.isclose(dot, 1.0):
            return np.eye(3)
        if np.isclose(dot, -1.0):
            # 180 degree rotation: pick arbitrary orthogonal axis
            ort = np.array([1.0, 0.0, 0.0])
            if np.allclose(np.abs(np.dot(z1, ort)), 1.0):
                ort = np.array([0.0, 1.0, 0.0])
            axis = np.cross(z1, ort)
            axis /= np.linalg.norm(axis)
            return R.from_rotvec(axis * np.pi).as_matrix()
        axis = np.cross(z1, z2)
        axis /= np.linalg.norm(axis)
        angle = np.arccos(dot)
        return R.from_rotvec(axis * angle).as_matrix()

    x1 = a1 / n1
    x2 = a2 / n2
    y1 = np.cross(z1, x1)
    y2 = np.cross(z2, x2)

    # build bases: columns are x,y,z
    A = np.column_stack((x1, y1, z1))
    B = np.column_stack((x2, y2, z2))

    # rotation that maps A -> B is R = B * A^T
    R_mat = B @ A.T
    return R_mat


def transform_coordinates_3d(coordinates: np.ndarray, sRT: np.ndarray):
    """
    Apply 3D affine transformation to pointcloud.

    Args:
        coordinates (np.ndarray): Point cloud of shape [3, N]
        sRT (np.ndarray): 4x4 transformation matrix

    Returns:
        np.ndarray: Transformed point cloud of shape [3, N]
    """
    assert coordinates.shape[0] == 3
    coordinates = np.vstack([coordinates, np.ones((1, coordinates.shape[1]), dtype=np.float32)])
    new_coordinates = sRT @ coordinates
    new_coordinates = new_coordinates[:3, :] / new_coordinates[3, :]
    return new_coordinates


def rotate_around_axis(pose, P1, vector, angle_delta):
    """
    Rotate an object around an axis in world coordinate system.

    Args:
        pose (np.ndarray): 4x4 object pose matrix
        P1 (np.ndarray): Rotation axis start point, shape (3,)
        vector (np.ndarray): Rotation axis direction vector, shape (3,)
        angle_delta (float): Rotation angle in degrees

    Returns:
        np.ndarray: Rotated 4x4 pose matrix
    """
    # Calculate rotation axis direction vector
    v = vector
    # Normalize direction vector
    u = v / np.linalg.norm(v)
    theta = np.radians(angle_delta)

    # Calculate matrix K for Rodrigues' rotation formula
    ux, uy, uz = u
    K = np.array([[0, -uz, uy], [uz, 0, -ux], [-uy, ux, 0]])

    # Calculate rotation matrix R
    identity_matrix = np.eye(3)
    R_rot = identity_matrix + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

    # Convert R to 4x4 form
    R_4x4 = np.eye(4)
    R_4x4[:3, :3] = R_rot

    # Build translation matrix T1
    T1 = np.eye(4)
    T1[:3, 3] = -P1

    # Build translation matrix T2
    T2 = np.eye(4)
    T2[:3, 3] = P1

    # Combine transformation matrices
    M = T2 @ R_4x4 @ T1

    # Apply transformation to original pose matrix
    new_pose = M @ pose

    return new_pose


def pose_from_position_quaternion(position, quaternion):
    """
    Construct a 4x4 pose matrix from position and quaternion.

    Args:
        position (np.ndarray): Position [x, y, z]
        quaternion (np.ndarray): Quaternion in [w, x, y, z] format

    Returns:
        np.ndarray: 4x4 pose matrix
    """
    # quaternion: [w, x, y, z]
    # position: [x, y, z]
    quaternion = np.array(quaternion)[[1, 2, 3, 0]]
    pose = np.eye(4)
    pose[:3, :3] = R.from_quat(quaternion).as_matrix()
    pose[:3, 3] = position
    return pose


# ===============================================
# Functions merged from client/layout/utils/transform_utils.py
# ===============================================


def transform_points(points, transform_matrix):
    """
    Transform point cloud using a 4x4 transformation matrix.
    Input format: (N, 3) - each row is a point.

    Args:
        points (np.ndarray): Point cloud of shape (N, 3)
        transform_matrix (np.ndarray): 4x4 transformation matrix

    Returns:
        np.ndarray: Transformed point cloud of shape (N, 3)
    """
    assert points.shape[1] == 3, "Input points must be Nx3 array"
    assert transform_matrix.shape == (4, 4), "Transform matrix must be 4x4"

    # Convert points to homogeneous coordinates (N x 4)
    ones = np.ones((points.shape[0], 1))
    points_homogeneous = np.hstack((points, ones))

    # Apply transformation matrix
    transformed_points_homogeneous = points_homogeneous @ transform_matrix.T

    # Convert back to non-homogeneous coordinates (N x 3)
    transformed_points = transformed_points_homogeneous[:, :3]

    return transformed_points


def farthest_point_sampling(pc, num_points):
    """
    Given a point cloud, sample num_points points that are the farthest apart.
    Uses Open3D farthest point sampling.

    Args:
        pc (np.ndarray): Point cloud of shape (N, 3)
        num_points (int): Number of points to sample

    Returns:
        np.ndarray: Sampled points of shape (num_points, 3)
    """
    import open3d as o3d

    assert pc.ndim == 2 and pc.shape[1] == 3, "pc must be a (N, 3) numpy array"
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    downpcd_farthest = pcd.farthest_point_down_sample(num_points)
    return np.asarray(downpcd_farthest.points)


# ===============================================
# Functions merged from solver_2d/solver.py
# ===============================================


def rotate_point_2d(px, py, angle, ox, oy):
    """
    Rotate a 2D point around a specified center point.

    Args:
        px (float): X coordinate of the point to rotate
        py (float): Y coordinate of the point to rotate
        angle (float): Rotation angle in radians
        ox (float): X coordinate of the rotation center
        oy (float): Y coordinate of the rotation center

    Returns:
        tuple: (xnew, ynew) - Rotated point coordinates
    """
    import math

    s, c = math.sin(angle), math.cos(angle)
    px, py = px - ox, py - oy
    xnew = px * c - py * s
    ynew = px * s + py * c
    return xnew + ox, ynew + oy


def compute_rectangle_intersection(bbox, plane_center, plane_width, plane_height):
    """
    Compute the intersection of two rectangles.

    Args:
        bbox (tuple): Bounding box as (center_x, center_y, width, height, angle)
        plane_center (tuple): Plane center as (x, y)
        plane_width (float): Width of the plane
        plane_height (float): Height of the plane

    Returns:
        tuple or None: Intersection rectangle as (center_x, center_y, width, height, 0)
                      or None if no intersection
    """
    # Unpack bounding box information
    bbox_center_x, bbox_center_y, bbox_width, bbox_height, _ = bbox

    # Calculate bounding box boundaries
    min_x = bbox_center_x - bbox_width / 2
    max_x = bbox_center_x + bbox_width / 2
    min_y = bbox_center_y - bbox_height / 2
    max_y = bbox_center_y + bbox_height / 2

    # Calculate plane boundaries
    plane_min_x = plane_center[0] - plane_width / 2
    plane_max_x = plane_center[0] + plane_width / 2
    plane_min_y = plane_center[1] - plane_height / 2
    plane_max_y = plane_center[1] + plane_height / 2

    # Calculate intersection boundaries
    intersect_min_x = max(min_x, plane_min_x)
    intersect_max_x = min(max_x, plane_max_x)
    intersect_min_y = max(min_y, plane_min_y)
    intersect_max_y = min(max_y, plane_max_y)

    # Check if intersection region is valid
    if intersect_min_x < intersect_max_x and intersect_min_y < intersect_max_y:
        # Calculate intersection rectangle center and size
        intersect_center_x = (intersect_min_x + intersect_max_x) / 2
        intersect_center_y = (intersect_min_y + intersect_max_y) / 2
        intersect_width = intersect_max_x - intersect_min_x
        intersect_height = intersect_max_y - intersect_min_y

        return (
            intersect_center_x,
            intersect_center_y,
            intersect_width,
            intersect_height,
            0,
        )
    else:
        # No valid intersection region
        return None
