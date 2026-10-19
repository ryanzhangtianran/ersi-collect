# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0
"""Minimal numpy-based cv_bridge fallback.

The real ``cv_bridge`` is built only for the system ROS Python and isn't
importable under IsaacSim's bundled Python. This implements the ``cv2_to_imgmsg``
/ ``imgmsg_to_cv2`` conversions the ROS publishers use, via numpy +
``sensor_msgs/Image``. Used only when the real ``cv_bridge`` is unavailable.
"""

import numpy as np
from sensor_msgs.msg import Image

# encoding -> (numpy dtype, channels)
_ENCODINGS = {
    "rgb8": (np.uint8, 3),
    "bgr8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "bgra8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "8UC1": (np.uint8, 1),
    "8UC3": (np.uint8, 3),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


class CvBridge:
    def cv2_to_imgmsg(self, cvim, encoding="passthrough", header=None):
        arr = np.ascontiguousarray(cvim)
        channels = 1 if arr.ndim == 2 else int(arr.shape[2])
        if encoding == "passthrough":
            encoding = {
                (np.uint8, 1): "mono8",
                (np.uint8, 3): "bgr8",
                (np.uint16, 1): "16UC1",
                (np.float32, 1): "32FC1",
            }.get((arr.dtype.type, channels), "")
        msg = Image()
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(arr.strides[0])
        msg.data = arr.tobytes()
        if header is not None:
            msg.header = header
        return msg

    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        dtype, channels = _ENCODINGS.get(img_msg.encoding, (np.uint8, 3))
        arr = np.frombuffer(bytes(img_msg.data), dtype=dtype)
        if channels == 1:
            return arr.reshape(img_msg.height, img_msg.width)
        return arr.reshape(img_msg.height, img_msg.width, channels)
