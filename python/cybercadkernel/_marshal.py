"""Small helpers to marshal Python sequences into ctypes buffers.

Kept separate so the object model in :mod:`cybercadkernel.kernel` reads as
geometry, not as pointer bookkeeping.
"""

from __future__ import annotations

import ctypes
from typing import Sequence

import numpy as np


def dbl_array(xy) -> tuple[ctypes.Array, int]:
    """Flatten a sequence of numbers (or (N,2)/(N,3) array) into a
    ``c_double[]`` and return ``(buffer, element_count)``."""
    arr = np.ascontiguousarray(np.asarray(xy, dtype=np.float64).ravel())
    buf = (ctypes.c_double * arr.size)(*arr.tolist())
    return buf, arr.size


def dbl_ptr(xy) -> ctypes.Array:
    return dbl_array(xy)[0]


def int_array(ids: Sequence[int]) -> tuple[ctypes.Array, int]:
    seq = list(ids)
    buf = (ctypes.c_int * len(seq))(*seq)
    return buf, len(seq)


def point_count(xy) -> int:
    """Number of 2-D points in a flat or (N,2) sequence."""
    arr = np.asarray(xy, dtype=np.float64)
    return int(arr.size // 2)


def point_count_3d(xyz) -> int:
    arr = np.asarray(xyz, dtype=np.float64)
    return int(arr.size // 3)
