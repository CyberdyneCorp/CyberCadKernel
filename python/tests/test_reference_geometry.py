"""Datum reference geometry (planes / axes).

These are exact fp64 vector-math constructors on the ABI: a plane from three
points and an axis from two points return an origin plus a **unit** normal /
direction. The task's headline check — "a reference plane from 3 points has the
expected normal" — lives here.
"""

import math

import numpy as np
import pytest

from cybercadkernel import Kernel


def _unit(v):
    v = np.asarray(v, dtype=np.float64)
    return v / np.linalg.norm(v)


def test_ref_plane_from_three_points_has_expected_normal(kernel: Kernel):
    # A plane through (0,0,0), (1,0,0), (0,1,0) is the XY plane: origin at p0,
    # unit +Z normal.
    plane = kernel.ref_plane_from_points((0, 0, 0), (1, 0, 0), (0, 1, 0))
    assert plane.origin == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert plane.normal == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)
    assert np.linalg.norm(plane.normal) == pytest.approx(1.0, abs=1e-9)


def test_ref_plane_from_tilted_points_normal(kernel: Kernel):
    # Points spanning the plane z = x. Its normal is (-1, 0, 1)/sqrt(2) up to
    # sign; assert the direction (via |dot|==1) and unit length.
    plane = kernel.ref_plane_from_points((0, 0, 0), (1, 0, 1), (0, 1, 0))
    n = np.asarray(plane.normal, dtype=np.float64)
    assert np.linalg.norm(n) == pytest.approx(1.0, abs=1e-9)
    expected = _unit((-1.0, 0.0, 1.0))
    assert abs(float(np.dot(n, expected))) == pytest.approx(1.0, abs=1e-9)


def test_ref_plane_offset_moves_origin_along_normal(kernel: Kernel):
    plane = kernel.ref_plane_offset((0, 0, 0), (0, 0, 1), 5.0)
    assert plane.origin == pytest.approx((0.0, 0.0, 5.0), abs=1e-9)
    assert plane.normal == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_ref_axis_from_two_points_unit_direction(kernel: Kernel):
    axis = kernel.ref_axis_from_points((0, 0, 0), (0, 0, 5))
    assert axis.origin == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert axis.direction == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)
    assert np.linalg.norm(axis.direction) == pytest.approx(1.0, abs=1e-9)


def test_ref_axis_from_diagonal_points(kernel: Kernel):
    axis = kernel.ref_axis_from_points((0, 0, 0), (1, 1, 1))
    d = np.asarray(axis.direction, dtype=np.float64)
    inv = 1.0 / math.sqrt(3.0)
    assert d == pytest.approx((inv, inv, inv), abs=1e-9)
    assert np.linalg.norm(d) == pytest.approx(1.0, abs=1e-9)
