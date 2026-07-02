"""Real geometry through the pythonic API.

Every assertion pins an exact / analytic geometric property (volume, area,
centroid, boolean result, bounding box, watertightness, STEP round-trip) — not a
trivially-true check. The numbers here match the proven manual build
(``brep_available == True``, box vol 1000, cut 875).
"""

import math
import os

import numpy as np
import pytest

from cybercadkernel import Kernel


# ── Extrude → exact volume + surface area ─────────────────────────────────────


def test_extrude_box_exact_volume_and_area(box):
    # Extrude a 10x10 square by 10 → a 10x10x10 solid.
    with box(10, 10, 10) as b:
        mp = b.mass_properties()
        assert mp.valid
        assert mp.volume == pytest.approx(1000.0, abs=1e-6)
        assert mp.area == pytest.approx(600.0, abs=1e-6)
        assert mp.center_of_mass == pytest.approx((5.0, 5.0, 5.0), abs=1e-6)


# ── Booleans → exact 875 / 1875 / 125 ─────────────────────────────────────────


@pytest.fixture
def overlapping_boxes(box):
    """Two 10x10x10 boxes overlapping in exactly one 5x5x5 corner.

    Box A sits at the origin; box B is translated by (5,5,5) so their
    intersection is the 5^3 = 125 corner cube. This is the exact configuration
    that yields the proven boolean numbers:

        cut    = 1000 - 125 = 875
        fuse   = 1000 + 1000 - 125 = 1875
        common = 125
    """
    a = box(10, 10, 10)
    b = box(10, 10, 10).translate(5, 5, 5)
    return a, b


def test_boolean_cut_is_875(overlapping_boxes):
    a, b = overlapping_boxes
    with a.cut(b) as diff:
        assert diff.mass_properties().volume == pytest.approx(875.0, abs=1e-6)


def test_boolean_fuse_is_1875(overlapping_boxes):
    a, b = overlapping_boxes
    with a.fuse(b) as fused:
        assert fused.mass_properties().volume == pytest.approx(1875.0, abs=1e-6)


def test_boolean_common_is_125(overlapping_boxes):
    a, b = overlapping_boxes
    with a.common(b) as inter:
        assert inter.mass_properties().volume == pytest.approx(125.0, abs=1e-6)


def test_cut_corner_box_is_875(box):
    # The literal proven case: box(10) minus a 5^3 box sharing the origin corner.
    with box(10, 10, 10) as big, box(5, 5, 5) as small:
        with big.cut(small) as diff:
            assert diff.mass_properties().volume == pytest.approx(875.0, abs=1e-6)


def test_operands_survive_boolean(box):
    # The ABI is functional: a cut returns a NEW body, operands stay valid.
    a = box(10, 10, 10)
    b = box(5, 5, 5)
    with a.cut(b) as diff:
        assert diff.mass_properties().volume == pytest.approx(875.0, abs=1e-6)
    assert a.mass_properties().volume == pytest.approx(1000.0, abs=1e-6)
    assert b.mass_properties().volume == pytest.approx(125.0, abs=1e-6)


# ── Mass properties + bounding box (exact B-rep, not the mesh) ─────────────────


def test_mass_properties_valid_and_consistent(box):
    with box(3, 4, 5) as b:
        mp = b.mass_properties()
        assert mp.valid
        assert mp.volume == pytest.approx(60.0, abs=1e-6)
        # closed box surface area = 2(3*4 + 4*5 + 3*5) = 94
        assert mp.area == pytest.approx(94.0, abs=1e-6)
        assert mp.center_of_mass == pytest.approx((1.5, 2.0, 2.5), abs=1e-6)


def test_exact_bounding_box(box):
    with box(3, 4, 5) as b:
        bb = b.bounding_box()
        assert bb.min == pytest.approx((0, 0, 0), abs=1e-6)
        assert bb.max == pytest.approx((3, 4, 5), abs=1e-6)
        assert bb.size == pytest.approx((3, 4, 5), abs=1e-6)
        assert bb.center == pytest.approx((1.5, 2.0, 2.5), abs=1e-6)


def test_principal_moments_of_cube(box):
    # A cube's three principal moments are equal (unit-density volume inertia).
    with box(10, 10, 10) as b:
        i1, i2, i3 = b.principal_moments()
        assert i1 == pytest.approx(i2, rel=1e-6)
        assert i2 == pytest.approx(i3, rel=1e-6)
        assert i1 > 0.0


# ── Revolve → analytic cylinder volume ────────────────────────────────────────


def test_revolve_makes_a_cylinder(kernel: Kernel):
    # Rectangle with one edge on the axis of revolution; a full 2*pi revolve
    # sweeps it into a solid cylinder. Radius 2, height 10 → volume = pi r^2 h.
    rect = [(0, 0), (2, 0), (2, 10), (0, 10)]
    with kernel.revolve(rect, 2 * math.pi) as cyl:
        expected = math.pi * (2**2) * 10
        assert cyl.mass_properties().volume == pytest.approx(expected, rel=1e-3)


# ── Tessellation → non-empty NumPy mesh, watertight ───────────────────────────


def test_tessellate_returns_nonempty_numpy_mesh(box):
    with box(10, 10, 10) as b:
        mesh = b.tessellate(deflection=0.1)
        assert isinstance(mesh.vertices, np.ndarray)
        assert isinstance(mesh.triangles, np.ndarray)
        assert mesh.vertices.dtype == np.float64
        assert mesh.triangles.dtype == np.int32
        assert mesh.vertices.shape[1] == 3
        assert mesh.triangles.shape[1] == 3
        assert mesh.triangle_count > 0
        assert mesh.vertex_count > 0
        assert not mesh.is_empty()
        # A closed box's mesh vertices span exactly the box extent.
        lo, hi = mesh.bounds()
        assert lo == pytest.approx([0, 0, 0], abs=1e-6)
        assert hi == pytest.approx([10, 10, 10], abs=1e-6)
        # Sum of triangle areas equals the exact surface area of the box.
        assert mesh.surface_area() == pytest.approx(600.0, abs=1e-6)


def test_tessellation_is_watertight(box, trimesh_or_skip):
    trimesh = trimesh_or_skip
    with box(10, 10, 10) as b:
        mesh = b.tessellate(deflection=0.1)
    tm = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.triangles, process=True)
    assert tm.is_watertight, "box tessellation must be a closed surface"
    # Mesh volume of an axis-aligned box is exact regardless of triangulation.
    assert abs(tm.volume) == pytest.approx(1000.0, abs=1e-6)


def test_face_meshes_cover_six_box_faces(box):
    with box(10, 10, 10) as b:
        assert len(b.face_ids()) == 6
        assert len(b.edge_ids()) == 12
        assert len(b.vertex_ids()) == 8
        faces = b.face_meshes(0.1)
        assert len(faces) == 6
        assert all(fm.mesh.triangle_count >= 2 for fm in faces)
        # Each face mesh is tagged with a distinct 1-based face id.
        assert sorted(fm.face_id for fm in faces) == [1, 2, 3, 4, 5, 6]


# ── STEP / IGES round-trip preserves volume ───────────────────────────────────


def test_step_round_trip_preserves_volume(kernel: Kernel, box, tmp_path):
    path = os.path.join(str(tmp_path), "box.step")
    with box(10, 10, 10) as b:
        original = b.mass_properties().volume
        b.step_export(path)
    assert os.path.getsize(path) > 0
    with kernel.step_import(path) as reimported:
        assert reimported.mass_properties().volume == pytest.approx(original, rel=1e-6)


def test_iges_round_trip_preserves_volume(kernel: Kernel, box, tmp_path):
    path = os.path.join(str(tmp_path), "box.iges")
    with box(6, 6, 6) as b:
        original = b.mass_properties().volume
        b.iges_export(path)
    assert os.path.getsize(path) > 0
    with kernel.iges_import(path) as reimported:
        assert reimported.mass_properties().volume == pytest.approx(original, rel=1e-4)


# ── Feature edits + transforms ────────────────────────────────────────────────


def test_fillet_reduces_volume_of_box(box):
    with box(10, 10, 10) as b:
        edges = b.edge_ids()
        with b.fillet_edges([edges[0]], radius=1.0) as filleted:
            v = filleted.mass_properties().volume
            assert v < 1000.0  # rounding removes material
            assert v > 990.0  # a single 1mm fillet removes only a sliver


def test_transform_translate_moves_bbox(box):
    with box(2, 2, 2) as b:
        with b.translate(10, 0, 0) as moved:
            bb = moved.bounding_box()
            assert bb.min == pytest.approx((10, 0, 0), abs=1e-6)
            assert bb.max == pytest.approx((12, 2, 2), abs=1e-6)


def test_scale_cubes_volume(box):
    with box(2, 2, 2) as b:  # volume 8
        with b.scale(2.0) as big:  # linear x2 → volume x8 → 64
            assert big.mass_properties().volume == pytest.approx(64.0, rel=1e-6)
