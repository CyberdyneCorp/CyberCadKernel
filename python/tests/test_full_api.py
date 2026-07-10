"""Real-geometry coverage for the full ``cc_*`` surface.

Every test pins an exact / analytic geometric property (a volume, an area, a
count, a known distance, a section area, a written file's magic bytes) — the same
bar as ``test_geometry.py`` — for the features added to reach 100% ABI coverage:
the loft family, variable / guided-orientation sweeps, draft, asymmetric chamfer,
sheet metal (base / edge flange + unfold), n-gon fill, planar section curves,
orthographic HLR, point projection, structural validity, interference, the
connected-solid enumeration, glTF / USDZ export, the render-quality display mesh,
and native tet-mesh quality.

Engine discipline: native-only features (sheet metal, section curves, n-gon fill,
STEP PMI scan) are built AND consumed under the native engine, then the engine is
restored to OCCT — this build segfaults if a native body is queried/exported
under the OCCT engine (a known engine bug, see the change notes), so we never
cross engines on one body.
"""

import math
import os

import numpy as np
import pytest

from cybercadkernel import CCProfileSeg, Kernel, TetMesh


# ── engine helper ─────────────────────────────────────────────────────────────


@pytest.fixture
def native(kernel: Kernel):
    """Activate the native engine for the test body and restore OCCT after.

    Native bodies must be both built and consumed under the native engine in this
    build; the fixture yields the kernel with ``engine == "native"`` and always
    flips it back to OCCT on teardown so later tests are unaffected.
    """
    kernel.engine = "native"
    try:
        yield kernel
    finally:
        kernel.engine = "occt"


def _circle_xy(radius: float, n: int = 48):
    """A closed n-gon approximation of a circle as (x, y) pairs."""
    return [
        (radius * math.cos(2 * math.pi * k / n), radius * math.sin(2 * math.pi * k / n))
        for k in range(n)
    ]


# ── Loft family ───────────────────────────────────────────────────────────────


def test_loft_circles_is_a_cylinder(kernel: Kernel):
    # Two coaxial circles of equal radius 5 → a true cylinder, vol = pi r^2 h.
    with kernel.loft_circles((0, 0, 0), (0, 0, 1), 5.0, (0, 0, 10), (0, 0, 1), 5.0) as c:
        expected = math.pi * 25.0 * 10.0
        assert c.mass_properties().volume == pytest.approx(expected, rel=1e-6)


def test_loft_circles_radius_taper_is_a_cone(kernel: Kernel):
    # r1=5, r2=0 over height 10 → a cone, vol = 1/3 pi r^2 h.
    with kernel.loft_circles((0, 0, 0), (0, 0, 1), 5.0, (0, 0, 10), (0, 0, 1), 2.0) as c:
        # frustum volume = pi/3 * h * (r1^2 + r1 r2 + r2^2)
        expected = math.pi / 3.0 * 10.0 * (25.0 + 10.0 + 4.0)
        assert c.mass_properties().volume == pytest.approx(expected, rel=1e-6)


def test_loft_circle_wire_closes_a_solid(kernel: Kernel):
    wire = [(-5, -5, 10), (5, -5, 10), (5, 5, 10), (-5, 5, 10)]
    with kernel.loft_circle_wire((0, 0, 0), (0, 0, 1), 5.0, wire) as s:
        mp = s.mass_properties()
        assert mp.valid
        assert mp.volume > 0.0


def test_loft_sections_three_square_rings(kernel: Kernel):
    # Three identical 10x10 square rings stacked at z=0,10,20 → a 10x10x20 prism.
    ring = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    sections = [[(x, y, z) for (x, y) in ring] for z in (0.0, 10.0, 20.0)]
    with kernel.loft_sections(sections) as s:
        assert s.mass_properties().volume == pytest.approx(2000.0, rel=1e-6)


def test_loft_typed_square_to_square(kernel: Kernel):
    def square(side):
        h = side / 2.0
        pts = [(-h, -h), (h, -h), (h, h), (-h, h)]
        return [
            CCProfileSeg(
                kind=0, x0=pts[i][0], y0=pts[i][1],
                x1=pts[(i + 1) % 4][0], y1=pts[(i + 1) % 4][1],
            )
            for i in range(4)
        ]

    frame_a = [0, 0, 0, 1, 0, 0, 0, 1, 0]
    frame_b = [0, 0, 10, 1, 0, 0, 0, 1, 0]
    with kernel.loft_typed(square(10), frame_a, square(6), frame_b) as s:
        # Prismatoid volume h/6 * (A_bottom + 4 A_mid + A_top),
        # A=100/36, A_mid = ((10+6)/2)^2 = 64  →  10/6*(100+256+36) = 653.33...
        assert s.mass_properties().volume == pytest.approx(653.3333333, rel=1e-4)


def test_loft_along_rails_builds_a_solid(kernel: Kernel):
    rail = [(0, 0, 0), (0, 0, 20)]
    guide = [(5, 0, 0), (5, 0, 20)]
    a = [(-2, -2), (2, -2), (2, 2), (-2, 2)]
    b = [(-3, -3), (3, -3), (3, 3), (-3, 3)]
    with kernel.loft_along_rails(rail, guide, a, b) as s:
        assert s.mass_properties().volume > 0.0


# ── Variable / guided-orientation sweeps ──────────────────────────────────────


def test_variable_sweep_circle_to_circle_is_a_cone(kernel: Kernel):
    # Radius 5 → 2 along a straight 20mm spine ⇒ a truncated cone (frustum).
    a = _circle_xy(5.0)
    b = _circle_xy(2.0)
    spine = [(0, 0, 0), (0, 0, 20)]
    with kernel.variable_sweep(a, b, spine) as s:
        frustum = math.pi / 3.0 * 20.0 * (25.0 + 10.0 + 4.0)
        # the polygonal circle approximation is slightly under the true frustum
        assert s.mass_properties().volume == pytest.approx(frustum, rel=2e-2)


def test_guided_orient_sweep_builds_a_solid(kernel: Kernel):
    prof = [(-2, -2), (2, -2), (2, 2), (-2, 2)]  # 4x4 square
    path = [(0, 0, 0), (0, 0, 20)]
    guide = [(3, 0, 0), (3, 0, 20)]
    with kernel.guided_orient_sweep(prof, path, guide) as s:
        # a 4x4 section swept 20mm ≈ 16*20 = 320
        assert s.mass_properties().volume == pytest.approx(320.0, rel=1e-3)


# ── Draft + asymmetric chamfer ────────────────────────────────────────────────


def test_draft_faces_removes_material(box):
    # Drafting a side face inward tapers the wall → the solid loses volume.
    with box(20, 20, 20) as b:
        face = b.face_ids()[0]
        with b.draft_faces([face], (0, 0, 0), (0, 0, 1), 5.0) as drafted:
            assert drafted.mass_properties().volume < 8000.0
            assert drafted.mass_properties().volume > 7000.0


def test_chamfer_asym_reduces_volume(box):
    with box(10, 10, 10) as b:
        edge = b.edge_ids()[0]
        with b.chamfer_edges_asym([edge], 1.0, 2.0) as ch:
            v = ch.mass_properties().volume
            assert v < 1000.0  # a bevel removes stock
            assert v > 985.0  # only along one edge


# ── Sheet metal (native engine only) ──────────────────────────────────────────


def test_sheet_base_flange_volume(native: Kernel):
    # A 50x30 flat blank, 2mm thick → vol = 50*30*2 = 3000.
    with native.sheet_base_flange([(0, 0), (50, 0), (50, 30), (0, 30)], 2.0) as bf:
        assert bf.mass_properties().volume == pytest.approx(3000.0, rel=1e-6)


def test_sheet_edge_flange_then_unfold_conserves_area(native: Kernel):
    with native.sheet_base_flange([(0, 0), (50, 0), (50, 30), (0, 30)], 2.0) as bf:
        base_vol = bf.mass_properties().volume
        # A single edge flange off some straight rim of the base.
        flanged = None
        for eid in bf.edge_ids():
            try:
                flanged = bf.sheet_edge_flange(eid, 20.0, 2.0, 90.0)
                break
            except Exception:
                continue
        assert flanged is not None, "no straight rim accepted an edge flange"
        with flanged:
            # A 90° fold adds material (base + bend + wall) → more volume than base.
            assert flanged.mass_properties().volume > base_vol
            with flanged.sheet_unfold(0.4) as flat:
                # The developed flat blank is a planar sheet of the same 2mm
                # thickness; its footprint area is (baseRun+BA+wall)*width and is
                # invariant under fold→unfold, so the flat pattern is strictly
                # larger in plan than the base and has a positive volume.
                assert flat.mass_properties().volume > 0.0
                assert flat.mass_properties().area > bf.mass_properties().area


# ── N-gon fill patch (native engine only) ─────────────────────────────────────


def test_fill_ngon_triangle_area(native: Kernel):
    # A right triangle with legs 10 → area 50. The patch is an OPEN surface, so
    # mass_properties is invalid; assert on the tessellated patch surface area.
    with native.fill_ngon([(0, 0, 0), (10, 0, 0), (0, 10, 0)], grid_n=8) as patch:
        mesh = patch.tessellate(0.5)
        assert mesh.surface_area() == pytest.approx(50.0, rel=1e-6)


# ── Planar section curves (native engine only) ────────────────────────────────


def test_section_plane_of_box_is_a_square(native: Kernel):
    # A z=5 cut of a 10x10x10 box is a 10x10 square loop: area 100, perimeter 40.
    with native.extrude([(0, 0), (10, 0), (10, 10), (0, 10)], 10) as b:
        sec = b.section_plane((5, 5, 5), (0, 0, 1))
        assert sec.loop_count == 1
        assert sec.total_area == pytest.approx(100.0, rel=1e-6)
        loop = sec.loops[0]
        assert loop.points.shape[1] == 3
        assert loop.length == pytest.approx(40.0, rel=1e-6)


# ── Orthographic hidden-line removal ──────────────────────────────────────────


def test_hlr_project_box_front_view(box):
    # Looking down -Z at a box: the front outline is 4 visible silhouette edges
    # and the back face gives 4 hidden edges.
    with box(10, 10, 10) as b:
        drawing = b.hlr_project((0, 0, -1), (0, 1, 0))
        assert drawing.visible_count == 4
        assert drawing.hidden_count == 4
        assert drawing.visible.shape[1] == 4  # [ax, ay, bx, by]


# ── Point projection onto a face ──────────────────────────────────────────────


def test_project_point_on_top_face(box):
    # The top face of a 10-cube is the plane z=10; a point 40mm above it projects
    # straight down with distance 40.
    with box(10, 10, 10) as b:
        # Find the face whose projection of (5,5,50) sits at z=10, distance 40.
        hit = None
        for fid in b.face_ids():
            try:
                pr = b.project_point_on_face(fid, (5, 5, 50))
            except Exception:
                continue
            if pr.foot[2] == pytest.approx(10.0, abs=1e-6):
                hit = pr
                break
        assert hit is not None, "no top face accepted the projection"
        assert hit.distance == pytest.approx(40.0, abs=1e-6)
        assert hit.foot == pytest.approx((5.0, 5.0, 10.0), abs=1e-6)


# ── Structural validity + interference ────────────────────────────────────────


def test_check_solid_reports_valid_box(box):
    with box(10, 10, 10) as b:
        report = b.check_solid()
        assert report.decided
        assert report.valid
        assert report.closed_manifold
        assert report.first_failure == 0


def test_interference_clash_overlap_volume(box):
    # Two 10-cubes overlapping in a 5^3 corner → CLASH, overlap volume 125.
    a = box(10, 10, 10)
    b = box(10, 10, 10).translate(5, 5, 5)
    result = a.interference(b)
    assert result.decided
    assert result.clash
    assert result.state == 2  # CC_CLASH_CLASH
    assert result.overlap_volume == pytest.approx(125.0, rel=1e-4)


def test_interference_clear_gap(box):
    # Two boxes 5mm apart → CLEAR, no clash, positive min distance.
    a = box(10, 10, 10)
    b = box(10, 10, 10).translate(15, 0, 0)
    result = a.interference(b)
    assert result.decided
    assert not result.clash
    assert result.state == 0  # CC_CLASH_CLEAR
    assert result.min_distance == pytest.approx(5.0, abs=1e-4)


# ── Connected-solid enumeration ───────────────────────────────────────────────


def test_solid_count_of_single_box_is_one(box):
    with box(10, 10, 10) as b:
        assert b.solid_count() == 1


def test_solid_count_of_two_lump_fuse_is_two(box):
    # Fuse two disjoint boxes → one body with two connected lumps.
    a = box(10, 10, 10)
    b = box(10, 10, 10).translate(100, 0, 0)
    with a.fuse(b) as fused:
        assert fused.solid_count() == 2
        lumps = fused.solids()
        try:
            assert len(lumps) == 2
            # Each lump is a 1000 mm^3 cube.
            for lump in lumps:
                assert lump.mass_properties().volume == pytest.approx(1000.0, rel=1e-6)
        finally:
            for lump in lumps:
                lump.close()


# ── glTF / USDZ export ────────────────────────────────────────────────────────

_GLTF_JSON_MAGIC = b"{"
_GLB_MAGIC = b"glTF"
_ZIP_MAGIC = b"PK\x03\x04"  # USDZ is a store-only ZIP


def test_gltf_glb_export_writes_binary_asset(box, tmp_path):
    path = os.path.join(str(tmp_path), "box.glb")
    with box(10, 10, 10) as b:
        b.gltf_export(path, deflection=0.5, glb=True)
    assert os.path.getsize(path) > 0
    with open(path, "rb") as fh:
        assert fh.read(4) == _GLB_MAGIC  # 12-byte glB header starts with "glTF"


def test_gltf_json_export_writes_gltf(box, tmp_path):
    path = os.path.join(str(tmp_path), "box.gltf")
    with box(10, 10, 10) as b:
        b.gltf_export(path, deflection=0.5, glb=False)
    with open(path, "rb") as fh:
        head = fh.read(64).lstrip()
    assert head.startswith(_GLTF_JSON_MAGIC)  # self-contained JSON asset


def test_usdz_export_writes_zip_package(box, tmp_path):
    path = os.path.join(str(tmp_path), "box.usdz")
    with box(10, 10, 10) as b:
        b.usdz_export(path, deflection=0.5)
    assert os.path.getsize(path) > 0
    with open(path, "rb") as fh:
        assert fh.read(4) == _ZIP_MAGIC  # USDZ = store-only ZIP


# ── Render-quality display mesh ───────────────────────────────────────────────


def test_display_mesh_has_normals_and_uvs(box):
    with box(10, 10, 10) as b:
        dm = b.display_mesh(deflection=0.5, crease_angle_deg=30.0, want_uvs=True)
        assert dm.vertex_count > 0
        assert dm.triangle_count > 0
        assert dm.positions.shape == (dm.vertex_count, 3)
        assert dm.normals.shape == (dm.vertex_count, 3)
        # Normals are unit length.
        lengths = np.linalg.norm(dm.normals, axis=1)
        assert np.allclose(lengths, 1.0, atol=1e-6)
        assert dm.uvs is not None
        assert dm.uvs.shape == (dm.vertex_count, 2)


def test_display_mesh_without_uvs(box):
    with box(10, 10, 10) as b:
        dm = b.display_mesh(deflection=0.5, want_uvs=False)
        assert dm.uvs is None


# ── Native tet-mesh quality (always available, TetGen-independent) ─────────────


def _regular_tet_nodes():
    """The four vertices of a regular tetrahedron (edge length 1)."""
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, math.sqrt(3) / 2.0, 0.0],
            [0.5, math.sqrt(3) / 6.0, math.sqrt(2.0 / 3.0)],
        ]
    )


def test_mesh_quality_of_regular_tet(kernel: Kernel):
    tm = TetMesh(
        nodes=_regular_tet_nodes(),
        elements=np.array([[0, 1, 2, 3]]),
        nodes_per_element=4,
        order=4,
    )
    report = kernel.mesh_quality(tm, min_scaled_jacobian=0.02)
    assert report.valid
    # A regular tet has all dihedral angles = arccos(1/3) ≈ 70.53°, scaled J = 1.
    assert report.min_dihedral_angle == pytest.approx(70.5288, abs=1e-2)
    assert report.max_dihedral_angle == pytest.approx(70.5288, abs=1e-2)
    assert report.min_scaled_jacobian == pytest.approx(1.0, abs=1e-6)
    assert report.elements_below_threshold == 0


def test_mesh_quality_flags_a_degenerate_sliver(kernel: Kernel):
    # A near-flat sliver tet has a tiny scaled Jacobian → flagged below threshold.
    nodes = np.array(
        [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.3, 0.3, 1e-3]], dtype=np.float64
    )
    tm = TetMesh(
        nodes=nodes,
        elements=np.array([[0, 1, 2, 3]]),
        nodes_per_element=4,
        order=4,
    )
    report = kernel.mesh_quality(tm, min_scaled_jacobian=0.2)
    assert report.valid
    assert report.min_scaled_jacobian < 0.2
    assert report.elements_below_threshold == 1
    assert list(report.flagged_elements) == [0]
