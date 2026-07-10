"""Engine selection, STEP PMI scan, and the HONEST-DECLINE contract for the
features this MIT desktop build does not link (measurement / curvature need
``CYBERCAD_HAS_NUMSCI``; tet meshing needs ``CYBERCAD_HAS_TETGEN``).

The binding is complete either way: an unavailable feature must raise
:class:`CyberCadError` carrying the engine's honest ``cc_last_error`` message —
NEVER return a fabricated number or crash. These tests assert exactly that, so
the binding is verified against a real engine response rather than a stub.
"""

import os

import pytest

from cybercadkernel import CyberCadError, Kernel


# ── Active engine selection ───────────────────────────────────────────────────


def test_engine_default_is_occt(kernel: Kernel):
    assert kernel.engine == "occt"


def test_engine_toggle_round_trips(kernel: Kernel):
    try:
        kernel.engine = "native"
        assert kernel.engine == "native"
        kernel.engine = "occt"
        assert kernel.engine == "occt"
    finally:
        kernel.engine = "occt"


def test_engine_rejects_unknown_name(kernel: Kernel):
    with pytest.raises(ValueError):
        kernel.engine = "bogus"


# ── STEP PMI scan (native engine) ─────────────────────────────────────────────


def test_step_pmi_scan_geometry_only_step_has_no_pmi(kernel: Kernel, box, tmp_path):
    # Export a plain solid (no PMI) under OCCT, then scan it under the native
    # engine. A geometry-only STEP has zero recognised PMI entities.
    path = os.path.join(str(tmp_path), "plain.step")
    with box(10, 10, 10) as b:
        b.step_export(path)
    try:
        kernel.engine = "native"
        summary = kernel.step_pmi_scan(path)
    finally:
        kernel.engine = "occt"
    assert summary.total == 0
    assert summary.dimensions == 0
    assert summary.tolerances == 0


# ── Honest declines: measurement / curvature (no CYBERCAD_HAS_NUMSCI) ─────────


def test_measure_distance_declines_without_numsci(box):
    # This build has no numerical-science backend; the measurement service must
    # decline honestly (raise), not fabricate a distance.
    with box(10, 10, 10) as b:
        faces = b.face_ids()
        with pytest.raises(CyberCadError):
            b.measure_distance(2, faces[0], 2, faces[1])


def test_measure_angle_declines_without_numsci(box):
    with box(10, 10, 10) as b:
        faces = b.face_ids()
        with pytest.raises(CyberCadError):
            b.measure_angle(2, faces[0], 2, faces[1])


def test_surface_curvature_declines_without_numsci(box):
    with box(10, 10, 10) as b:
        with pytest.raises(CyberCadError):
            b.surface_curvature(b.face_ids()[0], 0.5, 0.5)


def test_edge_curvature_declines_without_numsci(box):
    with box(10, 10, 10) as b:
        with pytest.raises(CyberCadError):
            b.edge_curvature(b.edge_ids()[0], 0.5)


# ── Honest declines: tet meshing (no CYBERCAD_HAS_TETGEN) ─────────────────────


def test_tet_mesh_declines_without_tetgen(box):
    # The default MIT build links no AGPL TetGen: cc_tet_mesh returns an empty
    # mesh with an "unavailable" error, which the wrapper surfaces as a raise.
    with box(10, 10, 10) as b:
        with pytest.raises(CyberCadError):
            b.tet_mesh(deflection=0.5)


def test_tet_mesh_surface_declines_without_tetgen(kernel: Kernel):
    import numpy as np

    verts = np.array(
        [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64
    )
    tris = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]], dtype=np.int32)
    with pytest.raises(CyberCadError):
        kernel.tet_mesh_surface(verts, tris)
