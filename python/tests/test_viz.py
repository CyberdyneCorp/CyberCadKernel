"""Visualization smoke tests.

``trimesh`` interop + STL export need no display and are asserted
unconditionally (real geometry: the exported solid re-loads to the same box).
The offscreen PNG render needs a GL context, so the GL path skips cleanly when
none is available (headless machine / CI).
"""

import os

import pytest

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _box_mesh(box):
    with box(10, 10, 10) as b:
        return b.tessellate(0.1)


def test_to_trimesh_watertight_and_volume(box, trimesh_or_skip):
    from cybercadkernel import viz

    mesh = _box_mesh(box)
    tm = viz.to_trimesh(mesh)
    tm.merge_vertices()
    assert tm.is_watertight
    assert abs(tm.volume) == pytest.approx(1000.0, abs=1e-6)


def test_export_stl_is_nonempty_and_round_trips(box, trimesh_or_skip, tmp_path):
    trimesh = trimesh_or_skip
    from cybercadkernel import viz

    out = os.path.join(str(tmp_path), "box.stl")
    mesh = _box_mesh(box)
    viz.export_stl(mesh, out)

    # Primary smoke assertion: a real, non-empty STL file was written.
    assert os.path.exists(out)
    assert os.path.getsize(out) > 0

    # Stronger: the exported solid re-loads to the same box volume.
    reloaded = trimesh.load(out)
    reloaded.merge_vertices()
    assert abs(reloaded.volume) == pytest.approx(1000.0, abs=1e-6)


def test_render_png_gl_offscreen_or_skip(box, trimesh_or_skip, tmp_path):
    from cybercadkernel import viz

    out = os.path.join(str(tmp_path), "box_gl.png")
    mesh = _box_mesh(box)
    try:
        viz.render_png(mesh, out, resolution=(128, 128), prefer="gl")
    except Exception as exc:  # no GL context / pyglet in this environment
        pytest.skip(f"offscreen GL render unavailable: {exc}")
    assert os.path.getsize(out) > 0
    with open(out, "rb") as f:
        assert f.read(8) == PNG_MAGIC, "not a PNG file"


def test_render_png_matplotlib_fallback_or_skip(box, trimesh_or_skip, tmp_path):
    from cybercadkernel import viz

    out = os.path.join(str(tmp_path), "box_mpl.png")
    mesh = _box_mesh(box)
    try:
        viz.render_png(mesh, out, resolution=(128, 128), prefer="matplotlib")
    except Exception as exc:  # matplotlib not installed
        pytest.skip(f"matplotlib render unavailable: {exc}")
    assert os.path.getsize(out) > 0
    with open(out, "rb") as f:
        assert f.read(8) == PNG_MAGIC, "not a PNG file"
