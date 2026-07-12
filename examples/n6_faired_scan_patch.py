"""N6 — Faired scan patch (noisy fitted surface -> minimal-energy fairing).

A noisy scan grid is fitted to a NURBS surface (``nurbs.fit_surface``), then
smoothed by ``nurbs.fair_surface`` — a minimal-energy thin-plate fairing that
stays within a tolerance of the input while damping the scan noise. The gallery
image shows the BEFORE (raw fit) and AFTER (faired) patches side by side so the
smoothing is visible.

Feature family: Wave-E fairing — ``cc_nurbs_fair_surface`` (over ``cc_nurbs_fit_surface``).

Exact-NURBS API exercised:
    nurbs.fit_surface(grid, ...)               -> raw fitted Surface (noisy)
    nurbs.fair_surface(surface, tol, ...)      -> faired Surface (smoothed)
    Surface.tessellate()                       -> per-surface display Mesh

Both patches are single real surfaces; they are placed side by side purely for
the before/after comparison (an offset applied to the faired copy for display).
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report

NAME = "n6_faired_scan_patch"


def _noisy_scan_grid():
    """An n_u x n_v scan grid over a gentle underlying dome, with additive noise
    standing in for scanner measurement error (row-major, U outer)."""
    rng = np.random.default_rng(7)
    n_u, n_v = 10, 10
    span = 20.0
    grid = []
    for i in range(n_u):
        for j in range(n_v):
            x = i / (n_u - 1) * span
            y = j / (n_v - 1) * span
            base = 3.0 * np.sin(x * 0.18) * np.cos(y * 0.16)   # smooth underlying shape
            z = base + rng.normal(0.0, 0.6)                    # scan noise
            grid.append((x, y, z))
    return np.array(grid, float), n_u, n_v


def _shifted_mesh(mesh, dx):
    """Return a copy of ``mesh``'s (vertices, triangles) shifted by dx in x, so
    the faired patch renders next to the raw one for the before/after view."""
    from types import SimpleNamespace

    v = np.asarray(mesh.vertices, float).reshape(-1, 3).copy()
    v[:, 0] += dx
    return SimpleNamespace(vertices=v, triangles=np.asarray(mesh.triangles).reshape(-1, 3))


def build():
    from cybercadkernel import nurbs

    grid, n_u, n_v = _noisy_scan_grid()

    raw = nurbs.fit_surface(grid, n_u, n_v, degree_u=3, degree_v=3,
                            n_ctrl_u=7, n_ctrl_v=7, param_method=1)
    faired = nurbs.fair_surface(raw, tol=1.5, keep_boundary=True)

    raw_mesh = raw.tessellate(TESS_NV, TESS_NV)
    faired_mesh = faired.tessellate(TESS_NV, TESS_NV)

    # Place BEFORE (raw) on the left, AFTER (faired) offset to the right.
    meshes = [raw_mesh, _shifted_mesh(faired_mesh, dx=26.0)]

    raw.close()
    faired.close()

    return dict(
        meshes=meshes,
        name=NAME,
        title="Faired scan patch (before / after)",
        description=(
            "A noisy scan grid is fitted to a NURBS surface (left, raw) then smoothed "
            "by a minimal-energy thin-plate fairing within tolerance (right, faired). "
            "The two single-surface patches are shown side by side to make the "
            "noise damping visible."
        ),
        feature="cc_nurbs_fair_surface (over cc_nurbs_fit_surface)",
        api_calls=["nurbs.fit_surface", "nurbs.fair_surface", "Surface.tessellate"],
        is_single_surface=False,  # two patches shown together for before/after
    )


if __name__ == "__main__":
    run_and_report(build)
