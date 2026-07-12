"""N5 — Reverse-engineered primitive (point cloud -> detected cylinder).

A noisy point cloud is sampled off a known cylinder, then
``nurbs.detect_primitive`` recovers the analytic primitive (type, axis, radius,
fit RMS). From the recovered parameters we rebuild the EXACT rational cylinder as
a NURBS surface and render it — closing the reverse-engineering loop from scan
points back to exact geometry.

Feature family: Wave-E reverse-engineering — ``cc_nurbs_detect_primitive``
(with the exact analytic ``cc_nurbs_cylinder`` conversion).

Exact-NURBS API exercised:
    nurbs.detect_primitive(points, rel_tol)    -> PrimitiveDetection (type/axis/radius)
    nurbs.cylinder(origin, axis, x, r, v0, v1) -> exact rational Surface
    Surface.tessellate()                       -> single-surface display Mesh
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report
from cybercadkernel.nurbs import PrimitiveType

NAME = "n5_reverse_engineered_primitive"

# The ground-truth cylinder we sample points off of (mm).
TRUE_RADIUS = 8.0
TRUE_HEIGHT = 30.0
NOISE_SIGMA = 0.05  # small radial/positional scan noise


def _sample_cylinder_cloud():
    """Sample a noisy point cloud off the ground-truth cylinder about the z-axis."""
    rng = np.random.default_rng(42)
    thetas = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False)
    zs = np.linspace(0.0, TRUE_HEIGHT, 20)
    pts = []
    for z in zs:
        for t in thetas:
            r = TRUE_RADIUS + rng.normal(0.0, NOISE_SIGMA)
            pts.append((r * np.cos(t), r * np.sin(t), z + rng.normal(0.0, NOISE_SIGMA)))
    return np.array(pts, float)


def build():
    from cybercadkernel import nurbs

    cloud = _sample_cylinder_cloud()

    det = nurbs.detect_primitive(cloud, rel_tol=0.02)
    if det.type != PrimitiveType.CYLINDER:
        raise RuntimeError(f"expected CYLINDER, detected type {det.type}")

    # Rebuild the exact rational cylinder from the recovered parameters. The
    # detected axis may point either way; build an orthonormal x-axis for it.
    axis = np.array(det.axis, float)
    axis = axis / (np.linalg.norm(axis) or 1.0)
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(axis, ref)
    x_axis = x_axis / (np.linalg.norm(x_axis) or 1.0)

    surface = nurbs.cylinder(det.center, axis, x_axis, det.radius, 0.0, TRUE_HEIGHT)
    mesh = surface.tessellate(TESS_NU, TESS_NV)
    surface.close()

    return dict(
        meshes=[mesh],
        name=NAME,
        title="Reverse-engineered cylinder (from a point cloud)",
        description=(
            f"A noisy point cloud sampled off a radius-{TRUE_RADIUS:.0f} cylinder is "
            f"run through detect_primitive, which recovers CYLINDER (radius "
            f"{det.radius:.3f}, RMS {det.rms:.4f}); the recovered parameters rebuild "
            "the EXACT rational cylinder shown here."
        ),
        feature="cc_nurbs_detect_primitive (-> exact cc_nurbs_cylinder)",
        api_calls=["nurbs.detect_primitive", "nurbs.cylinder", "Surface.tessellate"],
        is_single_surface=True,
    )


if __name__ == "__main__":
    run_and_report(build)
