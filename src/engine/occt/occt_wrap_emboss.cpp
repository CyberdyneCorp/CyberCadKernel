// OCCT engine adapter — WRAP-EMBOSS feature (Phase-3 owned TU).
//
// Defines OcctEngine::wrap_emboss (declared in occt_engine.h). It wraps a 2D
// profile onto a cylindrical face and pads it radially to add (boss) / remove
// (deboss) material.
//
// Phase-3 robustness rework
// --------------------------
// The baseline built the pad with BRepOffsetAPI_ThruSections (solid, ruled). The
// side loft between the two wrapped section wires is fine, but the wrapped section
// wires are NOT planar (they live on the cylinder), so the solid-mode ThruSections
// planar CAP construction fails / produces a non-valid shape on a dense,
// high-curvature profile — the emboss then either fails or drops to a coarse
// polygon, losing fidelity.
//
// The robust path builds the pad's boundary explicitly and sews it:
//   * inner cap  — an exact trimmed CYLINDRICAL face at radius rIn, bounded by the
//                  wrapped profile drawn in the surface's own (u,v) space (so its
//                  boundary edges are exact helical arcs on that cylinder);
//   * outer cap  — the same at radius rOut;
//   * side walls — one ruled face per profile edge (BRepFill::Face) between the
//                  inner arc edge and the outer arc edge, its radial sides being
//                  the exact straight segments I_i->O_i shared with the neighbours.
// Because every shared boundary is geometrically EXACT (arcs shared with the caps,
// straight radials shared between side walls), BRepBuilderAPI_Sewing stitches them
// at a tiny tolerance into a closed shell, which ShapeFix_Solid orients into a
// watertight solid. Gate on BRepCheck_Analyzer::IsValid.
//
// Honesty / never-regress: if the sewn solid is null or not valid, fall back to the
// baseline dense-then-coarse ThruSections pad (today's behaviour); if that too
// fails, return an error. A small radial overlap makes the pad cross the body
// surface so the fuse/cut is a clean (non-coincident-face) boolean.
//
// OCCT-only TU: every OpenCASCADE include lives here; no OCCT type escapes into a
// public/shared header. Compiled only for iOS (CYBERCAD_HAS_OCCT); the host build
// omits it and the stub's wrap_emboss default returns 0.

#include "engine/occt/occt_engine.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

// ── Wrap-emboss OCCT builders (this TU only) ──────────────────────────────────
#include <gp_Ax3.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Cylinder.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepLib.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Solid.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

namespace cyber {

namespace {

// A wrapped profile point in the cylinder's own parameter space: u = angle
// (arc-length / R), v = axial coordinate (already offset by the face's V-mid).
struct UV {
    double u;
    double v;
};

// Densify each profile edge by its arc (theta) span so the wrapped boundary FOLLOWS
// the cylinder instead of chording across it (matches the baseline). Axial edges
// (dTheta≈0) stay single-segment. Capped at NMAX to bound the vertex/face count.
std::vector<UV> densifyUV(const double* profileXY, int count, double R, double vMid) {
    const double chordTol = 0.5;  // mm
    const double dThetaMax =
        (R > chordTol) ? 2.0 * std::acos(std::max(-1.0, 1.0 - chordTol / R)) : 0.3;
    const int NMAX = 64;
    std::vector<UV> dense;
    for (int i = 0; i < count; ++i) {
        const double ax = profileXY[i * 2], ay = profileXY[i * 2 + 1];
        const double bx = profileXY[((i + 1) % count) * 2],
                     by = profileXY[((i + 1) % count) * 2 + 1];
        const double dTheta = std::fabs(bx - ax) / R;
        const int n = std::max(1, std::min(NMAX, static_cast<int>(std::ceil(dTheta / dThetaMax))));
        for (int k = 0; k < n; ++k) {
            const double t = static_cast<double>(k) / n;
            const double px = ax + (bx - ax) * t, py = ay + (by - ay) * t;
            dense.push_back({px / R, py + vMid});
        }
    }
    return dense;
}

// Build the exact trimmed cylindrical cap face at radius r, bounded by the wrapped
// profile drawn in the surface's own (u,v) space. Its boundary edges are helical
// arcs ON that cylinder; the closed boundary wire is returned in `outWire` so the
// side-wall loft can reuse the identical arcs for a watertight sew. Returns a null
// face on failure.
TopoDS_Face buildCap(const gp_Ax3& axes, double r, const std::vector<UV>& uv,
                     TopoDS_Wire& outWire) {
    Handle(Geom_CylindricalSurface) surf = new Geom_CylindricalSurface(axes, r);
    BRepBuilderAPI_MakeWire wireMaker;
    const int n = static_cast<int>(uv.size());
    for (int i = 0; i < n; ++i) {
        const UV& a = uv[i];
        const UV& b = uv[(i + 1) % n];
        const gp_Pnt2d p0(a.u, a.v), p1(b.u, b.v);
        Handle(Geom2d_TrimmedCurve) seg = GCE2d_MakeSegment(p0, p1);
        if (seg.IsNull()) {
            return TopoDS_Face();
        }
        BRepBuilderAPI_MakeEdge mkEdge(seg, surf);
        if (!mkEdge.IsDone()) {
            return TopoDS_Face();
        }
        wireMaker.Add(mkEdge.Edge());
    }
    if (!wireMaker.IsDone()) {
        return TopoDS_Face();
    }
    outWire = wireMaker.Wire();
    BRepLib::BuildCurves3d(outWire);  // give the pcurve edges their 3D representation
    BRepBuilderAPI_MakeFace mkFace(surf, outWire, Standard_True);
    if (!mkFace.IsDone()) {
        return TopoDS_Face();
    }
    return mkFace.Face();
}

}  // namespace

// Irreducible geometry (cylinder wrap + explicit cap/side build + sew + heal, with
// a ThruSections fallback) — systems band.
ShapeResult OcctEngine::wrap_emboss(EngineShape body, int faceId, const double* profileXY,
                                    int count, double depth, int boss) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* bodyShape = occt::unwrap(body);
        if (bodyShape == nullptr || profileXY == nullptr || count < 3 || depth <= 0) {
            return make_error("wrap_emboss: invalid body/profile input");
        }
        const TopoDS_Shape shape = *bodyShape;
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(shape, ids, 1);
        if (faces.empty()) {
            return make_error("wrap_emboss: face id not found");
        }
        BRepAdaptor_Surface surf(faces.front());
        if (surf.GetType() != GeomAbs_Cylinder) {
            return make_error("wrap_emboss: face is not cylindrical");
        }
        const gp_Cylinder cyl = surf.Cylinder();
        const double R = cyl.Radius();
        if (R < 1e-6) {
            return make_error("wrap_emboss: degenerate cylinder radius");
        }
        const gp_Ax3 pos = cyl.Position();
        // Centre the profile on the face's own axial (V) middle, so a profile passed
        // centred at py = 0 lands mid-face regardless of where V = 0 sits.
        Standard_Real umin, umax, vmin, vmax;
        BRepTools::UVBounds(faces.front(), umin, umax, vmin, vmax);
        const double vMid = (vmin + vmax) / 2;

        // A tiny radial overlap so the pad crosses the body's cylindrical surface:
        // the fuse (boss) / cut (deboss) is then a clean, non-coincident-face
        // boolean. Kept << depth so the body's volume delta stays ~ profile*depth.
        const double overlap = std::max(1.0e-3, depth * 0.02);
        const double rIn = boss ? (R - overlap) : (R - depth);
        const double rOut = boss ? (R + depth) : (R + overlap);
        if (rIn <= 1e-6) {
            return make_error("wrap_emboss: emboss depth exceeds radius");
        }

        // Legacy wrap (arc-length px, axial py) -> gp_Pnt at radius r, for the
        // ThruSections fallback (kept identical to the baseline).
        const gp_Vec axisO(pos.Location().XYZ()), axisD(pos.Direction()), xd(pos.XDirection()),
            yd(pos.YDirection());
        auto wrap = [&](double px, double py, double r) -> gp_Pnt {
            const double th = px / R;
            const gp_Vec radial = xd.Multiplied(std::cos(th)).Added(yd.Multiplied(std::sin(th)));
            const gp_Vec p = axisO.Added(axisD.Multiplied(py + vMid)).Added(radial.Multiplied(r));
            return gp_Pnt(p.X(), p.Y(), p.Z());
        };

        // ── Robust sewn pad: explicit caps + ruled side walls, sewn + healed. ──
        const std::vector<UV> uv = densifyUV(profileXY, count, R, vMid);
        auto buildSewnPad = [&]() -> TopoDS_Shape {
            TopoDS_Wire wireIn, wireOut;
            const TopoDS_Face capIn = buildCap(pos, rIn, uv, wireIn);
            const TopoDS_Face capOut = buildCap(pos, rOut, uv, wireOut);
            if (capIn.IsNull() || capOut.IsNull()) {
                return TopoDS_Shape();
            }
            // Side wall: one ruled shell (NOT solid — no planar cap, which is exactly
            // what defeats ThruSections here) lofted between the two cap boundary
            // wires. Its top/bottom boundaries are the same helical arcs as the caps,
            // so the sew is watertight.
            BRepOffsetAPI_ThruSections side(Standard_False /*shell*/, Standard_True /*ruled*/);
            side.AddWire(wireIn);
            side.AddWire(wireOut);
            side.Build();
            if (!side.IsDone() || side.Shape().IsNull()) {
                return TopoDS_Shape();
            }
            BRepBuilderAPI_Sewing sew(1.0e-4);  // exact shared boundaries -> tiny tol
            sew.Add(capIn);
            sew.Add(capOut);
            sew.Add(side.Shape());
            sew.Perform();
            const TopoDS_Shape sewed = sew.SewedShape();
            if (sewed.IsNull()) {
                return TopoDS_Shape();
            }
            // Pull out a shell (the sewed result is a shell or a compound of one).
            TopoDS_Shell shell;
            if (sewed.ShapeType() == TopAbs_SHELL) {
                shell = TopoDS::Shell(sewed);
            } else {
                TopExp_Explorer ex(sewed, TopAbs_SHELL);
                if (ex.More()) {
                    shell = TopoDS::Shell(ex.Current());
                }
            }
            if (shell.IsNull()) {
                return TopoDS_Shape();
            }
            ShapeFix_Shell fixShell(shell);
            fixShell.Perform();
            shell = fixShell.Shell();
            ShapeFix_Solid fixSolid;
            const TopoDS_Solid solid = fixSolid.SolidFromShell(shell);
            if (solid.IsNull() || !BRepCheck_Analyzer(solid).IsValid()) {
                return TopoDS_Shape();
            }
            return TopoDS_Shape(solid);
        };

        // ── Baseline ThruSections pad (coarse fallback; never regress). ──
        auto buildWire = [&](const std::vector<std::pair<double, double>>& pts,
                             double r) -> TopoDS_Wire {
            BRepBuilderAPI_MakePolygon poly;
            for (const auto& p : pts) {
                poly.Add(wrap(p.first, p.second, r));
            }
            poly.Close();
            return poly.Wire();
        };
        auto tryThruSections = [&](const std::vector<std::pair<double, double>>& pts)
            -> TopoDS_Shape {
            BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
            gen.AddWire(buildWire(pts, rIn));
            gen.AddWire(buildWire(pts, rOut));
            gen.Build();
            if (!gen.IsDone()) {
                return TopoDS_Shape();
            }
            const TopoDS_Shape s = gen.Shape();
            if (s.IsNull() || !BRepCheck_Analyzer(s).IsValid()) {
                return TopoDS_Shape();
            }
            return s;
        };

        TopoDS_Shape pad = buildSewnPad();
        if (pad.IsNull()) {  // robust sew failed -> baseline dense-then-coarse loft
            std::vector<std::pair<double, double>> dense;
            for (const auto& p : uv) {
                dense.push_back({p.u * R, p.v - vMid});
            }
            pad = tryThruSections(dense);
        }
        if (pad.IsNull()) {
            std::vector<std::pair<double, double>> coarse;
            for (int i = 0; i < count; ++i) {
                coarse.push_back({profileXY[i * 2], profileXY[i * 2 + 1]});
            }
            pad = tryThruSections(coarse);
        }
        if (pad.IsNull()) {
            return make_error("wrap_emboss: could not build the wrapped pad");
        }
        const TopoDS_Shape result = boss ? TopoDS_Shape(BRepAlgoAPI_Fuse(shape, pad))
                                         : TopoDS_Shape(BRepAlgoAPI_Cut(shape, pad));
        return occt::addIfValid(result, "wrap_emboss: invalid boolean result");
    });
}

}  // namespace cyber
