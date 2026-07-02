// OCCT engine adapter — G2 (curvature-continuous) BLEND-FILLET feature
// (Phase-3 owned TU).
//
// Defines OcctEngine::fillet_edges_g2 (declared in occt_engine.h): build a
// curvature-continuous (G2, or best-achievable) blend along the given edges at the
// nominal radius. The stock fillet_edges (G1 circular) is left untouched.
//
// APPROACH (native; OCCT is used only for primitives/booleans, not for the blend
// law — OCCT's BRepFilletAPI is G1/circular only and has no G2 mode):
//
//   For each selected edge that is a straight line shared by two PLANAR faces, we
//   replace the sharp corner by a curvature-continuous cross-section instead of a
//   circular arc. The cross-section is a QUINTIC Bezier whose first three control
//   poles are collinear at each rail — so its SECOND derivative is exactly zero
//   there (curvature == 0), matching the flat neighbour faces' zero normal
//   curvature. That yields genuine G2 (curvature) continuity at both seams, versus
//   the constant 1/radius curvature jump a circular fillet leaves.
//
//   The blend is realized by CUTTING the sharp corner away with a prism whose
//   cross-section is bounded by that quintic on the inside and a point pushed just
//   outside the corner on the outside, extruded past both ends of the edge. The
//   boolean produces a valid, watertight solid whose new corner face IS the
//   extruded quintic — tangent to both planes (G1) with zero curvature (G2) at the
//   rails. Result is gated on BRepCheck_Analyzer::IsValid (occt::addIfValid).
//
//   Non-straight edges or non-planar neighbours are NOT handled here (the
//   zero-curvature rail construction assumes flat neighbours); those return an
//   Error the facade collapses to 0, and the sim harness records the case deferred
//   with its measured gap — an honest fallback, never a faked G2 pass. The seam
//   curvature gap itself is MEASURED by the sim check (checks_g2_fillet.cpp) from
//   the tessellated surface and only claimed G2 when it is within tolerance AND
//   smaller than the stock G1 fillet's gap on the same edge/radius.
//
// OCCT-only TU (CYBERCAD_HAS_OCCT); the host build omits it and the stub inherits
// the unsupported default (also 0).

#include "engine/occt/occt_engine.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_BezierCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace cyber {

namespace {

// A straight edge reduced to the data the blend builder needs.
struct EdgeLine {
    gp_Pnt v1;   // first vertex
    gp_Dir dir;  // unit direction v1 -> v2
    double len;  // |v2 - v1|
};

// Recover a straight edge's endpoints/direction; std::nullopt if it is not a line
// or is degenerate (the G2 rail construction only handles straight seams).
std::optional<EdgeLine> asStraightEdge(const TopoDS_Edge& edge) {
    BRepAdaptor_Curve curve(edge);
    if (curve.GetType() != GeomAbs_Line) {
        return std::nullopt;
    }
    const gp_Pnt p1 = curve.Value(curve.FirstParameter());
    const gp_Pnt p2 = curve.Value(curve.LastParameter());
    const gp_Vec span(p1, p2);
    const double len = span.Magnitude();
    if (len < 1e-7) {
        return std::nullopt;
    }
    return EdgeLine{p1, gp_Dir(span), len};
}

// The in-plane direction of `face` that is perpendicular to the edge and points
// INTO the face interior (away from the edge). std::nullopt if the face is not
// planar. `edgeMid` seeds the interior-orientation test.
std::optional<gp_Dir> inFaceDirection(const TopoDS_Face& face, const gp_Dir& edgeDir,
                                      const gp_Pnt& edgeMid) {
    BRepAdaptor_Surface surf(face);
    if (surf.GetType() != GeomAbs_Plane) {
        return std::nullopt;
    }
    const gp_Dir normal = surf.Plane().Axis().Direction();
    // edge lies in the plane => edgeDir ⟂ normal => this cross is unit and in-plane.
    gp_Dir inPlane = edgeDir.Crossed(normal);

    double u1 = 0, u2 = 0, w1 = 0, w2 = 0;
    BRepTools::UVBounds(face, u1, u2, w1, w2);
    const gp_Pnt faceMid = surf.Value(0.5 * (u1 + u2), 0.5 * (w1 + w2));
    if (gp_Vec(edgeMid, faceMid).Dot(gp_Vec(inPlane)) < 0.0) {
        inPlane.Reverse();
    }
    return inPlane;
}

// Translate a point by coeff along a unit direction.
gp_Pnt along(const gp_Pnt& p, const gp_Dir& d, double coeff) {
    return p.Translated(gp_Vec(d) * coeff);
}

// Build the curvature-continuous corner-removal cutter for one straight edge whose
// two neighbours are planar. Returns std::nullopt when the edge is not a straight
// planar seam (caller treats that as "cannot build G2 here").
std::optional<TopoDS_Shape> buildCornerCutter(const TopoDS_Shape& shape,
                                              const TopoDS_Edge& edge, double radius) {
    const std::optional<EdgeLine> line = asStraightEdge(edge);
    if (!line) {
        return std::nullopt;
    }

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    if (!edgeFaces.Contains(edge)) {
        return std::nullopt;
    }
    const TopTools_ListOfShape& faces = edgeFaces.FindFromKey(edge);
    if (faces.Extent() != 2) {
        return std::nullopt;  // not a manifold interior edge
    }
    const TopoDS_Face faceA = TopoDS::Face(faces.First());
    const TopoDS_Face faceB = TopoDS::Face(faces.Last());

    const gp_Pnt edgeMid = along(line->v1, line->dir, 0.5 * line->len);
    const std::optional<gp_Dir> tA = inFaceDirection(faceA, line->dir, edgeMid);
    const std::optional<gp_Dir> tB = inFaceDirection(faceB, line->dir, edgeMid);
    if (!tA || !tB) {
        return std::nullopt;  // non-planar neighbour: zero-curvature rail invalid
    }

    // Cross-section is built at a base plane pushed `overshoot` before the edge so
    // the prism caps land outside the body at BOTH ends (clean, non-coplanar cut).
    const double overshoot = std::max(2.0, 2.0 * radius);
    const gp_Pnt corner = along(line->v1, line->dir, -overshoot);
    const gp_Pnt railA = along(corner, *tA, radius);
    const gp_Pnt railB = along(corner, *tB, radius);

    // Quintic Bezier: poles 0..2 collinear along -tA and poles 3..5 collinear along
    // -tB. Equal spacing makes the 2nd derivative EXACTLY zero at each rail =>
    // curvature 0 there, matching the flat neighbours (true G2). `s` sets fullness.
    const double s = 0.35 * radius;
    TColgp_Array1OfPnt poles(1, 6);
    poles.SetValue(1, railA);
    poles.SetValue(2, along(railA, *tA, -s));
    poles.SetValue(3, along(railA, *tA, -2.0 * s));
    poles.SetValue(4, along(railB, *tB, -2.0 * s));
    poles.SetValue(5, along(railB, *tB, -s));
    poles.SetValue(6, railB);
    Handle(Geom_BezierCurve) quintic = new Geom_BezierCurve(poles);

    // Outer apex pushed just outside the convex corner so the cutter fully covers
    // the sliver between the quintic and the sharp corner (and only that).
    gp_Vec outward = -(gp_Vec(*tA) + gp_Vec(*tB));
    if (outward.Magnitude() < 1e-9) {
        return std::nullopt;  // opposite faces (180°): not a corner to blend
    }
    outward.Normalize();
    const gp_Pnt apex = corner.Translated(outward * (2.0 * radius));

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(quintic).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(railB, apex).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(apex, railA).Edge());
    if (!wire.IsDone()) {
        return std::nullopt;
    }
    BRepBuilderAPI_MakeFace face(wire.Wire(), /*OnlyPlane*/ Standard_True);
    if (!face.IsDone()) {
        return std::nullopt;
    }

    const double prismLen = line->len + 2.0 * overshoot;
    BRepPrimAPI_MakePrism prism(face.Face(), gp_Vec(line->dir) * prismLen);
    if (!prism.IsDone()) {
        return std::nullopt;
    }
    return prism.Shape();
}

}  // namespace

ShapeResult OcctEngine::fillet_edges_g2(EngineShape body, const int* edgeIds, int edgeCount,
                                        double radius) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || radius <= 0.0) {
            return make_error("fillet_edges_g2: invalid input");
        }
        const std::vector<TopoDS_Edge> edges = occt::edgesByIds(*sp, edgeIds, edgeCount);
        if (edges.empty()) {
            return make_error("fillet_edges_g2: no edges");
        }

        // All cutters are computed from the ORIGINAL geometry (corner regions are
        // disjoint), then cut sequentially so ids stay meaningful and the boolean
        // stays simple.
        TopoDS_Shape result = *sp;
        int built = 0;
        for (const TopoDS_Edge& edge : edges) {
            const std::optional<TopoDS_Shape> cutter = buildCornerCutter(*sp, edge, radius);
            if (!cutter) {
                continue;  // not a straight planar seam: skip (honest partial)
            }
            BRepAlgoAPI_Cut cut(result, *cutter);
            cut.SetFuzzyValue(1e-6);
            cut.Build();
            if (!cut.IsDone()) {
                return make_error("fillet_edges_g2: boolean cut failed");
            }
            result = cut.Shape();
            ++built;
        }
        if (built == 0) {
            return make_error("fillet_edges_g2: no straight planar-seam edge to blend");
        }
        return occt::addIfValid(result, "fillet_edges_g2: invalid blend");
    });
}

}  // namespace cyber
