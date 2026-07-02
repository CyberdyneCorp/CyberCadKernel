// OCCT engine adapter — FULL-ROUND-FILLET feature (Phase-3 owned TU).
//
// Defines the face-consuming rolling-ball blend declared in occt_engine.h:
//   full_round_fillet(body, faceId)                  auto-detect the two opposite
//                                                    neighbours of faceId
//   full_round_fillet_faces(body, left, middle, right)  consume middle, blend
//                                                    tangent to left + right
//
// A full round rolls a ball of the largest fitting radius along the strip between
// two side faces, replacing the narrow middle face with a blend surface tangent to
// both. OCCT has no stock "consume this face and blend its neighbours" call, and
// BRepFilletAPI_MakeFillet REFUSES the critical radius (= half the strip width) at
// which the two edge fillets would merge and consume the middle face — so we build
// the rolling ball NATIVELY: a solid cylinder of radius = width/2 whose axis lies
// on the strip mid-plane, tangent to both side planes, is used to carve the two
// top corners away (result = body − (top-slab − cylinder)). The blend surface is
// the cylinder wall, exactly tangent (G1) to both neighbour planes at the seams,
// and the middle face is provably gone.
//
// The native rolling-ball blend is only geometrically well defined when the two
// neighbours are PLANAR and (anti-)parallel (a constant-radius strip). When that
// eligibility check fails, or the boolean result is not BRepCheck_Analyzer::IsValid,
// we FALL BACK to a standard edge fillet on the two seam edges (a valid, lower-
// fidelity result). The facade returns the valid shape either way; the sim checks
// distinguish the true full round (exact tangent-cylinder volume + centred blend
// axis) from the fallback and record the fallback case as deferred with the
// measured gap — never a faked G1/consumption pass.
//
// OCCT-only TU (CYBERCAD_HAS_OCCT); the host build omits it and the stub inherits
// the unsupported default (0). No OCCT type escapes into a public/shared header.

#include "engine/occt/occt_engine.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepGProp.hxx>
#include <BRepGProp_Face.hxx>
#include <GProp_GProps.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>

namespace cyber {

namespace {

// ── small geometric helpers ───────────────────────────────────────────────────

double edgeLength(const TopoDS_Edge& e) {
    GProp_GProps p;
    BRepGProp::LinearProperties(e, p);
    return p.Mass();
}

gp_Pnt edgeMidpoint(const TopoDS_Edge& e) {
    BRepAdaptor_Curve c(e);
    return c.Value(0.5 * (c.FirstParameter() + c.LastParameter()));
}

gp_Vec edgeDirection(const TopoDS_Edge& e) {
    BRepAdaptor_Curve c(e);
    gp_Pnt p;
    gp_Vec d;
    c.D1(0.5 * (c.FirstParameter() + c.LastParameter()), p, d);
    if (d.Magnitude() > 1e-12) { d.Normalize(); }
    return d;
}

// Outward normal of a planar face at its parametric centre, flipped to point away
// from the body centroid so "inward" is unambiguous.
gp_Dir faceOutwardNormal(const TopoDS_Face& face, const gp_Pnt& bodyCentre) {
    BRepGProp_Face gf(face);
    Standard_Real u0, u1, v0, v1;
    gf.Bounds(u0, u1, v0, v1);
    gp_Pnt c;
    gp_Vec n;
    gf.Normal(0.5 * (u0 + u1), 0.5 * (v0 + v1), c, n);
    if (n.Magnitude() > 1e-12) { n.Normalize(); }
    gp_Vec outward(bodyCentre, c);
    if (n.Dot(outward) < 0) { n.Reverse(); }
    return gp_Dir(n);
}

bool isPlanar(const TopoDS_Face& f) {
    return BRepAdaptor_Surface(f).GetType() == GeomAbs_Plane;
}

// Edges shared by two faces (compared by identity).
std::vector<TopoDS_Edge> sharedEdges(const TopoDS_Face& a, const TopoDS_Face& b) {
    std::vector<TopoDS_Edge> out;
    for (TopExp_Explorer ea(a, TopAbs_EDGE); ea.More(); ea.Next()) {
        for (TopExp_Explorer eb(b, TopAbs_EDGE); eb.More(); eb.Next()) {
            if (ea.Current().IsSame(eb.Current())) {
                out.push_back(TopoDS::Edge(ea.Current()));
                break;
            }
        }
    }
    return out;
}

// The other face (not `middle`) sharing edge `e`, using the edge→face ancestry map.
TopoDS_Face neighbourAcross(const TopoDS_Shape& shape, const TopoDS_Edge& e,
                            const TopoDS_Face& middle) {
    TopTools_IndexedDataMapOfShapeListOfShape ef;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ef);
    if (!ef.Contains(e)) { return TopoDS_Face(); }
    for (const TopoDS_Shape& fs : ef.FindFromKey(e)) {
        if (!fs.IsSame(middle)) { return TopoDS::Face(fs); }
    }
    return TopoDS_Face();
}

// Everything the blend needs, resolved once from either entry point.
struct SeamInfo {
    bool ok = false;               // seam edges + faces found
    TopoDS_Edge eL, eR;            // the two seam edges (middle↔left, middle↔right)
    TopoDS_Face left, right;       // neighbour side faces
    gp_Dir nMiddleOut;             // outward normal of the middle face
    double width = 0.0;            // perpendicular gap between the seam edges
    double seamLen = 0.0;          // length of the longer seam edge
    gp_Vec axisDir;                // rolling-ball axis direction (along the strip)
    gp_Pnt seamMid;                // midpoint between the two seam-edge midpoints
    gp_Pnt bodyCentre;             // body centre of mass (for normal orientation)
    double bboxDiag = 0.0;         // for boolean-tool margins
};

double bboxDiagonal(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    // A cheap, robust size proxy: 2× the radius of gyration bound is overkill; use
    // the linear extent from the mass distribution instead via the bounding of the
    // two most-distant seam points is not available here, so fall back to a safe
    // constant scaled by the volume^(1/3).
    const double v = std::abs(g.Mass());
    const double s3 = (v > 1e-9) ? std::cbrt(v) : 1.0;
    return 4.0 * s3 + 1.0;  // generous margin; booleans tolerate oversize tools
}

void fillCommon(const TopoDS_Shape& shape, const TopoDS_Face& middle, SeamInfo& info) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(shape, g);
    info.bodyCentre = g.CentreOfMass();
    info.bboxDiag = bboxDiagonal(shape);
    info.nMiddleOut = faceOutwardNormal(middle, info.bodyCentre);
    info.width = BRepExtrema_DistShapeShape(info.eL, info.eR).Value();
    info.seamLen = std::max(edgeLength(info.eL), edgeLength(info.eR));
    info.axisDir = edgeDirection(info.eL);
    const gp_Pnt mL = edgeMidpoint(info.eL);
    const gp_Pnt mR = edgeMidpoint(info.eR);
    info.seamMid = gp_Pnt(0.5 * (mL.X() + mR.X()), 0.5 * (mL.Y() + mR.Y()),
                          0.5 * (mL.Z() + mR.Z()));
}

// Explicit left/middle/right → seam edges are the shared edges.
SeamInfo resolveFromFaces(const TopoDS_Shape& shape, const TopoDS_Face& left,
                          const TopoDS_Face& middle, const TopoDS_Face& right) {
    SeamInfo info;
    const std::vector<TopoDS_Edge> sL = sharedEdges(middle, left);
    const std::vector<TopoDS_Edge> sR = sharedEdges(middle, right);
    if (sL.empty() || sR.empty()) { return info; }
    auto longest = [](const std::vector<TopoDS_Edge>& v) {
        return *std::max_element(v.begin(), v.end(), [](const TopoDS_Edge& a, const TopoDS_Edge& b) {
            return edgeLength(a) < edgeLength(b);
        });
    };
    info.eL = longest(sL);
    info.eR = longest(sR);
    info.left = left;
    info.right = right;
    fillCommon(shape, middle, info);
    info.ok = true;
    return info;
}

// Single face id → the two longest edges of the middle face are the seams; their
// across-neighbours are the side faces.
SeamInfo resolveAuto(const TopoDS_Shape& shape, const TopoDS_Face& middle) {
    SeamInfo info;
    std::vector<TopoDS_Edge> edges;
    for (TopExp_Explorer ex(middle, TopAbs_EDGE); ex.More(); ex.Next()) {
        edges.push_back(TopoDS::Edge(ex.Current()));
    }
    if (edges.size() < 2) { return info; }
    std::sort(edges.begin(), edges.end(),
              [](const TopoDS_Edge& a, const TopoDS_Edge& b) { return edgeLength(a) > edgeLength(b); });
    info.eL = edges[0];
    info.eR = edges[1];
    info.left = neighbourAcross(shape, info.eL, middle);
    info.right = neighbourAcross(shape, info.eR, middle);
    if (info.left.IsNull() || info.right.IsNull()) { return info; }
    fillCommon(shape, middle, info);
    info.ok = true;
    return info;
}

// The rolling ball is only a well-defined constant-radius full round when both
// neighbours are planar and their outward normals are (anti-)parallel.
bool rollingBallEligible(const SeamInfo& info) {
    if (!isPlanar(info.left) || !isPlanar(info.right)) { return false; }
    if (info.width < 1e-6 || info.seamLen < 1e-6) { return false; }
    const gp_Dir nL = faceOutwardNormal(info.left, info.bodyCentre);
    const gp_Dir nR = faceOutwardNormal(info.right, info.bodyCentre);
    return nL.Dot(nR) < -0.98;  // opposite side walls (parallel strip)
}

// Build the native rolling-ball blend: carve the two top corners with a cylinder of
// radius width/2 tangent to both side planes. Returns a null shape on any failure.
TopoDS_Shape buildRollingBall(const TopoDS_Shape& body, const SeamInfo& info) {
    const double r = 0.5 * info.width;
    const double m = info.bboxDiag;                       // generous tool margin
    const gp_Vec inward = gp_Vec(info.nMiddleOut).Multiplied(-r);
    const gp_Pnt axisPt = info.seamMid.Translated(inward);  // on the strip mid-plane

    // Solid cylinder along the strip, centred on the seam, tangent to both walls.
    const double cylLen = info.seamLen + 2.0 * m;
    const gp_Pnt cylBase = axisPt.Translated(info.axisDir.Multiplied(-(0.5 * info.seamLen + m)));
    const gp_Ax2 cylAx(cylBase, gp_Dir(info.axisDir));
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(cylAx, r, cylLen).Shape();
    if (cyl.IsNull()) { return TopoDS_Shape(); }

    // Oriented slab covering the whole top of the strip (frame Z=outward, X=axis).
    const gp_Dir zAx(info.nMiddleOut);
    const gp_Dir xAx(info.axisDir);
    const gp_Vec widthVec = gp_Vec(info.nMiddleOut).Crossed(info.axisDir);
    const gp_Dir widthDir(widthVec);
    const gp_Pnt corner = axisPt.Translated(gp_Vec(widthDir).Multiplied(-(r + m)))
                              .Translated(info.axisDir.Multiplied(-(0.5 * info.seamLen + m)));
    const gp_Ax2 boxAx(corner, zAx, xAx);
    const double dx = info.seamLen + 2.0 * m;  // along axis (X)
    const double dy = 2.0 * (r + m);           // along width (Y = Z×X)
    const double dz = 2.0 * r + m;             // outward (Z), covers axis→top and above
    TopoDS_Shape slab = BRepPrimAPI_MakeBox(boxAx, dx, dy, dz).Shape();
    if (slab.IsNull()) { return TopoDS_Shape(); }

    // tool = slab − cylinder = the two corner slivers above the tangent cylinder.
    BRepAlgoAPI_Cut mkTool(slab, cyl);
    if (!mkTool.IsDone()) { return TopoDS_Shape(); }
    // result = body − tool = body with the middle face rounded off into the cylinder.
    BRepAlgoAPI_Cut mkRes(body, mkTool.Shape());
    if (!mkRes.IsDone()) { return TopoDS_Shape(); }
    return mkRes.Shape();
}

// Honest fallback: a standard edge fillet on the two seam edges, at a radius below
// the strip half-width so it builds cleanly (it does NOT consume the middle face).
// Tries a couple of radii and returns the first valid result (null if none).
TopoDS_Shape fallbackEdgeFillet(const TopoDS_Shape& body, const SeamInfo& info) {
    const double half = 0.5 * info.width;
    for (const double frac : {0.45, 0.30, 0.20}) {
        const double r = half * frac;
        if (r <= 1e-6) { continue; }
        BRepFilletAPI_MakeFillet fillet(body);
        fillet.Add(r, info.eL);
        fillet.Add(r, info.eR);
        fillet.Build();
        if (fillet.IsDone() && !fillet.Shape().IsNull() &&
            BRepCheck_Analyzer(fillet.Shape()).IsValid()) {
            return fillet.Shape();
        }
    }
    return TopoDS_Shape();
}

// Shared tail: try the native rolling ball, then the fallback fillet; gate both on
// BRepCheck_Analyzer::IsValid (via occt::addIfValid).
ShapeResult finishBlend(const TopoDS_Shape& body, const SeamInfo& info, const char* op) {
    if (rollingBallEligible(info)) {
        const TopoDS_Shape rolled = buildRollingBall(body, info);
        if (!rolled.IsNull() && occt::isValid(rolled)) {
            return occt::wrap(rolled);
        }
    }
    const TopoDS_Shape fb = fallbackEdgeFillet(body, info);
    if (!fb.IsNull()) {
        return occt::addIfValid(fb, "full_round_fillet: invalid fallback fillet");
    }
    return make_error(std::string(op) + ": could not build a rolling-ball blend or fallback fillet");
}

}  // namespace

ShapeResult OcctEngine::full_round_fillet(EngineShape body, int faceId) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("full_round_fillet: invalid body"); }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> mid = occt::facesByIds(*sp, ids, 1);
        if (mid.empty()) { return make_error("full_round_fillet: middle face not found"); }
        const SeamInfo info = resolveAuto(*sp, mid.front());
        if (!info.ok) { return make_error("full_round_fillet: could not identify seam neighbours"); }
        return finishBlend(*sp, info, "full_round_fillet");
    });
}

ShapeResult OcctEngine::full_round_fillet_faces(EngineShape body, int leftFaceId, int middleFaceId,
                                                int rightFaceId) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("full_round_fillet_faces: invalid body"); }
        const int ids[3] = {leftFaceId, middleFaceId, rightFaceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, ids, 3);
        if (faces.size() != 3) {
            return make_error("full_round_fillet_faces: left/middle/right face id not found");
        }
        // facesByIds preserves the request order (skipping only out-of-range ids); a
        // full set of 3 means [left, middle, right].
        const SeamInfo info = resolveFromFaces(*sp, faces[0], faces[1], faces[2]);
        if (!info.ok) {
            return make_error("full_round_fillet_faces: no shared seam edges between faces");
        }
        return finishBlend(*sp, info, "full_round_fillet_faces");
    });
}

}  // namespace cyber
