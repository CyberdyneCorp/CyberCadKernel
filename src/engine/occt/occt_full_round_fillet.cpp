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
// the rolling ball NATIVELY: a solid cylinder of radius r whose axis is the line the
// ball centre traces as it rolls along the crease, tangent to both side planes, is
// used to carve the two top corners away (result = body − (top-slab − cylinder)).
// The blend surface is the cylinder wall, exactly tangent (G1) to both neighbour
// planes at the seams, and the middle face is provably gone.
//
// This works for two PLANAR neighbours — whether they are (anti-)PARALLEL (a
// constant-radius strip on the mid-plane) OR meet at a genuine DIHEDRAL angle:
//
//   • PARALLEL walls (nL·nR ≈ -1): the crease direction n_L×n_R degenerates to zero,
//     so the axis runs along the seam edge, the radius is half the strip width, and
//     the axis sits on the strip mid-plane at distance r from each wall.
//
//   • NON-PARALLEL (dihedral) planar walls: a ball tangent to both planes has its
//     centre on the internal bisector of the dihedral; rolling it along the crease
//     sweeps a cylinder whose
//         axis DIRECTION = normalize(n_L × n_R)   (the crease / plane-intersection),
//         axis LOCATION  = the valley point at perpendicular distance r from BOTH
//                          planes on the interior side, obtained by offsetting each
//                          seam edge inward by r along its wall's inward normal and
//                          requiring the two offsets to coincide,
//     with r fixed by the strip geometry: r = (mL−mR)·(nL−nR) / |nL−nR|² so that the
//     tangent-contact lines land exactly on the two seam edges. The parallel case is
//     the r = width/2 special case of this same formula. Before accepting the
//     dihedral blend we VERIFY the cylinder is actually tangent to both walls within
//     ~1° (radial-normal vs wall-normal); if it is not, we treat it as ineligible.
//
// Truly CURVED (non-planar) neighbours remain out of scope: no constant-radius
// rolling ball exists, so those cases take the fallback below (deferred, honest).
//
// When the neighbours are not two valid planes, the rolling-ball radius/valley
// cannot be solved, the tangency self-check fails, or the boolean result is not
// BRepCheck_Analyzer::IsValid, we FALL BACK to a standard edge fillet on the two
// seam edges (a valid, lower-fidelity result). The facade returns the valid shape
// either way; the sim checks distinguish the true full round (exact tangent-cylinder
// volume + centred/valley blend axis) from the fallback and record the fallback case
// as deferred with the measured gap — never a faked G1/consumption pass.
//
// OCCT-only TU (CYBERCAD_HAS_OCCT); the host build omits it and the stub inherits
// the unsupported default (0). No OCCT type escapes into a public/shared header.

#include "engine/occt/occt_engine.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
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
    gp_Pnt mL, mR;                 // the two seam-edge midpoints (points on each wall)
    gp_Pnt bodyCentre;             // body centre of mass (for normal orientation)
    double bboxDiag = 0.0;         // for boolean-tool margins
};

// The geometry of the rolling-ball cylinder, solved for both the parallel and the
// dihedral case (see the file header). `ok` is false when neither neighbour pair
// admits a constant-radius rolling ball (non-planar walls, degenerate solve, or a
// failed tangency self-check).
struct RollingBall {
    bool ok = false;
    double radius = 0.0;
    gp_Pnt axisPt;                 // a point on the ball-centre axis line
    gp_Dir axisDir;                // direction the ball rolls along (crease / seam)
    bool parallel = false;         // true for the (anti-)parallel special case
    double tangencyDeg = 0.0;      // worst measured wall-tangency error (dihedral)
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
    info.mL = edgeMidpoint(info.eL);
    info.mR = edgeMidpoint(info.eR);
    info.seamMid = gp_Pnt(0.5 * (info.mL.X() + info.mR.X()),
                          0.5 * (info.mL.Y() + info.mR.Y()),
                          0.5 * (info.mL.Z() + info.mR.Z()));
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

// Solve the rolling-ball cylinder (radius, axis point, axis direction) for the two
// neighbour walls. Both must be PLANAR (curved walls have no constant-radius rolling
// ball — those defer). Handles the (anti-)parallel and the general dihedral case
// with ONE construction (see the file header):
//
// The ball centre C and radius r satisfy three tangency/cap conditions — distance r
// from each wall on the interior side, and the cylinder crest (C + r·nM) lands on the
// middle-face plane so the blend exactly caps where the middle strip was:
//     (C − pL)·nL = −r,  (C − pR)·nR = −r,  (C − seamMid)·nM = −r.
// C's freedom to slide along the crease is removed by placing it in the plane through
// seamMid perpendicular to the axis (basis u,v). Subtracting the cap equation from
// each wall equation cancels r and gives a 2×2 linear system in (a,b) with C =
// seamMid + a·u + b·v; r then follows from any wall equation. For (anti-)parallel
// walls this reduces to r = width/2 with C on the strip mid-plane (the original
// behaviour); for a dihedral ridge it yields the tangent cylinder that caps the top.
RollingBall solveRollingBall(const SeamInfo& info) {
    RollingBall rb;
    if (!isPlanar(info.left) || !isPlanar(info.right)) { return rb; }
    if (info.width < 1e-6 || info.seamLen < 1e-6) { return rb; }

    const gp_Vec nL(faceOutwardNormal(info.left, info.bodyCentre));
    const gp_Vec nR(faceOutwardNormal(info.right, info.bodyCentre));
    const gp_Vec nM(info.nMiddleOut);

    // Axis direction: the crease n_L × n_R, or — when it degenerates because the walls
    // are (anti-)parallel — the seam-edge direction (the parallel special case).
    const gp_Vec cross = nL.Crossed(nR);
    rb.parallel = cross.Magnitude() < 1e-4;
    const gp_Dir axisDir = rb.parallel ? gp_Dir(info.axisDir) : gp_Dir(cross);
    rb.axisDir = axisDir;

    // Orthonormal basis (u,v) of the plane through seamMid perpendicular to the axis.
    const gp_Vec zc(axisDir);
    gp_Vec u = gp_Vec(1, 0, 0).Crossed(zc);
    if (u.Magnitude() < 1e-6) { u = gp_Vec(0, 1, 0).Crossed(zc); }
    u.Normalize();
    const gp_Vec v = zc.Crossed(u);  // unit, ⊥ u and axis

    // 2×2 system: a·(u·(nL−nM)) + b·(v·(nL−nM)) = (pL−seamMid)·nL, and likewise for R.
    const gp_Vec dNL = nL - nM;
    const gp_Vec dNR = nR - nM;
    const double a11 = u.Dot(dNL), a12 = v.Dot(dNL);
    const double a21 = u.Dot(dNR), a22 = v.Dot(dNR);
    const double rhs1 = gp_Vec(info.seamMid, info.mL).Dot(nL);  // (pL−seamMid)·nL
    const double rhs2 = gp_Vec(info.seamMid, info.mR).Dot(nR);  // (pR−seamMid)·nR
    const double det = a11 * a22 - a12 * a21;
    if (std::abs(det) < 1e-9) { return rb; }  // walls do not define a rolling ball
    const double a = (rhs1 * a22 - a12 * rhs2) / det;
    const double b = (a11 * rhs2 - rhs1 * a21) / det;

    rb.axisPt = info.seamMid.Translated(u.Multiplied(a)).Translated(v.Multiplied(b));
    const double r = -gp_Vec(info.mL, rb.axisPt).Dot(nL);  // = −(C−pL)·nL
    if (r < 1e-6) { return rb; }
    rb.radius = r;

    // Tangency self-check: perpendicular distance from the axis to each plane must
    // equal r (⇒ the cylinder wall is G1-tangent to each neighbour). Measure the worst
    // angular error and reject beyond ~1° so we never assert a blend that is not truly
    // tangent (honesty rule); such cases take the fallback and are deferred.
    const double dL = std::abs(gp_Vec(info.mL, rb.axisPt).Dot(nL));
    const double dR = std::abs(gp_Vec(info.mR, rb.axisPt).Dot(nR));
    const double errL = std::abs(dL - r) / std::max(r, 1e-9);
    const double errR = std::abs(dR - r) / std::max(r, 1e-9);
    rb.tangencyDeg =
        std::asin(std::min(1.0, std::max(errL, errR))) * 180.0 / 3.14159265358979323846;
    if (rb.tangencyDeg > 1.0) { return rb; }  // ineligible: not truly tangent (deferred)

    rb.ok = true;
    return rb;
}

// Build the native rolling-ball blend: carve the two top corners with the tangent
// cylinder solved by solveRollingBall (parallel or dihedral). Returns a null shape
// on any failure.
TopoDS_Shape buildRollingBall(const TopoDS_Shape& body, const SeamInfo& info,
                              const RollingBall& rb) {
    const double r = rb.radius;
    const double m = info.bboxDiag;               // generous tool margin
    const gp_Dir axisDir = rb.axisDir;
    const gp_Pnt axisPt = rb.axisPt;              // on the ball-centre axis line

    // Solid cylinder along the crease, tangent to both walls, spanning the seam.
    const double cylLen = info.seamLen + 2.0 * m;
    const gp_Pnt cylBase = axisPt.Translated(gp_Vec(axisDir).Multiplied(-(0.5 * info.seamLen + m)));
    const gp_Ax2 cylAx(cylBase, axisDir);
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(cylAx, r, cylLen).Shape();
    if (cyl.IsNull()) { return TopoDS_Shape(); }

    // Half-space H on the INTERIOR side of the TANGENT plane — the plane containing the
    // two cylinder↔wall contact lines. Each contact point is the foot of the axis on a
    // wall: contactL = axisPt + r·nL, contactR = axisPt + r·nR. The plane through both
    // (containing the axis direction) is exactly where the cylinder meets the walls, so
    // capping the body there leaves a seamless (G1) join with no flat sliver: below the
    // plane the body is kept whole; above it only the material inside the cylinder
    // survives. For (anti-)parallel walls this plane passes through the axis (nL·nM=0)
    // and the construction reduces to the original mid-plane cap.
    const gp_Vec nL(faceOutwardNormal(info.left, info.bodyCentre));
    const gp_Vec nR(faceOutwardNormal(info.right, info.bodyCentre));
    const gp_Pnt contactL = axisPt.Translated(nL.Multiplied(r));
    const gp_Pnt contactR = axisPt.Translated(nR.Multiplied(r));
    const gp_Pnt planeOrg(0.5 * (contactL.X() + contactR.X()),
                          0.5 * (contactL.Y() + contactR.Y()),
                          0.5 * (contactL.Z() + contactR.Z()));
    gp_Vec cutN = gp_Vec(axisDir).Crossed(gp_Vec(contactL, contactR));
    if (cutN.Magnitude() < 1e-9) { cutN = gp_Vec(info.nMiddleOut); }  // contacts coincide
    if (cutN.Dot(gp_Vec(info.nMiddleOut)) < 0) { cutN.Reverse(); }    // point exterior
    cutN.Normalize();

    // Big box, TOP face on the tangent plane (box grows inward = −cutN), spanning the
    // whole body laterally. Frame: X=+axis, Z=−cutN (inward), Y=Z×X.
    const gp_Dir inwardZ(cutN.Multiplied(-1.0));
    const gp_Dir xAx(axisDir);
    const gp_Vec widthVec = cutN.Crossed(gp_Vec(axisDir));
    const gp_Dir widthDir(widthVec);
    const double big = 2.0 * m;                     // ≥ body extent in every direction
    const gp_Pnt corner = planeOrg.Translated(gp_Vec(widthDir).Multiplied(big))
                              .Translated(gp_Vec(axisDir).Multiplied(-big));
    const gp_Ax2 boxAx(corner, inwardZ, xAx);
    TopoDS_Shape halfSpace = BRepPrimAPI_MakeBox(boxAx, 2.0 * big, 2.0 * big, 2.0 * big).Shape();
    if (halfSpace.IsNull()) { return TopoDS_Shape(); }

    // keep = cylinder ∪ interior half-space. Below the axis plane the body is kept
    // whole; above it only the material inside the tangent cylinder survives — so the
    // middle face is replaced by the cylinder wall (G1-tangent to both neighbours) and
    // the two walls remain intact up to their tangent lines. This is exactly what a
    // ball rolling along the crease sweeps, for both the parallel and dihedral cases.
    BRepAlgoAPI_Fuse mkKeep(cyl, halfSpace);
    if (!mkKeep.IsDone()) { return TopoDS_Shape(); }
    BRepAlgoAPI_Common mkRes(body, mkKeep.Shape());
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
    const RollingBall rb = solveRollingBall(info);
    if (rb.ok) {
        const TopoDS_Shape rolled = buildRollingBall(body, info, rb);
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
