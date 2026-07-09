// OCCT engine adapter — CONSTRUCT capability group.
//
// Definitions of the OcctEngine construct-group methods declared in
// occt_engine.h (extrude / revolve / loft / sweep / thread / shank / profile
// builders). This is an OCCT-only TU: it may include OpenCASCADE headers, but no
// OCCT type escapes into any public or shared header. It compiles only for iOS
// where the trimmed OCCT static libs are linked (CYBERCAD_HAS_OCCT); there is no
// host OCCT, so the whole file is unconditionally OCCT (no CYBERCAD_HAS_OCCT
// guards — the build simply omits this TU on the host).
//
// Ported behaviour-for-behaviour from the app's Objective-C++ bridge
// (cybercad/CyberCad/Kernel/Bridge/KernelBridge.mm, CYBERCAD_HAS_OCCT path): the
// same degenerate-input guards and BRepCheck_Analyzer::IsValid gating are
// preserved. Where the bridge returned CCShapeId 0 (degenerate / invalid), the
// method returns an Error the facade collapses to 0/nil + cc_last_error; where it
// returned a registry id, the method returns the validated shape wrapped for the
// registry via occt::addIfValid. Cross-cutting helpers (occtGuard / wrap / unwrap
// / addIfValid / facesByIds / tessellateShape) come from occt_engine.{h,cpp}.

#include "engine/occt/occt_engine.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

// ── OCCT builders (this TU only) ──────────────────────────────────────────────
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Cylinder.hxx>
#include <gp_XY.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_TransitionMode.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepFill_TypeOfContact.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopoDS_Wire.hxx>

namespace cyber {

// ── Construct-group private helpers ───────────────────────────────────────────
// Ported verbatim from the bridge's anonymous namespace. These are construct-only
// (loft-rail framing, typed-wire / spline edges, helix sampling, thread sweep) so
// they live here rather than in the shared spine.
namespace {

// Centroid of an x,y profile (2 doubles per point). Used to recenter a section on
// the rail endpoint before placing it.
gp_XY profileCentroid2D(const double* xy, int count) {
    double cx = 0, cy = 0;
    for (int i = 0; i < count; ++i) {
        cx += xy[i * 2];
        cy += xy[i * 2 + 1];
    }
    return gp_XY(cx / count, cy / count);
}

// Build a closed section wire from an x,y profile centered on its centroid and
// placed in the plane through `origin` whose local x maps to `uDir`, local y to
// `vDir`. Returns false on a degenerate frame / wire.
bool buildRailSectionWire(const double* xy, int count, const gp_Pnt& origin, const gp_Vec& uDir,
                          const gp_Vec& vDir, TopoDS_Wire& out) {
    if (xy == nullptr || count < 3) {
        return false;
    }
    const gp_XY c = profileCentroid2D(xy, count);
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < count; ++i) {
        const double u = xy[i * 2] - c.X(), v = xy[i * 2 + 1] - c.Y();
        poly.Add(gp_Pnt(origin.X() + u * uDir.X() + v * vDir.X(),
                        origin.Y() + u * uDir.Y() + v * vDir.Y(),
                        origin.Z() + u * uDir.Z() + v * vDir.Z()));
    }
    poly.Close();
    if (!poly.IsDone()) {
        return false;
    }
    out = poly.Wire();
    return true;
}

// An orthonormal (uDir, vDir) frame in the plane perpendicular to `tangent`,
// keeping uDir close to world +X / +Y so the two sections stay aligned (no twist)
// along a planar rail. Returns false if the tangent is degenerate.
bool perpendicularFrame(gp_Vec tangent, gp_Vec& uDir, gp_Vec& vDir) {
    if (tangent.Magnitude() < 1.0e-9) {
        return false;
    }
    tangent.Normalize();
    gp_Vec ref(0, 0, 1);
    if (fabs(tangent.Dot(ref)) > 0.95) {
        ref = gp_Vec(1, 0, 0);  // tangent ~parallel to Z → use X
    }
    uDir = tangent.Crossed(ref);
    if (uDir.Magnitude() < 1.0e-9) {
        return false;
    }
    uDir.Normalize();
    vDir = tangent.Crossed(uDir);
    if (vDir.Magnitude() < 1.0e-9) {
        return false;
    }
    vDir.Normalize();
    return true;
}

// Build a spine wire from rail points (x,y,z triplets). A BSpline through the
// points for a smooth rail (>= 3 points), else a single straight edge for 2
// points. Also returns the start/end tangents for orienting the sections. Returns
// false on a degenerate rail (coincident points / too few).
bool buildSpineWire(const double* xyz, int count, TopoDS_Wire& out, gp_Pnt& startPt,
                    gp_Vec& startTan, gp_Pnt& endPt, gp_Vec& endTan) {
    if (xyz == nullptr || count < 2) {
        return false;
    }
    startPt = gp_Pnt(xyz[0], xyz[1], xyz[2]);
    endPt = gp_Pnt(xyz[(count - 1) * 3], xyz[(count - 1) * 3 + 1], xyz[(count - 1) * 3 + 2]);
    if (startPt.Distance(endPt) < 1.0e-6) {
        return false;
    }
    startTan = gp_Vec(gp_Pnt(xyz[0], xyz[1], xyz[2]), gp_Pnt(xyz[3], xyz[4], xyz[5]));
    endTan = gp_Vec(gp_Pnt(xyz[(count - 2) * 3], xyz[(count - 2) * 3 + 1], xyz[(count - 2) * 3 + 2]),
                    endPt);
    if (count == 2) {
        const TopoDS_Edge e = BRepBuilderAPI_MakeEdge(startPt, endPt).Edge();
        out = BRepBuilderAPI_MakeWire(e).Wire();
        return true;
    }
    TColgp_Array1OfPnt pts(1, count);
    for (int i = 0; i < count; ++i) {
        pts.SetValue(i + 1, gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
    }
    GeomAPI_PointsToBSpline fit(pts);
    if (!fit.IsDone()) {
        return false;
    }
    Handle(Geom_BSplineCurve) curve = fit.Curve();
    if (curve.IsNull()) {
        return false;
    }
    const TopoDS_Edge e = BRepBuilderAPI_MakeEdge(curve).Edge();
    out = BRepBuilderAPI_MakeWire(e).Wire();
    return true;
}

// Build a closed profile (x,y pairs) into a planar face on z=0. Returns false on a
// degenerate profile.
bool buildProfileFace(const double* profileXY, int pointCount, TopoDS_Face& out) {
    if (pointCount < 3 || profileXY == nullptr) {
        return false;
    }
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < pointCount; ++i) {
        poly.Add(gp_Pnt(profileXY[i * 2], profileXY[i * 2 + 1], 0.0));
    }
    poly.Close();
    if (!poly.IsDone()) {
        return false;
    }
    BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
    if (!face.IsDone()) {
        return false;
    }
    out = face.Face();
    return true;
}

// Build a closed profile (x,y pairs, z=0) extruded +Z into a validated solid.
// Returns false (leaving `out` untouched) on a degenerate profile or an invalid
// result.
bool buildExtrusion(const double* profileXY, int pointCount, double depth, TopoDS_Shape& out) {
    if (!(depth > 1.0e-6)) {
        return false;  // zero/negative height → degenerate prism (OCCT can hang)
    }
    TopoDS_Face face;
    if (!buildProfileFace(profileXY, pointCount, face)) {
        return false;
    }
    BRepPrimAPI_MakePrism prism(face, gp_Vec(0.0, 0.0, depth));
    prism.Build();
    if (!prism.IsDone()) {
        return false;
    }
    const TopoDS_Shape solid = prism.Shape();
    if (!BRepCheck_Analyzer(solid).IsValid()) {
        return false;
    }
    out = solid;
    return true;
}

// Extrude an outer polygon with TRUE circular holes (one circle edge per hole) by
// `depth`. Returns false on degenerate input / invalid result.
bool buildExtrusionWithHoles(const double* outerXY, int outerCount, const double* holesCR,
                             int holeCount, double depth, TopoDS_Shape& out) {
    if (!(depth > 1.0e-6)) {
        return false;
    }
    TopoDS_Face outerFace;
    if (!buildProfileFace(outerXY, outerCount, outerFace)) {
        return false;
    }
    BRepBuilderAPI_MakeFace mf(outerFace);
    for (int i = 0; i < holeCount; ++i) {
        const double cx = holesCR[i * 3], cy = holesCR[i * 3 + 1], r = holesCR[i * 3 + 2];
        if (!(r > 1.0e-6)) {
            continue;
        }
        const gp_Circ circ(gp_Ax2(gp_Pnt(cx, cy, 0.0), gp_Dir(0.0, 0.0, 1.0)), r);
        const TopoDS_Edge e = BRepBuilderAPI_MakeEdge(circ);
        const TopoDS_Wire w = BRepBuilderAPI_MakeWire(e);
        mf.Add(TopoDS::Wire(w.Reversed()));  // hole wire: opposite orientation to the outer
    }
    if (!mf.IsDone()) {
        return false;
    }
    BRepPrimAPI_MakePrism prism(mf.Face(), gp_Vec(0.0, 0.0, depth));
    prism.Build();
    if (!prism.IsDone()) {
        return false;
    }
    const TopoDS_Shape solid = prism.Shape();
    if (!BRepCheck_Analyzer(solid).IsValid()) {
        return false;
    }
    out = solid;
    return true;
}

// Extrude an outer polygon with arbitrary POLYGON holes (each hole a closed point
// loop) by `depth`. `holesXY` concatenates every hole's (x,y) pairs; `holeCounts[i]`
// is hole i's vertex count. Returns false on degenerate input / invalid result.
bool buildExtrusionWithPolyHoles(const double* outerXY, int outerCount, const double* holesXY,
                                 const int* holeCounts, int holeCount, double depth,
                                 TopoDS_Shape& out) {
    if (!(depth > 1.0e-6)) {
        return false;
    }
    TopoDS_Face outerFace;
    if (!buildProfileFace(outerXY, outerCount, outerFace)) {
        return false;
    }
    BRepBuilderAPI_MakeFace mf(outerFace);
    int offset = 0;
    for (int i = 0; i < holeCount; ++i) {
        const int n = holeCounts[i];
        if (n >= 3) {
            BRepBuilderAPI_MakePolygon poly;
            for (int j = 0; j < n; ++j) {
                poly.Add(gp_Pnt(holesXY[(offset + j) * 2], holesXY[(offset + j) * 2 + 1], 0.0));
            }
            poly.Close();
            if (poly.IsDone()) {
                mf.Add(TopoDS::Wire(poly.Wire().Reversed()));  // hole: opposite orientation
            }
        }
        offset += n;
    }
    if (!mf.IsDone()) {
        return false;
    }
    BRepPrimAPI_MakePrism prism(mf.Face(), gp_Vec(0.0, 0.0, depth));
    prism.Build();
    if (!prism.IsDone()) {
        return false;
    }
    const TopoDS_Shape solid = prism.Shape();
    if (!BRepCheck_Analyzer(solid).IsValid()) {
        return false;
    }
    out = solid;
    return true;
}

// Build one B-spline edge through the kind-3 segment's points (window into the
// call's splineXY x,y side-channel). Falls back to a straight edge for 2 points.
bool addSplineEdge(const ProfileSeg& s, const double* splineXY, int splineXYCount,
                   BRepBuilderAPI_MakeWire& wire) {
    if (splineXY == nullptr || s.ptCount < 2) {
        return false;
    }
    if ((s.ptOffset + s.ptCount) * 2 > splineXYCount) {
        return false;  // bounds guard
    }
    const double* base = splineXY + static_cast<size_t>(s.ptOffset) * 2;
    if (s.ptCount == 2) {
        const gp_Pnt p0(base[0], base[1], 0.0), p1(base[2], base[3], 0.0);
        if (p0.Distance(p1) < 1.0e-7) {
            return true;  // degenerate → skip, not fatal
        }
        wire.Add(BRepBuilderAPI_MakeEdge(p0, p1).Edge());
        return true;
    }
    TColgp_Array1OfPnt pts(1, s.ptCount);
    for (int k = 0; k < s.ptCount; ++k) {
        pts.SetValue(k + 1, gp_Pnt(base[k * 2], base[k * 2 + 1], 0.0));
    }
    GeomAPI_PointsToBSpline fit(pts);
    if (!fit.IsDone()) {
        return false;
    }
    Handle(Geom_BSplineCurve) curve = fit.Curve();
    if (curve.IsNull()) {
        return false;
    }
    wire.Add(BRepBuilderAPI_MakeEdge(curve).Edge());
    return true;
}

// Build a closed wire (z=0) from typed segments — lines and TRUE arc/circle/spline
// edges — so extruded curved boundaries become single B-rep edges. Returns false
// if the segments don't form a valid closed wire.
bool buildTypedWire(const ProfileSeg* segs, int segCount, TopoDS_Wire& out,
                    const double* splineXY = nullptr, int splineXYCount = 0) {
    if (segs == nullptr || segCount < 1) {
        return false;
    }
    BRepBuilderAPI_MakeWire wire;
    for (int i = 0; i < segCount; ++i) {
        const ProfileSeg& s = segs[i];
        if (s.kind == 0) {  // line
            const gp_Pnt p0(s.x0, s.y0, 0.0), p1(s.x1, s.y1, 0.0);
            if (p0.Distance(p1) < 1.0e-7) {
                continue;  // skip a degenerate edge
            }
            wire.Add(BRepBuilderAPI_MakeEdge(p0, p1).Edge());
        } else if (s.kind == 1) {  // arc of a circle
            if (!(s.r > 1.0e-6)) {
                return false;
            }
            const gp_Circ circ(gp_Ax2(gp_Pnt(s.cx, s.cy, 0.0), gp_Dir(0.0, 0.0, 1.0)), s.r);
            const double u1 = std::min(s.a0, s.a1), u2 = std::max(s.a0, s.a1);
            if (!(u2 - u1 > 1.0e-7)) {
                return false;
            }
            wire.Add(BRepBuilderAPI_MakeEdge(circ, u1, u2).Edge());
        } else if (s.kind == 3) {  // B-spline edge
            if (!addSplineEdge(s, splineXY, splineXYCount, wire)) {
                return false;
            }
        } else {  // full circle
            if (!(s.r > 1.0e-6)) {
                return false;
            }
            const gp_Circ circ(gp_Ax2(gp_Pnt(s.cx, s.cy, 0.0), gp_Dir(0.0, 0.0, 1.0)), s.r);
            wire.Add(BRepBuilderAPI_MakeEdge(circ).Edge());
        }
    }
    if (!wire.IsDone()) {
        return false;
    }
    out = wire.Wire();
    return true;
}

// Place a wire built in the global XOY plane onto a plane frame (origin + u + v) via a rigid
// motion (dimensions preserved). Mirrors the app's KernelBridge.mm placeWireOnFrame so
// cc_loft_typed's frame semantics match the oracle. Returns false on a degenerate frame.
bool placeWireOnFrame(TopoDS_Wire& w, const double* f) {
    gp_Vec u(f[3], f[4], f[5]), v(f[6], f[7], f[8]);
    gp_Vec n = u.Crossed(v);
    if (u.Magnitude() < 1.0e-9 || v.Magnitude() < 1.0e-9 || n.Magnitude() < 1.0e-9) {
        return false;
    }
    gp_Ax3 dst(gp_Pnt(f[0], f[1], f[2]), gp_Dir(n), gp_Dir(u));
    gp_Trsf t;
    t.SetDisplacement(gp_Ax3(), dst);
    BRepBuilderAPI_Transform xf(w, t, Standard_True);
    if (!xf.IsDone()) {
        return false;
    }
    w = TopoDS::Wire(xf.Shape());
    return true;
}

// Revolve a closed profile (x,y pairs, z=0) about the Y axis through the origin by
// `angleRadians` (the caller positions the profile so its axis is x=0).
bool buildRevolution(const double* profileXY, int pointCount, double angleRadians,
                     TopoDS_Shape& out) {
    if (!(angleRadians > 1.0e-6)) {
        return false;  // zero/negative sweep → degenerate (OCCT can hang)
    }
    TopoDS_Face face;
    if (!buildProfileFace(profileXY, pointCount, face)) {
        return false;
    }
    const gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0));
    BRepPrimAPI_MakeRevol revol(face, axis, angleRadians);
    revol.Build();
    if (!revol.IsDone()) {
        return false;
    }
    const TopoDS_Shape solid = revol.Shape();
    if (!BRepCheck_Analyzer(solid).IsValid()) {
        return false;
    }
    out = solid;
    return true;
}

// Sample a (possibly TAPERED) helix (x=r·cosθ, y=r·sinθ, z=rise·f) as an OCCT
// point array over `turns`, capping the per-turn samples so a many-turn helix
// stays a bounded spine for MakePipeShell. The spine radius interpolates linearly
// from `radiusBottom` (z=0) to `radiusTop` (z=rise). Returns false on degenerate
// input.
bool sampleHelix(double radiusBottom, double radiusTop, double pitch, double turns,
                 int samplesPerTurn, TColgp_Array1OfPnt& outPts, int& count) {
    if (radiusBottom <= 0 || radiusTop <= 0 || pitch <= 0 || turns <= 0) {
        return false;
    }
    const int perTurn = std::max(2, std::min(24, samplesPerTurn));  // cap (crash discipline)
    count = std::max(2, static_cast<int>(std::lround(turns * perTurn)));
    const double rise = pitch * turns;  // total Z over all turns
    outPts.Resize(1, count + 1, Standard_False);
    for (int i = 0; i <= count; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(count);  // 0…1 across turns
        const double theta = f * turns * 2.0 * M_PI;
        const double r = radiusBottom + (radiusTop - radiusBottom) * f;  // taper the spine radius
        outPts.SetValue(i + 1, gp_Pnt(r * cos(theta), r * sin(theta), rise * f));
    }
    count += 1;  // point count (steps + 1)
    return true;
}

// Build the helix spine wire (a single BSpline edge through the sampled points).
bool helixSpineWire(const TColgp_Array1OfPnt& pts, TopoDS_Wire& out) {
    GeomAPI_PointsToBSpline fit(pts);
    if (!fit.IsDone()) {
        return false;
    }
    Handle(Geom_BSplineCurve) curve = fit.Curve();
    if (curve.IsNull()) {
        return false;
    }
    const TopoDS_Edge e = BRepBuilderAPI_MakeEdge(curve).Edge();
    BRepBuilderAPI_MakeWire mk(e);
    if (!mk.IsDone()) {
        return false;
    }
    out = mk.Wire();
    return true;
}

// Sweep `profile` along `spine`, keeping it RADIAL by binding it to an AUXILIARY
// spine = the central Z axis line (SetMode(axisWire, contact)), so the section's
// local X always points at the axis and does NOT Frenet-rotate along the helix.
// Returns an Error on any failure (caller falls back to the round profile).
ShapeResult sweepRadialThread(const TopoDS_Wire& spine, const TopoDS_Wire& profile, double axisZ0,
                              double axisZ1) {
    BRepOffsetAPI_MakePipeShell mk(spine);
    // Auxiliary spine: the central Z axis over the helix's Z extent. Keep-contact so
    // the profile's frame is taken relative to this line at each station.
    const TopoDS_Edge axisEdge =
        BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, axisZ0), gp_Pnt(0, 0, axisZ1)).Edge();
    BRepBuilderAPI_MakeWire axisMk(axisEdge);
    if (!axisMk.IsDone()) {
        return make_error("thread: auxiliary axis spine failed");
    }
    mk.SetMode(axisMk.Wire(), Standard_True /*keep contact with the aux spine → radial*/);
    mk.Add(profile, Standard_False /*withContact*/, Standard_True /*withCorrection*/);
    if (!mk.IsReady()) {
        return make_error("thread: radial pipe not ready");
    }
    mk.Build();
    if (!mk.IsDone()) {
        return make_error("thread: radial pipe build failed");
    }
    if (!mk.MakeSolid()) {
        return make_error("thread: radial pipe make-solid failed");
    }
    return occt::addIfValid(mk.Shape(), "thread: invalid radial sweep");
}

// Shared V/triangular helical-thread builder, generalised to a TAPERED (conical)
// thread: the major radius runs from `majorTipMM` at z=0 (tip end) to `majorTopMM`
// at z=rise (head end); equal radii ⇒ the original cylindrical thread. The V
// section is placed at the tip station and swept radially along the tapering
// spine, falling back to a round profile if the radial sweep can't form a valid
// solid. Returns an Error on any failure. (Irreducible geometry — systems band.)
ShapeResult buildHelicalThread(double majorTopMM, double majorTipMM, double pitchMM, double turns,
                               double depthMM, double flankAngleDeg, double pointsPerMM,
                               int samplesPerTurn) {
    if (majorTopMM <= 0 || majorTipMM <= 0 || pitchMM <= 0 || turns <= 0 || depthMM <= 0 ||
        pointsPerMM <= 0 || flankAngleDeg <= 0 || flankAngleDeg >= 180) {
        return make_error("thread: degenerate parameters");
    }
    const double scale = pointsPerMM;
    const double depth = depthMM * scale;
    const double pitch = pitchMM * scale;
    // Pitch-line radius at each end: midway down the thread depth so the V apex
    // reaches the major radius and the root sits at majorRadius − depth.
    const double pitchRBottom = (majorTipMM - depthMM / 2.0) * scale;  // z = 0 (tip)
    const double pitchRTop = (majorTopMM - depthMM / 2.0) * scale;     // z = rise (head)
    if (pitchRBottom <= 0 || pitchRTop <= 0) {
        return make_error("thread: pitch radius non-positive");
    }

    // (Possibly tapering) helix spine through the pitch-line radii.
    TColgp_Array1OfPnt pts(1, 2);
    int n = 0;
    if (!sampleHelix(pitchRBottom, pitchRTop, pitch, turns, samplesPerTurn, pts, n) || n < 2) {
        return make_error("thread: helix sampling failed");
    }
    TopoDS_Wire spine;
    if (!helixSpineWire(pts, spine)) {
        return make_error("thread: spine wire failed");
    }
    const double z0 = pts.Value(1).Z(), z1 = pts.Value(n).Z();

    // V (isosceles triangle) profile in the plane spanned by the radial (local X,
    // → +X at the helix start, where θ=0) and the AXIS (local Y → Z). Base = pitch
    // tall along the axis so adjacent threads nearly meet; apex projects radially
    // outward by `depth`. half-base is set from the flank angle so the included
    // angle at the apex matches `flankAngleDeg` (ISO ≈ 60°).
    const gp_Pnt start = pts.Value(1);  // on +X radial, z = z0 (the tip end)
    const double halfBase =
        std::min(pitch / 2.0, depth * tan((flankAngleDeg * M_PI / 180.0) / 2.0));
    if (halfBase <= 0) {
        return make_error("thread: degenerate V half-base");
    }
    const double rOuter = start.X() + depth;  // apex radius (+X)
    // Three vertices: root-bottom, apex (outward, mid-height), root-top.
    BRepBuilderAPI_MakePolygon tri;
    tri.Add(gp_Pnt(start.X(), 0, start.Z() - halfBase));
    tri.Add(gp_Pnt(rOuter, 0, start.Z()));
    tri.Add(gp_Pnt(start.X(), 0, start.Z() + halfBase));
    tri.Close();
    if (!tri.IsDone()) {
        return make_error("thread: V profile failed");
    }
    const TopoDS_Wire vProfile = tri.Wire();

    // Sweep the V radially. If the radial sweep can't form a valid solid, fall back
    // to a round profile (orientation-safe) so we never ship a twisted body.
    ShapeResult v = sweepRadialThread(spine, vProfile, z0, z1);
    if (v.has_value()) {
        return v;
    }

    // Round-profile fallback: a circle of radius depth/2 swept along the spine
    // (BRepOffsetAPI_MakePipe is orientation-safe — no aux spine needed).
    const double rr = std::max(1.0e-3, depth / 2.0);
    const gp_Circ circ(gp_Ax2(start, gp_Dir(1, 0, 0)), rr);
    const TopoDS_Edge ce = BRepBuilderAPI_MakeEdge(circ).Edge();
    BRepBuilderAPI_MakeWire cwire(ce);
    if (!cwire.IsDone()) {
        return make_error("thread: round fallback wire failed");
    }
    BRepBuilderAPI_MakeFace cface(cwire.Wire(), Standard_True);
    if (!cface.IsDone()) {
        return make_error("thread: round fallback face failed");
    }
    BRepOffsetAPI_MakePipe pipe(spine, cface.Face());
    pipe.Build();
    if (!pipe.IsDone()) {
        return make_error("thread: round fallback pipe failed");
    }
    return occt::addIfValid(pipe.Shape(), "thread: invalid round fallback");
}

}  // namespace

// ── Legacy mesh extrude ───────────────────────────────────────────────────────

Result<MeshData> OcctEngine::extrude_mesh(const double* profileXY, int pointCount, double depth) {
    return occt::occtGuard([&]() -> Result<MeshData> {
        if (pointCount < 3 || profileXY == nullptr) {
            return make_error("extrude_mesh: need >= 3 profile points");
        }
        // Profile wire (z=0) → planar face → prism along +Z, then tessellate at 0.1.
        TopoDS_Shape solid;
        if (!buildExtrusion(profileXY, pointCount, depth, solid)) {
            return make_error("extrude_mesh: degenerate profile/height or invalid solid");
        }
        return occt::tessellateShape(solid, 0.1);
    });
}

// ── Extrusions ────────────────────────────────────────────────────────────────

ShapeResult OcctEngine::solid_extrude(const double* profileXY, int pointCount, double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        TopoDS_Shape solid;
        if (!buildExtrusion(profileXY, pointCount, depth, solid)) {
            return make_error("solid_extrude: degenerate profile/height or invalid solid");
        }
        return occt::addIfValid(solid, "solid_extrude: invalid solid");
    });
}

ShapeResult OcctEngine::solid_extrude_holes(const double* outerXY, int outerCount,
                                            const double* holesCenterRadius, int holeCount,
                                            double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        TopoDS_Shape solid;
        if (!buildExtrusionWithHoles(outerXY, outerCount, holesCenterRadius, holeCount, depth,
                                     solid)) {
            return make_error("solid_extrude_holes: degenerate input or invalid solid");
        }
        return occt::addIfValid(solid, "solid_extrude_holes: invalid solid");
    });
}

ShapeResult OcctEngine::solid_extrude_polyholes(const double* outerXY, int outerCount,
                                                const double* holesXY, const int* holeCounts,
                                                int holeCount, double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        TopoDS_Shape solid;
        if (!buildExtrusionWithPolyHoles(outerXY, outerCount, holesXY, holeCounts, holeCount, depth,
                                         solid)) {
            return make_error("solid_extrude_polyholes: degenerate input or invalid solid");
        }
        return occt::addIfValid(solid, "solid_extrude_polyholes: invalid solid");
    });
}

ShapeResult OcctEngine::solid_extrude_profile(const ProfileSeg* segs, int segCount,
                                              const double* holesCenterRadius, int holeCount,
                                              const double* splineXY, int splineXYCount,
                                              double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (!(depth > 1.0e-6)) {
            return make_error("solid_extrude_profile: degenerate height");
        }
        TopoDS_Wire outer;
        if (!buildTypedWire(segs, segCount, outer, splineXY, splineXYCount)) {
            return make_error("solid_extrude_profile: invalid outer wire");
        }
        BRepBuilderAPI_MakeFace mf(outer, Standard_True);
        if (!mf.IsDone()) {
            return make_error("solid_extrude_profile: outer face failed");
        }
        for (int i = 0; i < holeCount; ++i) {
            const double cx = holesCenterRadius[i * 3], cy = holesCenterRadius[i * 3 + 1],
                         r = holesCenterRadius[i * 3 + 2];
            if (!(r > 1.0e-6)) {
                continue;
            }
            const gp_Circ circ(gp_Ax2(gp_Pnt(cx, cy, 0.0), gp_Dir(0.0, 0.0, 1.0)), r);
            const TopoDS_Wire w = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(circ).Edge());
            mf.Add(TopoDS::Wire(w.Reversed()));
        }
        if (!mf.IsDone()) {
            return make_error("solid_extrude_profile: face with holes failed");
        }
        BRepPrimAPI_MakePrism prism(mf.Face(), gp_Vec(0.0, 0.0, depth));
        prism.Build();
        if (!prism.IsDone()) {
            return make_error("solid_extrude_profile: prism failed");
        }
        return occt::addIfValid(prism.Shape(), "solid_extrude_profile: invalid solid");
    });
}

ShapeResult OcctEngine::solid_extrude_profile_polyholes(
    const ProfileSeg* segs, int segCount, const double* holesCenterRadius, int circleCount,
    const double* polyXY, const int* polyCounts, int polyCount, const double* splineXY,
    int splineXYCount, double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (!(depth > 1.0e-6)) {
            return make_error("solid_extrude_profile_polyholes: degenerate height");
        }
        TopoDS_Wire outer;
        // TRUE arcs/lines/splines → clean outer rim edges.
        if (!buildTypedWire(segs, segCount, outer, splineXY, splineXYCount)) {
            return make_error("solid_extrude_profile_polyholes: invalid outer wire");
        }
        BRepBuilderAPI_MakeFace mf(outer, Standard_True);
        if (!mf.IsDone()) {
            return make_error("solid_extrude_profile_polyholes: outer face failed");
        }
        for (int i = 0; i < circleCount; ++i) {  // round holes stay TRUE circles
            const double cx = holesCenterRadius[i * 3], cy = holesCenterRadius[i * 3 + 1],
                         r = holesCenterRadius[i * 3 + 2];
            if (!(r > 1.0e-6)) {
                continue;
            }
            const gp_Circ circ(gp_Ax2(gp_Pnt(cx, cy, 0.0), gp_Dir(0.0, 0.0, 1.0)), r);
            mf.Add(TopoDS::Wire(
                BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(circ).Edge()).Wire().Reversed()));
        }
        int offset = 0;  // arbitrary holes as polygon wires
        for (int i = 0; i < polyCount; ++i) {
            const int n = polyCounts[i];
            if (n >= 3) {
                BRepBuilderAPI_MakePolygon poly;
                for (int j = 0; j < n; ++j) {
                    poly.Add(gp_Pnt(polyXY[(offset + j) * 2], polyXY[(offset + j) * 2 + 1], 0.0));
                }
                poly.Close();
                if (poly.IsDone()) {
                    mf.Add(TopoDS::Wire(poly.Wire().Reversed()));
                }
            }
            offset += n;
        }
        if (!mf.IsDone()) {
            return make_error("solid_extrude_profile_polyholes: face with holes failed");
        }
        BRepPrimAPI_MakePrism prism(mf.Face(), gp_Vec(0.0, 0.0, depth));
        prism.Build();
        if (!prism.IsDone()) {
            return make_error("solid_extrude_profile_polyholes: prism failed");
        }
        return occt::addIfValid(prism.Shape(), "solid_extrude_profile_polyholes: invalid solid");
    });
}

// ── Revolutions ───────────────────────────────────────────────────────────────

ShapeResult OcctEngine::solid_revolve_profile(const ProfileSeg* segs, int segCount, double ax,
                                              double ay, double adx, double ady,
                                              const double* splineXY, int splineXYCount,
                                              double angleRadians) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (!(angleRadians > 1.0e-6) || !(adx * adx + ady * ady > 1.0e-18)) {
            return make_error("solid_revolve_profile: degenerate angle/axis");
        }
        TopoDS_Wire outer;
        // TRUE arcs/lines/splines → a smooth revolved face.
        if (!buildTypedWire(segs, segCount, outer, splineXY, splineXYCount)) {
            return make_error("solid_revolve_profile: invalid outer wire");
        }
        BRepBuilderAPI_MakeFace mf(outer, Standard_True);
        if (!mf.IsDone()) {
            return make_error("solid_revolve_profile: face failed");
        }
        const gp_Ax1 axis(gp_Pnt(ax, ay, 0.0),
                          gp_Dir(adx, ady, 0.0));  // axis in the profile's plane (z=0)
        BRepPrimAPI_MakeRevol revol(mf.Face(), axis, angleRadians);
        revol.Build();
        if (!revol.IsDone()) {
            return make_error("solid_revolve_profile: revolve failed");
        }
        return occt::addIfValid(revol.Shape(), "solid_revolve_profile: invalid solid");
    });
}

ShapeResult OcctEngine::solid_revolve(const double* profileXY, int pointCount, double angleRadians) {
    return occt::occtGuard([&]() -> ShapeResult {
        TopoDS_Shape solid;
        if (!buildRevolution(profileXY, pointCount, angleRadians, solid)) {
            return make_error("solid_revolve: degenerate profile/angle or invalid solid");
        }
        return occt::addIfValid(solid, "solid_revolve: invalid solid");
    });
}

// ── Lofts ─────────────────────────────────────────────────────────────────────

ShapeResult OcctEngine::solid_loft(const double* bottomXY, int bottomCount, const double* topXY,
                                   int topCount, double depth) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (bottomXY == nullptr || topXY == nullptr || bottomCount < 3 || topCount < 3 ||
            !(depth > 1.0e-6)) {
            return make_error("solid_loft: degenerate input");
        }
        BRepBuilderAPI_MakePolygon bottom;
        for (int i = 0; i < bottomCount; ++i) {
            bottom.Add(gp_Pnt(bottomXY[i * 2], bottomXY[i * 2 + 1], 0.0));
        }
        bottom.Close();
        BRepBuilderAPI_MakePolygon top;
        for (int i = 0; i < topCount; ++i) {
            top.Add(gp_Pnt(topXY[i * 2], topXY[i * 2 + 1], depth));
        }
        top.Close();
        if (!bottom.IsDone() || !top.IsDone()) {
            return make_error("solid_loft: section wire failed");
        }
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        gen.AddWire(bottom.Wire());
        gen.AddWire(top.Wire());
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("solid_loft: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "solid_loft: invalid solid");
    });
}

ShapeResult OcctEngine::solid_loft_wires(const double* aXYZ, int aCount, const double* bXYZ,
                                         int bCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (aXYZ == nullptr || bXYZ == nullptr || aCount < 3 || bCount < 3) {
            return make_error("solid_loft_wires: degenerate input");
        }
        BRepBuilderAPI_MakePolygon a;
        for (int i = 0; i < aCount; ++i) {
            a.Add(gp_Pnt(aXYZ[i * 3], aXYZ[i * 3 + 1], aXYZ[i * 3 + 2]));
        }
        a.Close();
        BRepBuilderAPI_MakePolygon b;
        for (int i = 0; i < bCount; ++i) {
            b.Add(gp_Pnt(bXYZ[i * 3], bXYZ[i * 3 + 1], bXYZ[i * 3 + 2]));
        }
        b.Close();
        if (!a.IsDone() || !b.IsDone()) {
            return make_error("solid_loft_wires: section wire failed");
        }
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        gen.AddWire(a.Wire());
        gen.AddWire(b.Wire());
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("solid_loft_wires: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "solid_loft_wires: invalid solid");
    });
}

// N-section ruled loft (≥3 sections; the generalisation of solid_loft_wires). Each
// section is a closed polygon from its slice of the flat (x,y,z) buffer; every
// section is added to one BRepOffsetAPI_ThruSections (solid, ruled) in order → the
// same construction the 2-section loft uses, just through more wires. This is the
// OCCT oracle the native N-section builder (loft.h build_loft_sections) verifies
// against; on a native decline the facade forwards the SAME arguments here.
ShapeResult OcctEngine::solid_loft_sections(const double* sectionsXYZ, const int* counts,
                                            int sectionCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (sectionsXYZ == nullptr || counts == nullptr || sectionCount < 2) {
            return make_error("solid_loft_sections: degenerate input");
        }
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        std::size_t off = 0;  // running offset into the flat (x,y,z) buffer, in doubles
        for (int k = 0; k < sectionCount; ++k) {
            const int cnt = counts[k];
            if (cnt < 3) return make_error("solid_loft_sections: section with < 3 points");
            BRepBuilderAPI_MakePolygon poly;
            for (int i = 0; i < cnt; ++i) {
                poly.Add(gp_Pnt(sectionsXYZ[off + i * 3], sectionsXYZ[off + i * 3 + 1],
                                sectionsXYZ[off + i * 3 + 2]));
            }
            poly.Close();
            if (!poly.IsDone()) return make_error("solid_loft_sections: section wire failed");
            gen.AddWire(poly.Wire());
            off += static_cast<std::size_t>(cnt) * 3;
        }
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("solid_loft_sections: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "solid_loft_sections: invalid solid");
    });
}

ShapeResult OcctEngine::loft_along_rail(const double* railXYZ, int railCount,
                                        const double* profileA_XY, int aCount,
                                        const double* profileB_XY, int bCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        // Hard input guards (OCCT crash discipline): null / too-few points.
        if (railXYZ == nullptr || profileA_XY == nullptr || profileB_XY == nullptr) {
            return make_error("loft_along_rail: null input");
        }
        if (railCount < 2 || aCount < 3 || bCount < 3) {
            return make_error("loft_along_rail: too few points");
        }
        // Spine from the rail + its end tangents (degenerate rail → bail).
        TopoDS_Wire spine;
        gp_Pnt startPt, endPt;
        gp_Vec startTan, endTan;
        if (!buildSpineWire(railXYZ, railCount, spine, startPt, startTan, endPt, endTan)) {
            return make_error("loft_along_rail: degenerate rail");
        }
        // Orient each section in the plane perpendicular to the rail at its end.
        gp_Vec uA, vA, uB, vB;
        if (!perpendicularFrame(startTan, uA, vA) || !perpendicularFrame(endTan, uB, vB)) {
            return make_error("loft_along_rail: degenerate rail tangent");
        }
        TopoDS_Wire wireA, wireB;
        if (!buildRailSectionWire(profileA_XY, aCount, startPt, uA, vA, wireA)) {
            return make_error("loft_along_rail: section A wire failed");
        }
        if (!buildRailSectionWire(profileB_XY, bCount, endPt, uB, vB, wireB)) {
            return make_error("loft_along_rail: section B wire failed");
        }
        // Sweep the morph between the two sections along the spine.
        BRepOffsetAPI_MakePipeShell mk(spine);
        mk.SetTransitionMode(BRepBuilderAPI_RoundCorner);  // round at rail kinks (right-angle bend)
        mk.Add(wireA, Standard_False /*withContact*/, Standard_True /*withCorrection*/);
        mk.Add(wireB, Standard_False, Standard_True);
        if (!mk.IsReady()) {
            return make_error("loft_along_rail: pipe shell not ready");
        }
        mk.Build();
        if (!mk.IsDone()) {
            return make_error("loft_along_rail: pipe shell build failed");
        }
        if (!mk.MakeSolid()) {
            return make_error("loft_along_rail: make-solid failed");
        }
        return occt::addIfValid(mk.Shape(), "loft_along_rail: invalid solid");
    });
}

// ── App-parity loft variants (ADDITIVE) ────────────────────────────────────────
// These MATCH the app's reference KernelBridge.mm construction exactly so that, during
// the OCCT-transition, the kernel is behaviourally identical to the app's inline bridge.

// Loft between two TRUE circles → a smooth conical/cylindrical B-rep (one side face, two
// circular edges), not a faceted polygon.
ShapeResult OcctEngine::loft_circles(const double* c1, const double* n1, double r1,
                                     const double* c2, const double* n2, double r2) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (c1 == nullptr || n1 == nullptr || c2 == nullptr || n2 == nullptr || !(r1 > 1.0e-6) ||
            !(r2 > 1.0e-6)) {
            return make_error("loft_circles: degenerate input");
        }
        const gp_Ax2 a1(gp_Pnt(c1[0], c1[1], c1[2]), gp_Dir(n1[0], n1[1], n1[2]));
        const gp_Ax2 a2(gp_Pnt(c2[0], c2[1], c2[2]), gp_Dir(n2[0], n2[1], n2[2]));
        const TopoDS_Wire w1 = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(a1, r1)));
        const TopoDS_Wire w2 = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(a2, r2)));
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        gen.AddWire(w1);
        gen.AddWire(w2);
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("loft_circles: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "loft_circles: invalid solid");
    });
}

// Loft a TRUE circle section to an arbitrary polygon wire — smooth circle rim, exact polygon.
ShapeResult OcctEngine::loft_circle_wire(const double* cc, const double* cn, double cr,
                                         const double* wXYZ, int wCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (cc == nullptr || cn == nullptr || wXYZ == nullptr || !(cr > 1.0e-6) || wCount < 3) {
            return make_error("loft_circle_wire: degenerate input");
        }
        const gp_Ax2 ax(gp_Pnt(cc[0], cc[1], cc[2]), gp_Dir(cn[0], cn[1], cn[2]));
        const TopoDS_Wire circle = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(ax, cr)));
        BRepBuilderAPI_MakePolygon poly;
        for (int i = 0; i < wCount; ++i) {
            poly.Add(gp_Pnt(wXYZ[i * 3], wXYZ[i * 3 + 1], wXYZ[i * 3 + 2]));
        }
        poly.Close();
        if (!poly.IsDone()) {
            return make_error("loft_circle_wire: polygon wire failed");
        }
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        gen.AddWire(circle);
        gen.AddWire(poly.Wire());
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("loft_circle_wire: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "loft_circle_wire: invalid solid");
    });
}

// Two-rail loft: the rail is the spine and the guide steers as an auxiliary spine; retry
// without the guide if the guided build fails, mirroring the app's two-pass loop.
ShapeResult OcctEngine::loft_along_rails(const double* railXYZ, int railCount,
                                         const double* guideXYZ, int guideCount,
                                         const double* profileA_XY, int aCount,
                                         const double* profileB_XY, int bCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (railXYZ == nullptr || profileA_XY == nullptr || profileB_XY == nullptr) {
            return make_error("loft_along_rails: null input");
        }
        if (railCount < 2 || aCount < 3 || bCount < 3) {
            return make_error("loft_along_rails: too few points");
        }
        TopoDS_Wire spine;
        gp_Pnt startPt, endPt;
        gp_Vec startTan, endTan;
        if (!buildSpineWire(railXYZ, railCount, spine, startPt, startTan, endPt, endTan)) {
            return make_error("loft_along_rails: degenerate rail");
        }
        gp_Vec uA, vA, uB, vB;
        if (!perpendicularFrame(startTan, uA, vA) || !perpendicularFrame(endTan, uB, vB)) {
            return make_error("loft_along_rails: degenerate rail tangent");
        }
        TopoDS_Wire wireA, wireB;
        if (!buildRailSectionWire(profileA_XY, aCount, startPt, uA, vA, wireA)) {
            return make_error("loft_along_rails: section A wire failed");
        }
        if (!buildRailSectionWire(profileB_XY, bCount, endPt, uB, vB, wireB)) {
            return make_error("loft_along_rails: section B wire failed");
        }
        // Build a guide wire; add it as an auxiliary spine when it is valid.
        TopoDS_Wire guide;
        bool haveGuide = false;
        if (guideXYZ != nullptr && guideCount >= 2) {
            gp_Pnt gs, ge;
            gp_Vec gts, gte;
            haveGuide = buildSpineWire(guideXYZ, guideCount, guide, gs, gts, ge, gte);
        }
        // Try the two-rail (guided) sweep first; on failure retry without the guide.
        for (int pass = 0; pass < 2; ++pass) {
            BRepOffsetAPI_MakePipeShell mk(spine);
            mk.SetTransitionMode(BRepBuilderAPI_RoundCorner);
            if (haveGuide && pass == 0) {
                mk.SetMode(guide, Standard_True /*CurvilinearEquivalence*/);
            }
            mk.Add(wireA, Standard_False /*withContact*/, Standard_True /*withCorrection*/);
            mk.Add(wireB, Standard_False, Standard_True);
            if (mk.IsReady()) {
                mk.Build();
                if (mk.IsDone() && mk.MakeSolid()) {
                    ShapeResult id = occt::addIfValid(mk.Shape(), "loft_along_rails: invalid solid");
                    if (id) {
                        return id;
                    }
                }
            }
            if (!haveGuide) {
                break;  // no guide → nothing to retry
            }
        }
        return make_error("loft_along_rails: pipe shell build failed");
    });
}

// Loft between two TYPED section profiles, each placed on its own plane frame.
ShapeResult OcctEngine::loft_typed(const ProfileSeg* segsA, int countA, const double* splineA,
                                   int splineACount, const double* frameA, const ProfileSeg* segsB,
                                   int countB, const double* splineB, int splineBCount,
                                   const double* frameB) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (frameA == nullptr || frameB == nullptr) {
            return make_error("loft_typed: null frame");
        }
        TopoDS_Wire wa, wb;
        if (!buildTypedWire(segsA, countA, wa, splineA, splineACount)) {
            return make_error("loft_typed: section A wire failed");
        }
        if (!buildTypedWire(segsB, countB, wb, splineB, splineBCount)) {
            return make_error("loft_typed: section B wire failed");
        }
        if (!placeWireOnFrame(wa, frameA) || !placeWireOnFrame(wb, frameB)) {
            return make_error("loft_typed: degenerate plane frame");
        }
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        gen.AddWire(wa);
        gen.AddWire(wb);
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("loft_typed: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "loft_typed: invalid solid");
    });
}

// ── Sweeps ────────────────────────────────────────────────────────────────────

ShapeResult OcctEngine::solid_sweep(const double* profileXY, int profileCount,
                                    const double* pathXYZ, int pathCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (profileXY == nullptr || pathXYZ == nullptr || profileCount < 3 || pathCount < 2) {
            return make_error("solid_sweep: degenerate input");
        }
        // Profile centroid (the profile is placed centered on the path start).
        double cx = 0, cy = 0;
        for (int i = 0; i < profileCount; ++i) {
            cx += profileXY[i * 2];
            cy += profileXY[i * 2 + 1];
        }
        cx /= profileCount;
        cy /= profileCount;
        const gp_Pnt p0(pathXYZ[0], pathXYZ[1], pathXYZ[2]);
        const gp_Pnt p1(pathXYZ[3], pathXYZ[4], pathXYZ[5]);
        // Start tangent and a perpendicular frame.
        gp_Vec tan(p0, p1);
        if (tan.Magnitude() < 1.0e-9) {
            return make_error("solid_sweep: degenerate start tangent");
        }
        tan.Normalize();
        // Pick a reference axis NOT parallel to the tangent so the cross products never
        // collapse — a hardcoded +Y "up" degenerates when the path runs vertically. For
        // a horizontal tangent this reproduces (nrm, up) = (cross(tan,Y), Y).
        const gp_Vec ref = (fabs(tan.Y()) > 0.9) ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
        gp_Vec nrm = tan.Crossed(ref);
        if (nrm.Magnitude() < 1.0e-6) {
            nrm = gp_Vec(1, 0, 0);
        } else {
            nrm.Normalize();
        }
        gp_Vec up = nrm.Crossed(tan);
        up.Normalize();
        // Build the profile wire in that start frame.
        BRepBuilderAPI_MakePolygon poly;
        for (int i = 0; i < profileCount; ++i) {
            const double u = profileXY[i * 2] - cx, v = profileXY[i * 2 + 1] - cy;
            poly.Add(gp_Pnt(p0.X() + u * nrm.X() + v * up.X(), p0.Y() + u * nrm.Y() + v * up.Y(),
                            p0.Z() + u * nrm.Z() + v * up.Z()));
        }
        poly.Close();
        if (!poly.IsDone()) {
            return make_error("solid_sweep: profile wire failed");
        }
        BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
        if (!face.IsDone()) {
            return make_error("solid_sweep: profile face failed");
        }
        // Spine polyline through the path points.
        BRepBuilderAPI_MakePolygon spine;
        for (int i = 0; i < pathCount; ++i) {
            spine.Add(gp_Pnt(pathXYZ[i * 3], pathXYZ[i * 3 + 1], pathXYZ[i * 3 + 2]));
        }
        if (!spine.IsDone()) {
            return make_error("solid_sweep: spine wire failed");
        }
        // Sweep the FACE so the result is a capped solid (a wire would give a shell).
        BRepOffsetAPI_MakePipe pipe(spine.Wire(), face.Face());
        pipe.Build();
        if (!pipe.IsDone()) {
            return make_error("solid_sweep: pipe failed");
        }
        return occt::addIfValid(pipe.Shape(), "solid_sweep: invalid solid");
    });
}

ShapeResult OcctEngine::twisted_sweep(const double* profileXY, int profileCount,
                                      const double* pathXYZ, int pathCount, double twistRadians,
                                      double scaleEnd) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (profileXY == nullptr || pathXYZ == nullptr || profileCount < 3 || pathCount < 2) {
            return make_error("twisted_sweep: degenerate input");
        }
        double cx = 0, cy = 0;
        for (int i = 0; i < profileCount; ++i) {
            cx += profileXY[i * 2];
            cy += profileXY[i * 2 + 1];
        }
        cx /= profileCount;
        cy /= profileCount;
        const gp_Vec up(0, 1, 0);
        // Loft through a rotated+scaled copy of the profile at every path station so the
        // section twists along the path. ThruSections connects vertex k → vertex k
        // between consecutive (closed) wires, so the twist must be gradual (it is).
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        int added = 0;
        for (int i = 0; i < pathCount; ++i) {
            const gp_Pnt c(pathXYZ[i * 3], pathXYZ[i * 3 + 1], pathXYZ[i * 3 + 2]);
            const int a = (i > 0) ? i - 1 : i;
            const int b = (i < pathCount - 1) ? i + 1 : i;  // local (neighbour) tangent
            gp_Vec tan(gp_Pnt(pathXYZ[a * 3], pathXYZ[a * 3 + 1], pathXYZ[a * 3 + 2]),
                       gp_Pnt(pathXYZ[b * 3], pathXYZ[b * 3 + 1], pathXYZ[b * 3 + 2]));
            if (tan.Magnitude() < 1.0e-9) {
                continue;
            }
            tan.Normalize();
            gp_Vec nrm = tan.Crossed(up);
            if (nrm.Magnitude() < 1.0e-6) {
                nrm = gp_Vec(1, 0, 0);
            } else {
                nrm.Normalize();
            }
            const double f = static_cast<double>(i) / static_cast<double>(pathCount - 1);
            const double ang = twistRadians * f;
            const double sc = 1.0 + (scaleEnd - 1.0) * f;
            const double ca = std::cos(ang), sa = std::sin(ang);
            BRepBuilderAPI_MakePolygon poly;
            for (int k = 0; k < profileCount; ++k) {
                const double u0 = (profileXY[k * 2] - cx) * sc, v0 = (profileXY[k * 2 + 1] - cy) * sc;
                const double u = u0 * ca - v0 * sa,
                             v = u0 * sa + v0 * ca;  // rotate the section about the tangent
                poly.Add(gp_Pnt(c.X() + u * nrm.X() + v * up.X(), c.Y() + u * nrm.Y() + v * up.Y(),
                                c.Z() + u * nrm.Z() + v * up.Z()));
            }
            poly.Close();
            if (!poly.IsDone()) {
                continue;
            }
            gen.AddWire(poly.Wire());
            ++added;
        }
        if (added < 2) {
            return make_error("twisted_sweep: fewer than 2 valid sections");
        }
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("twisted_sweep: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "twisted_sweep: invalid solid");
    });
}

ShapeResult OcctEngine::guided_sweep(const double* profileXY, int profileCount,
                                     const double* pathXYZ, int pathCount, const double* guideXYZ,
                                     int guideCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (profileXY == nullptr || pathXYZ == nullptr || guideXYZ == nullptr) {
            return make_error("guided_sweep: null input");
        }
        if (profileCount < 3 || pathCount < 2 || guideCount < 2) {
            return make_error("guided_sweep: too few points");
        }
        double cx = 0, cy = 0;
        for (int i = 0; i < profileCount; ++i) {
            cx += profileXY[i * 2];
            cy += profileXY[i * 2 + 1];
        }
        cx /= profileCount;
        cy /= profileCount;
        const gp_Vec up(0, 1, 0);
        // The guide sampled at parameter fraction f (its vertices treated as evenly
        // spaced in parameter — exact for a 2-point guide).
        auto guideAt = [&](double f) -> gp_Pnt {
            double s = f * (guideCount - 1);
            int i0 = std::min(static_cast<int>(std::floor(s)), guideCount - 2);
            const double t = s - i0;
            const gp_Pnt a(guideXYZ[i0 * 3], guideXYZ[i0 * 3 + 1], guideXYZ[i0 * 3 + 2]);
            const gp_Pnt b(guideXYZ[(i0 + 1) * 3], guideXYZ[(i0 + 1) * 3 + 1],
                           guideXYZ[(i0 + 1) * 3 + 2]);
            return gp_Pnt(a.X() + (b.X() - a.X()) * t, a.Y() + (b.Y() - a.Y()) * t,
                          a.Z() + (b.Z() - a.Z()) * t);
        };
        // The guide's distance from the path at the start sets the reference width; each
        // station scales by how far the guide has splayed from the path there.
        const gp_Pnt p0(pathXYZ[0], pathXYZ[1], pathXYZ[2]);
        const double d0 = p0.Distance(guideAt(0.0));
        if (d0 < 1.0e-6) {
            return make_error("guided_sweep: guide coincident with path start");
        }
        // Loft through the guide-scaled section at every path station.
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        int added = 0;
        for (int i = 0; i < pathCount; ++i) {
            const gp_Pnt c(pathXYZ[i * 3], pathXYZ[i * 3 + 1], pathXYZ[i * 3 + 2]);
            const int a = (i > 0) ? i - 1 : i;
            const int b = (i < pathCount - 1) ? i + 1 : i;
            gp_Vec tan(gp_Pnt(pathXYZ[a * 3], pathXYZ[a * 3 + 1], pathXYZ[a * 3 + 2]),
                       gp_Pnt(pathXYZ[b * 3], pathXYZ[b * 3 + 1], pathXYZ[b * 3 + 2]));
            if (tan.Magnitude() < 1.0e-9) {
                continue;
            }
            tan.Normalize();
            gp_Vec nrm = tan.Crossed(up);
            if (nrm.Magnitude() < 1.0e-6) {
                nrm = gp_Vec(1, 0, 0);
            } else {
                nrm.Normalize();
            }
            const double f = static_cast<double>(i) / static_cast<double>(pathCount - 1);
            const double sc = c.Distance(guideAt(f)) / d0;  // guide-driven section scale
            BRepBuilderAPI_MakePolygon poly;
            for (int k = 0; k < profileCount; ++k) {
                const double u = (profileXY[k * 2] - cx) * sc, v = (profileXY[k * 2 + 1] - cy) * sc;
                poly.Add(gp_Pnt(c.X() + u * nrm.X() + v * up.X(), c.Y() + u * nrm.Y() + v * up.Y(),
                                c.Z() + u * nrm.Z() + v * up.Z()));
            }
            poly.Close();
            if (!poly.IsDone()) {
                continue;
            }
            gen.AddWire(poly.Wire());
            ++added;
        }
        if (added < 2) {
            return make_error("guided_sweep: fewer than 2 valid sections");
        }
        gen.Build();
        if (!gen.IsDone()) {
            return make_error("guided_sweep: thru-sections failed");
        }
        return occt::addIfValid(gen.Shape(), "guided_sweep: invalid solid");
    });
}

// ── Guide-ORIENTED sweep — the ORACLE for cc_guided_orient_sweep ───────────────
// Section ORIENTATION (not scale) fixed by a guide wire: BRepOffsetAPI_MakePipeShell
// with SetMode(guide, CurvilinearEquivalence=false, KeepContact=NoContact) — the
// default plane-trihedron law GeomFill_GuideTrihedronPlan (rigid per-station frame
// [N,B,T], N from the guide point in the plane perpendicular to the spine tangent). The
// profile is placed at the spine start in the frame the guide implies there; Add() with
// WithCorrection reframes each station to the guide trihedron. This is the independent
// oracle the native straight-spine builder is verified against on volume AND bbox.
ShapeResult OcctEngine::guided_orient_sweep(const double* profileXY, int profileCount,
                                            const double* pathXYZ, int pathCount,
                                            const double* guideXYZ, int guideCount) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (profileXY == nullptr || pathXYZ == nullptr || guideXYZ == nullptr) {
            return make_error("guided_orient_sweep: null input");
        }
        if (profileCount < 3 || pathCount < 2 || guideCount < 2) {
            return make_error("guided_orient_sweep: too few points");
        }
        // Spine wire (the path polyline).
        BRepBuilderAPI_MakePolygon spineP;
        for (int i = 0; i < pathCount; ++i)
            spineP.Add(gp_Pnt(pathXYZ[i * 3], pathXYZ[i * 3 + 1], pathXYZ[i * 3 + 2]));
        if (!spineP.IsDone()) return make_error("guided_orient_sweep: spine wire failed");
        const TopoDS_Wire spine = spineP.Wire();
        // Guide wire (the orientation-steering polyline).
        BRepBuilderAPI_MakePolygon guideP;
        for (int i = 0; i < guideCount; ++i)
            guideP.Add(gp_Pnt(guideXYZ[i * 3], guideXYZ[i * 3 + 1], guideXYZ[i * 3 + 2]));
        if (!guideP.IsDone()) return make_error("guided_orient_sweep: guide wire failed");
        const TopoDS_Wire guide = guideP.Wire();

        // Start frame implied by the guide: T = spine tangent at start; N0 = the guide
        // point in the plane through P0 perpendicular to T, minus P0 (perpendicular-plane
        // correspondence); B0 = T × N0. The profile is placed at P0 in (N0, B0).
        const gp_Pnt p0(pathXYZ[0], pathXYZ[1], pathXYZ[2]);
        gp_Vec T(p0, gp_Pnt(pathXYZ[(pathCount - 1) * 3], pathXYZ[(pathCount - 1) * 3 + 1],
                            pathXYZ[(pathCount - 1) * 3 + 2]));
        if (T.Magnitude() < 1.0e-9) return make_error("guided_orient_sweep: degenerate spine");
        T.Normalize();
        const double target = p0.XYZ().Dot(T.XYZ());
        gp_Pnt pprime;
        bool found = false;
        for (int i = 0; i + 1 < guideCount && !found; ++i) {
            const gp_Pnt ga(guideXYZ[i * 3], guideXYZ[i * 3 + 1], guideXYZ[i * 3 + 2]);
            const gp_Pnt gb(guideXYZ[(i + 1) * 3], guideXYZ[(i + 1) * 3 + 1],
                            guideXYZ[(i + 1) * 3 + 2]);
            const double da = ga.XYZ().Dot(T.XYZ()), db = gb.XYZ().Dot(T.XYZ());
            const double lo = std::min(da, db), hi = std::max(da, db);
            if (target < lo - 1.0e-9 || target > hi + 1.0e-9) continue;
            const double denom = db - da;
            const double u = std::fabs(denom) > 1.0e-12 ? (target - da) / denom : 0.0;
            const double uc = std::min(1.0, std::max(0.0, u));
            pprime = gp_Pnt(ga.X() + (gb.X() - ga.X()) * uc, ga.Y() + (gb.Y() - ga.Y()) * uc,
                            ga.Z() + (gb.Z() - ga.Z()) * uc);
            found = true;
        }
        if (!found) return make_error("guided_orient_sweep: guide misses spine start plane");
        gp_Vec N0(p0, pprime);
        if (N0.Magnitude() < 1.0e-6) return make_error("guided_orient_sweep: degenerate guide N");
        N0.Normalize();
        const gp_Vec B0 = T.Crossed(N0);

        double cx = 0, cy = 0;
        for (int i = 0; i < profileCount; ++i) { cx += profileXY[i * 2]; cy += profileXY[i * 2 + 1]; }
        cx /= profileCount;
        cy /= profileCount;
        BRepBuilderAPI_MakePolygon prof;
        for (int i = 0; i < profileCount; ++i) {
            const double u = profileXY[i * 2] - cx, v = profileXY[i * 2 + 1] - cy;
            prof.Add(gp_Pnt(p0.X() + u * N0.X() + v * B0.X(), p0.Y() + u * N0.Y() + v * B0.Y(),
                            p0.Z() + u * N0.Z() + v * B0.Z()));
        }
        prof.Close();
        if (!prof.IsDone()) return make_error("guided_orient_sweep: profile wire failed");

        BRepOffsetAPI_MakePipeShell mk(spine);
        mk.SetMode(guide, Standard_False /*CurvilinearEquivalence*/,
                   BRepFill_NoContact /*guide plane trihedron, no rotation*/);
        mk.Add(prof.Wire(), Standard_False /*WithContact*/, Standard_True /*WithCorrection*/);
        if (!mk.IsReady()) return make_error("guided_orient_sweep: pipe shell not ready");
        mk.Build();
        if (!mk.IsDone()) return make_error("guided_orient_sweep: pipe shell build failed");
        if (!mk.MakeSolid()) return make_error("guided_orient_sweep: make-solid failed");
        return occt::addIfValid(mk.Shape(), "guided_orient_sweep: invalid solid");
    });
}

// ── Wrap / emboss (cylindrical face) ──────────────────────────────────────────
// OcctEngine::wrap_emboss is defined in its own TU, occt_wrap_emboss.cpp, so the
// Phase-3 robust sewn-pad rework can be owned there in a disjoint file.

// ── Threads / shank ───────────────────────────────────────────────────────────

ShapeResult OcctEngine::helical_thread(double majorRadiusMM, double pitchMM, double turns,
                                       double depthMM, double flankAngleDeg, double pointsPerMM,
                                       int samplesPerTurn) {
    return occt::occtGuard([&]() -> ShapeResult {
        // Cylindrical thread = a tapered thread with equal top/tip radii. Tag the
        // built shape with its turns/pitch so a later fuse/cut into a shaft can be
        // evaluated by the fine-thread gate (parallel-acceleration §"Fine-thread
        // boolean gate").
        ShapeResult r = buildHelicalThread(majorRadiusMM, majorRadiusMM, pitchMM, turns, depthMM,
                                           flankAngleDeg, pointsPerMM, samplesPerTurn);
        return occt::tagAsThread(std::move(r), turns, pitchMM);
    });
}

ShapeResult OcctEngine::tapered_thread(double topRadiusMM, double tipRadiusMM, double pitchMM,
                                       double turns, double depthMM, double flankAngleDeg,
                                       double pointsPerMM, int samplesPerTurn) {
    return occt::occtGuard([&]() -> ShapeResult {
        // A conical thread: full `topRadiusMM` near the head (z = rise) shrinking to
        // `tipRadiusMM` at the tip (z = 0). topRadiusMM == tipRadiusMM ⇒ cylindrical.
        // Tagged with turns/pitch for the fine-thread gate, as with helical_thread.
        ShapeResult r = buildHelicalThread(topRadiusMM, tipRadiusMM, pitchMM, turns, depthMM,
                                           flankAngleDeg, pointsPerMM, samplesPerTurn);
        return occt::tagAsThread(std::move(r), turns, pitchMM);
    });
}

ShapeResult OcctEngine::tapered_shank(double radiusMM, double fullHeightMM, double taperHeightMM,
                                      double pointsPerMM) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (radiusMM <= 0 || fullHeightMM <= 0 || taperHeightMM <= 0 || pointsPerMM <= 0) {
            return make_error("tapered_shank: degenerate parameters");
        }
        const double r = radiusMM * pointsPerMM;
        const double zTaper = taperHeightMM * pointsPerMM;  // tip → taper-start
        const double zTop = (taperHeightMM + fullHeightMM) * pointsPerMM;
        const double tipR = std::max(1.0e-3, r * 0.02);  // ≈ a sharp point (not 0 → valid revolve)
        // Silhouette in the X–Z plane (X = radius from the Z axis, revolved about Z):
        //   tip (0,0) → taper-start (r, zTaper) → top-outer (r, zTop) → top-axis (0, zTop).
        // The axis edge (X=0) closes the profile on the rotation axis.
        BRepBuilderAPI_MakePolygon poly;
        poly.Add(gp_Pnt(tipR, 0, 0));      // the point (tip), z = 0
        poly.Add(gp_Pnt(r, 0, zTaper));    // taper start: full radius
        poly.Add(gp_Pnt(r, 0, zTop));      // top outer: full radius (head end)
        poly.Add(gp_Pnt(0, 0, zTop));      // top, on the axis
        poly.Add(gp_Pnt(0, 0, 0));         // back down the axis to the tip
        poly.Close();
        if (!poly.IsDone()) {
            return make_error("tapered_shank: silhouette wire failed");
        }
        BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
        if (!face.IsDone()) {
            return make_error("tapered_shank: silhouette face failed");
        }
        const gp_Ax1 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));  // revolve about Z
        BRepPrimAPI_MakeRevol revol(face.Face(), axis, 2.0 * M_PI);
        revol.Build();
        if (!revol.IsDone()) {
            return make_error("tapered_shank: revolve failed");
        }
        return occt::addIfValid(revol.Shape(), "tapered_shank: invalid solid");
    });
}

}  // namespace cyber
