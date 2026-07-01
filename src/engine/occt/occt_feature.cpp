// OCCT engine adapter — FEATURE capability group.
//
// Feature edits on an existing solid: fillet / chamfer edges, shell (hollow),
// planar-face offset / replace / retarget, whole-face fillet, planar split, and
// the outer-loop offset boundary polyline. These are the OcctEngine methods
// declared in occt_engine.h and are DEFINED here, ported one-for-one from the
// app's KernelBridge.mm (the CYBERCAD_HAS_OCCT paths of cc_fillet_edges,
// cc_fillet_edges_variable, cc_chamfer_edges, cc_shell, cc_offset_face,
// cc_replace_face, cc_replace_face_to_plane, cc_fillet_face, cc_split_plane,
// cc_offset_face_boundary). Behaviour is preserved exactly: the same degenerate-
// input guards and the same BRepCheck_Analyzer::IsValid gate (via occt::addIfValid).
//
// This is an OCCT-only TU: every OpenCASCADE include lives here and no OCCT type
// escapes into a public/shared header. It compiles only for iOS (CYBERCAD_HAS_OCCT);
// there is no host OCCT. The shared spine (occt::unwrap / occtGuard / addIfValid /
// mapFaces / edgesByIds / facesByIds) is reused, not duplicated.
//
// Each early `return 0` in the source becomes `return make_error(...)`; the facade
// collapses the Result to 0/nil + cc_last_error, so the caller keeps its last valid
// body exactly as before. occtGuard translates OCCT's Standard_Failure (which does
// NOT derive from std::exception) so the facade's guard records it.

#include "engine/occt/occt_engine.h"

#include <cmath>
#include <vector>

// ── Feature-group OCCT builders (adapter TU only) ─────────────────────────────
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_JoinType.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax1.hxx>

namespace cyber {

// ── fillet / chamfer edges ────────────────────────────────────────────────────

ShapeResult OcctEngine::fillet_edges(EngineShape body, const int* edgeIds, int edgeCount,
                                     double radius) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || radius <= 0) { return make_error("fillet_edges: invalid input"); }
        const std::vector<TopoDS_Edge> edges = occt::edgesByIds(*sp, edgeIds, edgeCount);
        if (edges.empty()) { return make_error("fillet_edges: no edges"); }
        BRepFilletAPI_MakeFillet fillet(*sp);
        for (const TopoDS_Edge& edge : edges) { fillet.Add(radius, edge); }
        fillet.Build();
        if (!fillet.IsDone()) { return make_error("fillet_edges: build failed"); }
        return occt::addIfValid(fillet.Shape(), "fillet_edges: invalid");  // caller keeps its body
    });
}

ShapeResult OcctEngine::fillet_edges_variable(EngineShape body, const int* edgeIds, int edgeCount,
                                              double radius1, double radius2) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || (radius1 <= 0 && radius2 <= 0)) {
            return make_error("fillet_edges_variable: invalid input");
        }
        const std::vector<TopoDS_Edge> edges = occt::edgesByIds(*sp, edgeIds, edgeCount);
        if (edges.empty()) { return make_error("fillet_edges_variable: no edges"); }
        BRepFilletAPI_MakeFillet fillet(*sp);
        // linearly-varying radius from radius1 to radius2 along each edge
        for (const TopoDS_Edge& edge : edges) { fillet.Add(radius1, radius2, edge); }
        fillet.Build();
        if (!fillet.IsDone()) { return make_error("fillet_edges_variable: build failed"); }
        return occt::addIfValid(fillet.Shape(), "fillet_edges_variable: invalid");
    });
}

ShapeResult OcctEngine::chamfer_edges(EngineShape body, const int* edgeIds, int edgeCount,
                                      double distance) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || distance <= 0) { return make_error("chamfer_edges: invalid input"); }
        const std::vector<TopoDS_Edge> edges = occt::edgesByIds(*sp, edgeIds, edgeCount);
        if (edges.empty()) { return make_error("chamfer_edges: no edges"); }
        BRepFilletAPI_MakeChamfer chamfer(*sp);
        for (const TopoDS_Edge& edge : edges) { chamfer.Add(distance, edge); }
        chamfer.Build();
        if (!chamfer.IsDone()) { return make_error("chamfer_edges: build failed"); }
        return occt::addIfValid(chamfer.Shape(), "chamfer_edges: invalid");
    });
}

ShapeResult OcctEngine::fillet_face(EngineShape body, int faceId, double radius) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || radius <= 0) { return make_error("fillet_face: invalid input"); }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, ids, 1);
        if (faces.empty()) { return make_error("fillet_face: no face"); }
        BRepFilletAPI_MakeFillet fillet(*sp);
        // round every edge of the picked face
        for (TopExp_Explorer ex(faces[0], TopAbs_EDGE); ex.More(); ex.Next()) {
            fillet.Add(radius, TopoDS::Edge(ex.Current()));
        }
        fillet.Build();
        if (!fillet.IsDone()) { return make_error("fillet_face: build failed"); }
        return occt::addIfValid(fillet.Shape(), "fillet_face: invalid");
    });
}

// ── shell (hollow) ────────────────────────────────────────────────────────────

ShapeResult OcctEngine::shell(EngineShape body, const int* faceIds, int faceCount,
                              double thickness) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr || thickness <= 0) { return make_error("shell: invalid input"); }
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, faceIds, faceCount);
        if (faces.empty()) { return make_error("shell: no faces"); }
        TopTools_ListOfShape facesToRemove;
        for (const TopoDS_Face& face : faces) { facesToRemove.Append(face); }
        // Negative offset hollows the body inward, leaving `thickness` walls and
        // opening the selected faces.
        BRepOffsetAPI_MakeThickSolid maker;
        maker.MakeThickSolidByJoin(*sp, facesToRemove, -thickness, 1.0e-3);
        maker.Build();
        if (!maker.IsDone()) { return make_error("shell: build failed"); }
        return occt::addIfValid(maker.Shape(), "shell: invalid");
    });
}

// ── planar-face offset / replace / retarget ───────────────────────────────────

ShapeResult OcctEngine::offset_face(EngineShape body, int faceId, double distance) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("offset_face: invalid body"); }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, ids, 1);
        if (faces.empty() || !(std::abs(distance) > 1.0e-6)) {
            return make_error("offset_face: invalid input");
        }
        const TopoDS_Face face = faces.front();
        // Move a PLANAR face along its outward normal, keeping a solid: extrude the
        // face into a prism and fuse (outward) or cut (inward) it with the body.
        // This is the Shapr3D "offset/move face" edit (a true solid modification),
        // unlike a bare surface offset which yields a zero-volume shell.
        BRepAdaptor_Surface surf(face);
        if (surf.GetType() != GeomAbs_Plane) {
            return make_error("offset_face: non-planar face");  // only planar faces for now
        }
        gp_Dir n = surf.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) { n.Reverse(); }  // outward normal
        gp_Vec vec(n);
        vec *= distance;  // signed: out (+) / in (−)
        BRepPrimAPI_MakePrism prism(face, vec);
        prism.Build();
        if (!prism.IsDone()) { return make_error("offset_face: prism failed"); }
        const TopoDS_Shape tool = prism.Shape();
        if (distance > 0) {
            BRepAlgoAPI_Fuse fuse(*sp, tool);
            if (!fuse.IsDone()) { return make_error("offset_face: fuse failed"); }
            return occt::addIfValid(fuse.Shape(), "offset_face: invalid");
        }
        BRepAlgoAPI_Cut cut(*sp, tool);
        if (!cut.IsDone()) { return make_error("offset_face: cut failed"); }
        return occt::addIfValid(cut.Shape(), "offset_face: invalid");
    });
}

ShapeResult OcctEngine::replace_face(EngineShape body, int faceId, double offset, double tiltDeg) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("replace_face: invalid body"); }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, ids, 1);
        if (faces.empty()) { return make_error("replace_face: no face"); }
        const TopoDS_Face face = faces.front();
        // Replace a PLANAR face by retargeting it to a new plane (offset along its
        // outward normal, then tilted about the face's in-plane X axis), trimming the
        // solid to that plane via a half-space cut. Convex bodies: the picked face
        // becomes the new plane and the side walls extend/trim to meet it.
        BRepAdaptor_Surface surf(face);
        if (surf.GetType() != GeomAbs_Plane) {
            return make_error("replace_face: non-planar face");  // planar faces only (for now)
        }
        const gp_Pln pln = surf.Plane();
        gp_Dir n = pln.Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) { n.Reverse(); }  // outward
        const gp_Pnt p0 = pln.Location();
        gp_Trsf rot;
        rot.SetRotation(gp_Ax1(p0, pln.XAxis().Direction()), tiltDeg * M_PI / 180.0);
        const gp_Dir nn = n.Transformed(rot);  // tilted normal
        const gp_Pnt pp(p0.X() + n.X() * offset, p0.Y() + n.Y() * offset, p0.Z() + n.Z() * offset);
        const gp_Pln newPln(pp, nn);

        Bnd_Box bb;
        BRepBndLib::Add(*sp, bb);
        if (bb.IsVoid()) { return make_error("replace_face: empty bbox"); }
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double L = 2.0 * (gp_Pnt(xmin, ymin, zmin).Distance(gp_Pnt(xmax, ymax, zmax)) + 1.0);
        BRepBuilderAPI_MakeFace mkFace(newPln, -L, L, -L, L);
        if (!mkFace.IsDone()) { return make_error("replace_face: face failed"); }
        const gp_Pnt ref(pp.X() + nn.X() * L * 0.25, pp.Y() + nn.Y() * L * 0.25,
                         pp.Z() + nn.Z() * L * 0.25);  // outward side
        BRepPrimAPI_MakeHalfSpace mkHS(mkFace.Face(), ref);
        if (!mkHS.IsDone()) { return make_error("replace_face: halfspace failed"); }
        BRepAlgoAPI_Cut cut(*sp, mkHS.Solid());  // remove the outward half → new plane is the face
        if (!cut.IsDone()) { return make_error("replace_face: cut failed"); }
        return occt::addIfValid(cut.Shape(), "replace_face: invalid");
    });
}

ShapeResult OcctEngine::replace_face_to_plane(EngineShape body, int faceId, double px, double py,
                                              double pz, double nx, double ny, double nz) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("replace_face_to_plane: invalid body"); }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*sp, ids, 1);
        if (faces.empty()) { return make_error("replace_face_to_plane: no face"); }
        const TopoDS_Face face = faces.front();
        BRepAdaptor_Surface surf(face);
        if (surf.GetType() != GeomAbs_Plane) {
            return make_error("replace_face_to_plane: non-planar face");  // planar faces only
        }
        gp_Dir nFace = surf.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) { nFace.Reverse(); }  // outward
        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nlen < 1e-9) { return make_error("replace_face_to_plane: degenerate normal"); }
        const gp_Pnt tp(px, py, pz);
        const gp_Pln newPln(tp, gp_Dir(nx / nlen, ny / nlen, nz / nlen));

        Bnd_Box bb;
        BRepBndLib::Add(*sp, bb);
        if (bb.IsVoid()) { return make_error("replace_face_to_plane: empty bbox"); }
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double L = 2.0 * (gp_Pnt(xmin, ymin, zmin).Distance(gp_Pnt(xmax, ymax, zmax)) + 1.0);
        BRepBuilderAPI_MakeFace mkFace(newPln, -L, L, -L, L);
        if (!mkFace.IsDone()) { return make_error("replace_face_to_plane: face failed"); }
        // Remove the side the picked face faced toward (its outward side) so the
        // target plane becomes the face and the body bulk is kept and trimmed to it.
        const gp_Pnt ref(tp.X() + nFace.X() * L * 0.25, tp.Y() + nFace.Y() * L * 0.25,
                         tp.Z() + nFace.Z() * L * 0.25);
        BRepPrimAPI_MakeHalfSpace mkHS(mkFace.Face(), ref);
        if (!mkHS.IsDone()) { return make_error("replace_face_to_plane: halfspace failed"); }
        BRepAlgoAPI_Cut cut(*sp, mkHS.Solid());
        if (!cut.IsDone()) { return make_error("replace_face_to_plane: cut failed"); }
        return occt::addIfValid(cut.Shape(), "replace_face_to_plane: invalid");
    });
}

// ── planar split ──────────────────────────────────────────────────────────────

ShapeResult OcctEngine::split_plane(EngineShape body, double ox, double oy, double oz, double nx,
                                    double ny, double nz, int keepPositive) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("split_plane: invalid body"); }
        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nlen < 1e-9) { return make_error("split_plane: degenerate normal"); }
        const gp_Dir n(nx / nlen, ny / nlen, nz / nlen);
        const gp_Pnt o(ox, oy, oz);
        const gp_Pln pln(o, n);

        Bnd_Box bb;
        BRepBndLib::Add(*sp, bb);
        if (bb.IsVoid()) { return make_error("split_plane: empty bbox"); }
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        bb.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const double L = 2.0 * (gp_Pnt(xmin, ymin, zmin).Distance(gp_Pnt(xmax, ymax, zmax)) + 1.0);
        BRepBuilderAPI_MakeFace mkFace(pln, -L, L, -L, L);
        if (!mkFace.IsDone()) { return make_error("split_plane: face failed"); }
        // Half-space sits on the side of its reference point; cutting removes that
        // side. To KEEP the +normal half, the tool must cover the −normal half.
        const double s = keepPositive ? -1.0 : 1.0;
        const gp_Pnt ref(o.X() + n.X() * s * L * 0.25, o.Y() + n.Y() * s * L * 0.25,
                         o.Z() + n.Z() * s * L * 0.25);
        BRepPrimAPI_MakeHalfSpace mkHS(mkFace.Face(), ref);
        if (!mkHS.IsDone()) { return make_error("split_plane: halfspace failed"); }
        BRepAlgoAPI_Cut cut(*sp, mkHS.Solid());
        if (!cut.IsDone()) { return make_error("split_plane: cut failed"); }
        return occt::addIfValid(cut.Shape(), "split_plane: invalid");
    });
}

// ── outer-loop offset boundary polyline ───────────────────────────────────────
// Note: offset_face_boundary is declared under the QUERY group in occt_engine.h,
// but its OCCT logic is ported here with the rest of the feature edits (per the
// group assignment). The facade owns the malloc/count buffer conversion; this
// method returns the x,y,z triplets as a std::vector<double>.

Result<std::vector<double>> OcctEngine::offset_face_boundary(EngineShape body, int faceId,
                                                             double distance) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* sp = occt::unwrap(body);
        if (sp == nullptr) { return make_error("offset_face_boundary: invalid body"); }
        const TopTools_IndexedMapOfShape fmap = occt::mapFaces(*sp);  // 1-based, matches face ids
        if (faceId < 1 || faceId > fmap.Extent()) {
            return make_error("offset_face_boundary: bad faceId");
        }
        const TopoDS_Face face = TopoDS::Face(fmap.FindKey(faceId));
        const TopoDS_Wire outer = BRepTools::OuterWire(face);  // offset just the outer loop
        if (outer.IsNull()) { return make_error("offset_face_boundary: no outer wire"); }
        BRepOffsetAPI_MakeOffset mko(outer, GeomAbs_Arc);
        mko.Perform(distance);
        if (!mko.IsDone()) { return make_error("offset_face_boundary: offset failed"); }
        const TopoDS_Shape res = mko.Shape();
        if (res.IsNull()) { return make_error("offset_face_boundary: null result"); }
        // The result is a wire (or a compound containing one) — take the first wire.
        TopoDS_Wire wire;
        if (res.ShapeType() == TopAbs_WIRE) {
            wire = TopoDS::Wire(res);
        } else {
            TopExp_Explorer ex(res, TopAbs_WIRE);
            if (ex.More()) { wire = TopoDS::Wire(ex.Current()); }
        }
        if (wire.IsNull()) { return make_error("offset_face_boundary: no wire"); }
        // Walk the wire's edges in order, discretizing each (lines → 2 pts, arcs → more).
        std::vector<double> pts;
        for (BRepTools_WireExplorer we(wire); we.More(); we.Next()) {
            BRepAdaptor_Curve curve(we.Current());
            GCPnts_TangentialDeflection disc(curve, 0.2, 0.2);
            const int n = disc.NbPoints();
            for (int k = 1; k <= n; ++k) {
                const gp_Pnt p = disc.Value(k);
                // skip a duplicate of the previous point (shared edge endpoints)
                if (pts.size() >= 3 && std::abs(pts[pts.size() - 3] - p.X()) < 1e-7 &&
                    std::abs(pts[pts.size() - 2] - p.Y()) < 1e-7 &&
                    std::abs(pts[pts.size() - 1] - p.Z()) < 1e-7) {
                    continue;
                }
                pts.push_back(p.X());
                pts.push_back(p.Y());
                pts.push_back(p.Z());
            }
        }
        if (static_cast<int>(pts.size() / 3) < 2) {
            return make_error("offset_face_boundary: too few points");
        }
        return pts;
    });
}

}  // namespace cyber
