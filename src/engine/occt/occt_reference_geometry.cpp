// OCCT engine adapter — REFERENCE-GEOMETRY feature (Phase-3 owned TU).
//
// Implements the DERIVED datum constructors declared in occt_engine.h:
//   ref_plane_from_face(body, faceId)  planar-face plane  -> [ox,oy,oz, nx,ny,nz]
//   ref_axis_from_edge(body, edgeId)   linear-edge axis   -> [ox,oy,oz, dx,dy,dz]
//   ref_axis_from_face(body, faceId)   cyl/cone-face axis -> [ox,oy,oz, dx,dy,dz]
// (The point-only trio — plane-from-3-points, offset plane, axis-from-2-points —
// is exact fp64 math done facade-side in cc_kernel.cpp and does NOT touch this TU.)
//
// These read an existing body's geometry through the shared occt spine helpers
// (unwrap / facesByIds / edgesByIds) and return a 6-double POD the facade copies
// into out6. Any non-planar face, non-linear edge, non-cyl/cone face, or unknown
// body/subshape id collapses to an Error (the facade's documented 0 sentinel).
//
// Notes on correctness:
//  - Plane origin is evaluated at the face's UV midpoint so it provably LIES on
//    the face (gp_Pln::Location merely lies on the infinite plane). The normal is
//    the plane axis, reversed for a TopAbs_REVERSED face so it points OUTWARD —
//    matching the convention used by offset_face/replace_face in occt_feature.cpp.
//  - ref_axis_from_face reuses the EXACT cylinder/cone axis extraction of
//    face_axis (occt_query.cpp) so cc_ref_axis_from_face == cc_face_axis bit-for-bit.
//  - gp_Dir is unit by construction, so the emitted normal/direction is unit
//    within fp64 rounding (well inside the spec's 1e-9 guarantee).
//
// OCCT-only TU (CYBERCAD_HAS_OCCT); the host build omits it and the stub inherits
// the unsupported default (also 0).

#include "engine/occt/occt_engine.h"

#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

namespace cyber {

Result<std::vector<double>> OcctEngine::ref_plane_from_face(EngineShape body, int faceId) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("ref_plane_from_face: unknown body");
        }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*shape, ids, 1);
        if (faces.empty()) {
            return make_error("ref_plane_from_face: face not found");
        }
        const TopoDS_Face face = faces.front();
        BRepAdaptor_Surface surf(face);
        if (surf.GetType() != GeomAbs_Plane) {
            return make_error("ref_plane_from_face: non-planar face");
        }
        // Outward normal: the plane axis, flipped for a reversed face (same
        // convention as offset_face / replace_face).
        gp_Dir n = surf.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) {
            n.Reverse();
        }
        // Origin AT the face's UV midpoint so it provably lies on the face.
        Standard_Real umin = 0, umax = 0, vmin = 0, vmax = 0;
        BRepTools::UVBounds(face, umin, umax, vmin, vmax);
        const gp_Pnt o = surf.Value(0.5 * (umin + umax), 0.5 * (vmin + vmax));
        return std::vector<double>{o.X(), o.Y(), o.Z(), n.X(), n.Y(), n.Z()};
    });
}

Result<std::vector<double>> OcctEngine::ref_axis_from_edge(EngineShape body, int edgeId) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("ref_axis_from_edge: unknown body");
        }
        const int ids[1] = {edgeId};
        const std::vector<TopoDS_Edge> edges = occt::edgesByIds(*shape, ids, 1);
        if (edges.empty()) {
            return make_error("ref_axis_from_edge: edge not found");
        }
        BRepAdaptor_Curve curve(edges.front());
        if (curve.GetType() != GeomAbs_Line) {
            return make_error("ref_axis_from_edge: non-linear edge");
        }
        const gp_Lin line = curve.Line();
        const gp_Pnt p = line.Location();
        const gp_Dir d = line.Direction();
        return std::vector<double>{p.X(), p.Y(), p.Z(), d.X(), d.Y(), d.Z()};
    });
}

Result<std::vector<double>> OcctEngine::ref_axis_from_face(EngineShape body, int faceId) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("ref_axis_from_face: unknown body");
        }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*shape, ids, 1);
        if (faces.empty()) {
            return make_error("ref_axis_from_face: face not found");
        }
        // Reuse the EXACT face_axis extraction so this matches cc_face_axis.
        BRepAdaptor_Surface surf(faces.front());
        gp_Ax1 axis;
        if (surf.GetType() == GeomAbs_Cylinder) {
            axis = surf.Cylinder().Axis();
        } else if (surf.GetType() == GeomAbs_Cone) {
            axis = surf.Cone().Axis();
        } else {
            return make_error("ref_axis_from_face: face has no axis");
        }
        const gp_Pnt p = axis.Location();
        const gp_Dir d = axis.Direction();
        return std::vector<double>{p.X(), p.Y(), p.Z(), d.X(), d.Y(), d.Z()};
    });
}

}  // namespace cyber
