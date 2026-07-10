// OCCT engine adapter — QUERY capability group.
//
// Defines the OcctEngine measurement / topology-inspection overrides declared in
// occt_engine.h: mass_properties, principal_moments, bounding_box, face_axis,
// subshape_ids, tangent_chain and outer_rim_chain. Like every occt_*.cpp this is
// an OCCT-only TU (it may include OpenCASCADE headers, but no OCCT type escapes
// into any public or shared header) and is NOT host-buildable — it compiles only
// for iOS where the trimmed OCCT static libs are linked (CYBERCAD_HAS_OCCT).
//
// Behaviour is a faithful re-homing of the app bridge's CYBERCAD_HAS_OCCT
// implementations of cc_mass_properties / cc_principal_moments / cc_bounding_box /
// cc_face_axis / cc_subshape_ids / cc_tangent_chain / cc_outer_rim_chain: same
// GProp / Bnd / BRepAdaptor / TopExp queries, and the same degenerate-input guards
// (empty maps, out-of-range ids, non-cylindrical faces, void boxes). These are
// pure queries — they build no shape — so there is no BRepCheck_Analyzer::IsValid
// gate here; the guard is on the input body and the geometric result.
//
// Buffer ownership: the methods return POD vectors only; the facade allocates the
// C buffers (finish_fixed / finish_ints -> cc_ints_free / cc_points_free), so no
// malloc crosses this boundary.
//
// OCCT signals failures via Standard_Failure (which does NOT derive from
// std::exception); occt::occtGuard rethrows it as std::runtime_error so the
// facade's outer guard records the message into the per-thread cc_last_error.

#include "engine/occt/occt_engine.h"

#include <cmath>
#include <set>
#include <vector>

// ── OCCT query headers (this TU only) ─────────────────────────────────────────
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

namespace cyber {

namespace {

// Unit tangent of edge `e` at vertex `v` (false if degenerate / a point edge).
// Query-local helper (not a shared spine primitive): only tangentChain needs it.
bool edgeTangentAt(const TopoDS_Edge& e, const TopoDS_Vertex& v, gp_Dir& out) {
    if (BRep_Tool::Degenerated(e)) {
        return false;
    }
    const Standard_Real p = BRep_Tool::Parameter(v, e);
    BRepAdaptor_Curve curve(e);
    gp_Pnt pnt;
    gp_Vec d1;
    curve.D1(p, pnt, d1);
    if (d1.Magnitude() < 1.0e-9) {
        return false;
    }
    out = gp_Dir(d1);
    return true;
}

// Grow `seeds` (1-based edge ids) to the connected set of tangent-continuous
// edges — edges meeting C1 (parallel tangents, ~<=15 deg) at a shared vertex — so
// filleting one edge of a smooth arc<->line contour rounds the whole contour,
// which is required because OCCT can't fillet a lone tangent arc edge.
std::vector<int> tangentChain(const TopoDS_Shape& shape, const std::vector<int>& seeds) {
    const TopTools_IndexedMapOfShape emap = occt::mapEdges(shape);
    TopTools_IndexedDataMapOfShapeListOfShape vmap;
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vmap);
    std::set<int> inChain;
    std::vector<int> queue;
    for (int s : seeds) {
        if (s >= 1 && s <= emap.Extent() && inChain.insert(s).second) {
            queue.push_back(s);
        }
    }
    const double tol = 0.966;  // |cos| of ~15 deg
    while (!queue.empty()) {
        const int id = queue.back();
        queue.pop_back();
        const TopoDS_Edge edge = TopoDS::Edge(emap.FindKey(id));
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2);
        for (const TopoDS_Vertex& v : {v1, v2}) {
            if (v.IsNull() || !vmap.Contains(v)) {
                continue;
            }
            gp_Dir t1;
            if (!edgeTangentAt(edge, v, t1)) {
                continue;
            }
            const TopTools_ListOfShape& incident = vmap.FindFromKey(v);
            for (TopTools_ListIteratorOfListOfShape it(incident); it.More(); it.Next()) {
                const TopoDS_Edge ae = TopoDS::Edge(it.Value());
                const int aid = emap.FindIndex(ae);
                if (aid < 1 || inChain.count(aid)) {
                    continue;
                }
                gp_Dir t2;
                if (!edgeTangentAt(ae, v, t2)) {
                    continue;
                }
                if (std::abs(t1.Dot(t2)) >= tol) {
                    inChain.insert(aid);
                    queue.push_back(aid);
                }
            }
        }
    }
    return std::vector<int>(inChain.begin(), inChain.end());
}

// Map cc_subshape_ids' kind selector to a TopAbs sub-shape type: 0 vertex,
// 1 edge, anything else face (matching the app bridge's ternary exactly).
TopAbs_ShapeEnum subshapeKind(int kind) {
    if (kind == 0) {
        return TopAbs_VERTEX;
    }
    if (kind == 1) {
        return TopAbs_EDGE;
    }
    return TopAbs_FACE;
}

}  // namespace

// ── Mass properties ───────────────────────────────────────────────────────────

Result<MassData> OcctEngine::mass_properties(EngineShape body) {
    return occt::occtGuard([&]() -> Result<MassData> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("mass_properties: unknown body");
        }
        GProp_GProps vol;
        BRepGProp::VolumeProperties(*shape, vol);
        GProp_GProps surf;
        BRepGProp::SurfaceProperties(*shape, surf);
        const gp_Pnt c = vol.CentreOfMass();
        MassData md;
        md.volume = vol.Mass();  // VolumeProperties -> mass == volume (unit density)
        md.area = surf.Mass();   // SurfaceProperties -> mass == area
        md.cx = c.X();
        md.cy = c.Y();
        md.cz = c.Z();
        md.valid = true;
        return md;
    });
}

// ── Principal moments of inertia ──────────────────────────────────────────────

Result<std::vector<double>> OcctEngine::principal_moments(EngineShape body) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("principal_moments: unknown body");
        }
        GProp_GProps vol;
        BRepGProp::VolumeProperties(*shape, vol);
        const GProp_PrincipalProps pp = vol.PrincipalProperties();
        Standard_Real i1 = 0, i2 = 0, i3 = 0;
        pp.Moments(i1, i2, i3);  // principal moments (unit density -> volume inertia)
        return std::vector<double>{i1, i2, i3};
    });
}

// ── Solid validity (MOAT M-GS GS6) ────────────────────────────────────────────
// The BRepCheck_Analyzer::IsValid ORACLE. This adapter reports the OVERALL verdict
// (the quantity the native mesh-level checker is verified against on the sim gate);
// the per-check breakdown fields mirror that verdict (OCCT always certifies), while
// the NativeEngine fills the richer per-check decomposition validated at the host
// gate. finite is always true for a built OCCT B-rep.
Result<ValidityData> OcctEngine::check_solid(EngineShape body) {
    return occt::occtGuard([&]() -> Result<ValidityData> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("check_solid: unknown body");
        }
        const bool ok = !shape->IsNull() && BRepCheck_Analyzer(*shape).IsValid();
        ValidityData out;
        out.valid = ok;
        out.closed = ok;
        out.oriented = ok;
        out.nondegenerate = ok;
        out.finite = !shape->IsNull();
        out.noSelfIntersection = ok;
        out.certified = true;
        return out;
    });
}

// ── Interference / clash of two solids (MOAT M-GS GS7) ─────────────────────────
// The BRepAlgoAPI_Common + BRepExtrema_DistShapeShape ORACLE the native mesh-level
// classifier is verified against on the sim gate. CLASH iff the COMMON has positive
// volume; otherwise the minimum boundary distance (DistShapeShape) decides TOUCHING
// (≈0, no interior overlap) vs CLEAR (>0). The witness box + point on CLASH are the
// COMMON solid's bounding box + its centre of mass — a genuine interior point.
Result<InterferenceData> OcctEngine::interference(EngineShape a, EngineShape b) {
    return occt::occtGuard([&]() -> Result<InterferenceData> {
        const TopoDS_Shape* sa = occt::unwrap(a);
        const TopoDS_Shape* sb = occt::unwrap(b);
        if (sa == nullptr || sb == nullptr) return make_error("interference: unknown body");

        InterferenceData out;

        // Overlap volume via COMMON. A clean empty/failed common ⇒ no interior overlap.
        double vc = 0.0;
        TopoDS_Shape common;
        {
            BRepAlgoAPI_Common k(*sa, *sb);
            if (k.IsDone() && !k.Shape().IsNull()) {
                common = k.Shape();
                GProp_GProps g;
                BRepGProp::VolumeProperties(common, g);
                vc = g.Mass();
            }
        }

        // Boundary clearance via DistShapeShape (0 on contact / penetration).
        double dist = 0.0;
        {
            BRepExtrema_DistShapeShape ext(*sa, *sb);
            if (ext.IsDone() && ext.NbSolution() > 0) dist = ext.Value();
        }

        if (vc > 1e-12) {
            out.state = 2;  // clash
            out.overlapVolume = vc;
            // Witness = the COMMON solid's tight box + its centre of mass.
            Bnd_Box box;
            BRepBndLib::AddOptimal(common, box, /*useTri=*/Standard_False,
                                   /*useShapeTol=*/Standard_False);
            if (!box.IsVoid()) {
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                out.hasWitness = true;
                out.witLoX = xmin; out.witLoY = ymin; out.witLoZ = zmin;
                out.witHiX = xmax; out.witHiY = ymax; out.witHiZ = zmax;
                GProp_GProps gc;
                BRepGProp::VolumeProperties(common, gc);
                const gp_Pnt c = gc.CentreOfMass();
                out.witPX = c.X(); out.witPY = c.Y(); out.witPZ = c.Z();
            }
            return out;
        }

        out.minDistance = dist;
        out.state = (dist <= 1e-7) ? 1 : 0;  // touching vs clear
        return out;
    });
}

// ── Exact axis-aligned bounding box (of the B-rep, not the tessellation) ───────

Result<std::vector<double>> OcctEngine::bounding_box(EngineShape body) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("bounding_box: unknown body");
        }
        Bnd_Box box;
        // AddOptimal(useTriangulation=false, useShapeTolerance=false) → the TIGHT
        // geometric bounding box. Plain BRepBndLib::Add inflates every face/edge box
        // by a sampling/deflection gap (~deflection·√3 per corner) — and uses a shape's
        // attached triangulation when present — so a boolean result (BRepAlgoAPI
        // attaches a mesh) reported a LOOSE box (e.g. a [0,3]³ fuse came back as
        // [-0.0087, 3.0087]) while the exact-modelling contract — and the native engine
        // — give the exact extents. AddOptimal builds precise geometric boxes that differ
        // from the true boundary only by shape tolerances; with useShapeTolerance=false
        // those collapse to Precision::Confusion, so this matches the native bounding_box
        // exactly for planar solids and is tighter (more correct) for every shape.
        BRepBndLib::AddOptimal(*shape, box, /*useTriangulation=*/Standard_False,
                               /*useShapeTolerance=*/Standard_False);
        if (box.IsVoid()) {
            return make_error("bounding_box: empty box");
        }
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        return std::vector<double>{xmin, ymin, zmin, xmax, ymax, zmax};
    });
}

// ── Axis of a cylindrical / conical face ──────────────────────────────────────

Result<std::vector<double>> OcctEngine::face_axis(EngineShape body, int faceId) {
    return occt::occtGuard([&]() -> Result<std::vector<double>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("face_axis: unknown body");
        }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*shape, ids, 1);
        if (faces.empty()) {
            return make_error("face_axis: face not found");
        }
        BRepAdaptor_Surface surf(faces.front());
        gp_Ax1 axis;
        if (surf.GetType() == GeomAbs_Cylinder) {
            axis = surf.Cylinder().Axis();
        } else if (surf.GetType() == GeomAbs_Cone) {
            axis = surf.Cone().Axis();
        } else {
            return make_error("face_axis: face has no axis");  // only cylinder/cone
        }
        const gp_Pnt p = axis.Location();
        const gp_Dir d = axis.Direction();
        return std::vector<double>{p.X(), p.Y(), p.Z(), d.X(), d.Y(), d.Z()};
    });
}

// ── DM4 point projection onto a face surface (GeomAPI_ProjectPointOnSurf) ──────

Result<ProjectionData> OcctEngine::project_point_on_face(EngineShape body, int faceId, double px,
                                                         double py, double pz) {
    return occt::occtGuard([&]() -> Result<ProjectionData> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("project_point_on_face: unknown body");
        }
        const int ids[1] = {faceId};
        const std::vector<TopoDS_Face> faces = occt::facesByIds(*shape, ids, 1);
        if (faces.empty()) {
            return make_error("project_point_on_face: face not found");
        }
        // Project onto the face's UNTRIMMED analytic surface (matches the native
        // infinite-surface foot; the native slice serves plane/cylinder/sphere).
        const Handle(Geom_Surface) surf = BRep_Tool::Surface(faces.front());
        if (surf.IsNull()) {
            return make_error("project_point_on_face: face carries no surface");
        }
        GeomAPI_ProjectPointOnSurf proj(gp_Pnt(px, py, pz), surf);
        if (!proj.IsDone() || proj.NbPoints() < 1) {
            return make_error("project_point_on_face: projection failed");
        }
        const gp_Pnt foot = proj.NearestPoint();
        ProjectionData out;
        out.footX = foot.X();
        out.footY = foot.Y();
        out.footZ = foot.Z();
        out.distance = proj.LowerDistance();
        return out;
    });
}

// ── Stable sub-shape ids for picking ──────────────────────────────────────────

Result<std::vector<int>> OcctEngine::subshape_ids(EngineShape body, int kind) {
    return occt::occtGuard([&]() -> Result<std::vector<int>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("subshape_ids: unknown body");
        }
        TopTools_IndexedMapOfShape map;
        TopExp::MapShapes(*shape, subshapeKind(kind), map);
        const int count = map.Extent();
        if (count <= 0) {
            return std::vector<int>{};  // no sub-shapes of this kind -> 0 (no error)
        }
        // Stable 1-based indices into the indexed map — the ids Swift uses to select
        // sub-shapes for fillet/chamfer/offset edits.
        std::vector<int> ids(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            ids[static_cast<std::size_t>(i)] = i + 1;
        }
        return ids;
    });
}

// ── Connected-solid enumeration (app-parity) ──────────────────────────────────

Result<int> OcctEngine::shape_solid_count(EngineShape body) {
    return occt::occtGuard([&]() -> Result<int> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr) {
            return make_error("shape_solid_count: unknown body");
        }
        int n = 0;
        for (TopExp_Explorer ex(*shape, TopAbs_SOLID); ex.More(); ex.Next()) {
            ++n;
        }
        return n;
    });
}

ShapeResult OcctEngine::shape_solid_at(EngineShape body, int index) {
    return occt::occtGuard([&]() -> ShapeResult {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr || index < 0) {
            return make_error("shape_solid_at: unknown body or negative index");
        }
        int i = 0;
        for (TopExp_Explorer ex(*shape, TopAbs_SOLID); ex.More(); ex.Next(), ++i) {
            if (i == index) {
                return ShapeResult(occt::wrap(ex.Current()));
            }
        }
        return make_error("shape_solid_at: index out of range");
    });
}

// ── Tangent-continuous edge chain ─────────────────────────────────────────────

Result<std::vector<int>> OcctEngine::tangent_chain(EngineShape body, const int* edgeIds,
                                                   int edgeCount) {
    return occt::occtGuard([&]() -> Result<std::vector<int>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr || edgeIds == nullptr || edgeCount <= 0) {
            return make_error("tangent_chain: unknown body or empty selection");
        }
        const std::vector<int> seeds(edgeIds, edgeIds + edgeCount);
        return tangentChain(*shape, seeds);  // empty chain -> 0 ids (no error)
    });
}

// ── Outer-rim edge chain (planar cap outer wire) ──────────────────────────────

Result<std::vector<int>> OcctEngine::outer_rim_chain(EngineShape body, const int* edgeIds,
                                                     int edgeCount) {
    return occt::occtGuard([&]() -> Result<std::vector<int>> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr || edgeIds == nullptr || edgeCount <= 0) {
            return make_error("outer_rim_chain: unknown body or empty selection");
        }
        const TopTools_IndexedMapOfShape emap = occt::mapEdges(*shape);
        TopTools_IndexedDataMapOfShapeListOfShape efmap;
        TopExp::MapShapesAndAncestors(*shape, TopAbs_EDGE, TopAbs_FACE, efmap);
        // Collect the seed edges' vertex points. The CAP face the seeds bound contains ALL
        // of them in its plane; a planar SIDE WALL contains only the one shared edge, so its
        // plane is far from the rest. Requiring all-seeds-coplanar with the face picks the cap
        // and rejects side walls — which previously dragged the OPPOSITE (bottom) rim in.
        std::vector<gp_Pnt> seedPts;
        for (int i = 0; i < edgeCount; ++i) {
            const int sid = edgeIds[i];
            if (sid < 1 || sid > emap.Extent()) {
                continue;
            }
            const TopoDS_Edge se = TopoDS::Edge(emap.FindKey(sid));
            TopoDS_Vertex va, vb;
            TopExp::Vertices(se, va, vb);
            if (!va.IsNull()) {
                seedPts.push_back(BRep_Tool::Pnt(va));
            }
            if (!vb.IsNull()) {
                seedPts.push_back(BRep_Tool::Pnt(vb));
            }
        }
        const double planeTol = 1.0;  // points; separates 0 (in-plane) from the body's thickness
        std::set<int> ids;
        for (int i = 0; i < edgeCount; ++i) {
            const int sid = edgeIds[i];
            if (sid < 1 || sid > emap.Extent()) {
                continue;
            }
            const TopoDS_Edge se = TopoDS::Edge(emap.FindKey(sid));
            if (!efmap.Contains(se)) {
                continue;
            }
            // Each planar CAP face the seed bounds -> add that face's OUTER wire edges, so a
            // tapped rim edge rounds the whole rim (skips hole wires). A face only qualifies
            // as a cap if ALL seed points lie in its plane (excludes vertical side walls).
            for (TopTools_ListIteratorOfListOfShape it(efmap.FindFromKey(se)); it.More(); it.Next()) {
                const TopoDS_Face f = TopoDS::Face(it.Value());
                BRepAdaptor_Surface surf(f);
                if (surf.GetType() != GeomAbs_Plane) {
                    continue;
                }
                const gp_Pln pln = surf.Plane();
                bool allCoplanar = true;
                for (const gp_Pnt& p : seedPts) {
                    if (pln.Distance(p) > planeTol) {
                        allCoplanar = false;
                        break;
                    }
                }
                if (!allCoplanar) {
                    continue;  // a side wall holds only some seeds -> not the cap
                }
                const TopoDS_Wire outer = BRepTools::OuterWire(f);
                for (TopExp_Explorer ex(outer, TopAbs_EDGE); ex.More(); ex.Next()) {
                    const int eid = emap.FindIndex(TopoDS::Edge(ex.Current()));
                    if (eid >= 1) {
                        ids.insert(eid);
                    }
                }
            }
        }
        return std::vector<int>(ids.begin(), ids.end());  // empty set -> 0 ids (no error)
    });
}

}  // namespace cyber
