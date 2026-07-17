// NativeEngine implementation — the clean-room native construct ops + fallthrough.
//
// This TU pulls in the heavy native geometry (src/native/construct + tessellate +
// topology + math), all OCCT-FREE and host-buildable. The OCCT engine is referenced
// ONLY inside CYBERCAD_HAS_OCCT (for the fallthrough target); without OCCT the
// fallthrough target is the always-compiled stub. No OCCT type appears in this
// file outside that guard, so it compiles on the host.
//
// See native_engine.h for the coexistence / shape-ownership contract.

#include "engine/native/native_engine.h"

#include "core/shape_provenance.h"  // process-wide native-body identity (cross-engine guard)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <set>
#include <utility>
#include <vector>

#include "engine/native/native_heal_hook.h"
#include "native/analysis/angle.h"
#include "native/analysis/curvature.h"
#include "native/analysis/inertia.h"    // GS5: signed-tetra inertia over M0
#include "native/analysis/validity.h"   // GS6: mesh-level B-rep validity report
#include "native/analysis/interference.h"  // GS7: mesh-level clash/interference
#ifdef CYBERCAD_HAS_NUMSCI
#include "native/analysis/distance.h"  // seed-and-refine minimizer (numsci-gated)
#endif
#include "native/blend/native_blend.h"
#include "native/boolean/native_boolean.h"
#include "native/boolean/thread_apply.h"
#include "native/construct/native_construct.h"
#include "native/drafting/native_drafting.h"
#include "native/section/native_section.h"
#include "native/exchange/native_exchange.h"
#include "native/feature/draft_faces.h"
#include "native/feature/wrap_emboss.h"
#include "native/sheetmetal/sheetmetal.h"  // MOAT M-SM: base/edge flange + unfold
#include "native/surface/native_surface.h"  // MOAT surface: bounded N-sided fill patch
#include "native/heal/native_heal.h"
#include "native/math/transform.h"  // M-TX: affine placement for native transforms
#include "native/tessellate/native_tessellate.h"
#include "native/topology/accessors.h"
#include "native/reference/reference.h"  // M-REF: datum + topology-reference reads

// DM1 split_plane composes the landed freeformHalfSpaceCut, whose freeform-wall seam
// trace (ssi::trace_intersection) is DEFINED only under CYBERCAD_HAS_NUMSCI. Including
// this header — and calling into it — is therefore gated on the substrate so the
// always-compiled native engine links without NUMSCI (native split then honestly
// declines, exactly as it did pre-DM1). The analytic paths are unaffected.
#ifdef CYBERCAD_HAS_NUMSCI
#include "native/boolean/split_plane.h"
#include "native/directmodel/replace_face.h"
#include "native/directmodel/replace_face_general.h"  // DM3 general offset/tilt retarget
#endif

// DM4 point projection is pure analytic geometry (no NUMSCI substrate) — always on.
#include "native/directmodel/project.h"

#ifdef CYBERCAD_HAS_OCCT
#include "engine/occt/occt_engine.h"
#endif

namespace cyber {

namespace {

namespace ntopo = cybercad::native::topology;
namespace nan = cybercad::native::analysis;
namespace ntess = cybercad::native::tessellate;
namespace ndraft = cybercad::native::drafting;
namespace nsec = cybercad::native::section;
namespace nmath = cybercad::native::math;
namespace nref = cybercad::native::reference;

// ── Native shape holder type-erased behind the registry's EngineShape ──────────
// Distinct from occt::OcctShape. A native void MUST NEVER be handed to the OCCT
// adapter, whose unwrap() does an UNCHECKED static_pointer_cast<OcctShape> and would
// read a NativeShape as garbage (a null/garbage TopoDS_Shape → BRepBndLib::Add
// crashes). Identifying a native body therefore CANNOT depend on any single engine
// instance's bookkeeping: cc_set_engine(1) builds a fresh NativeEngine each time, so
// a body built under one instance would look "unknown" (→ OCCT) to the next.
//
// PROCESS-WIDE IDENTITY. Every live NativeShape registers its own address on
// construction and removes it on destruction, so isNative() is a stable,
// instance-independent fact carried by the shape itself. This is the robust fix for
// the boolean-parity crash (build under engine 1, run boolean under a second engine-1
// instance → the second instance no longer misclassifies the operands as OCCT bodies
// and never forwards a native void to OCCT).
//
// The backing store is the SHARED, engine-agnostic cyber::*_native_shape registry in
// src/core/shape_provenance — the SAME set the OCCT adapter queries at its unwrap()
// boundary to REFUSE a foreign native body (the symmetric half of this guard: OCCT
// active, handed a native body). This thin wrapper keeps the existing call sites
// (add/remove/contains) but routes them to that one source of truth.
class NativeShapeRegistry {
public:
    static void add(const void* p) { register_native_shape(p); }
    static void remove(const void* p) { unregister_native_shape(p); }
    static bool contains(const void* p) { return is_native_shape(p); }
};

struct NativeShape {
    ntopo::Shape shape;       // B-rep bodies; left default/empty for mesh bodies
    ntess::Mesh mesh;         // populated for imported STL mesh bodies (issue #5)
    bool isMesh = false;      // true => serve the mesh directly (no B-rep)
    // MOAT M-SM: closed-form fold parameters of a single-bend sheet-metal part,
    // recorded so cc_sheet_unfold(body, kFactor) develops it EXACTLY (no fragile mesh
    // reverse-engineering). Additive, process-internal — never crosses the cc_* ABI.
    cybercad::native::sheetmetal::FoldRecord fold{};
    NativeShape() { NativeShapeRegistry::add(this); }
    ~NativeShape() { NativeShapeRegistry::remove(this); }
    NativeShape(const NativeShape&) = delete;
    NativeShape& operator=(const NativeShape&) = delete;
};

EngineShape wrapNative(ntopo::Shape shape) {
    auto holder = std::make_shared<NativeShape>();
    holder->shape = std::move(shape);
    return std::static_pointer_cast<void>(holder);
}

// Wrap a folded sheet-metal body WITH its closed-form fold record attached.
EngineShape wrapNativeFold(ntopo::Shape shape,
                           const cybercad::native::sheetmetal::FoldRecord& fold) {
    auto holder = std::make_shared<NativeShape>();
    holder->shape = std::move(shape);
    holder->fold = fold;
    return std::static_pointer_cast<void>(holder);
}

// Wrap an imported triangle-soup mesh as a mesh-backed native body. Every native
// body-consuming op (tessellate / mass_properties / bounding_box / face_meshes /
// subshape_ids) branches on holder->isMesh to serve this mesh directly.
EngineShape wrapNativeMesh(ntess::Mesh mesh) {
    auto holder = std::make_shared<NativeShape>();
    holder->mesh = std::move(mesh);
    holder->isMesh = true;
    return std::static_pointer_cast<void>(holder);
}

namespace ncst = cybercad::native::construct;

// The revolution axis the cc_solid_revolve contract implies: the Y axis through
// the origin, in the profile plane (z=0). Mirrors the OCCT adapter's solid_revolve
// (BRepPrimAPI_MakeRevol about gp_Ax1(origin, {0,1,0})).
constexpr ncst::RevolveAxis kRevolveYAxis{0.0, 0.0, 0.0, 1.0};

// ── Tier-A (#4b) input marshalling: engine POD → native construct POD ───────────
// Map the engine's ProfileSeg (mirrored from CCProfileSeg) to the native construct
// ProfileSegment 1:1 (construct/ stays OCCT-free with its own POD). Both mirror the
// same CCProfileSeg layout, so this is a field copy.
std::vector<ncst::ProfileSegment> toNativeSegs(const ProfileSeg* segs, int segCount) {
    std::vector<ncst::ProfileSegment> out;
    if (!segs || segCount <= 0) return out;
    out.reserve(static_cast<std::size_t>(segCount));
    for (int i = 0; i < segCount; ++i) {
        const ProfileSeg& s = segs[i];
        out.push_back(ncst::ProfileSegment{s.kind, s.x0, s.y0, s.x1, s.y1, s.cx, s.cy, s.r, s.a0,
                                           s.a1, s.ptOffset, s.ptCount});
    }
    return out;
}

// Parse the packed circular-hole array (cx,cy,r triples) into CircleHole PODs.
std::vector<ncst::CircleHole> toCircleHoles(const double* holesCenterRadius, int holeCount) {
    std::vector<ncst::CircleHole> out;
    if (!holesCenterRadius || holeCount <= 0) return out;
    out.reserve(static_cast<std::size_t>(holeCount));
    for (int i = 0; i < holeCount; ++i)
        out.push_back(ncst::CircleHole{holesCenterRadius[i * 3], holesCenterRadius[i * 3 + 1],
                                       holesCenterRadius[i * 3 + 2]});
    return out;
}

// Parse the packed polygon-hole arrays: `polyXY` is a flat x,y stream; `polyCounts`
// gives the vertex count of each of `polyCount` holes, consumed in order.
std::vector<std::vector<cybercad::native::math::Point3>> toPolyHoles(const double* polyXY,
                                                                     const int* polyCounts,
                                                                     int polyCount) {
    std::vector<std::vector<cybercad::native::math::Point3>> out;
    if (!polyXY || !polyCounts || polyCount <= 0) return out;
    out.reserve(static_cast<std::size_t>(polyCount));
    int off = 0;
    for (int h = 0; h < polyCount; ++h) {
        const int n = polyCounts[h];
        std::vector<cybercad::native::math::Point3> loop;
        if (n > 0) {
            loop.reserve(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i)
                loop.push_back({polyXY[(off + i) * 2], polyXY[(off + i) * 2 + 1], 0.0});
        }
        out.push_back(std::move(loop));
        off += n > 0 ? n : 0;
    }
    return out;
}

// Default tessellation deflection when a body-consuming op does not carry one
// (mass_properties / bounding_box derive from a mesh). A tight value keeps the
// area/volume within the convergence bound the native tessellator guarantees.
// Planar prisms and analytic revolves converge (near-)exactly even at a loose
// deflection, but a TWISTED ruled loft (a bilinear saddle side face) only
// converges as the mesh subdivides the twist, so this must be tight enough that
// the worst supported case — a square lofted to a 45°-rotated square — lands its
// mesh-derived volume within the parity tolerance of the OCCT oracle (at 0.005 the
// rotated-square-twist volume is 14.515 vs OCCT 14.438, rel ≈ 5.3e-3, well under
// the 5e-2 gate; 0.05 gave 15.38 / rel 6.5e-2 and failed).
constexpr double kPropertyDeflection = 0.005;

// Convert a native mesh into the ABI MeshData (flat vertex/triangle arrays).
MeshData toMeshData(const ntess::Mesh& m) {
    MeshData out;
    out.vertices.reserve(m.vertices.size() * 3);
    for (const auto& p : m.vertices) {
        out.vertices.push_back(p.x);
        out.vertices.push_back(p.y);
        out.vertices.push_back(p.z);
    }
    out.triangles.reserve(m.triangles.size() * 3);
    for (const auto& t : m.triangles) {
        out.triangles.push_back(static_cast<int>(t.a));
        out.triangles.push_back(static_cast<int>(t.b));
        out.triangles.push_back(static_cast<int>(t.c));
    }
    return out;
}

// Self-verify that a freshly built native solid meshes ROBUSTLY watertight — closed
// at EVERY deflection in a small ladder, not just one. Used by ops whose native
// builder attempts a hard tiling that may or may not weld (Tier-D helical threads):
// the builder can return non-null topology whose ruled-band + cap seams only weld at
// some deflections, so shipping it as native would leak an open mesh into
// mass/bbox/tessellate. Verifying across the ladder means the engine keeps a thread
// native ONLY when it is genuinely watertight, and otherwise falls through to OCCT —
// honest coexistence, never a faked or leaky solid. A closed solid also encloses a
// positive volume, which we require.
bool robustlyWatertight(const ntopo::Shape& s) {
    if (s.isNull()) return false;
    for (const double d : {0.05, 0.02, 0.01, 0.005}) {
        ntess::MeshParams p;
        p.deflection = d;
        const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
        if (!ntess::isWatertight(m)) return false;
        if (!(std::fabs(ntess::enclosedVolume(m)) > 0.0)) return false;
    }
    return true;
}

// Robust watertight self-verify for a possibly-MULTI-solid import (the native STEP
// reader returns a single Solid for one MANIFOLD_SOLID_BREP or a Compound of Solids
// for several). A single Solid delegates to robustlyWatertight; a Compound requires
// EVERY member solid to independently pass (any leaky member → the whole import
// declines to OCCT, never shipping a partially-open multi-body). An empty compound is
// rejected. Meshing the Compound as one shape would let a leak in member A be masked,
// so we verify each solid on its own.
bool robustlyWatertightImport(const ntopo::Shape& s) {
    if (s.isNull()) return false;
    if (s.type() != ntopo::ShapeType::Compound) return robustlyWatertight(s);
    int members = 0;
    for (ntopo::Explorer ex(s, ntopo::ShapeType::Solid); ex.more(); ex.next()) {
        ++members;
        if (!robustlyWatertight(ex.current())) return false;
    }
    return members > 0;
}

// Watertight-valid enclosed volume of a native solid at a fine deflection, or a
// negative sentinel if the mesh is not watertight (so the boolean self-verify can
// reject a leaky result). Planar polyhedra mesh exactly, so this is the exact
// volume for the boolean's domain.
double watertightVolume(const ntopo::Shape& s) {
    if (s.isNull()) return -1.0;
    ntess::MeshParams p;
    p.deflection = 0.005;
    const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
    if (!ntess::isWatertight(m)) return -1.0;
    return std::fabs(ntess::enclosedVolume(m));
}

// CURVED-SLICE self-verify (NATIVE-REWRITE.md #5 residual: axis-aligned box ⟷
// axis-parallel cylinder). When BOTH operands are recognised as an axis-aligned box
// and an axis-parallel cylinder, the expected result volume is ANALYTIC (closed form,
// not a mesh estimate), so the guard checks the result's watertight mesh volume
// against that analytic value:
//   cut  (box − cyl through hole) = boxVol − πr²·boxAxialDepth
//   fuse (box ∪ cyl boss)         = boxVol + πr²·protrudingLength
//   common(box ∩ cyl segment)     = πr²·overlapLength
// Because the result carries a TRUE curved surface, its watertight mesh only
// approximates the analytic volume (deflection-bounded), so the tolerance is the
// tessellation bound (1% relative + a small absolute floor), NOT the exact-planar
// 1e-6. Returns {matched, applicable}: applicable=false means "not a recognised
// box-cylinder pair" and the caller should use the exact planar set-algebra check.
struct CurvedCheck {
    bool applicable = false;
    bool matched = false;
};
CurvedCheck curvedBooleanVerified(const ntopo::Shape& result, const ntopo::Shape& a,
                                  const ntopo::Shape& b, int op) {
    namespace cv = cybercad::native::boolean::curved;
    const auto aBox = cv::recogniseBox(a);
    const auto bBox = cv::recogniseBox(b);
    const auto aCyl = aBox ? std::nullopt : cv::recogniseCylinder(a);
    const auto bCyl = bBox ? std::nullopt : cv::recogniseCylinder(b);
    const bool pair = (aBox && bCyl) || (aCyl && bBox);
    if (!pair) return {};  // not applicable → planar path decides

    const cv::AABox box = aBox ? *aBox : *bBox;
    const cv::AxisCylinder cyl = aCyl ? *aCyl : *bCyl;
    const int axis = cyl.axis;
    const double r2 = cyl.radius * cyl.radius;

    double expected = 0.0;
    switch (op) {
        case 0: {  // fuse (boss): protruding length past the box hi face
            const double protrude = std::max(0.0, cyl.hi - box.hi[axis]);
            expected = box.volume() + cv::kPi * r2 * protrude;
            break;
        }
        case 1: {  // cut a − b: round through hole removes a full box-depth cylinder
            expected = box.volume() - cv::kPi * r2 * box.size(axis);
            break;
        }
        case 2: {  // common: cylinder segment clipped to the box axial extent
            const double lo = std::max(cyl.lo, box.lo[axis]);
            const double hi = std::min(cyl.hi, box.hi[axis]);
            expected = cv::kPi * r2 * std::max(0.0, hi - lo);
            break;
        }
        default: return {/*applicable=*/true, /*matched=*/false};
    }
    if (!(expected > 0.0)) return {true, false};
    const double vr = watertightVolume(result);
    if (vr < 0.0) return {true, false};  // not watertight
    const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
    return {true, std::fabs(vr - expected) <= tol};
}

// SELF-VERIFY the native boolean result against SET-ALGEBRA volume (mandatory guard,
// NATIVE-REWRITE.md #5). The result must be a closed watertight 2-manifold AND carry
// the volume the op's set algebra predicts from the operands, to a relative tolerance:
//   fuse   V = volA + volB − volCommon
//   cut    V = volA − volCommon
//   common V = volCommon
// where volCommon is measured by the native common operation on the SAME operands
// (its own watertightness is required too). This catches a boolean that is closed but
// geometrically WRONG (a mis-classified fragment) — such a result is DISCARDED and the
// engine falls through / errors rather than emit a wrong solid. For the planar domain
// every mesh is exact, so the tolerance is tight (1e-6 relative, 1e-9 absolute floor).
// For the CURVED box-cylinder slice the ANALYTIC check (curvedBooleanVerified) takes
// over — its expected volume is closed-form, not a mesh estimate of the operands.
// SSI STAGE S5-a self-verify (openspec add-native-ssi-curved-boolean). The SSI-driven
// path (src/native/boolean/ssi_boolean) builds the curved boolean of two transversal
// elementary curved solids from the S3 TraceSet. Its host analytic oracle is the
// STEINMETZ solid: the COMMON of two equal-radius cylinders whose axes cross at right
// angles has exact enclosed volume 16·r³/3. When the operands are recognised as two
// such cylinders and op == common, the guard checks the result's watertight mesh
// volume against that closed form (deflection-bounded tolerance, like the box-cylinder
// analytic guard). Returns {applicable, matched}; applicable == false means "not a
// recognised Steinmetz pair" and the caller uses the generic set-algebra check.
//
// This keeps the mandatory watertight + correct-volume guard in the ENGINE (next to
// the OCCT fallback), so the library stays OCCT-free and a bad SSI candidate is
// DISCARDED → OCCT, never faked.
CurvedCheck ssiCurvedBooleanVerified(const ntopo::Shape& result, const ntopo::Shape& a,
                                     const ntopo::Shape& b, int op) {
#if defined(CYBERCAD_HAS_NUMSCI)
    if (op != 2) return {};  // only COMMON has an analytic closed-form oracle in S5
    namespace nb = cybercad::native::boolean;
    namespace nm = cybercad::native::math;
    namespace cv = cybercad::native::boolean::curved;  // kPi
    const auto csA = nb::ssidetail::recogniseCurvedSolid(a);
    const auto csB = nb::ssidetail::recogniseCurvedSolid(b);
    if (!csA || !csB) return {};
    using CK = nb::ssidetail::CurvedKind;

    // ── Steinmetz arm: two EQUAL-R PERPENDICULAR cylinders (common = 16 r³/3). ──
    if (csA->kind == CK::Cylinder && csB->kind == CK::Cylinder) {
        if (std::fabs(csA->radius - csB->radius) > 1e-6) return {};
        const double axisDot = std::fabs(nm::dot(csA->frame.z.vec(), csB->frame.z.vec()));
        if (axisDot > 1e-4) return {};  // not perpendicular → no closed form here
        const double r = csA->radius;
        const double expected = 16.0 * r * r * r / 3.0;  // Steinmetz bicylinder common volume
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-e arm: COAXIAL cone(frustum)∩cylinder COMMON (single analytic circle). ──
    // Overlap = solid of revolution of r ≤ min(r_cone(s), Rc) over the shared axial span
    // [sLo,sHi]; r_cone crosses Rc once at s*, giving two frustum bands whose volume is
    // closed-form: V = V_frustum(rBot→Rc over [sLo,s*]) + V_frustum(Rc→rTop over [s*,sHi]),
    // V_frustum(ra,rb,Δh) = (π Δh/3)(ra²+ra·rb+rb²). Mirrors the buildConeCylCommon geometry.
    const auto* cone = csA->kind == CK::Cone ? &*csA : (csB->kind == CK::Cone ? &*csB : nullptr);
    const auto* cyl = csA->kind == CK::Cylinder ? &*csA : (csB->kind == CK::Cylinder ? &*csB : nullptr);
    if (cone && cyl) {
        const nm::Vec3 zc = cone->frame.z.vec();
        if (nm::norm(nm::cross(zc, cyl->frame.z.vec())) > 1e-6) return {};  // axes not parallel
        const nm::Vec3 d = cyl->frame.origin - cone->frame.origin;
        if (nm::norm(d - zc * nm::dot(d, zc)) > 1e-6) return {};  // origins not colinear → not coaxial
        const double tanA = std::tan(cone->semiAngle);
        if (std::fabs(tanA) < 1e-9) return {};
        const double Rc = cyl->radius;
        const double base = nm::dot(cyl->frame.origin - cone->frame.origin, zc);
        const double sgn = nm::dot(cyl->frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
        double cylSLo = base + sgn * cyl->vLo, cylSHi = base + sgn * cyl->vHi;
        if (cylSLo > cylSHi) std::swap(cylSLo, cylSHi);
        const double sLo = std::max(cone->vLo, cylSLo);
        const double sHi = std::min(cone->vHi, cylSHi);
        if (!(sHi - sLo > 1e-6)) return {};
        const double sStar = (Rc - cone->radius) / tanA;
        if (!(sStar - sLo > 1e-6) || !(sHi - sStar > 1e-6)) return {};
        auto rCone = [&](double s) { return cone->radius + s * tanA; };
        auto frustum = [&](double ra, double rb, double dh) {
            return cv::kPi * dh / 3.0 * (ra * ra + ra * rb + rb * rb);
        };
        const double rBot = std::min(rCone(sLo), Rc);
        const double rTop = std::min(rCone(sHi), Rc);
        const double expected =
            frustum(rBot, Rc, sStar - sLo) + frustum(Rc, rTop, sHi - sStar);
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-f arm: COAXIAL cone(frustum)∩sphere COMMON (single analytic circle). ──
    // The overlap cross-section at axial s has radius min(r_cone(s), r_sphere(s)); r_cone
    // crosses r_sphere once at the seam s*. Below s* the cone limits (a frustum segment),
    // above s* the sphere limits (a spherical segment) — both closed-form. Mirrors the
    // buildConeSphereCommon geometry. Applicable only for the clean single-crossing config;
    // anything else returns {} → the generic set-algebra check applies.
    const auto* sph = csA->kind == CK::Sphere ? &*csA : (csB->kind == CK::Sphere ? &*csB : nullptr);
    if (cone && sph) {
        const nm::Vec3 zc = cone->frame.z.vec();
        const nm::Vec3 d = sph->frame.origin - cone->frame.origin;
        const double sc = nm::dot(d, zc);
        if (nm::norm(d - zc * sc) > 1e-6) return {};  // sphere centre off axis → not coaxial
        const double tanA = std::tan(cone->semiAngle);
        if (std::fabs(tanA) < 1e-9) return {};
        const double R0 = cone->radius, Rs = sph->radius;
        if (!(Rs > 1e-9)) return {};
        // Seam quadratic (1+tanA²)s² + 2(R0·tanA − sc)s + (sc² + R0² − Rs²) = 0.
        const double Aq = 1.0 + tanA * tanA;
        const double Bq = 2.0 * (R0 * tanA - sc);
        const double Cq = sc * sc + R0 * R0 - Rs * Rs;
        const double disc = Bq * Bq - 4.0 * Aq * Cq;
        if (disc <= 1e-12) return {};
        const double sqd = std::sqrt(disc);
        const double roots[2] = {(-Bq + sqd) / (2.0 * Aq), (-Bq - sqd) / (2.0 * Aq)};
        int nin = 0;
        double sStar = 0.0;
        for (const double r : roots)
            if (r > cone->vLo + 1e-6 && r < cone->vHi - 1e-6) { ++nin; sStar = r; }
        if (nin != 1) return {};
        auto rCone = [&](double s) { return R0 + s * tanA; };
        if (!(rCone(sStar) > 1e-9)) return {};
        // In-cone pole (single-crossing config): the axial pole inside the cone extent.
        const double sPoleP = sc + Rs, sPoleM = sc - Rs;
        const bool inP = sPoleP > cone->vLo && sPoleP < cone->vHi && rCone(sPoleP) > 0.0;
        const bool inM = sPoleM > cone->vLo && sPoleM < cone->vHi && rCone(sPoleM) > 0.0;
        if (inP == inM) return {};  // not exactly one interior pole → not the clean config
        const double inDir = inP ? 1.0 : -1.0;
        const double sPole = sc + Rs * inDir;
        const double coneNear = (inDir > 0.0) ? cone->vLo : cone->vHi;
        // Cone-limited span [coneNear, s*]; sphere-limited span [s*, sPole] (ordered).
        const double a1 = std::min(coneNear, sStar), b1 = std::max(coneNear, sStar);
        const double a2 = std::min(sStar, sPole), b2 = std::max(sStar, sPole);
        auto frustum = [&](double ra, double rb, double dh) {
            return cv::kPi * dh / 3.0 * (ra * ra + ra * rb + rb * rb);
        };
        auto sphF = [&](double s) { const double z = s - sc; return Rs * Rs * z - z * z * z / 3.0; };
        const double expected = frustum(rCone(a1), rCone(b1), b1 - a1) + cv::kPi * (sphF(b2) - sphF(a2));
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-l arm: COAXIAL torus∩cylinder COMMON (two analytic circle seams). ──
    // The COMMON is the ρ ≤ Rc part of the torus tube, revolved about the axis. In the
    // meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0); the cylinder
    // is the vertical chord ρ = Rc (d = Rc − R). By Pappus V = 2π·(R·A_seg + M), where the
    // INNER (ρ ≤ Rc) circular segment has area A_seg = πr² − (r²·acos(d/r) − d·√(r²−d²)) and
    // first moment about ρ=R of M = −(2/3)(r²−d²)^{3/2}. Airtight closed form; mirrors the
    // buildTorusCylCommon geometry. Applicable only for the clean two-circle poke-through.
    const auto* tor = csA->kind == CK::Torus ? &*csA : (csB->kind == CK::Torus ? &*csB : nullptr);
    if (tor && cyl) {
        const nm::Vec3 zc = tor->frame.z.vec();
        if (nm::norm(nm::cross(zc, cyl->frame.z.vec())) > 1e-6) return {};  // axes not parallel
        const nm::Vec3 d = cyl->frame.origin - tor->frame.origin;
        if (nm::norm(d - zc * nm::dot(d, zc)) > 1e-6) return {};  // origins not colinear → not coaxial
        const double R = tor->radius, r = tor->minorRadius, Rc = cyl->radius;
        if (!(r > 1e-9) || !(R > r + 1e-9) || !(Rc > 1e-9)) return {};
        const double dc = Rc - R;
        if (!(std::fabs(dc) < r - 1e-6)) return {};  // not a proper two-circle crossing
        const double root = std::sqrt(std::max(r * r - dc * dc, 0.0));
        const double aCap = r * r * std::acos(std::clamp(dc / r, -1.0, 1.0)) - dc * root;  // ρ>Rc segment
        const double aSeg = cv::kPi * r * r - aCap;                                         // ρ≤Rc segment
        const double mom = -(2.0 / 3.0) * root * root * root;                                // first moment
        const double expected = 2.0 * cv::kPi * (R * aSeg + mom);
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-m arm: COAXIAL torus∩sphere COMMON (two analytic circle seams). ──
    // The sphere centre sits ON the torus axis AT the torus centre (the symmetric pose). In
    // the meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0); the sphere
    // is the circle ρ²+z²=Rs². Both meet at the SAME radius ρ* = (R²−r²+Rs²)/(2R) and z=±z0,
    // z0=√(r²−(ρ*−R)²). The COMMON is the ρ ≤ √(Rs²−z²) part of the tube, revolved. By Pappus
    // V = 2π·[ (Rs²−R²−r²)·z0 + R·(z0·√(r²−z0²) + r²·asin(z0/r)) ] (the −z²/+z² terms cancel).
    // Airtight closed form; mirrors the buildTorusSphereCommon geometry. Applicable only for
    // the clean symmetric two-circle poke-through.
    if (tor && sph) {
        const nm::Vec3 zc = tor->frame.z.vec();
        const nm::Vec3 d = sph->frame.origin - tor->frame.origin;
        if (nm::norm(d) > 1e-6) return {};  // sphere not at the torus centre → not the clean pose
        const double R = tor->radius, r = tor->minorRadius, Rs = sph->radius;
        if (!(r > 1e-9) || !(R > r + 1e-9) || !(Rs > 1e-9)) return {};
        const double rhoStar = (R * R - r * r + Rs * Rs) / (2.0 * R);
        if (!(rhoStar > 1e-9)) return {};
        const double dc = rhoStar - R;
        if (!(std::fabs(dc) < r - 1e-6)) return {};  // not a proper two-circle crossing
        const double z0 = std::sqrt(std::max(r * r - dc * dc, 0.0));
        const double innerInt = z0 * std::sqrt(std::max(r * r - z0 * z0, 0.0)) +
                                r * r * std::asin(std::clamp(z0 / r, -1.0, 1.0));
        const double expected = 2.0 * cv::kPi * ((Rs * Rs - R * R - r * r) * z0 + R * innerInt);
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-n arm: COAXIAL torus∩cone COMMON (two analytic circle seams). ──
    // The cone is a COAXIAL frustum whose OBLIQUE wall ρ = a + b·z (b = ±tanα) crosses the tube
    // in a SLANTED chord (the S5-l cylinder is the b=0 vertical-chord special case). In the
    // meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0). The COMMON is the
    // ρ ≤ a+b·z part of the tube revolved. Working about the tube centre (ρ'=ρ−R), the chord has
    // unit normal m̂=(1,−b)/√(1+b²) into the DISCARDED (ρ>line) region and signed offset
    // t0=(a−R)/√(1+b²). The discarded circular segment has area A_d = r²acos(t0/r)−t0√(r²−t0²)
    // and ρ'-moment (1/√(1+b²))·(2/3)(r²−t0²)^{3/2}, so the KEPT segment has area A_seg=πr²−A_d
    // and ρ'-moment M=−(1/√(1+b²))·(2/3)(r²−t0²)^{3/2}, and by Pappus V=2π(R·A_seg+M). Airtight
    // closed form; REDUCES to the S5-l torus∩cylinder formula at b=0. Mirrors buildTorusConeCommon.
    if (tor && cone) {
        const nm::Vec3 zc = tor->frame.z.vec();
        if (nm::norm(nm::cross(zc, cone->frame.z.vec())) > 1e-6) return {};  // axes not parallel
        const nm::Vec3 d = cone->frame.origin - tor->frame.origin;
        if (nm::norm(d - zc * nm::dot(d, zc)) > 1e-6) return {};  // origins not colinear → not coaxial
        const double R = tor->radius, r = tor->minorRadius;
        if (!(r > 1e-9) || !(R > r + 1e-9)) return {};
        const double tanA = std::tan(cone->semiAngle);
        if (std::fabs(tanA) < 1e-6) return {};  // near-cylindrical → S5-l owns it
        const double base = nm::dot(d, zc);
        const double sgn = nm::dot(cone->frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
        const double b = sgn * tanA;
        const double a = cone->radius - sgn * base * tanA;
        // Proper two-circle poke-through: two distinct real roots strictly inside the tube.
        const double Aq = 1.0 + b * b, Bq = 2.0 * b * (a - R), Cq = (a - R) * (a - R) - r * r;
        const double disc = Bq * Bq - 4.0 * Aq * Cq;
        if (!(disc > 1e-9)) return {};
        const double sq = std::sqrt(disc);
        double z1 = (-Bq - sq) / (2.0 * Aq), z2 = (-Bq + sq) / (2.0 * Aq);
        if (z1 > z2) std::swap(z1, z2);
        if (!(z2 - z1 > 1e-6) || !(a + b * z1 > 1e-9) || !(a + b * z2 > 1e-9)) return {};
        const double invNorm = 1.0 / std::sqrt(1.0 + b * b);
        const double t0 = (a - R) * invNorm;
        if (!(std::fabs(t0) < r - 1e-9)) return {};
        const double root = std::sqrt(std::max(r * r - t0 * t0, 0.0));
        const double aCap = r * r * std::acos(std::clamp(t0 / r, -1.0, 1.0)) - t0 * root;  // discarded
        const double aSeg = cv::kPi * r * r - aCap;                                         // kept (ρ≤line)
        const double mom = -invNorm * (2.0 / 3.0) * root * root * root;                     // kept ρ'-moment
        const double expected = 2.0 * cv::kPi * (R * aSeg + mom);
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }

    // ── S5-o arm: COAXIAL torus∩torus COMMON (two analytic circle seams). ──
    // Two coaxial ring tori: tube A the meridian disk radius r1 centred (R1,zA), tube B radius r2
    // centred (R2,zB). The COMMON is the revolved LENS (disk A ∩ disk B). Two circles meet at the
    // chord offset a = (D²+r1²−r2²)/(2D) from A along the centre line (D = centre distance),
    // half-chord h = √(r1²−a²). By Pappus the lens revolves to 2π times the sum of the two
    // circular-segment first-moments about the axis; the equal-h moment terms cancel, leaving
    // V = 2π·(R1·A_segA + R2·A_segB), A_segA = r1²acos(a/r1)−a·h, A_segB = r2²acos((D−a)/r2)−(D−a)·h.
    // Airtight closed form; mirrors the buildTorusTorusCommon geometry.
    if (csA->kind == CK::Torus && csB->kind == CK::Torus) {
        const nm::Vec3 zc = csA->frame.z.vec();
        if (nm::norm(nm::cross(zc, csB->frame.z.vec())) > 1e-6) return {};  // axes not parallel
        const nm::Vec3 d = csB->frame.origin - csA->frame.origin;
        if (nm::norm(d - zc * nm::dot(d, zc)) > 1e-6) return {};  // origins not colinear → not coaxial
        const double R1 = csA->radius, r1 = csA->minorRadius;
        const double R2 = csB->radius, r2 = csB->minorRadius;
        if (!(r1 > 1e-9) || !(R1 > r1 + 1e-9)) return {};
        if (!(r2 > 1e-9) || !(R2 > r2 + 1e-9)) return {};
        const double cr = R2 - R1, cz = nm::dot(d, zc);   // meridian centre offset (ρ, z)
        const double D = std::hypot(cr, cz);
        if (!(D > 1e-9) || !(D < r1 + r2 - 1e-6) || !(D > std::fabs(r1 - r2) + 1e-6)) return {};
        const double a = (D * D + r1 * r1 - r2 * r2) / (2.0 * D);
        const double h2 = r1 * r1 - a * a;
        if (!(h2 > 1e-12)) return {};
        const double h = std::sqrt(h2);
        const double aSegA = r1 * r1 * std::acos(std::clamp(a / r1, -1.0, 1.0)) - a * h;
        const double aSegB = r2 * r2 * std::acos(std::clamp((D - a) / r2, -1.0, 1.0)) - (D - a) * h;
        const double expected = 2.0 * cv::kPi * (R1 * aSegA + R2 * aSegB);
        const double vr = watertightVolume(result);
        if (vr < 0.0) return {true, false};  // not watertight
        const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
        return {true, std::fabs(vr - expected) <= tol};
    }
    return {};
#else
    (void)result; (void)a; (void)b; (void)op;
    return {};  // no substrate → no SSI path → generic check applies
#endif
}

bool booleanResultVerified(const ntopo::Shape& result, const ntopo::Shape& a,
                           const ntopo::Shape& b, int op) {
    // Curved slice first: an analytic-volume check for a box-cylinder pair.
    if (const CurvedCheck cc = curvedBooleanVerified(result, a, b, op); cc.applicable)
        return cc.matched;
    // SSI Stage S5-a: the Steinmetz analytic oracle for a transversal cylinder∩cylinder
    // common. A recognised pair whose mesh volume matches 16r³/3 is accepted; a mismatch
    // is DISCARDED → OCCT.
    if (const CurvedCheck cc = ssiCurvedBooleanVerified(result, a, b, op); cc.applicable)
        return cc.matched;

    const double vr = watertightVolume(result);
    if (vr < 0.0) return false;  // not watertight

    const double va = watertightVolume(a);
    const double vb = watertightVolume(b);
    if (va < 0.0 || vb < 0.0) return true;  // operands not measurable → trust watertight

    // Measure the overlap volume with the native common op (independent path).
    const ntopo::Shape common =
        cybercad::native::boolean::boolean_solid(a, b, cybercad::native::boolean::Op::Common);
    const double vc = common.isNull() ? 0.0 : std::max(0.0, watertightVolume(common));

    double expected = 0.0;
    switch (op) {
        case 0: expected = va + vb - vc; break;  // fuse
        case 1: expected = va - vc; break;       // cut a−b
        case 2: expected = vc; break;            // common
        default: return false;
    }
    if (!(expected > 0.0)) return false;  // an empty/degenerate result is not a valid solid

    // Curved operands (a recognised cylinder/sphere/cone/torus solid) mesh with an
    // O(deflection) tessellation bias, and that bias appears INDEPENDENTLY in va, vb, vc
    // and vr — it does NOT cancel across the set-algebra, so the exact-planar 1e-6 band is
    // unattainable even for a correct watertight curved CUT/FUSE (the S5 curved-boolean
    // families: COMMON is intercepted by the analytic oracle above, but CUT/FUSE fall
    // through to here). The result mesh itself matches the EXACT set-algebra to <0.05%
    // (measured); the residual seen here is the operand-mesh bias (~1% at deflection
    // 0.005). Widen ONLY when an operand is analytically curved, to a deflection-bounded
    // band (2% — the same order the NURBS orchestrator's analyticVolumeBandOk uses). A
    // wrong/leaky result differs by a whole-feature volume, so it is still rejected
    // (DISAGREED=0); an all-planar pair meshes exactly and keeps the strict 1e-6 contract.
    bool curvedOperand = false;
#if defined(CYBERCAD_HAS_NUMSCI)
    // recogniseCurvedSolid lives in the numsci-gated ssidetail namespace (it consumes the
    // S3 tracer path); a no-numsci build has no curved-boolean families, so the strict
    // planar band is the only path and this stays false.
    curvedOperand =
        cybercad::native::boolean::ssidetail::recogniseCurvedSolid(a).has_value() ||
        cybercad::native::boolean::ssidetail::recogniseCurvedSolid(b).has_value();
#endif
    const double relBand = curvedOperand ? 2e-2 : 1e-6;
    const double tol = std::max(relBand * expected, 1e-9);
    return std::fabs(vr - expected) <= tol;
}

// SELF-VERIFY a native BLEND result (Phase 4 #6 native-blends). MANDATORY guard: a
// blend is accepted native ONLY when its result is a valid watertight solid with a
// SANE VOLUME SIGN vs the original body — else the candidate is DISCARDED and the op
// falls through to OCCT (never a wrong/leaky solid). The expected sign per op:
//   * chamfer / fillet — a convex-edge blend REDUCES volume: 0 < Vr < Vorig.
//   * offset_face grow (distance > 0) — GROWS: Vr > Vorig.
//   * offset_face shrink / shell — stays positive but SMALLER: 0 < Vr < Vorig.
// `wantGrow` selects the grow inequality; otherwise the result must be a valid
// watertight positive-volume solid strictly smaller than the original.
bool blendResultVerified(const ntopo::Shape& result, const ntopo::Shape& original, bool wantGrow) {
    const double vr = watertightVolume(result);
    if (vr <= 0.0) return false;  // not watertight or empty → reject
    const double vo = watertightVolume(original);
    if (vo <= 0.0) return true;   // original not measurable → trust watertight+positive
    const double tol = std::max(1e-6 * vo, 1e-9);
    if (wantGrow) return vr > vo + tol;  // offset grow
    return vr < vo - tol;                // chamfer / fillet / shell / offset-shrink
}

// SELF-VERIFY a native WRAP-EMBOSS result (Phase 4 #7 native-wrap-emboss). MANDATORY
// guard: the volume change is ANALYTIC. An emboss (boss=1) RAISES a pad so the volume
// GROWS by ≈ footprint area × height; a deboss (boss=0) RECESSES a pocket so it SHRINKS
// by ≈ footprint area × depth. The footprint area is the TRUE signed (shoelace) polygon
// area — exact for the rectangle (== its bbox) and correct for any N-vertex polygon
// (T2) where the bbox would badly over-state a thin / L-shaped loop. Because the
// profile's px is ALREADY arc-length, that (u,v) area is the flat profile area,
// independent of R; the small curvature term (h/2R) stays inside the tolerance. The
// result carries TRUE curved (cylindrical) faces, so its watertight mesh volume only
// approximates the analytic target — deflection-bounded tolerance (1% relative + a small
// floor). A NULL or failing candidate is DISCARDED -> OCCT cc_wrap_emboss.
bool wrapEmbossVerified(const ntopo::Shape& result, const ntopo::Shape& original, int faceId,
                        const double* profileXY, int count, double height, int boss) {
    const double vr = watertightVolume(result);
    if (vr <= 0.0) return false;  // not watertight or empty → reject
    const double vo = watertightVolume(original);
    if (vo <= 0.0) return true;   // original not measurable → trust watertight+positive
    if (profileXY == nullptr || count < 3 || !(height > 0.0)) return false;

    // F5 FREEFORM (curved) base: a sphere-cap pole boss has an EXACT spherical-shell-sector
    // volume delta (2π(1−cosφ0)·((R+h)³−R³)/3), which does NOT equal footArea×height. When
    // the picked face is a recognised sphere-cap dome wall (boss=1), gate against that closed
    // form instead of the developable-cylinder area×height rule. Here the signed volume is
    // load-bearing, so ALSO require the landed orientation invariant (isConsistentlyOriented):
    // an inconsistently-wound-but-watertight shell has a meaningless signed volume. The
    // cylinder arm (below) keeps its original watertight-only acceptance — its planar-facet
    // window/pad soup can weld watertight without a globally consistent winding while still
    // metering the correct footArea×height delta, so requiring consistency there would REGRESS
    // the landed rectangular/polygon cylinder cases.
    if (boss == 1) {
        const double dSphere =
            cybercad::native::feature::spherePoleBossVolumeDelta(original, faceId, profileXY,
                                                                 count, height);
        if (dSphere > 0.0) {
            ntess::MeshParams mp;
            mp.deflection = 0.005;
            const ntess::Mesh m = ntess::SolidMesher{mp}.mesh(result);
            if (!ntess::isConsistentlyOriented(m)) return false;
            const double expected = vo + dSphere;
            const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
            return std::fabs(vr - expected) <= tol;
        }
    }

    double a2 = 0.0;  // twice the signed shoelace area
    for (int i = 0; i < count; ++i) {
        const int j = (i + 1) % count;
        a2 += profileXY[i * 2] * profileXY[j * 2 + 1] - profileXY[j * 2] * profileXY[i * 2 + 1];
    }
    const double footArea = std::fabs(a2) * 0.5;
    if (!(footArea > 0.0)) return false;
    const double sign = (boss == 1) ? 1.0 : -1.0;  // emboss grows, deboss shrinks
    const double expected = vo + sign * footArea * height;
    if (!(expected > 0.0)) return false;  // a pocket cannot remove more than the body holds
    const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
    return std::fabs(vr - expected) <= tol;
}

}  // namespace

// ── Fallback engine factory (build-specific) ───────────────────────────────────
// The always-compiled stub provides make_stub_engine(); the OCCT adapter provides
// OcctEngine only under CYBERCAD_HAS_OCCT.
std::shared_ptr<IEngine> make_stub_engine();

std::shared_ptr<IEngine> make_native_fallback_engine() {
#ifdef CYBERCAD_HAS_OCCT
    return std::make_shared<OcctEngine>();
#else
    return make_stub_engine();
#endif
}

// ADDITIVE — the bare analytic RING-TORUS primitive (see native_engine.h). Builds
// the native single-face Kind::Torus solid via construct::build_torus and wraps it as
// a process-native EngineShape so the exact-NURBS boolean recognises it. An engine-
// agnostic factory: it does NOT touch active_engine() or any IEngine vtable — the
// torus is always a native body (the OCCT engine has no equivalent bare-torus-face
// primitive the boolean's recogniseCurvedSolid consumes).
ShapeResult make_native_torus(const double centre[3], const double axis[3], double majorRadius,
                              double minorRadius) {
    if (centre == nullptr || axis == nullptr) {
        return ShapeResult::fail(cyber::make_error("cc_torus: null centre/axis"));
    }
    const nmath::Vec3 a{axis[0], axis[1], axis[2]};
    const double an = nmath::norm(a);
    if (!(an > 1e-12)) {
        return ShapeResult::fail(cyber::make_error("cc_torus: degenerate (zero-length) axis"));
    }
    const nmath::Point3 c{centre[0], centre[1], centre[2]};
    const nmath::Dir3 axDir{a * (1.0 / an)};
    ntopo::Shape solid = ncst::build_torus(majorRadius, minorRadius, c, axDir);
    if (solid.isNull()) {
        return ShapeResult::fail(cyber::make_error(
            "cc_torus: not a ring torus (require majorRadius > minorRadius > 0)"));
    }
    return ShapeResult::ok(wrapNative(std::move(solid)));
}

// ── M8 scoped-unlink DRY-RUN rehearsal (NON-SHIPPING) ───────────────────────────
// Under -DCYBERCAD_M8_REHEARSAL=ON (implies no OCCT), the build's DEFAULT active
// engine becomes a NativeEngine whose only fallback is the stub — the exact
// post-unlink product wiring, WITHOUT deleting src/engine/occt or touching any
// shipped default. Every cc_* op then either serves natively or hits the stub's
// honest engine_unsupported decline (never an OCCT arm). This override replaces
// the no-OCCT stub create_default_engine() (guarded out in stub_engine.cpp under
// the same macro), so tests that never call cc_set_engine(1) still measure the
// native-only path. Measurement-only: see openspec/DROP-OCCT-READINESS.md §6.
#if defined(CYBERCAD_M8_REHEARSAL) && !defined(CYBERCAD_HAS_OCCT)
std::shared_ptr<IEngine> create_default_engine() {
    return std::make_shared<NativeEngine>(make_native_fallback_engine());
}
#endif

// ── engine-INTERNAL native-heal hook (Phase 4 #4 native-healing) ────────────────
// Native builder → mandatory self-verify → OCCT fallback, the SAME discipline every
// native op follows (see native_heal_hook.h). Reached internally by the engine, NOT
// via cc_* — no ABI/IEngine change. healShell already self-verifies (watertight +
// enclosed volume > 0) before returning Healed, so a Healed result is trustworthy;
// an Unhealed result carries the honest measured residual and defers to OCCT.
HealOutcome tryNativeHeal(const ntopo::Shape& shape, double tolerance) {
    HealOutcome outcome;
    namespace nheal = cybercad::native::heal;
    outcome.native = nheal::healShell(shape, nheal::HealOptions{tolerance});
    if (outcome.native.status == nheal::HealStatus::Healed) {
        outcome.source = HealSource::Native;  // self-verified watertight + valid
        return outcome;
    }
#ifdef CYBERCAD_HAS_OCCT
    // DEFER: the native slice is out of scope for this defect. The OCCT sewing /
    // ShapeFix oracle (cyber::occt::sewAndFix) runs on a TopoDS representation of
    // the soup, entirely inside src/engine/occt/ — src/native/** stays OCCT-free.
    // (This slice heals native-built soups; wiring a native→TopoDS converter for an
    // arbitrary imported soup is the STEP-import track's job. Until then the honest
    // outcome is Unhealed: the caller keeps the pristine input for OCCT.)
    outcome.source = HealSource::Unhealed;
#else
    outcome.source = HealSource::Unhealed;  // no OCCT linked → honest deferral
#endif
    return outcome;
}

// ── construction / lifecycle ────────────────────────────────────────────────────

NativeEngine::NativeEngine(std::shared_ptr<IEngine> fallback) : fallback_(std::move(fallback)) {}

IEngine& NativeEngine::fallback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fallback_) fallback_ = make_native_fallback_engine();
    return *fallback_;
}

std::string NativeEngine::name() const { return "native"; }
bool NativeEngine::available() const { return true; }

void NativeEngine::set_parallel(bool enabled) { fallback().set_parallel(enabled); }
bool NativeEngine::parallel_enabled() const { return fallback().parallel_enabled(); }
void NativeEngine::set_gpu_tessellation(bool enabled) { fallback().set_gpu_tessellation(enabled); }
bool NativeEngine::gpu_tessellation_enabled() const { return fallback().gpu_tessellation_enabled(); }

bool NativeEngine::isNative(const EngineShape& handle) const {
    // Instance-INDEPENDENT: a native body is identified by the process-wide live-
    // NativeShape registry, not this engine instance's bookkeeping, so a body built
    // under an earlier cc_set_engine(1) instance is still recognised by a later one
    // (and is NEVER misclassified as an OCCT body and forwarded to OCCT's unwrap).
    if (!handle) return false;
    return NativeShapeRegistry::contains(handle.get());
}

EngineShape NativeEngine::track(EngineShape handle) const {
    // Registration happens in the NativeShape ctor (process-wide), so track() is now
    // just a pass-through kept for call-site symmetry with wrapNative().
    return handle;
}

// ── NATIVE construct ops (fall through to OCCT for deferred cases) ──────────────

ShapeResult NativeEngine::solid_extrude(const double* profileXY, int pointCount, double depth) {
    ntopo::Shape solid = cybercad::native::construct::build_prism(profileXY, pointCount, depth);
    if (solid.isNull()) {
        // Degenerate input or a case the native builder defers: hand the SAME
        // arguments to the fallback engine (honest coexistence — no faking).
        return fallback().solid_extrude(profileXY, pointCount, depth);
    }
    return track(wrapNative(std::move(solid)));
}

ShapeResult NativeEngine::solid_revolve(const double* profileXY, int pointCount,
                                        double angleRadians) {
    ntopo::Shape solid = cybercad::native::construct::build_revolution(profileXY, pointCount,
                                                                       kRevolveYAxis, angleRadians);
    if (solid.isNull()) {
        return fallback().solid_revolve(profileXY, pointCount, angleRadians);
    }
    return track(wrapNative(std::move(solid)));
}

// ── body-consuming ops: native for a native body, else fallback ─────────────────

Result<MeshData> NativeEngine::tessellate(EngineShape body, double deflection) {
    if (!isNative(body)) return fallback().tessellate(body, deflection);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return toMeshData(holder->mesh);  // imported STL soup
    ntess::MeshParams params;
    if (deflection > 0.0) params.deflection = deflection;
    ntess::SolidMesher mesher(params);
    return toMeshData(mesher.mesh(holder->shape));
}

Result<std::vector<FaceMeshData>> NativeEngine::face_meshes(EngineShape body, double deflection) {
    if (!isNative(body)) return fallback().face_meshes(body, deflection);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) {
        // A triangle soup has no per-face topology: return the whole mesh as one
        // face (faceId 1), reusing toMeshData's flat layout.
        const MeshData md = toMeshData(holder->mesh);
        FaceMeshData fmd;
        fmd.faceId = 1;
        fmd.vertices = md.vertices;
        fmd.triangles = md.triangles;
        std::vector<FaceMeshData> single;
        single.push_back(std::move(fmd));
        return single;
    }
    ntess::MeshParams params;
    if (deflection > 0.0) params.deflection = deflection;
    ntess::FaceMesher fm(params);

    std::vector<FaceMeshData> out;
    int faceId = 0;
    for (ntopo::Explorer ex(holder->shape, ntopo::ShapeType::Face); ex.more(); ex.next()) {
        ++faceId;  // 1-based, matching the subshape-id convention
        const ntess::Mesh m = fm.mesh(ex.current());
        FaceMeshData fmd;
        fmd.faceId = faceId;
        fmd.vertices.reserve(m.vertices.size() * 3);
        for (const auto& p : m.vertices) {
            fmd.vertices.push_back(p.x);
            fmd.vertices.push_back(p.y);
            fmd.vertices.push_back(p.z);
        }
        fmd.triangles.reserve(m.triangles.size() * 3);
        for (const auto& t : m.triangles) {
            fmd.triangles.push_back(static_cast<int>(t.a));
            fmd.triangles.push_back(static_cast<int>(t.b));
            fmd.triangles.push_back(static_cast<int>(t.c));
        }
        out.push_back(std::move(fmd));
    }
    return out;
}

Result<MassData> NativeEngine::mass_properties(EngineShape body) {
    if (!isNative(body)) return fallback().mass_properties(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    // Imported STL: measure the welded soup directly (area always valid; volume +
    // valid only when the soup is watertight — "volume-if-closed"). A B-rep body
    // meshes at the property deflection first.
    ntess::Mesh brepMesh;
    if (!holder->isMesh) {
        ntess::MeshParams params;
        params.deflection = kPropertyDeflection;
        brepMesh = ntess::SolidMesher(params).mesh(holder->shape);
    }
    const ntess::Mesh& mesh = holder->isMesh ? holder->mesh : brepMesh;

    MassData out;
    out.area = ntess::surfaceArea(mesh);
    // enclosedVolume is signed with +Z-outward CCW winding; a well-formed native
    // solid meshes outward, so take the magnitude for the reported (positive) volume.
    out.volume = std::fabs(ntess::enclosedVolume(mesh));
    // Centroid from the same signed-tetra decomposition (divergence theorem):
    // C = (1/V) · ⅛ Σ (aᵢ+bᵢ+cᵢ scaled)… computed here as the volume-weighted
    // tetra centroid sum over the origin fan.
    double cx = 0.0, cy = 0.0, cz = 0.0, vol6 = 0.0;
    for (const auto& t : mesh.triangles) {
        const auto& A = mesh.vertices[t.a];
        const auto& B = mesh.vertices[t.b];
        const auto& C = mesh.vertices[t.c];
        const double v = A.x * (B.y * C.z - B.z * C.y) - A.y * (B.x * C.z - B.z * C.x) +
                         A.z * (B.x * C.y - B.y * C.x);  // 6·(signed tetra volume)
        vol6 += v;
        cx += v * (A.x + B.x + C.x);
        cy += v * (A.y + B.y + C.y);
        cz += v * (A.z + B.z + C.z);
    }
    if (std::fabs(vol6) > 1e-12) {
        const double inv = 1.0 / (4.0 * vol6);  // ¼ from the tetra centroid, /vol6
        out.cx = cx * inv;
        out.cy = cy * inv;
        out.cz = cz * inv;
    }
    out.valid = ntess::isWatertight(mesh) && out.volume > 0.0;
    return out;
}

Result<std::vector<double>> NativeEngine::bounding_box(EngineShape body) {
    if (!isNative(body)) return fallback().bounding_box(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) {
        // AABB over the imported soup vertices (the mesh IS the geometry).
        if (holder->mesh.vertices.empty())
            return make_error("bounding_box: imported STL mesh is empty");
        const auto& v0 = holder->mesh.vertices.front();
        double lo[3] = {v0.x, v0.y, v0.z}, hi[3] = {v0.x, v0.y, v0.z};
        for (const auto& p : holder->mesh.vertices) {
            const double c[3] = {p.x, p.y, p.z};
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], c[k]);
                hi[k] = std::max(hi[k], c[k]);
            }
        }
        return std::vector<double>{lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]};
    }
    // Bound the TESSELLATED body, not the raw B-rep vertices. A revolved solid's
    // topological vertices sit only at the angular STATIONS (e.g. θ = 0/120/240 for
    // a full turn), so a vertex-only AABB misses the circular extremes between them
    // (a full-radius error on a tube). Meshing at a tight deflection samples every
    // curved face densely, so the mesh AABB is within `deflection` of the exact
    // B-rep box — matching OCCT's BRepBndLib to the same bound the parity gate uses.
    // For planar prisms the mesh vertices are exact, so this is exact there too.
    ntess::MeshParams params;
    params.deflection = kPropertyDeflection;
    const ntess::Mesh mesh = ntess::SolidMesher(params).mesh(holder->shape);
    if (mesh.vertices.empty()) return make_error("bounding_box: native body meshed empty");
    const auto& v0 = mesh.vertices.front();
    double lo[3] = {v0.x, v0.y, v0.z}, hi[3] = {v0.x, v0.y, v0.z};
    for (const auto& p : mesh.vertices) {
        const double c[3] = {p.x, p.y, p.z};
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], c[k]);
            hi[k] = std::max(hi[k], c[k]);
        }
    }
    return std::vector<double>{lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]};
}

// Native topology query: Vertex/Edge/Face counts via the native Explorer, matching
// OCCT's TopExp::MapShapes + 1-based ids. kind 0 = vertex, 1 = edge, else face
// (the SAME selector the OCCT adapter's subshapeKind uses). An OCCT body forwards.
Result<std::vector<int>> NativeEngine::subshape_ids(EngineShape body, int kind) {
    if (!isNative(body)) return fallback().subshape_ids(body, kind);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) {
        // A triangle soup has no B-rep topology: expose the mesh vertices (kind 0,
        // ids 1..vertexCount) and triangles as faces (kind 2, ids 1..triangleCount);
        // there are no distinct edge sub-shapes (kind 1 → empty).
        std::size_t n = 0;
        if (kind == 0) n = holder->mesh.vertices.size();
        else if (kind != 1) n = holder->mesh.triangles.size();
        std::vector<int> ids(n);
        for (std::size_t i = 0; i < n; ++i) ids[i] = static_cast<int>(i) + 1;
        return ids;
    }
    const ntopo::ShapeType type = kind == 0   ? ntopo::ShapeType::Vertex
                                  : kind == 1 ? ntopo::ShapeType::Edge
                                              : ntopo::ShapeType::Face;
    // MapShapes dedups shared sub-shapes by isSame (a shared vertex/edge is counted
    // once), so the count equals OCCT's indexed-map extent; ids are 1..count.
    const ntopo::ShapeMap map = ntopo::mapShapes(holder->shape, type);
    std::vector<int> ids(map.size());
    for (std::size_t i = 0; i < map.size(); ++i) ids[i] = static_cast<int>(i) + 1;
    return ids;
}

// ── Connected-solid enumeration (app-parity) ──────────────────────────────────
// The native Explorer over ShapeType::Solid emits the root itself when the root IS a
// solid and dedups shared solids by isSame — identical to OCCT's TopExp_Explorer for the
// app's use (splitting a disjoint boolean union into its connected lumps). A mesh body has
// no B-rep solids → 0. An OCCT body forwards to the fallback (the OCCT explorer).
Result<int> NativeEngine::shape_solid_count(EngineShape body) {
    if (!isNative(body)) return fallback().shape_solid_count(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return 0;
    int n = 0;
    for (ntopo::Explorer ex(holder->shape, ntopo::ShapeType::Solid); ex.more(); ex.next()) ++n;
    return n;
}

ShapeResult NativeEngine::shape_solid_at(EngineShape body, int index) {
    if (!isNative(body)) return fallback().shape_solid_at(body, index);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh || index < 0) return make_error("shape_solid_at: no solid at index");
    int i = 0;
    for (ntopo::Explorer ex(holder->shape, ntopo::ShapeType::Solid); ex.more(); ex.next(), ++i) {
        if (i == index) return wrapNative(ex.current());
    }
    return make_error("shape_solid_at: index out of range");
}

// ── MOAT M-GS GS3/GS4 analysis resolution ───────────────────────────────────
// A resolved measurement operand: its analysis::Entity plus the raw geometry kind
// (for per-cell guards) and face orientation (for the curvature-sign flip). The
// Entity's edge/face pointers alias the child TShape geometry, which the root
// shape keeps alive for the whole call.
namespace {
struct ResolvedSub {
    nan::Entity entity;
    int kind = 0;  // 0 vertex, 1 edge, 2 face
    ntopo::Orientation orient = ntopo::Orientation::Forward;
    const ntopo::FaceSurface* face = nullptr;
    const ntopo::EdgeCurve* edge = nullptr;
};

// Resolve a 1-based sub-shape id to a measurement Entity, or an honest decline.
// A non-identity placement is declined (world-coordinate safety), never guessed.
std::optional<ResolvedSub> resolveSub(const ntopo::Shape& root, int subKind, int subId,
                                      std::string& err) {
    const ntopo::ShapeType type = subKind == 0   ? ntopo::ShapeType::Vertex
                                  : subKind == 1 ? ntopo::ShapeType::Edge
                                                 : ntopo::ShapeType::Face;
    const ntopo::ShapeMap map = ntopo::mapShapes(root, type);
    if (subId < 1 || subId > static_cast<int>(map.size())) { err = "sub-shape id out of range"; return std::nullopt; }
    const ntopo::Shape& sub = map.shape(subId);
    if (!sub.location().isIdentity()) { err = "located sub-shape (non-identity placement) not supported"; return std::nullopt; }
    const auto& geom = sub.tshape()->geometry();
    ResolvedSub r; r.kind = subKind; r.orient = sub.orientation();
    if (subKind == 0) {
        const auto* p = std::get_if<nan::Point3>(&geom);
        if (!p) { err = "vertex has no point geometry"; return std::nullopt; }
        r.entity = nan::Entity::ofVertex(*p);
    } else if (subKind == 1) {
        const auto* c = std::get_if<ntopo::EdgeCurve>(&geom);
        if (!c) { err = "edge has no curve geometry"; return std::nullopt; }
        r.edge = c;
        r.entity = nan::Entity::ofEdge(*c, sub.tshape()->firstParam(), sub.tshape()->lastParam());
    } else {
        const auto* s = std::get_if<ntopo::FaceSurface>(&geom);
        if (!s) { err = "face has no surface geometry"; return std::nullopt; }
        r.face = s;
        r.entity = nan::Entity::ofFace(*s, 0.0, 1.0, 0.0, 1.0);  // benign window; windowless cells only
    }
    return r;
}
}  // namespace

Result<std::vector<double>> NativeEngine::surface_curvature(EngineShape body, int faceId,
                                                            double u, double v) {
    if (!isNative(body)) return fallback().surface_curvature(body, faceId, u, v);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("surface_curvature: mesh body has no surface geometry");
    std::string err;
    auto rs = resolveSub(holder->shape, 2, faceId, err);
    if (!rs) return make_error("surface_curvature: " + err);
    auto cur = nan::surfaceCurvature(*rs->face, u, v);
    if (!cur) return make_error("surface_curvature: declined (parametric singularity)");
    nan::SurfaceCurvature c = *cur;
    if (rs->orient == ntopo::Orientation::Reversed) {  // face normal reversed → flip H, k1/k2 (K invariant)
        const double k1 = -c.k2, k2 = -c.k1;           // keep k1 ≥ k2 after negation
        c.H = -c.H; c.k1 = k1; c.k2 = k2;
    }
    return std::vector<double>{c.K, c.H, c.k1, c.k2};
}

Result<std::vector<double>> NativeEngine::edge_curvature(EngineShape body, int edgeId, double t) {
    if (!isNative(body)) return fallback().edge_curvature(body, edgeId, t);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("edge_curvature: mesh body has no curve geometry");
    std::string err;
    auto rs = resolveSub(holder->shape, 1, edgeId, err);
    if (!rs) return make_error("edge_curvature: " + err);
    auto k = nan::edgeCurvature(*rs->edge, t);
    if (!k) return make_error("edge_curvature: declined (stationary/cusp point)");
    return std::vector<double>{*k};
}

Result<std::vector<double>> NativeEngine::measure_angle(EngineShape body, int subKindA, int subIdA,
                                                        int subKindB, int subIdB) {
    if (!isNative(body)) return fallback().measure_angle(body, subKindA, subIdA, subKindB, subIdB);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("measure_angle: mesh body has no B-rep geometry");
    std::string err;
    auto a = resolveSub(holder->shape, subKindA, subIdA, err);
    if (!a) return make_error("measure_angle: operand A " + err);
    auto b = resolveSub(holder->shape, subKindB, subIdB, err);
    if (!b) return make_error("measure_angle: operand B " + err);
    auto th = nan::angle(a->entity, b->entity);
    if (!th) return make_error("measure_angle: declined (only line·line, plane·plane, line·plane)");
    return std::vector<double>{*th};
}

Result<std::vector<double>> NativeEngine::measure_distance(EngineShape body, int subKindA,
                                                           int subIdA, int subKindB, int subIdB) {
#ifdef CYBERCAD_HAS_NUMSCI
    if (!isNative(body)) return fallback().measure_distance(body, subKindA, subIdA, subKindB, subIdB);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("measure_distance: mesh body has no B-rep geometry");
    std::string err;
    auto a = resolveSub(holder->shape, subKindA, subIdA, err);
    if (!a) return make_error("measure_distance: operand A " + err);
    auto b = resolveSub(holder->shape, subKindB, subIdB, err);
    if (!b) return make_error("measure_distance: operand B " + err);
    // A curved-face operand needs a certified (u,v) trim window we do not synthesise
    // here → honest decline (host scope: vertex / edge / planar-face cells).
    auto nonPlanarFace = [](const ResolvedSub& r) {
        return r.kind == 2 && r.face && r.face->kind != ntopo::FaceSurface::Kind::Plane;
    };
    if (nonPlanarFace(*a) || nonPlanarFace(*b))
        return make_error("measure_distance: curved-face operand declined (needs a trim window)");
    auto d = nan::minDistance(a->entity, b->entity);
    if (!d) return make_error("measure_distance: declined (non-certifiable minimizer)");
    return std::vector<double>{d->distance, d->p1.x, d->p1.y, d->p1.z, d->p2.x, d->p2.y, d->p2.z};
#else
    (void)body; (void)subKindA; (void)subIdA; (void)subKindB; (void)subIdB;
    return make_error("measure_distance: requires a CYBERCAD_HAS_NUMSCI build");
#endif
}

// ── Fallthrough helper for body-consuming ops with NO native implementation ─────
// A native body handed to such an op cannot be served (and must NOT be forwarded
// to OCCT, which would misread the void). Return a clean, honest Error; an OCCT
// body forwards normally.
#define CC_NATIVE_BODY_UNSUPPORTED(op, body)                                       \
    do {                                                                           \
        if (isNative(body))                                                        \
            return make_error("operation not supported on a native body yet: " op \
                              " (native scope: extrude/revolve + tessellate/mass/bbox)"); \
    } while (0)

// ── construct fallthrough ───────────────────────────────────────────────────────

// Legacy mesh extrude (cc_extrude): ATTEMPT NATIVE FIRST, fall through to OCCT on an
// honest decline. The native prism is the SAME build_prism solid_extrude uses (the
// OCCT adapter's extrude_mesh likewise builds a prism then tessellates at 0.1), so a
// case the native builder handles produces the identical geometry — meshed at the
// SAME 0.1 deflection here for a bbox/extents-identical result. A degenerate profile
// or a case the native builder defers returns a NULL solid → forward the SAME
// arguments to the fallback (honest coexistence — OCCT still catches the rest, no
// faking, no forwarding of a native void).
Result<MeshData> NativeEngine::extrude_mesh(const double* p, int n, double d) {
    ntopo::Shape solid = cybercad::native::construct::build_prism(p, n, d);
    if (solid.isNull()) return fallback().extrude_mesh(p, n, d);
    ntess::MeshParams params;
    params.deflection = 0.1;  // match the OCCT adapter's legacy 0.1 tessellation
    ntess::SolidMesher mesher(params);
    return toMeshData(mesher.mesh(solid));
}
// ── Tier-B (#4b) NATIVE ruled loft, incl. the T1 MISMATCHED-count breadth. NATIVE
// for equal-count sections AND for mismatched counts (the loop is resampled at the
// union of arc-length params — geometry-preserving; see construct/loft.h). Falls
// through for the deferred cases (a non-planar section, a degenerate/point section,
// guided/rail lofts). Because the mismatched path now builds a NON-trivial tiling,
// the result is SELF-VERIFIED robustly watertight with a positive enclosed volume
// before being kept native; any candidate that fails → forward the SAME args to
// OCCT (honest coexistence, never a faked or leaky solid). ────────────────────────
ShapeResult NativeEngine::solid_loft(const double* b, int bc, const double* t, int tc, double d) {
    ntopo::Shape solid = ncst::build_loft(b, bc, t, tc, d);
    if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
        return fallback().solid_loft(b, bc, t, tc, d);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::solid_loft_wires(const double* a, int ac, const double* b, int bc) {
    ntopo::Shape solid = ncst::build_loft_wires(a, ac, b, bc);
    if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
        return fallback().solid_loft_wires(a, ac, b, bc);
    return track(wrapNative(std::move(solid)));
}
// N-SECTION (≥3) ruled loft — the generalisation of solid_loft_wires. NATIVE for a
// chain of PLANAR sections (equal OR mismatched vertex counts): (N−1) ruled bands +
// two end caps, internal sections shared as vertex rings (loft.h build_loft_sections).
// The stacked skin is a non-trivial tiling, so the result is SELF-VERIFIED robustly
// watertight with a positive enclosed volume before being kept native; a non-planar /
// point-collapsed section, a self-folding chain, or any candidate that fails the gate
// → forward the SAME args to OCCT solid_loft_sections (honest coexistence, never a
// faked or leaky solid).
ShapeResult NativeEngine::solid_loft_sections(const double* s, const int* c, int sc) {
    ntopo::Shape solid = ncst::build_loft_sections(s, c, sc);
    if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
        return fallback().solid_loft_sections(s, c, sc);
    return track(wrapNative(std::move(solid)));
}
// ── Tier-C (#4b) NATIVE sweep. NATIVE for a STRAIGHT spine → an EXACT directional
// prism, AND for a SMOOTH CURVED spine → an RMF-transported ruled-band tube
// (deflection-bounded, watertight at working deflections), both cross-checked vs
// BRepOffsetAPI_MakePipe. A TIGHT-CURVATURE / self-intersecting spine, a degenerate
// profile, or < 2 path points → build_sweep returns NULL and we forward the SAME args
// to OCCT MakePipe (honest coexistence, no faking — see construct/sweep.h). ─────────
ShapeResult NativeEngine::solid_sweep(const double* p, int pc, const double* path, int pathc) {
    ntopo::Shape solid = ncst::build_sweep(p, pc, path, pathc);
    if (solid.isNull()) return fallback().solid_sweep(p, pc, path, pathc);
    return track(wrapNative(std::move(solid)));
}
// twisted_sweep: NATIVE for the plain reduction (twist ≈ 0, scale ≈ 1) AND for a REAL
// twist/scale — build_twisted_sweep densifies the spine to a bounded per-band twist and
// builds the Frenet-framed ruled ThruSections tube. The result is SELF-VERIFIED robustly
// watertight with a positive enclosed volume before being kept native; a self-folding
// tube (build returns NULL) or a candidate that fails the gate → forward the SAME args to
// OCCT twisted_sweep (honest coexistence, never a faked or leaky solid).
ShapeResult NativeEngine::twisted_sweep(const double* p, int pc, const double* path, int pathc,
                                        double tw, double se) {
    ntopo::Shape solid = ncst::build_twisted_sweep(p, pc, path, pathc, tw, se);
    if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
        return fallback().twisted_sweep(p, pc, path, pathc, tw, se);
    return track(wrapNative(std::move(solid)));
}
// ── Tier-2#4 (#4b) NATIVE general sweep: loft_along_rail + guided_sweep ──────────
// loft_along_rail is NATIVE for a STRAIGHT rail (a ruled loft between the two
// equal-count sections placed perpendicular to the rail tangent, matching
// MakePipeShell on a straight rail) AND for a SMOOTH CURVED rail (an RMF-transported
// section morph densified to a bounded per-band turn); mismatched section counts →
// NULL, and a rail too tight to weld fails the self-verify below → OCCT MakePipeShell.
// guided_sweep is NATIVE when the guide-scaled per-station Frenet ThruSections tube
// welds watertight and does not self-fold; a coincident guide start, degenerate
// input, or a SELF-INTERSECTING tube (which needs surface-surface intersection —
// Tier 4) → NULL → OCCT. Both results are SELF-VERIFIED robustly watertight before
// being kept native (never a faked or leaky solid).
ShapeResult NativeEngine::loft_along_rail(const double* r, int rc, const double* a, int ac,
                                          const double* b, int bc) {
    ntopo::Shape solid = ncst::build_loft_along_rail(r, rc, a, ac, b, bc);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().loft_along_rail(r, rc, a, ac, b, bc);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::guided_sweep(const double* p, int pc, const double* path, int pathc,
                                       const double* g, int gc) {
    ntopo::Shape solid = ncst::build_guided_sweep(p, pc, path, pathc, g, gc);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().guided_sweep(p, pc, path, pathc, g, gc);
    return track(wrapNative(std::move(solid)));
}
// guided_orient_sweep — the section ORIENTATION is fixed by a guide wire (OCCT
// MakePipeShell+SetMode(guide), NoContact / GeomFill_GuideTrihedronPlan rigid [N,B,T]
// law). NATIVE for a straight spine, where the perpendicular-plane∩guide intersection is
// parametrization-independent so native and OCCT agree SPATIALLY (bbox), not merely on
// volume — the M7a-hardened acceptance. The tube must weld robustly watertight (a coarse
// rotating guide that will not weld → fallback); a curved spine / degenerate guide →
// build_guided_orient_sweep returns NULL → OCCT.
ShapeResult NativeEngine::guided_orient_sweep(const double* p, int pc, const double* path,
                                              int pathc, const double* g, int gc) {
    ntopo::Shape solid = ncst::build_guided_orient_sweep(p, pc, path, pathc, g, gc);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().guided_orient_sweep(p, pc, path, pathc, g, gc);
    return track(wrapNative(std::move(solid)));
}
// variable_sweep — the variable-section / guide+spine sweep (moat-vsweep). A section that
// MORPHS from A→B along the spine, optionally guide-SCALED. NATIVE for a straight or
// smooth-curved (planar) spine that welds watertight (the no-guide case reuses the landed
// loft_along_rail morph; a guided morph runs the RMF/perp-framed guide-scaled morph tube).
// SELF-VERIFIED robustly watertight + positive enclosed volume before being kept native; a
// mismatched-count / degenerate / coincident-guide-start / self-folding morph (build returns
// NULL) or a candidate that fails the gate → forward the SAME args to the OCCT MakePipeShell
// multi-section oracle (honest coexistence, never a faked or leaky solid).
ShapeResult NativeEngine::variable_sweep(const double* a, int ac, const double* b, int bc,
                                         const double* sp, int spc, const double* g, int gc) {
    ntopo::Shape solid = ncst::build_variable_sweep(a, ac, b, bc, sp, spc, g, gc);
    if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
        return fallback().variable_sweep(a, ac, b, bc, sp, spc, g, gc);
    return track(wrapNative(std::move(solid)));
}
// ── NATIVE wrap-emboss (Phase 4 #7 native-wrap-emboss) with mandatory self-verify ──
// Native tracks on a native body's CYLINDER lateral face: a raised RECTANGULAR pad
// (control), a recessed rectangular POCKET (T1 deboss), and an N-vertex POLYGON footprint
// embossed or debossed (T2). The OCCT-free builder (src/native/feature/wrap_emboss.h)
// wraps the footprint onto the cylinder and rebuilds the whole solid as a deflection-
// bounded planar-facet soup welded watertight via the boolean assembleSolid (the
// pad-wall∩cylinder seam is expressed by shared footprint vertices, not a fragile
// boolean). The result is SELF-VERIFIED (watertight + signed volume: emboss GROWS,
// deboss SHRINKS by ≈ footprint area × height) and DISCARDED on failure. A native body
// the slice declines (non-cylindrical / freeform base — T3, off-end or ≥2π footprint,
// self-intersecting profile, deboss depth ≥ radius) canNOT be forwarded to OCCT (OCCT
// would misread the native void) → honest error. An OCCT body forwards to the Phase-3
// OCCT cc_wrap_emboss oracle unconditionally.
ShapeResult NativeEngine::wrap_emboss(EngineShape body, int faceId, const double* p, int c, double d,
                                      int boss) {
    if (!isNative(body)) return fallback().wrap_emboss(body, faceId, p, c, d, boss);
    const auto* h = static_cast<const NativeShape*>(body.get());
    ntopo::Shape result =
        cybercad::native::feature::wrap_emboss(h->shape, faceId, p, c, d, boss);
    if (result.isNull() || !wrapEmbossVerified(result, h->shape, faceId, p, c, d, boss))
        return make_error(
            "native wrap_emboss: no verified watertight result for this native body "
            "(non-cylindrical / freeform base / off-end or >2π footprint / self-"
            "intersecting profile / deboss depth ≥ radius → OCCT-only)");
    return track(wrapNative(std::move(result)));
}
// ── Tier-D (#4b) threads + tapered shank ──────────────────────────────────────
// tapered_shank is NATIVE: a shank silhouette revolved 360° about Z (reusing the
// native revolve). Wide at the head, a true point at the tip; watertight, exact/
// deflection-bounded vs BRepPrimAPI_MakeRevol. A degenerate parameter → NULL → OCCT.
//
// helical_thread / tapered_thread run NATIVE (a V/triangular section swept RADIALLY
// along the pitch-line helix via the axis-aux-spine law, tiled into ruled bands +
// planar caps — construct/thread.h), guarded against self-intersection at fine pitch /
// large depth / overlapping turns. The per-turn ruled-band/cap seams weld watertight via
// the mesher's canonical shared-edge points (edge_mesher CanonicalEndpoints /
// face_mesher BoundaryAnchors), so a well-formed thread SELF-VERIFIES as robustly
// watertight across a deflection ladder (robustlyWatertight) and is kept native. A
// FINE-PITCH / self-intersecting thread fails that gate (a self-overlapping mesh is
// non-manifold) and the SAME arguments fall through to the OCCT MakePipeShell oracle —
// never a faked or leaky solid (honest coexistence, see NATIVE-REWRITE.md Tier D).
ShapeResult NativeEngine::helical_thread(double mr, double pi, double tu, double de, double fa,
                                         double pp, int sp) {
    ntopo::Shape solid = ncst::build_helical_thread(mr, pi, tu, de, fa, pp, sp);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().helical_thread(mr, pi, tu, de, fa, pp, sp);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::tapered_thread(double tr, double tip, double pi, double tu, double de,
                                         double fa, double pp, int sp) {
    ntopo::Shape solid = ncst::build_tapered_thread(tr, tip, pi, tu, de, fa, pp, sp);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().tapered_thread(tr, tip, pi, tu, de, fa, pp, sp);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::tapered_shank(double r, double fh, double th, double pp) {
    ntopo::Shape solid = ncst::build_tapered_shank(r, fh, th, pp);
    if (solid.isNull()) return fallback().tapered_shank(r, fh, th, pp);
    return track(wrapNative(std::move(solid)));
}
// ── Tier-A (#4b) NATIVE holed / typed-profile extrude + typed-profile revolve ───
// Each tries the native builder first; a NULL Shape means the native path defers
// this sub-case (spline outer edge, off-axis-arc revolve, degenerate input) and we
// forward the SAME arguments to the fallback engine (honest coexistence, no faking).

ShapeResult NativeEngine::solid_extrude_holes(const double* o, int oc, const double* h, int hc,
                                              double d) {
    ntopo::Shape solid = ncst::build_prism_with_holes(o, oc, toCircleHoles(h, hc), {}, d);
    if (solid.isNull()) return fallback().solid_extrude_holes(o, oc, h, hc, d);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::solid_extrude_polyholes(const double* o, int oc, const double* h,
                                                  const int* hcs, int hc, double d) {
    ntopo::Shape solid = ncst::build_prism_with_holes(o, oc, {}, toPolyHoles(h, hcs, hc), d);
    if (solid.isNull()) return fallback().solid_extrude_polyholes(o, oc, h, hcs, hc, d);
    return track(wrapNative(std::move(solid)));
}
// solid_extrude_profile / _polyholes route through build_prism_profile_spline, which
// handles kind-3 SPLINE outer edges (Tier-1 residual) via the splineXY side channel
// and delegates a line/arc/full-circle-only profile to the already-native
// build_prism_profile. A spline-bearing profile expands to a dense polyline through
// the fitted NURBS; the result is SELF-VERIFIED robustly watertight before being kept
// native (a self-crossing / degenerate profile → NULL or fails verify → OCCT, never
// faked). A pure line/arc profile is exact and passes verify trivially.
ShapeResult NativeEngine::solid_extrude_profile(const ProfileSeg* s, int sc, const double* h, int hc,
                                                const double* sx, int sxc, double d) {
    ntopo::Shape solid =
        ncst::build_prism_profile_spline(toNativeSegs(s, sc), sx, sxc, toCircleHoles(h, hc), {}, d);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().solid_extrude_profile(s, sc, h, hc, sx, sxc, d);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::solid_extrude_profile_polyholes(const ProfileSeg* s, int sc,
                                                          const double* h, int cc, const double* px,
                                                          const int* pcs, int pc, const double* sx,
                                                          int sxc, double d) {
    ntopo::Shape solid = ncst::build_prism_profile_spline(
        toNativeSegs(s, sc), sx, sxc, toCircleHoles(h, cc), toPolyHoles(px, pcs, pc), d);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().solid_extrude_profile_polyholes(s, sc, h, cc, px, pcs, pc, sx, sxc, d);
    return track(wrapNative(std::move(solid)));
}
// solid_revolve_profile routes through build_revolution_profile_spline, which adds the
// two Tier-1 residuals: a kind-3 SPLINE meridian and an OFF-AXIS circular arc (a TORUS
// surface of revolution via the native Torus, src/native/math/torus.h). A profile with
// neither residual delegates to the already-native build_revolution_profile (line →
// Plane/Cylinder/Cone, on-axis arc → Sphere). The curved result is SELF-VERIFIED
// robustly watertight before being kept native; a spindle torus / axis-crossing
// generatrix (self-intersecting surface of revolution — Tier-4 SSI), a partial-turn
// residual revolve, or any candidate that fails verify → NULL / OCCT (never faked).
ShapeResult NativeEngine::solid_revolve_profile(const ProfileSeg* s, int sc, double ax, double ay,
                                                double adx, double ady, const double* sx, int sxc,
                                                double a) {
    const ncst::RevolveAxis axis{ax, ay, adx, ady};
    ntopo::Shape solid = ncst::build_revolution_profile_spline(toNativeSegs(s, sc), sx, sxc, axis, a);
    if (solid.isNull() || !robustlyWatertight(solid))
        return fallback().solid_revolve_profile(s, sc, ax, ay, adx, ady, sx, sxc, a);
    return track(wrapNative(std::move(solid)));
}

// ── feature fallthrough ─────────────────────────────────────────────────────────

// ── NATIVE blends (Phase 4 #6 native-blends) with mandatory self-verify ──────────
//
// Each blend op is NATIVE when the body is a native PLANAR-FACED solid and the picked
// feature is in the tractable domain (a convex planar-dihedral edge for chamfer/
// fillet; a planar face for offset; a convex planar solid for shell). The native
// builder (src/native/blend) edits the solid's planar-polygon soup and re-welds a
// watertight solid; the result is then SELF-VERIFIED (watertight + sane volume sign
// vs the original — chamfer/fillet/shell shrink, offset grows or shrinks) and
// DISCARDED if it fails. A native body that the native builder cannot handle (curved,
// concave, ≠2-face edge, oversized) or that fails self-verify canNOT be forwarded to
// OCCT (OCCT would misread the native void), so it returns an honest error — never a
// wrong/leaky or faked solid. An OCCT body forwards to the OCCT BRepFilletAPI/
// BRepOffsetAPI oracle unconditionally.
//
// STILL OCCT-FALLTHROUGH (native builder returns NULL / self-verify discards):
// curved-face inputs, CONCAVE edges, variable-radius (fillet_edges_variable),
// fillet_face, an edge shared by ≠2 faces, multi-edge fillet interference, non-convex
// shell — labelled, verified, never faked.

namespace nblend = cybercad::native::blend;

ShapeResult NativeEngine::fillet_edges(EngineShape body, const int* e, int ec, double r) {
    if (!isNative(body)) return fallback().fillet_edges(body, e, ec, r);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // Each candidate is checked with ITS OWN correctly-signed self-verify: a CONVEX
    // blend REDUCES volume (wantGrow=false, 0 < Vr < Vo), a CONCAVE fillet ADDS material
    // and GROWS volume (wantGrow=true, Vr > Vo — the same branch offset_face grow uses).
    // The first candidate passing its guard wins; because a convex candidate can never
    // pass grow and a concave never passes shrink, the sign cannot be spoofed. NULL from
    // all / any failed self-verify → OCCT-only (never a wrong or leaky solid).

    // 1. Planar dihedral fillet (tangent-cylinder) — verified SHRINK.
    ntopo::Shape result = nblend::fillet_edges(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 2. CONVEX circular crease (cylinder lateral ↔ coaxial planar cap) → torus canal,
    //    REMOVES material — verified SHRINK.
    result = nblend::curved_fillet_edge(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 3. CONCAVE circular crease (boss cylinder ↔ larger coaxial shoulder) → material-
    //    side torus canal, ADDS material — verified GROW.
    result = nblend::concave_fillet_edge(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/true))
        return track(wrapNative(std::move(result)));

    // 4. CONVEX circular crease on a CONE FRUSTUM ↔ coaxial planar cap → coaxial torus
    //    band swept the tilted minor angle from the cone-wall seam to the cap seam,
    //    REMOVES material — verified SHRINK. (Cone faces exist only on native bodies of
    //    revolution, so an OCCT body never reaches here; the cylinder is σ=0 above.)
    result = nblend::cone_fillet_edge(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 5. CONVEX circular crease on a SPHERE ↔ coaxial planar cap → coaxial torus band swept
    //    the minor angle from the sphere-wall seam to the cap seam, REMOVES material —
    //    verified SHRINK. (Sphere faces exist only on native bodies of revolution, so an
    //    OCCT body never reaches here; the cylinder/cone cases are handled above.)
    result = nblend::sphere_fillet_edge(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 6. CYL↔CYL CANAL crease (Steinmetz bicylinder COMMON — two EQUAL-radius cylinders
    //    whose axes cross ORTHOGONALLY) → two coaxial canal strips (crease planes z=±x,
    //    G1-tangent to both walls, tapering to zero width at the two shared poles) welded
    //    to the trimmed lune walls PURELY in the assembly layer, REMOVES material —
    //    verified SHRINK. The two crease arcs cross at the poles as a DEGENERATE PINCH
    //    (dihedral→180°, strip cross-section→0), NOT a finite trihedral corner: the two
    //    strips share the two canonical pole vertices, so no corner patch is needed and the
    //    shell welds watertight (with an INTERNAL orientation + removed-volume self-verify so
    //    a large-radius pole fold can never pass). The builder rounds the WHOLE crossing
    //    crease (all four arcs) — the only watertight resolution for a topology whose arcs
    //    meet at the poles; a single-arc fillet cannot close the poles (the original decline).
    //    A native Steinmetz body exists only on native SSI-boolean solids, so an OCCT body
    //    never reaches here; recognition is WHOLESALE from the boolean's planar-facet soup.
    result = nblend::canal_fillet_edge(h->shape, e, ec, r);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // T2 (ELLIPTICAL-crease fillet, cylinder ↔ oblique plane) is an HONEST DECLINE → OCCT-
    // fallthrough, NO native builder (no dead code): T2 needs a native body carrying a true
    // Cylinder face + oblique Plane face meeting at an Ellipse edge. No OCCT-FREE constructor
    // produces that topology (native booleans are planar-faced only, and the SSI curved
    // boolean recognizes only quadric↔quadric pairs), so the body is never a NativeShape and
    // the elliptical path is UNREACHABLE natively; OCCT owns it (ref: Rc=5,H=10,60° oblique,
    // r=1 → filleted 383.454285, Δ=−9.244796 vs MakeFillet).
    return make_error(
        "native fillet_edges: no verified watertight result for this native body "
        "(non-circular curved crease / blind-hole rim / Rc<2r convex / ≠cyl-plane rim / "
        "variable / non-Steinmetz cyl-cyl / interference → OCCT-only)");
}
ShapeResult NativeEngine::fillet_edges_variable(EngineShape body, const int* e, int ec, double r1,
                                                double r2) {
    if (!isNative(body)) return fallback().fillet_edges_variable(body, e, ec, r1, r2);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // NATIVE slice: a variable-radius rolling-ball fillet on a CONVEX circular
    // cylinder↔coaxial-cap rim with a LINEAR law r(θ)=r1+(r2−r1)·θ/2π — a swept
    // variable-r torus canal, G1 at both non-circular (helix/spiral) seams. A convex
    // variable fillet REMOVES material → verified SHRINK (wantGrow=false), the same
    // branch the constant convex fillet uses.
    ntopo::Shape result = nblend::variable_fillet_edge(h->shape, e, ec, r1, r2);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));
    // Out of the native domain (non-circular / non-linear / concave-variable / cyl-cyl
    // canal / Rc<2·max(r) / failed self-verify) → OCCT BRepFilletAPI (variable). A native
    // void cannot be forwarded to OCCT; report honestly (the body is native here).
    return make_error(
        "native fillet_edges_variable: no verified watertight result for this native "
        "body (non-circular / non-linear / concave-variable / cyl-cyl canal / "
        "Rc<2·max(r1,r2) → OCCT-only)");
}
ShapeResult NativeEngine::chamfer_edges(EngineShape body, const int* e, int ec, double d) {
    if (!isNative(body)) return fallback().chamfer_edges(body, e, ec, d);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // Candidates in order; each gated by the identical SHRINK self-verify (a chamfer
    // REMOVES material). The first passing candidate wins; NULL/failed → next → OCCT.

    // 1. PLANAR dihedral chamfer — slice the convex corner with the setback plane.
    //    (Sequential per-edge clip; welds NON-adjacent edge sets.)
    ntopo::Shape result = nblend::chamfer_edges(h->shape, e, ec, d);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 1b. CONVEX-CORNER chamfer weld — the sequential path (1) DECLINES a set of
    //     mutually-ADJACENT convex edges sharing a corner (the first cut removes the
    //     shared corner, losing the next edge). This weld resolves all chamfer planes
    //     up front against the ORIGINAL soup, then applies all cuts; the corner facet
    //     forms from the exposed rings. All-planar → welds through the same
    //     assembleSolid path with no tessellator change. Same SHRINK self-verify.
    result = nblend::chamfer_corner(h->shape, e, ec, d);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 2. CONVEX circular crease (cylinder lateral ↔ coaxial planar cap) → CONE-FRUSTUM
    //    straight bevel (C0, not G1) between the two setback circles — verified SHRINK.
    result = nblend::curved_chamfer_edge(h->shape, e, ec, d);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    return make_error(
        "native chamfer_edges: no verified watertight result for this native body "
        "(curved face / concave edge / ≠2-face edge / asymmetric / non-circular / "
        "Rc<d convex / cyl-cyl rim → OCCT-only)");
}
ShapeResult NativeEngine::chamfer_edges_asym(EngineShape body, const int* e, int ec, double d1,
                                             double d2) {
    if (!isNative(body)) return fallback().chamfer_edges_asym(body, e, ec, d1, d2);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // T1: an ASYMMETRIC two-distance chamfer on a CONVEX circular cylinder↔coaxial-cap
    // rim — an OBLIQUE cone-frustum bevel (C0 at two DIFFERENT angles) between the setback
    // circles (Rc, H−s·d1) and (Rc−d2, H). d1 = the axial wall setback, d2 = the radial
    // cap setback. A chamfer REMOVES material → verified SHRINK (wantGrow=false), the same
    // gate the symmetric chamfer uses; d1 == d2 reproduces the symmetric result. NULL /
    // unverified (non-circular / concave / tilted / Rc ≤ d2 / wall < d1 / multi-edge) →
    // honest error → OCCT BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face).
    ntopo::Shape result = nblend::curved_chamfer_edge_asym(h->shape, e, ec, d1, d2);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));
    return make_error(
        "native chamfer_edges_asym: no verified watertight result for this native body "
        "(non-circular / concave / tilted cap / Rc≤d2 / wall<d1 / cyl-cyl rim / "
        "multi-edge → OCCT-only)");
}
ShapeResult NativeEngine::shell(EngineShape body, const int* f, int fc, double t) {
    if (!isNative(body)) return fallback().shell(body, f, fc, t);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // 1. PLANAR convex shell — half-space cavity + BSP-CSG cut. Verified SHRINK.
    ntopo::Shape result = nblend::shell(h->shape, f, fc, t);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    // 2. CURVED shell — a capped CYLINDER or CONE FRUSTUM hollowed to a uniform wall by
    //    an analytic inward offset of the curved wall, one planar cap left OPEN. Rebuilt
    //    as a watertight facet soup (curved_shell.h); the wall volume is closed-form. A
    //    hollow REMOVES material → verified SHRINK. Cone faces exist only on native bodies
    //    of revolution, so an OCCT body never reaches here.
    result = nblend::curved_shell(h->shape, f, fc, t);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return track(wrapNative(std::move(result)));

    return make_error(
        "native shell: no verified watertight wall for this native body "
        "(non-convex planar / stepped-multi-wall / sphere / freeform / tilted cap / both "
        "caps or wall removed / thickness too large → OCCT-only)");
}
ShapeResult NativeEngine::offset_face(EngineShape body, int f, double d) {
    if (!isNative(body)) return fallback().offset_face(body, f, d);
    const auto* h = static_cast<const NativeShape*>(body.get());
    // 1. PLANAR face on an all-planar solid — slide the cap along its normal, drag the
    //    side faces. Grow (d>0) increases volume; shrink (d<0) decreases it.
    ntopo::Shape result = nblend::offset_face(h->shape, f, d);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/d > 0.0))
        return track(wrapNative(std::move(result)));

    // 2. CURVED face — the CYLINDER lateral wall of a capped cylinder, offset RADIALLY
    //    (radius Rc → Rc+d). The offset of a cylinder surface is a coaxial cylinder, so the
    //    capped body is re-radiused analytically (wall band + two disc caps, planar-facet
    //    weld, no tessellator change). Same correctly-signed volume self-verify: grow d>0 →
    //    Vr>Vo, shrink d<0 → 0<Vr<Vo. A picked planar face is served by (1); a cone / sphere
    //    / stepped / multi-cylinder body → NULL → OCCT (BRepOffsetAPI).
    result = nblend::curved_offset_face(h->shape, f, d);
    if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/d > 0.0))
        return track(wrapNative(std::move(result)));

    return make_error(
        "native offset_face: no verified watertight result for this native body "
        "(non-cylinder curved face / stepped / cone / sphere / degenerate offset → OCCT-only)");
}
// ── NATIVE DM3 general move-face (additive; derives the target plane, reuses DM2) ──
// The app's `cc_replace_face(body, faceId, offset, tiltDeg)` — retarget a planar face
// by OFFSETTING it along its outward normal and TILTING it about the face's parametric
// X-axis. An OCCT body is UNCHANGED — it forwards to the OCCT half-space-cut oracle.
// A NATIVE B-rep body is served for the PURE-OFFSET case (tiltDeg ≈ 0): directmodel::
// replaceFaceOffsetTilt derives the target plane (o + n̂_F·offset, n̂_F) and re-solves
// via the landed DM2 replaceFaceToPlane (the SAME watertight self-verify + closed-form
// V₀+A·offset oracle). A NON-ZERO tilt rotates about OCCT's face-parametrization X-axis
// — a foreign convention we do not reproduce for a native body — so it, a non-planar
// picked face, and a non-all-planar solid yield NULL and the SAME honest decline the
// prior fall-through produced (a native void is NEVER handed to OCCT).
ShapeResult NativeEngine::replace_face(EngineShape body, int f, double o, double t) {
    if (!isNative(body)) return fallback().replace_face(body, f, o, t);

#ifdef CYBERCAD_HAS_NUMSCI
    // The re-solve reuses replaceFaceToPlane, whose tilted probe traces the NUMSCI-only
    // seam; without the substrate this path is absent and we fall to the honest decline.
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (!holder->isMesh) {
        namespace dm = cybercad::native::directmodel;
        dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
        ntopo::Shape result = dm::replaceFaceOffsetTilt(holder->shape, f, o, t, &why);
        if (!result.isNull() && watertightVolume(result) > 0.0)
            return track(wrapNative(std::move(result)));
    }
#endif
    // Honest decline: identical to the pre-DM3 native-body behaviour (never → OCCT).
    return make_error("operation not supported on a native body yet: replace_face"
                      " (native scope: pure-offset planar retarget; tilt / non-planar → OCCT-only)");
}
// ── NATIVE DM2 move-face (additive; consumes DM1 split + boolean + construct) ─────
// The app's push/pull "move a planar face to a target plane", re-solving the adjacent
// planar faces so the solid stays watertight. An OCCT body is UNCHANGED — it forwards
// to the OCCT half-space-cut oracle byte-for-byte. A NATIVE B-rep body is re-solved by
// directmodel::replaceFaceToPlane (src/native/directmodel/replace_face.h), which
// composes the landed verbs: a parallel pull is one DM1 splitByPlane, a parallel push
// is a build_prism slab fused on, and a tilted/mixed move is grow-then-trim (one Fuse +
// one tilted cut). The candidate is accepted ONLY when it passes the module's re-solve
// self-verify (watertight closed 2-manifold, single lump χ=2, distinct-plane count
// preserved, moved face on the target plane, enclosed volume == the closed-form
// V₀+A·d̄) AND the engine's own watertightVolume audit. Anything outside the convex-
// planar slice — a curved neighbour (non-all-planar), a non-planar picked face, a
// degenerate / topology-changing target — yields NULL and the SAME honest decline the
// prior fall-through produced (a native void is NEVER handed to OCCT).
ShapeResult NativeEngine::replace_face_to_plane(EngineShape body, int f, double px, double py,
                                                double pz, double nx, double ny, double nz) {
    if (!isNative(body)) return fallback().replace_face_to_plane(body, f, px, py, pz, nx, ny, nz);

#ifdef CYBERCAD_HAS_NUMSCI
    // splitByPlane's freeform probe (used by the tilted re-solve) traces the NUMSCI-only
    // seam; without the substrate this path is absent and we fall to the honest decline
    // below (the SAME behaviour as the pre-DM2 native engine).
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (!holder->isMesh) {
        namespace dm = cybercad::native::directmodel;
        namespace nm = cybercad::native::math;
        dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
        ntopo::Shape result = dm::replaceFaceToPlane(holder->shape, f, nm::Point3{px, py, pz},
                                                     nm::Vec3{nx, ny, nz}, &why);
        if (!result.isNull() && watertightVolume(result) > 0.0)
            return track(wrapNative(std::move(result)));
    }
#endif
    // Honest decline: identical to the pre-DM2 native-body behaviour (never → OCCT).
    return make_error("operation not supported on a native body yet: replace_face_to_plane"
                      " (native scope: convex planar-polyhedron move-face → OCCT-only)");
}
// ── NATIVE DRAFT ANGLE (additive; consumes DM1 split + the DM2 face read + mesh audit) ─
// The app's molding/manufacturing draft: taper one or more PLANAR side faces of a solid
// about a planar NEUTRAL plane by an angle, so the walls taper along a PULL direction for
// mold release. An OCCT body forwards to the OCCT BRepOffsetAPI_DraftAngle oracle. A
// NATIVE all-planar (prismatic) body is drafted by feature::draftFaces: each drafted
// plane is derived from the ORIGINAL face geometry (pivot on its trace with the neutral
// plane) and applied as an inward nb::splitByPlane trim, then the composite is
// SELF-VERIFIED (watertight closed 2-manifold, single lump χ=2, consistently oriented,
// volume strictly SMALLER than the original — a draft only removes stock). Anything
// outside the prismatic slice — a curved base (non-all-planar), a non-planar neutral
// (degenerate pull), a face perpendicular to the pull axis (a cap, no trace line), a
// degenerate/≥90° angle, or a self-intersecting draft — yields NULL and the SAME honest
// decline the prior fall-through produced (a native void is NEVER handed to OCCT).
ShapeResult NativeEngine::draft_faces(EngineShape body, const int* faceIds, int faceCount,
                                      const double* neutralOrigin, const double* pullDir,
                                      double angleDeg) {
    if (!isNative(body))
        return fallback().draft_faces(body, faceIds, faceCount, neutralOrigin, pullDir, angleDeg);

#ifdef CYBERCAD_HAS_NUMSCI
    // The inward trims reuse splitByPlane's tilted probe, which traces the NUMSCI-only
    // seam; without the substrate this path is absent and we fall to the honest decline.
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (!holder->isMesh && faceIds != nullptr && faceCount >= 1 && neutralOrigin != nullptr &&
        pullDir != nullptr) {
        namespace ndraft = cybercad::native::feature;
        namespace nm = cybercad::native::math;
        ndraft::DraftFacesDecline why = ndraft::DraftFacesDecline::Ok;
        const double angleRad = angleDeg * 3.14159265358979323846 / 180.0;
        ntopo::Shape result = ndraft::draftFaces(
            holder->shape, faceIds, faceCount, angleRad,
            nm::Point3{neutralOrigin[0], neutralOrigin[1], neutralOrigin[2]},
            nm::Vec3{pullDir[0], pullDir[1], pullDir[2]}, &why);
        // draftFaces already self-verifies watertight + χ=2 + oriented + shrink; the
        // engine's own watertightVolume audit is a redundant last gate (never a leaky solid).
        if (!result.isNull() && watertightVolume(result) > 0.0)
            return track(wrapNative(std::move(result)));
    }
#endif
    // Honest decline: a native void is NEVER handed to OCCT (which would misread it).
    return make_error("operation not supported on a native body yet: draft_faces"
                      " (native scope: prismatic planar-face draft about a planar neutral;"
                      " curved base / non-planar neutral / cap face / self-intersecting → OCCT-only)");
}

// ── MOAT M-SM sheet metal (first slice; native-only, closed-form arbiter) ───────
// OCCT core has NO sheet-metal module, so these are native-only and NEVER forwarded:
// a case the native builder cannot robustly build HONEST-DECLINES with a clean error
// (a measured SheetMetalDecline), it is never faked and never handed to OCCT. Every
// built solid self-verifies watertight / χ=2 / oriented at the closed-form volume.
namespace {
namespace nsm = cybercad::native::sheetmetal;
const char* smDeclineText(nsm::SheetMetalDecline d) {
    switch (d) {
        case nsm::SheetMetalDecline::BadProfile:        return "degenerate profile (<3 pts / zero area)";
        case nsm::SheetMetalDecline::BadThickness:      return "thickness must be > 0";
        case nsm::SheetMetalDecline::BadParam:          return "bad parameter (height<0 / radius<0 / angle outside (0,180))";
        case nsm::SheetMetalDecline::EdgeNotFound:      return "edge id out of range on the base";
        case nsm::SheetMetalDecline::EdgeNotStraight:   return "the bend line is not a straight edge";
        case nsm::SheetMetalDecline::NotSingleBendPart: return "not a recognised single-bend part (base+one edge flange)";
        case nsm::SheetMetalDecline::SelfCollision:     return "the fold self-intersects (flange re-enters the base)";
        case nsm::SheetMetalDecline::VerifyFailed:      return "the built solid failed the closed-form self-verify";
        default:                                        return "unsupported";
    }
}
}  // namespace

ShapeResult NativeEngine::sheet_base_flange(const double* profileXY, int pointCount,
                                            double thickness) {
    nsm::SheetMetalDecline why = nsm::SheetMetalDecline::Ok;
    ntopo::Shape solid = nsm::baseFlange(profileXY, pointCount, thickness, &why);
    if (!solid.isNull() && watertightVolume(solid) > 0.0)
        return track(wrapNative(std::move(solid)));
    return make_error(std::string("sheet_base_flange (native, no OCCT sheet-metal oracle): ") +
                      smDeclineText(why));
}

ShapeResult NativeEngine::sheet_edge_flange(EngineShape body, int edgeId, double height,
                                            double bendRadius, double angleDeg) {
    if (!isNative(body))
        return make_error("sheet_edge_flange: sheet metal is native-only (build the base under"
                          " cc_set_engine(1); OCCT has no sheet-metal module)");
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh)
        return make_error("sheet_edge_flange: an imported mesh body is not a sheet-metal base");
    nsm::SheetMetalDecline why = nsm::SheetMetalDecline::Ok;
    nsm::FoldRecord fold{};
    const double angleRad = angleDeg * 3.14159265358979323846 / 180.0;
    ntopo::Shape result =
        nsm::edgeFlange(holder->shape, edgeId, height, bendRadius, angleRad, &why, &fold);
    if (!result.isNull() && watertightVolume(result) > 0.0)
        return track(wrapNativeFold(std::move(result), fold));
    return make_error(std::string("sheet_edge_flange (native, no OCCT sheet-metal oracle): ") +
                      smDeclineText(why));
}

ShapeResult NativeEngine::sheet_unfold(EngineShape body, double kFactor) {
    if (!isNative(body))
        return make_error("sheet_unfold: sheet metal is native-only (OCCT has no sheet-metal module)");
    const auto* holder = static_cast<const NativeShape*>(body.get());
    nsm::SheetMetalDecline why = nsm::SheetMetalDecline::Ok;
    ntopo::Shape blank = nsm::unfold(holder->fold, kFactor, /*out=*/nullptr, &why);
    if (!blank.isNull() && watertightVolume(blank) > 0.0)
        return track(wrapNative(std::move(blank)));
    return make_error(std::string("sheet_unfold (native, no OCCT sheet-metal oracle): ") +
                      smDeclineText(why));
}
// ── NATIVE M3 fillet_face (additive; reuses the landed multi-edge dihedral fillet) ──
// The app's `cc_fillet_face(body, faceId, radius)` — round EVERY edge bounding the
// picked face at constant radius (mirrors OcctEngine::fillet_face, which Adds a fillet
// on every edge of the face). An OCCT body forwards to the OCCT BRepFilletAPI oracle.
// A NATIVE all-planar body is served by nblend::fillet_face, which collects the CONVEX
// planar-dihedral bounding edges of the picked planar face (probed with the SAME
// filletArc guard the dihedral fillet uses) and re-solves them through the byte-frozen
// nblend::fillet_edges tangent-cylinder blend (open-seam weld). The candidate is
// accepted ONLY under the engine's SHRINK self-verify (0 < Vr < Vo — a face fillet
// REMOVES material), the SAME gate fillet_edges uses. A curved solid / non-planar
// picked face / concave-or-curved bounding edge / interfering radius yields NULL and
// the SAME honest decline the prior hard-decline produced (a native void is NEVER
// handed to OCCT).
ShapeResult NativeEngine::fillet_face(EngineShape body, int f, double r) {
    if (!isNative(body)) return fallback().fillet_face(body, f, r);
    const auto* h = static_cast<const NativeShape*>(body.get());
    if (!h->isMesh && r > 0.0) {
        ntopo::Shape result = nblend::fillet_face(h->shape, f, r);
        if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
            return track(wrapNative(std::move(result)));
    }
    // Honest decline: identical to the pre-M3 native-body behaviour (never → OCCT).
    return make_error(
        "native fillet_face: no verified watertight result for this native body "
        "(curved face / non-planar picked face / concave-or-curved bounding edge / "
        "interfering radius → OCCT-only)");
}
// ── NATIVE DM1 plane split (additive; the ONLY DM1 change) ────────────────────────
// An OCCT body is UNCHANGED — it forwards to the OCCT split_plane oracle byte-for-byte.
// A NATIVE B-rep body is split by composing the two landed verbs
// (native/boolean/split_plane.h: freeformHalfSpaceCut for a freeform wall, else the BSP
// cut against a discard half-space box), then the piece is accepted ONLY when it passes
// the engine's mandatory watertight audit. A native body the composition cannot robustly
// split (a curved perpendicular slice, a grazing-tangent plane, a degenerate/coincident
// plane, a multi-freeform operand, or a mesh-only body) returns the SAME clean error the
// prior unconditional fall-through produced — a native void is NEVER handed to OCCT
// (whose unwrap would misread it), so the decline is reported honestly, never faked.
ShapeResult NativeEngine::split_plane(EngineShape body, double ox, double oy, double oz, double nx,
                                      double ny, double nz, int keep) {
    if (!isNative(body)) return fallback().split_plane(body, ox, oy, oz, nx, ny, nz, keep);

#ifdef CYBERCAD_HAS_NUMSCI
    // The native split composes freeformHalfSpaceCut, whose seam trace is NUMSCI-only.
    // Without the substrate this path is absent and we fall straight to the honest
    // decline below (the SAME behaviour as the pre-DM1 native engine).
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (!holder->isMesh) {
        namespace nb = cybercad::native::boolean;
        namespace nm = cybercad::native::math;
        nb::HalfSpaceCutDecline why = nb::HalfSpaceCutDecline::Ok;
        ntopo::Shape piece = nb::splitByPlane(holder->shape, nm::Point3{ox, oy, oz},
                                              nm::Vec3{nx, ny, nz}, keep != 0, 0.008, &why);
        if (!piece.isNull() && watertightVolume(piece) > 0.0)
            return track(wrapNative(std::move(piece)));
    }
#endif
    // Honest decline: identical to the pre-DM1 native-body behaviour (never → OCCT).
    return make_error("operation not supported on a native body yet: split_plane"
                      " (native scope: extrude/revolve + tessellate/mass/bbox)");
}
// ── NATIVE M3 full-round fillet (additive; the r = w/2 prismatic tangent-cylinder cap) ──
// The app's `cc_full_round_fillet[_faces]` — replace a narrow middle face with a full
// round tangent to its two neighbour walls, consuming the middle face. An OCCT body
// forwards to the OCCT full-round oracle. A NATIVE all-planar body is served by
// nblend::full_round_fillet[_faces] for the ANALYTIC PRISMATIC case (two PARALLEL
// planar walls a distance w apart, a straight middle strip): the rolling ball of
// radius r = w/2 is the special case of the tangent-cylinder blend, so the cap is
// built by re-solving the two seam edges through the byte-frozen nblend::fillet_edges
// at radius w/2 — the two arcs meet tangentially on the strip mid-plane and the
// middle face is consumed. Accepted ONLY under the engine's SHRINK self-verify
// (0 < Vr < Vo — a convex full round REMOVES material). A DIHEDRAL (non-parallel)
// middle, a curved wall, a non-planar middle, or a closed-seam/annulus (which need the
// M2 valley-solve / closed-seam weld) yields NULL and the SAME honest decline the prior
// hard-decline produced (a native void is NEVER handed to OCCT).
ShapeResult NativeEngine::full_round_fillet(EngineShape body, int f) {
    if (!isNative(body)) return fallback().full_round_fillet(body, f);
    const auto* h = static_cast<const NativeShape*>(body.get());
    if (!h->isMesh) {
        ntopo::Shape result = nblend::full_round_fillet(h->shape, f);
        if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
            return track(wrapNative(std::move(result)));
    }
    return make_error(
        "native full_round_fillet: no verified watertight result for this native body "
        "(dihedral / curved wall / non-planar middle / closed-seam annulus → OCCT-only; "
        "gates on M2 valley-solve + closed-seam weld)");
}
ShapeResult NativeEngine::full_round_fillet_faces(EngineShape body, int l, int m, int r) {
    if (!isNative(body)) return fallback().full_round_fillet_faces(body, l, m, r);
    const auto* h = static_cast<const NativeShape*>(body.get());
    if (!h->isMesh) {
        ntopo::Shape result = nblend::full_round_fillet_faces(h->shape, l, m, r);
        if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
            return track(wrapNative(std::move(result)));
    }
    return make_error(
        "native full_round_fillet_faces: no verified watertight result for this native "
        "body (dihedral / curved wall / non-planar middle / closed-seam annulus / no "
        "shared seams → OCCT-only; gates on M2 valley-solve + closed-seam weld)");
}
// ── NATIVE M3 G2 (CURVATURE-CONTINUOUS) blend fillet — the Class-B drop-OCCT slice ──
// The stock native fillet_edges rolls a ball into a convex planar dihedral and tiles a
// CIRCULAR-ARC section (constant curvature 1/r → G1: tangent-continuous, but the
// curvature JUMPS to 0 on the flat neighbours). A G2 blend replaces that section with a
// zero-END-CURVATURE quintic (blend/fillet_edges_g2.h): poles {P0,P1,P2} collinear along
// the face-1 tangent and {P5,P4,P3} along the face-2 tangent make B''=0 at both rails, so
// the section curvature is IDENTICALLY 0 there — curvature-continuous across the seam.
// Everything else (setback clip, facet tiling, watertight weld) is the G1 machinery; ONLY
// the section curve changes. NATIVE when the picked edges are CONVEX straight seams between
// two PLANAR faces (the tractable G2 family, mirroring the OCCT occt_g2_fillet.cpp
// reference which is likewise a quintic-section boolean, since OCCT's BRepFilletAPI is
// G1/circular-only). Accepted ONLY under the SHRINK self-verify (0 < Vr < Vo — a convex
// blend removes material). A CONCAVE edge / CURVED neighbour / non-planar solid / freeform
// substrate (the DEEP G2 residual — the real moat) → NULL → OCCT. A merely-G1 blend is
// NEVER emitted here: the section is the zero-end-curvature quintic or nothing.
ShapeResult NativeEngine::fillet_edges_g2(EngineShape body, const int* e, int ec, double r) {
    if (!isNative(body)) return fallback().fillet_edges_g2(body, e, ec, r);
    const auto* h = static_cast<const NativeShape*>(body.get());
    if (!h->isMesh) {
        ntopo::Shape result = nblend::fillet_edges_g2(h->shape, e, ec, r);
        if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
            return track(wrapNative(std::move(result)));
    }
    return make_error(
        "native fillet_edges_g2: no verified watertight G2 blend for this native body "
        "(concave edge / curved neighbour / non-planar / freeform substrate → OCCT-only; "
        "the deep general curvature-continuous surface remains the moat)");
}

// ── NATIVE planar-polyhedron boolean (fuse / cut / common) with self-verify ───────
//
// Phase 4 #5 (native-booleans). NATIVE when BOTH operands are native bodies whose
// faces are ALL PLANAR (boxes / prisms / convex or simple-concave polyhedra): the
// BSP-CSG core (src/native/boolean) computes the face-face intersection splits,
// classifies each fragment inside/outside/on the other solid, assembles + welds the
// surviving fragments into a watertight Solid. The result is then SELF-VERIFIED —
// closed watertight 2-manifold AND the set-algebra volume (fuse = A+B−∩, cut = A−∩,
// common = ∩) to a tight tolerance (planar meshes are exact). A result that fails the
// guard is DISCARDED (never emitted — no wrong/leaky solid).
//
// FALL-THROUGH (honest coexistence, cross-checked vs the BRepAlgoAPI/BOPAlgo oracle):
//   * either operand is an OCCT body (built by a fallthrough op)      → OCCT boolean.
//   * either operand has a CURVED face (cylinder/sphere/cone/free-form) → the native
//     builder returns NULL; both are native voids OCCT cannot read, so we return a
//     clean error rather than fake a result (the native domain is planar-only).
//   * the native result fails self-verify (a near-tangent/degenerate config the
//     planar algorithm cannot robustly handle)                        → same error.
// The engine NEVER hands a native void to OCCT (its unwrap would misread it), so a
// native-native boolean the planar path cannot do is reported honestly, not faked.
ShapeResult NativeEngine::boolean_op(EngineShape a, EngineShape b, int op) {
    const bool aNative = isNative(a);
    const bool bNative = isNative(b);

    // Mixed or all-OCCT operands: the OCCT engine owns both voids → forward.
    if (!aNative && !bNative) return fallback().boolean_op(a, b, op);
    if (aNative != bNative) {
        // One native, one OCCT: neither engine can read both voids. Honest error.
        return make_error(
            "boolean_op: mixed native/OCCT operands are not supported "
            "(build both bodies under the same active engine)");
    }

    // Both native: attempt the planar-polyhedron boolean.
    const auto* ha = static_cast<const NativeShape*>(a.get());
    const auto* hb = static_cast<const NativeShape*>(b.get());
    ntopo::Shape result = cybercad::native::boolean::boolean_solid(ha->shape, hb->shape, op);

    if (result.isNull() || !booleanResultVerified(result, ha->shape, hb->shape, op)) {
        // A curved-face input, a degenerate/near-tangent config, or a result that
        // failed the watertight/volume self-verify. Both operands are native voids
        // OCCT cannot read, so we cannot forward — report honestly (never fake).
        return make_error(
            "boolean_op: native planar-polyhedron boolean did not produce a verified "
            "watertight result for these operands (curved faces or a degenerate/near-"
            "tangent configuration are outside the native planar domain)");
    }
    return track(wrapNative(std::move(result)));
}
// thread_apply — apply a helical thread to a shaft (op 0 FUSE crest / 1 CUT groove).
// The native attempt reuses the landed planar BSP boolean: recognise the tractable input
// (cylinder shaft + coaxial helical thread), facet both operands into consistently-
// oriented planar-triangle solids, run boolean_solid, and self-verify WATERTIGHT + χ=2 +
// consistently-oriented + a two-sided closed-form-volume band (src/native/boolean/
// thread_apply.h). A verified result is kept native; ANY decline falls through to the OCCT
// per-turn oracle — never a leaky/misoriented/wrong solid, never a native void to OCCT.
//
// MEASURED: a multi-turn helical thread declines today (the native build_thread solid is
// watertight but NOT consistently oriented — sameDirectionEdgeCount != 0 — so it is an
// invalid BSP operand, and the near-tangent helical root ↔ shaft-wall contact fragments the
// dense-soup BSP into T-junction cracks). The self-verify catches both; OCCT owns the case.
// The sharpened next blocker is an orientation-coherent thread builder + robust dense-soup
// CSG with T-junction repair (M7b). A mixed native/OCCT or OCCT-only pair forwards to OCCT.
ShapeResult NativeEngine::thread_apply(EngineShape shaft, EngineShape thread, int op) {
    const bool sNative = isNative(shaft);
    const bool tNative = isNative(thread);
    // Not both native: the OCCT engine owns at least one void → forward (mixed is not a
    // native case; forwarding a native void to OCCT would misread it, but the OCCT engine
    // only ever sees its own bodies here — a mixed pair means the caller built under
    // different engines, which the OCCT oracle rejects honestly).
    if (!sNative || !tNative) return fallback().thread_apply(shaft, thread, op);

    const auto* hs = static_cast<const NativeShape*>(shaft.get());
    const auto* ht = static_cast<const NativeShape*>(thread.get());
    if (hs->isMesh || ht->isMesh) return fallback().thread_apply(shaft, thread, op);

    namespace nbo = cybercad::native::boolean;
    nbo::ThreadApplyDecline why = nbo::ThreadApplyDecline::Ok;
    ntopo::Shape result = nbo::threadApply(hs->shape, ht->shape, op, /*deflection=*/0.05, &why);
    if (!result.isNull() && why == nbo::ThreadApplyDecline::Ok)
        return track(wrapNative(std::move(result)));
    // Honest decline (measured reason recorded via `why`) → OCCT per-turn oracle.
    return fallback().thread_apply(shaft, thread, op);
}

// ── query / reference fallthrough ────────────────────────────────────────────────

// Native edge_polylines: one EdgePolylineData per edge, in the SAME 1-based
// mapShapes(Edge) order subshape_ids / cc_fillet_edges / cc_chamfer_edges use, each a
// world-placed 3D polyline. A straight edge yields its two endpoints; a curved native
// edge (a circle from a revolve) is deflection-discretized by the shared EdgeCache —
// the same discretizer the solid mesher stitches seams with, so the ids and geometry
// an app/harness resolves here match exactly what the blend ops act on. Without this,
// a native body's edges were unqueryable (the old fallthrough refused a native body),
// so cc_fillet_edges / cc_chamfer_edges could never resolve an edge id and always
// returned 0 — the sim-parity failure this fixes. An OCCT body still forwards.
Result<std::vector<EdgePolylineData>> NativeEngine::edge_polylines(EngineShape body) {
    if (!isNative(body)) return fallback().edge_polylines(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    // Deflection matched to the OCCT adapter's 0.2 tangential-deflection discretization
    // (a straight edge collapses to 2 points; a curved edge subdivides). minSegs=1 so a
    // line is exactly its two endpoints, matching mapShapes' single edge polyline.
    ntess::EdgeCache cache(/*deflection=*/0.2, /*minSegs=*/1, /*maxSegs=*/64);
    const ntopo::ShapeMap map = ntopo::mapShapes(holder->shape, ntopo::ShapeType::Edge);
    std::vector<EdgePolylineData> edges(map.size());
    for (std::size_t i = 0; i < map.size(); ++i) {
        EdgePolylineData& e = edges[i];
        e.edgeId = static_cast<int>(i) + 1;  // 1-based, matches subshape_ids
        const ntess::EdgeDiscretization& d = cache.discretize(map.shape(static_cast<int>(i) + 1));
        if (d.points.size() < 2) continue;  // degenerate → empty polyline (id preserved)
        e.points.reserve(d.points.size() * 3);
        for (const auto& p : d.points) {
            e.points.push_back(p.x);
            e.points.push_back(p.y);
            e.points.push_back(p.z);
        }
    }
    return edges;
}
namespace {

// Squared distance from point `p` to triangle (a,b,c) — Ericson, Real-Time
// Collision Detection (region tests on the Voronoi regions of the triangle).
inline double pointTriangleDistSq(const nmath::Point3& p, const nmath::Point3& a,
                                  const nmath::Point3& b, const nmath::Point3& c) noexcept {
    const nmath::Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = nmath::dot(ab, ap), d2 = nmath::dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return nmath::normSquared(ap);
    const nmath::Vec3 bp = p - b;
    const double d3 = nmath::dot(ab, bp), d4 = nmath::dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return nmath::normSquared(bp);
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return nmath::normSquared(ap - ab * v);
    }
    const nmath::Vec3 cp = p - c;
    const double d5 = nmath::dot(ab, cp), d6 = nmath::dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return nmath::normSquared(cp);
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return nmath::normSquared(ap - ac * w);
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return nmath::normSquared((b - p) + (c - b) * w) ;
    }
    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom, w = vc * denom;
    return nmath::normSquared(ap - ab * v - ac * w);
}

// Distance from `p` to the NEAREST triangle of the mesh — i.e. how far `p` is off
// the solid's (faceted) boundary. Used to SELF-VERIFY that a traced silhouette
// point actually sits on the solid before it is emitted (a partial-quadric
// generator in empty space is ~R away and forces an honest decline, while a point
// on a full quadric is at most the chord sagitta ≈ deflection away).
inline double minDistToMesh(const nmath::Point3& p, const ntess::Mesh& mesh,
                            const std::vector<std::array<std::uint32_t, 3>>& tris) noexcept {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& t : tris) {
        const double d2 =
            pointTriangleDistSq(p, mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]);
        if (d2 < best) best = d2;
    }
    return std::sqrt(best);
}

// Outward analytic normal of an ANALYTIC face at a world point on it. Only the
// kinds that can survive the face-kind gate (Plane / Cylinder / Sphere) are needed;
// anything else returns a null vector (treated as NON-smooth → drawn, the safe
// default). Used to decide whether a shared edge is a SMOOTH (tangent) seam that
// the HLR must suppress — matching OCCT's outline-only treatment of curved bodies.
inline nmath::Vec3 analyticFaceNormal(const ntopo::FaceSurface& fs, const ntopo::Location& loc,
                                      const nmath::Point3& wp) noexcept {
    auto placeP = [&](const nmath::Point3& p) {
        return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
    };
    auto placeV = [&](const nmath::Vec3& v) {
        return loc.isIdentity() ? v : loc.transform().applyToVector(v);
    };
    using K = ntopo::FaceSurface::Kind;
    switch (fs.kind) {
        case K::Plane:
            return nmath::Dir3{placeV(fs.frame.z.vec())}.vec();  // constant over the plane
        case K::Cylinder: {
            const nmath::Point3 O = placeP(fs.frame.origin);
            const nmath::Vec3 ax = nmath::Dir3{placeV(fs.frame.z.vec())}.vec();
            const nmath::Vec3 rel = wp - O;
            return nmath::Dir3{rel - ax * nmath::dot(rel, ax)}.vec();  // radial
        }
        case K::Sphere:
            return nmath::Dir3{wp - placeP(fs.frame.origin)}.vec();  // radial from centre
        case K::Cone: {
            // Outward normal (constant along a ruling): tilt the radial direction by
            // the half-angle away from the axis. n = cosα·radial − sinα·axis.
            const nmath::Point3 O = placeP(fs.frame.origin);
            const nmath::Vec3 ax = nmath::Dir3{placeV(fs.frame.z.vec())}.vec();
            const nmath::Vec3 rel = wp - O;
            const nmath::Vec3 radial = rel - ax * nmath::dot(rel, ax);
            if (nmath::norm(radial) <= 0.0) return nmath::Vec3{0, 0, 0};  // apex — ambiguous
            const nmath::Vec3 rhat = nmath::Dir3{radial}.vec();
            const double ca = std::cos(fs.semiAngle), sa = std::sin(fs.semiAngle);
            return nmath::Dir3{rhat * ca - ax * sa}.vec();
        }
        case K::Torus: {
            // Outward normal points radially out of the tube: from the tube-centre
            // circle point at this world point toward the point.
            const nmath::Point3 O = placeP(fs.frame.origin);
            const nmath::Vec3 ax = nmath::Dir3{placeV(fs.frame.z.vec())}.vec();
            const nmath::Vec3 rel = wp - O;
            const nmath::Vec3 radialV = rel - ax * nmath::dot(rel, ax);  // in-plane offset
            if (nmath::norm(radialV) <= 0.0) return nmath::Vec3{0, 0, 0};
            const nmath::Vec3 rhat = nmath::Dir3{radialV}.vec();
            const nmath::Point3 tubeCentre = O + rhat * fs.radius;  // on the major circle
            return nmath::Dir3{wp - tubeCentre}.vec();
        }
        default:
            return nmath::Vec3{0, 0, 0};
    }
}

}  // namespace

// ── drafting: orthographic HLR over the polyhedral core + quadric silhouettes ────
// Build the M0 boundary tessellation as the occluder + the topological (straight)
// edges as the lines to draw, then run the OCCT-FREE orthographic_hlr core. CYLINDER
// and SPHERE faces additionally contribute their closed-form SILHOUETTE outline
// (drafting/silhouette.h) fed through the SAME occlusion + split path. Cone / torus /
// freeform faces (a silhouette this slice does not trace) are DECLINED — never a
// wrong classification. An OCCT body forwards to the HLRBRep_Algo oracle.
Result<DrawingData> NativeEngine::hlr_project(EngineShape body, const double viewDir[3],
                                              const double up[3], HlrOptionsData opts) {
    if (!isNative(body)) return fallback().hlr_project(body, viewDir, up, opts);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh)
        return make_error("hlr_project: a native mesh body carries no B-rep topology (declined)");
    const ntopo::Shape& shape = holder->shape;

    // View basis validity: viewDir must be non-null and NOT parallel to `up`, else
    // the drawing-plane basis is undefined (decline, never guess).
    const nmath::Vec3 vd{viewDir[0], viewDir[1], viewDir[2]};
    const nmath::Vec3 uh{up[0], up[1], up[2]};
    if (nmath::norm(vd) < nmath::kLinearTolerance)
        return make_error("hlr_project: degenerate view direction");
    if (nmath::norm(nmath::cross(vd, uh)) < nmath::kLinearTolerance)
        return make_error("hlr_project: up hint parallel to view direction (declined)");

    // Face-kind gate. A PLANE face has no silhouette (its outline is a topological
    // edge, already drawn). CYLINDER / SPHERE / CONE / TORUS faces carry an ANALYTIC
    // silhouette this service traces in closed form (drafting/silhouette.h); collect
    // them. A FREEFORM kind (BSpline / Bezier) is DECLINED — its silhouette is not
    // robustly traceable analytically, so we never guess it. NOTE: a native REVOLVE
    // builds a torus as rational-B-spline bands (Kind::BSpline), NOT a Kind::Torus
    // face, so a revolve-built torus lands in the freeform decline below with a
    // sharpened reason; a Kind::Torus face (STEP-imported) is traced.
    std::vector<ntopo::Shape> curvedFaces;
    for (ntopo::Explorer ex(shape, ntopo::ShapeType::Face); ex.more(); ex.next()) {
        const auto srf = ntopo::surfaceOf(ex.current());
        if (!srf || srf->surface == nullptr)
            return make_error("hlr_project: face carries no surface (declined)");
        using K = ntopo::FaceSurface::Kind;
        const K k = srf->surface->kind;
        if (k == K::Plane) continue;
        if (k == K::Cylinder || k == K::Sphere || k == K::Cone || k == K::Torus) {
            curvedFaces.push_back(ex.current());
            continue;
        }
        return make_error(
            "hlr_project: freeform (B-spline/Bezier) silhouette declined — the analytic "
            "silhouette tracer handles plane/cylinder/sphere/cone/torus faces; a native "
            "revolve builds a torus as B-spline bands, so a revolve-built torus declines "
            "here (STEP-imported analytic tori are traced)");
    }

    // Occluder = the M0 boundary mesh at the caller-chosen deflection (never a hidden
    // default: opts.deflection <= 0 uses the mesher's own documented default).
    ntess::MeshParams mp;
    if (opts.deflection > 0.0) mp.deflection = opts.deflection;
    const ntess::Mesh mesh = ntess::SolidMesher(mp).mesh(shape);
    std::vector<std::array<std::uint32_t, 3>> tris;
    tris.reserve(mesh.triangles.size());
    for (const ntess::Triangle& t : mesh.triangles) tris.push_back({t.a, t.b, t.c});
    ndraft::Occluder occ{&mesh.vertices, &tris};

    // Straight edges to draw: discretize each topological edge (a polyhedral edge is
    // a single straight segment; a generic polyline is fed as consecutive straight
    // segments) into world points + index pairs. Matches edge_polylines' cache.
    //
    // DE-DUPLICATE coincident edges. The native B-rep emits one edge node PER
    // ADJACENT FACE (edge-sharing is deferred — see native_construct_parity), so a
    // box carries 24 edge nodes for its 12 physical edges. Drawing every node would
    // draw each edge TWICE (correct classification, but doubled count / length /
    // overdraw). Keying each edge by its quantized, order-independent endpoint pair
    // collapses the coincident duplicates so the drawing emits each edge ONCE —
    // matching the OCCT oracle's edge set exactly.
    auto quantize = [](const nmath::Point3& p) {
        constexpr double kGrid = 1e7;  // 0.1 µm — well under the 1e-5 mm parity tol
        return std::array<long long, 3>{static_cast<long long>(std::llround(p.x * kGrid)),
                                        static_cast<long long>(std::llround(p.y * kGrid)),
                                        static_cast<long long>(std::llround(p.z * kGrid))};
    };
    std::set<std::array<long long, 6>> seenEdges;
    std::vector<nmath::Point3> edgeVertices;
    std::vector<ndraft::EdgeIndices> edges;
    // Fine deflection for the DRAWING discretization of CURVED topological edges (a
    // cylinder's end circles, projecting to ellipses) so their polyline density and
    // total length track the OCCT oracle's tangential-deflection sampling. A STRAIGHT
    // edge collapses to minSegs=1 (2 points) at ANY deflection, so the polyhedral
    // output is BYTE-IDENTICAL; only genuinely curved edges (which the polyhedral core
    // never carried) subdivide finer.
    ntess::EdgeCache cache(/*deflection=*/0.01, /*minSegs=*/1, /*maxSegs=*/256);
    auto edgeKey = [&](const ntess::EdgeDiscretization& d) {
        const std::array<long long, 3> qa = quantize(d.points.front());
        const std::array<long long, 3> qb = quantize(d.points.back());
        const bool aFirst = qa <= qb;  // order-independent key (endpoints sorted)
        std::array<long long, 6> key{};
        for (int k = 0; k < 3; ++k) {
            key[k] = (aFirst ? qa : qb)[k];
            key[3 + k] = (aFirst ? qb : qa)[k];
        }
        return key;
    };

    // SMOOTH-EDGE SUPPRESSION (curved bodies only). Native cc_solid_revolve builds a
    // curved solid as several tangent angular sectors, so its B-rep carries internal
    // sector-seam generators and coplanar cap spokes that are NOT feature lines — the
    // OCCT HLR oracle draws only SHARP edges + silhouette outlines and suppresses
    // these. An edge is SMOOTH when its adjacent faces meet tangentially (their
    // outward normals are parallel at the edge). We key each face's edge by its
    // endpoint pair (phase-independent) and collect the adjacent-face normals; an edge
    // whose faces are all mutually tangent is dropped. This branch runs ONLY when the
    // body has a curved face, so a purely planar body's edge set (and its 13/13
    // polyhedral parity) is BYTE-IDENTICAL.
    std::map<std::array<long long, 6>, std::vector<nmath::Vec3>> edgeFaceNormals;
    if (!curvedFaces.empty()) {
        for (ntopo::Explorer fx(shape, ntopo::ShapeType::Face); fx.more(); fx.next()) {
            const auto fsr = ntopo::surfaceOf(fx.current());
            if (!fsr || fsr->surface == nullptr) continue;
            for (ntopo::Explorer eex(fx.current(), ntopo::ShapeType::Edge); eex.more(); eex.next()) {
                const auto& de = cache.discretize(eex.current());
                if (de.points.size() < 2) continue;
                // Sample the analytic normal at an INTERIOR sample of the edge, NOT at
                // a discretization vertex: a STRAIGHT seam discretizes to just its two
                // endpoints, so de.points[size/2] is an ENDPOINT — and for a cone/torus
                // seam that endpoint can be the APEX (radial 0 → null normal), which
                // silently dropped the seam and left it drawn. Probe a few interior
                // points (chord fractions + discretization interiors) and take the first
                // with a well-defined normal so a cone/torus seam is correctly compared.
                nmath::Vec3 n{0, 0, 0};
                const nmath::Point3 a = de.points.front(), b = de.points.back();
                for (double t : {0.5, 0.375, 0.625, 0.25, 0.75}) {
                    const nmath::Point3 mid{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                                            a.z + (b.z - a.z) * t};
                    n = analyticFaceNormal(*fsr->surface, fsr->location, mid);
                    if (nmath::norm(n) > 0.5) break;
                }
                if (nmath::norm(n) <= 0.5 && de.points.size() > 2)
                    n = analyticFaceNormal(*fsr->surface, fsr->location,
                                           de.points[de.points.size() / 2]);
                if (nmath::norm(n) > 0.5) edgeFaceNormals[edgeKey(de)].push_back(n);
            }
        }
    }
    auto isSmoothEdge = [&](const std::array<long long, 6>& key) {
        const auto it = edgeFaceNormals.find(key);
        if (it == edgeFaceNormals.end() || it->second.size() < 2) return false;
        const nmath::Vec3& n0 = it->second.front();
        for (std::size_t j = 1; j < it->second.size(); ++j)
            if (std::fabs(nmath::dot(n0, it->second[j])) <= 0.99) return false;  // a sharp pair
        return true;  // every adjacent face tangent → smooth seam, suppress
    };

    const ntopo::ShapeMap map = ntopo::mapShapes(shape, ntopo::ShapeType::Edge);
    for (std::size_t i = 0; i < map.size(); ++i) {
        const ntess::EdgeDiscretization& d = cache.discretize(map.shape(static_cast<int>(i) + 1));
        if (d.points.size() < 2) continue;
        const std::array<long long, 6> key = edgeKey(d);
        if (!seenEdges.insert(key).second) continue;   // coincident duplicate → skip
        if (!curvedFaces.empty() && isSmoothEdge(key)) continue;  // tangent seam → suppress
        const auto base = static_cast<std::uint32_t>(edgeVertices.size());
        for (const auto& p : d.points) edgeVertices.push_back(p);
        for (std::uint32_t k = 0; k + 1 < d.points.size(); ++k)
            edges.push_back({base + k, base + k + 1});
    }

    // ── Quadric silhouette augmentation ─────────────────────────────────────────
    // For each CYLINDER / SPHERE face, trace the closed-form silhouette locus
    // (n·viewDir = 0) and append it as extra edge polylines (straight generators /
    // a discretized great circle). Each emitted point is SELF-VERIFIED to lie on the
    // occluder boundary (within a faceting-sized tolerance); if any outline point is
    // NOT on the boundary the silhouette is not robustly on this solid (a partial
    // quadric, or a view parallel to the axis) and the WHOLE projection is declined —
    // never a partial or fabricated outline. The appended edges then flow through the
    // SAME occlusion + visibility-split pass as every straight edge, so their
    // visible/hidden classification "just works".
    const double meshDefl = mp.deflection > 0.0 ? mp.deflection : 0.1;
    double maxSilhouetteOff = 0.0;  // worst facet inset of an emitted silhouette point
    if (!curvedFaces.empty()) {
        for (const ntopo::Shape& face : curvedFaces) {
            const auto srf = ntopo::surfaceOf(face);
            const ntopo::FaceSurface& fs = *srf->surface;
            const ntopo::Location& loc = srf->location;
            auto placeP = [&](const nmath::Point3& p) {
                return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
            };
            auto placeV = [&](const nmath::Vec3& v) {
                return loc.isIdentity() ? v : loc.transform().applyToVector(v);
            };
            nmath::Ax3 wf;
            wf.origin = placeP(fs.frame.origin);
            wf.x = nmath::Dir3{placeV(fs.frame.x.vec())};
            wf.y = nmath::Dir3{placeV(fs.frame.y.vec())};
            wf.z = nmath::Dir3{placeV(fs.frame.z.vec())};

            // Axial trim extent [vLo,vHi] along Z from the face's bounding edge
            // samples — needed by the ruled kinds (cylinder / cone).
            auto axialTrim = [&](double& vLo, double& vHi) -> bool {
                vLo = std::numeric_limits<double>::infinity();
                vHi = -std::numeric_limits<double>::infinity();
                const nmath::Vec3 zc = wf.z.vec();
                for (ntopo::Explorer eex(face, ntopo::ShapeType::Edge); eex.more(); eex.next()) {
                    const auto& de = cache.discretize(eex.current());
                    for (const auto& p : de.points) {
                        const double vv = nmath::dot(p - wf.origin, zc);
                        vLo = std::min(vLo, vv);
                        vHi = std::max(vHi, vv);
                    }
                }
                return vHi > vLo;
            };

            ndraft::SilhouetteResult sil;
            using K = ntopo::FaceSurface::Kind;
            if (fs.kind == K::Cylinder) {
                double vLo, vHi;
                if (!axialTrim(vLo, vHi))
                    return make_error("hlr_project: cylinder face has no trim extent (declined)");
                sil = ndraft::cylinderSilhouette(wf, fs.radius, vLo, vHi, vd);
            } else if (fs.kind == K::Cone) {
                double vLo, vHi;
                if (!axialTrim(vLo, vHi))
                    return make_error("hlr_project: cone face has no trim extent (declined)");
                // fs.radius is the CONE reference radius at v=0 (native segmentSurface),
                // fs.semiAngle its half-angle. The ruling runs between the rim radii at
                // vLo/vHi measured along the axis (frame origin = axial origin, not apex).
                sil = ndraft::coneSilhouette(wf, fs.radius, fs.semiAngle, vLo, vHi, vd);
            } else if (fs.kind == K::Torus) {
                // fs.radius = MAJOR radius, fs.minorRadius = tube radius.
                sil = ndraft::torusSilhouette(wf, fs.radius, fs.minorRadius, vd, meshDefl);
            } else {  // Sphere
                sil = ndraft::sphereSilhouette(wf.origin, fs.radius, vd, meshDefl);
            }
            if (!sil.traced)
                return make_error(std::string("hlr_project: ") +
                                  (sil.declineReason ? sil.declineReason : "silhouette declined"));

            // Self-verify tolerance: scale-aware. A point on a FULL quadric is at
            // most the chord sagitta (≈ deflection) off the inscribed facets; a
            // partial-quadric point in the trimmed-away region is ~R away — a huge
            // separation, so a generous fraction of R rejects partials while never
            // false-declining a full quadric under a coarse mesh.
            const double coverTol = std::max(0.25 * fs.radius, 3.0 * meshDefl + 1e-6);
            for (const ndraft::SilhouettePolyline& pl : sil.outlines) {
                if (pl.points.size() < 2) continue;
                // DE-DUPLICATE across the several angular SECTOR faces a curved solid
                // is built from: every sector shares the same axis, so all of them
                // trace the SAME generators / great circle. Key each outline by its
                // endpoint pair (as for topological edges) so the shared outline is
                // emitted ONCE, matching the oracle's single silhouette.
                const std::array<long long, 3> qa = quantize(pl.points.front());
                const std::array<long long, 3> qb = quantize(pl.points.back());
                const bool aFirst = qa <= qb;
                std::array<long long, 6> okey{};
                for (int k = 0; k < 3; ++k) {
                    okey[k] = (aFirst ? qa : qb)[k];
                    okey[3 + k] = (aFirst ? qb : qa)[k];
                }
                if (!seenEdges.insert(okey).second) continue;  // same outline from another sector
                // Self-verify every outline point lies on the occluder boundary, and
                // record the worst facet inset (used to size the self-grazing offset).
                for (const nmath::Point3& p : pl.points) {
                    const double off = minDistToMesh(p, mesh, tris);
                    if (off > coverTol)
                        return make_error(
                            "hlr_project: traced silhouette leaves the solid boundary "
                            "(partial quadric declined)");
                    maxSilhouetteOff = std::max(maxSilhouetteOff, off);
                }
                const auto base = static_cast<std::uint32_t>(edgeVertices.size());
                for (const nmath::Point3& p : pl.points) edgeVertices.push_back(p);
                for (std::uint32_t k = 0; k + 1 < pl.points.size(); ++k)
                    edges.push_back({base + k, base + k + 1});
            }
        }
    }

    ndraft::OrthographicView view;
    view.viewDir = nmath::Dir3{vd};
    view.up = nmath::Dir3{uh};
    ndraft::HlrParams prm;
    if (opts.samplesPerEdge > 0) prm.samplesPerEdge = opts.samplesPerEdge;
    if (opts.surfaceOffset > 0.0) prm.surfaceOffset = opts.surfaceOffset;
    // Curved silhouette present: the occluder is the INSCRIBED facet mesh, whose
    // chords sit *inside* the true surface by up to the chord sagitta. A convex limb
    // sample at the default micro-offset can self-graze those inset facets and be
    // spuriously classified HIDDEN. Push the sample out past the WORST measured facet
    // inset (maxSilhouetteOff, the true sagitta the mesher achieved — which can exceed
    // the requested deflection) so it clears its own inscribed occluder. Polyhedral
    // inputs never take this branch (curvedFaces empty), so their classification is
    // BYTE-IDENTICAL.
    if (!curvedFaces.empty() && opts.surfaceOffset <= 0.0)
        prm.surfaceOffset = std::max(meshDefl, 3.0 * maxSilhouetteOff + 1e-6);

    const ndraft::HlrResult hlr = ndraft::projectOrthographic(edgeVertices, edges, occ, view, prm);

    DrawingData out;
    out.visible.reserve(hlr.visible.size());
    for (const ndraft::Segment2D& s : hlr.visible)
        out.visible.push_back(DrawingSegmentData{s.ax, s.ay, s.bx, s.by});
    out.hidden.reserve(hlr.hidden.size());
    for (const ndraft::Segment2D& s : hlr.hidden)
        out.hidden.push_back(DrawingSegmentData{s.ax, s.ay, s.bx, s.by});
    return out;
}

// ── drafting: planar SECTION CURVES over the analytic core ──────────────────────
// Intersect every face of the native B-rep with the cut plane via the OCCT-free
// section service (M1-SSI + topology consumed read-only) and return the closed
// section loops. Honest DECLINE (mapped to an Error the facade surfaces) for the
// oblique cylinder cut / coincident-tangent plane / non-closing / freeform cases —
// never a wrong or open section. A mesh body carries no B-rep, so it is declined.
Result<SectionData> NativeEngine::section_plane(EngineShape body, const double origin[3],
                                                const double normal[3]) {
    if (!isNative(body)) return fallback().section_plane(body, origin, normal);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh)
        return make_error("section_plane: a native mesh body carries no B-rep topology (declined)");
    const nmath::Vec3 nrm{normal[0], normal[1], normal[2]};
    if (nmath::norm(nrm) < nmath::kLinearTolerance)
        return make_error("section_plane: degenerate cut-plane normal (declined)");

    // Build the cut-plane frame with a well-defined in-plane X (⟂ the normal).
    const nmath::Dir3 n{nrm};
    const double axc = std::fabs(n.x()), ayc = std::fabs(n.y()), azc = std::fabs(n.z());
    const nmath::Vec3 pick = (axc <= ayc && axc <= azc) ? nmath::Vec3{1, 0, 0}
                             : (ayc <= azc)             ? nmath::Vec3{0, 1, 0}
                                                        : nmath::Vec3{0, 0, 1};
    const nmath::Plane cutPlane{nmath::Ax3::fromAxisAndRef(
        nmath::Point3{origin[0], origin[1], origin[2]}, n, nmath::Dir3{nmath::cross(n.vec(), pick)})};

    const nsec::SectionResult r = nsec::sectionByPlane(holder->shape, cutPlane);
    if (r.status != nsec::SectionStatus::Ok)
        return make_error(std::string("section_plane: ") + r.reason);

    SectionData out;
    out.loops.reserve(r.loops.size());
    for (const nsec::SectionLoop& lp : r.loops) {
        SectionLoopData d;
        d.pointsXYZ.reserve(lp.points.size() * 3);
        for (const nmath::Point3& p : lp.points) {
            d.pointsXYZ.push_back(p.x);
            d.pointsXYZ.push_back(p.y);
            d.pointsXYZ.push_back(p.z);
        }
        d.shape = static_cast<int>(lp.shape);
        d.length = lp.length();
        d.area = lp.area(cutPlane.pos.x, cutPlane.pos.y, cutPlane.pos.origin);
        out.loops.push_back(std::move(d));
    }
    out.totalLength = r.totalLength();
    out.totalArea = r.totalArea();
    return out;
}

// GS5 native inertia: principal moments of inertia (unit-density volume inertia)
// from the M0 boundary triangulation via signed-tetra second moments about the
// centroid + a symmetric-3×3 eigen. SELF-VERIFY: the mesh must be WATERTIGHT
// (principalInertia's precondition) — an open/non-closed body has no defined
// enclosed inertia and DECLINES with an error rather than a wrong tensor. Matches
// OCCT GProp_PrincipalProps on the sim gate (exact for planar solids, deflection-
// scaled for curved). An OCCT body forwards to the GProp oracle.
Result<std::vector<double>> NativeEngine::principal_moments(EngineShape body) {
    if (!isNative(body)) return fallback().principal_moments(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    // A B-rep body meshes at the property deflection first; an imported STL soup is
    // measured directly (inertia is defined only if that soup is watertight).
    ntess::Mesh brepMesh;
    if (!holder->isMesh) {
        ntess::MeshParams params;
        params.deflection = kPropertyDeflection;
        brepMesh = ntess::SolidMesher(params).mesh(holder->shape);
    }
    const ntess::Mesh& mesh = holder->isMesh ? holder->mesh : brepMesh;
    auto inertia = nan::principalInertia(mesh);
    if (!inertia)
        return make_error(
            "principal_moments: declined (mesh not watertight / zero volume — no "
            "defined inertia)");
    return std::vector<double>{inertia->moments[0], inertia->moments[1], inertia->moments[2]};
}

// DM4 native projection: the closed-form foot-of-perpendicular of a point on a
// plane / cylinder / sphere face's analytic surface (directmodel::projectPointOnFace).
// A cone / torus / freeform face, or an AMBIGUOUS pose (point on a cylinder axis / at
// a sphere centre), is honestly DECLINED with an error — never a fabricated foot. An
// OCCT body forwards to the GeomAPI_ProjectPointOnSurf oracle.
Result<ProjectionData> NativeEngine::project_point_on_face(EngineShape body, int f, double px,
                                                           double py, double pz) {
    if (!isNative(body)) return fallback().project_point_on_face(body, f, px, py, pz);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh)
        return make_error("project_point_on_face: a native mesh body carries no analytic "
                          "surface (declined)");
    namespace dm = cybercad::native::directmodel;
    namespace nm = cybercad::native::math;
    dm::ProjectDecline why = dm::ProjectDecline::Ok;
    const auto r = dm::projectPointOnFace(holder->shape, f, nm::Point3{px, py, pz}, &why);
    if (!r)
        return make_error("project_point_on_face: declined (non-analytic / ambiguous face — "
                          "native scope: plane / cylinder / sphere → OCCT-only)");
    ProjectionData out;
    out.footX = r->foot.x;
    out.footY = r->foot.y;
    out.footZ = r->foot.z;
    out.distance = r->distance;
    return out;
}

// GS6 native validity: a structural-validity report over the M0 boundary mesh
// (closed 2-manifold, consistent outward orientation, no degenerate/self-
// intersecting faces, finite coords). SELF-VERIFY / HONEST DECLINE: a coplanar-
// overlap self-intersection the transversal predicate cannot decide leaves
// `certified=false` and `valid=false` — the checker NEVER emits a false "valid".
// Matches BRepCheck_Analyzer::IsValid on the sim gate. An OCCT body forwards to
// the BRepCheck oracle.
Result<ValidityData> NativeEngine::check_solid(EngineShape body) {
    if (!isNative(body)) return fallback().check_solid(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    ntess::Mesh brepMesh;
    if (!holder->isMesh) {
        ntess::MeshParams params;
        params.deflection = kPropertyDeflection;
        brepMesh = ntess::SolidMesher(params).mesh(holder->shape);
    }
    const ntess::Mesh& mesh = holder->isMesh ? holder->mesh : brepMesh;
    const nan::ValidityReport rep = nan::checkSolidMesh(mesh);
    ValidityData out;
    out.closed = rep.closed;
    out.oriented = rep.oriented;
    out.nondegenerate = rep.nondegenerate;
    out.finite = rep.finite;
    out.noSelfIntersection = rep.noSelfIntersection;
    out.certified = rep.selfIntersectionCertified;
    out.valid = rep.valid();
    return out;
}

// ── MOAT M-GS GS7 — CLASH / INTERFERENCE of two native solids ────────────────────
// Both operands must be native (a mixed native/OCCT pair is rejected honestly; an
// all-OCCT pair forwards to the BRepAlgoAPI_Common oracle). Both are meshed at the
// property deflection; the mesh-level classifier (src/native/analysis/interference.h)
// decides CLASH / TOUCHING / CLEAR + the witness. On CLASH the overlap VOLUME is the
// native boolean COMMON, guarded by a TWO-SIDED self-verify:
//   (i)  the COMMON solid must be watertight (else the volume is not trustworthy), and
//   (ii) it must be bounded by min(V(A),V(B)) — an overlap cannot exceed either operand
//        (the independent set-algebra sanity bound the boolean self-verify also uses).
// If the native COMMON is not robustly available (null / non-watertight / out-of-band)
// the whole verdict is DECLINED (Error) so the facade falls through to OCCT — a wrong
// overlap volume is NEVER returned. A clash with no measurable volume likewise declines.
Result<InterferenceData> NativeEngine::interference(EngineShape a, EngineShape b) {
    const bool aNative = isNative(a);
    const bool bNative = isNative(b);
    if (!aNative && !bNative) return fallback().interference(a, b);
    if (aNative != bNative)
        return make_error(
            "interference: mixed native/OCCT operands are not supported "
            "(build both bodies under the same active engine)");

    const auto* ha = static_cast<const NativeShape*>(a.get());
    const auto* hb = static_cast<const NativeShape*>(b.get());

    // Mesh both operands (a mesh body IS its geometry; a B-rep body meshes first).
    ntess::MeshParams mp;
    mp.deflection = kPropertyDeflection;
    ntess::Mesh brepA, brepB;
    if (!ha->isMesh) brepA = ntess::SolidMesher(mp).mesh(ha->shape);
    if (!hb->isMesh) brepB = ntess::SolidMesher(mp).mesh(hb->shape);
    const ntess::Mesh& meshA = ha->isMesh ? ha->mesh : brepA;
    const ntess::Mesh& meshB = hb->isMesh ? hb->mesh : brepB;

    nan::InterferenceResult ir = nan::meshInterference(meshA, meshB, kPropertyDeflection);
    if (ir.state == nan::ClashState::Unknown)
        return make_error(
            "interference: mesh evidence is ambiguous (non-watertight operand or a "
            "boundary point the ray-parity classifier declined) — verdict declined");

    InterferenceData out;
    out.minDistance = ir.minDistance;
    out.hasWitness = ir.hasWitness;
    out.witLoX = ir.witnessLo.x; out.witLoY = ir.witnessLo.y; out.witLoZ = ir.witnessLo.z;
    out.witHiX = ir.witnessHi.x; out.witHiY = ir.witnessHi.y; out.witHiZ = ir.witnessHi.z;
    out.witPX = ir.witnessPoint.x; out.witPY = ir.witnessPoint.y; out.witPZ = ir.witnessPoint.z;

    if (ir.state != nan::ClashState::Clash) {
        out.state = (ir.state == nan::ClashState::Touching) ? 1 : 0;  // touching / clear
        out.overlapVolume = 0.0;
        return out;
    }

    // CLASH: the overlap VOLUME is the native boolean COMMON. Only a B-rep pair can be
    // fed to the boolean; a mesh-soup operand has no B-rep to intersect → decline to OCCT.
    if (ha->isMesh || hb->isMesh)
        return make_error(
            "interference: clash detected but a mesh-soup operand has no B-rep overlap "
            "volume (native COMMON needs B-rep operands) — volume declined to OCCT");

    const ntopo::Shape common = cybercad::native::boolean::boolean_solid(
        ha->shape, hb->shape, cybercad::native::boolean::Op::Common);
    const double vc = watertightVolume(common);  // <0 ⇒ null / not watertight
    if (vc <= 0.0)
        return make_error(
            "interference: clash detected but the native COMMON overlap volume is not "
            "robustly available (curved/near-tangent operand) — volume declined to OCCT");

    // TWO-SIDED self-verify: an overlap cannot exceed either operand's own volume.
    const double vA = watertightVolume(ha->shape);
    const double vB = watertightVolume(hb->shape);
    if (vA > 0.0 && vB > 0.0) {
        const double cap = std::min(vA, vB);
        const double tol = std::max(1e-6 * cap, 1e-9);
        if (vc > cap + tol)
            return make_error(
                "interference: native COMMON volume exceeds the smaller operand — the "
                "overlap is not trustworthy (self-verify), declined to OCCT");
    }

    out.state = 2;  // clash
    out.overlapVolume = vc;

    // Sharpen the witness from the COMMON solid itself: its tight mesh AABB + its
    // signed-tetra centroid — a point guaranteed to lie in the overlap INTERIOR,
    // matching the OCCT oracle's witness (the COMMON bbox + centre of mass). This
    // supersedes the mesh-classifier's boundary witness with the true overlap region.
    {
        const ntess::Mesh cm = ntess::SolidMesher(mp).mesh(common);
        if (!cm.vertices.empty()) {
            nmath::Point3 lo = cm.vertices.front(), hi = cm.vertices.front();
            for (const auto& p : cm.vertices) {
                lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
                lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
                lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
            }
            double cx = 0, cy = 0, cz = 0, v6 = 0;
            for (const auto& t : cm.triangles) {
                const auto& A = cm.vertices[t.a]; const auto& B = cm.vertices[t.b];
                const auto& C = cm.vertices[t.c];
                const double v = A.x * (B.y * C.z - B.z * C.y) - A.y * (B.x * C.z - B.z * C.x) +
                                 A.z * (B.x * C.y - B.y * C.x);
                v6 += v; cx += v * (A.x + B.x + C.x); cy += v * (A.y + B.y + C.y);
                cz += v * (A.z + B.z + C.z);
            }
            out.hasWitness = true;
            out.witLoX = lo.x; out.witLoY = lo.y; out.witLoZ = lo.z;
            out.witHiX = hi.x; out.witHiY = hi.y; out.witHiZ = hi.z;
            if (std::fabs(v6) > 1e-12) {
                const double inv = 1.0 / (4.0 * v6);
                out.witPX = cx * inv; out.witPY = cy * inv; out.witPZ = cz * inv;
            } else {
                out.witPX = 0.5 * (lo.x + hi.x); out.witPY = 0.5 * (lo.y + hi.y);
                out.witPZ = 0.5 * (lo.z + hi.z);
            }
        }
    }
    return out;
}

// ── M-REF reference / topology reads (NATIVE, OCCT-FREE) ─────────────────────────
// Datum + topology-reference queries computed from the native B-rep (src/native/
// reference). A native body is served here; an OCCT body forwards. On an HONEST
// DECLINE (nullopt) a native body returns a clean Error so the facade falls
// through to OCCT — the void is NEVER handed to OCCT. A mesh body has no B-rep
// topology → clean error (not forwarded).
namespace {
// Resolve a 1-based sub-shape id to its world-placed Shape (empty on out-of-range).
ntopo::Shape refSubShape(const ntopo::Shape& root, ntopo::ShapeType type, int id) {
    const ntopo::ShapeMap map = ntopo::mapShapes(root, type);
    if (id < 1 || id > static_cast<int>(map.size())) return ntopo::Shape{};
    return map.shape(id);
}
}  // namespace

Result<std::vector<double>> NativeEngine::face_axis(EngineShape body, int f) {
    if (!isNative(body)) return fallback().face_axis(body, f);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("face_axis: mesh body has no B-rep geometry");
    const ntopo::Shape face = refSubShape(holder->shape, ntopo::ShapeType::Face, f);
    if (face.isNull()) return make_error("face_axis: face not found");
    auto ax = nref::faceAxis(face);
    if (!ax) return make_error("face_axis: face has no axis (only cylinder/cone)");
    return std::vector<double>(ax->begin(), ax->end());
}
Result<std::vector<double>> NativeEngine::ref_plane_from_face(EngineShape body, int f) {
    if (!isNative(body)) return fallback().ref_plane_from_face(body, f);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("ref_plane_from_face: mesh body has no B-rep geometry");
    const ntopo::Shape face = refSubShape(holder->shape, ntopo::ShapeType::Face, f);
    if (face.isNull()) return make_error("ref_plane_from_face: face not found");
    auto pl = nref::refPlaneFromFace(face);
    if (!pl) return make_error("ref_plane_from_face: non-planar face");
    return std::vector<double>(pl->begin(), pl->end());
}
Result<std::vector<double>> NativeEngine::ref_axis_from_edge(EngineShape body, int e) {
    if (!isNative(body)) return fallback().ref_axis_from_edge(body, e);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("ref_axis_from_edge: mesh body has no B-rep geometry");
    const ntopo::Shape edge = refSubShape(holder->shape, ntopo::ShapeType::Edge, e);
    if (edge.isNull()) return make_error("ref_axis_from_edge: edge not found");
    auto ax = nref::refAxisFromEdge(edge);
    if (!ax) return make_error("ref_axis_from_edge: non-linear edge");
    return std::vector<double>(ax->begin(), ax->end());
}
Result<std::vector<double>> NativeEngine::ref_axis_from_face(EngineShape body, int f) {
    if (!isNative(body)) return fallback().ref_axis_from_face(body, f);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("ref_axis_from_face: mesh body has no B-rep geometry");
    const ntopo::Shape face = refSubShape(holder->shape, ntopo::ShapeType::Face, f);
    if (face.isNull()) return make_error("ref_axis_from_face: face not found");
    auto ax = nref::refAxisFromFace(face);
    if (!ax) return make_error("ref_axis_from_face: face has no axis");
    return std::vector<double>(ax->begin(), ax->end());
}
Result<std::vector<int>> NativeEngine::tangent_chain(EngineShape body, const int* e, int ec) {
    if (!isNative(body)) return fallback().tangent_chain(body, e, ec);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh || e == nullptr || ec <= 0)
        return make_error("tangent_chain: mesh body or empty selection");
    auto chain = nref::tangentChain(holder->shape, std::vector<int>(e, e + ec));
    if (!chain) return make_error("tangent_chain: freeform edge in walk (deferred to oracle)");
    return *chain;  // possibly empty (no chain) — a valid answer, matches OCCT
}
Result<std::vector<int>> NativeEngine::outer_rim_chain(EngineShape body, const int* e, int ec) {
    if (!isNative(body)) return fallback().outer_rim_chain(body, e, ec);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh || e == nullptr || ec <= 0)
        return make_error("outer_rim_chain: mesh body or empty selection");
    return nref::outerRimChain(holder->shape, std::vector<int>(e, e + ec));  // empty = no cap
}
Result<std::vector<double>> NativeEngine::offset_face_boundary(EngineShape body, int f, double d) {
    if (!isNative(body)) return fallback().offset_face_boundary(body, f, d);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    if (holder->isMesh) return make_error("offset_face_boundary: mesh body has no B-rep geometry");
    const ntopo::Shape face = refSubShape(holder->shape, ntopo::ShapeType::Face, f);
    if (face.isNull()) return make_error("offset_face_boundary: bad faceId");
    auto pts = nref::offsetFaceBoundary(face, d);
    if (!pts) return make_error("offset_face_boundary: declined (non-polygonal / arc-join / self-intersect)");
    return *pts;
}

// ── NATIVE affine transforms (M-TX) ─────────────────────────────────────────────
// translate / rotate / uniform-scale / mirror / place_on_frame on a NATIVE body are
// served natively by APPLYING a math::Transform to the body (they used to hard-error
// on a native body). The machinery is the placement path certified by
// tests/sim/native_transform_fuzz.mm (native topology::Shape::located(math::Transform)
// + SolidMesher, differentially fuzzed vs OCCT BRepBuilderAPI_Transform(gp_Trsf) +
// BRepGProp AND a closed-form similarity image). A NON-native (OCCT) body forwards to
// the OCCT adapter exactly as before; a native body is NEVER forwarded to OCCT (its
// unwrap would misread the void).

namespace {
// Apply an affine math::Transform to a native body, returning a fresh native handle
// or nullptr on an honest decline (the caller then returns a clean error — never
// OCCT). B-rep bodies use Shape::located()+SolidMesher (the fuzzed path); a mesh
// (imported STL) body transforms its vertices/normals directly. A NON-INVERTIBLE
// (singular / zero-scale) linear part collapses the solid → decline. A mirror (det<0)
// is kept: the placement flips the signed enclosed-volume sign yet stays a valid
// watertight positive-|vol| solid — the located() convention the fuzzer certifies.
// B-rep results self-verify robustly watertight with positive |vol| before being kept
// (a similarity/rigid/mirror placement of a watertight solid stays watertight, so this
// holds for any legitimate body; a surprise declines honestly rather than shipping a
// leaky solid).
EngineShape applyNativeTransform(const NativeShape& holder, const nmath::Transform& xf) {
    if (!xf.inverse().has_value()) return nullptr;  // singular / degenerate → decline
    if (holder.isMesh) {
        ntess::Mesh m = holder.mesh;  // deep copy; transform in place
        for (auto& v : m.vertices) v = xf.applyToPoint(v);
        if (m.hasNormals())
            for (auto& n : m.normals) n = xf.applyToDir(n);
        return wrapNativeMesh(std::move(m));
    }
    ntopo::Shape located = holder.shape.located(ntopo::Location{xf});
    if (located.isNull()) return nullptr;
    if (!robustlyWatertight(located) || !(watertightVolume(located) > 0.0)) return nullptr;
    return wrapNative(std::move(located));
}
}  // namespace

ShapeResult NativeEngine::scale_shape(EngineShape body, double f) {
    if (!isNative(body)) return fallback().scale_shape(body, f);
    if (!(f > 0.0)) return make_error("scale_shape: non-positive factor");
    const auto* holder = static_cast<const NativeShape*>(body.get());
    EngineShape out = applyNativeTransform(*holder, nmath::Transform::scaleOf(nmath::Point3{0, 0, 0}, f));
    if (!out) return make_error("scale_shape: native placement declined (degenerate)");
    return track(std::move(out));
}
ShapeResult NativeEngine::scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                            double f) {
    if (!isNative(body)) return fallback().scale_shape_about(body, cx, cy, cz, f);
    if (!(f > 0.0)) return make_error("scale_shape_about: non-positive factor");
    const auto* holder = static_cast<const NativeShape*>(body.get());
    EngineShape out =
        applyNativeTransform(*holder, nmath::Transform::scaleOf(nmath::Point3{cx, cy, cz}, f));
    if (!out) return make_error("scale_shape_about: native placement declined (degenerate)");
    return track(std::move(out));
}
ShapeResult NativeEngine::rotate_shape_about(EngineShape body, double cx, double cy, double cz,
                                             double ax, double ay, double az, double a) {
    if (!isNative(body)) return fallback().rotate_shape_about(body, cx, cy, cz, ax, ay, az, a);
    if (ax * ax + ay * ay + az * az < 1.0e-12) return make_error("rotate_shape_about: zero axis");
    const auto* holder = static_cast<const NativeShape*>(body.get());
    const nmath::Dir3 axis(ax, ay, az);  // Dir3 ctor normalizes the axis
    EngineShape out = applyNativeTransform(
        *holder, nmath::Transform::rotationOf(nmath::Point3{cx, cy, cz}, axis, a));
    if (!out) return make_error("rotate_shape_about: native placement declined");
    return track(std::move(out));
}
ShapeResult NativeEngine::mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                                       double ny, double nz) {
    if (!isNative(body)) return fallback().mirror_shape(body, px, py, pz, nx, ny, nz);
    const double nn = nx * nx + ny * ny + nz * nz;
    if (nn < 1.0e-12) return make_error("mirror_shape: zero plane normal");
    // Reflection across the plane through (px,py,pz) with UNIT normal u:
    //   L = I − 2 u uᵀ (det = −1),  t = 2 (p·u) u   (matches OCCT gp_Trsf::SetMirror).
    const double inv = 1.0 / std::sqrt(nn);
    const double ux = nx * inv, uy = ny * inv, uz = nz * inv;
    const nmath::Mat3 L(1 - 2 * ux * ux, -2 * ux * uy, -2 * ux * uz, -2 * uy * ux,
                        1 - 2 * uy * uy, -2 * uy * uz, -2 * uz * ux, -2 * uz * uy,
                        1 - 2 * uz * uz);
    const double pd = px * ux + py * uy + pz * uz;
    const nmath::Vec3 t{2 * pd * ux, 2 * pd * uy, 2 * pd * uz};
    const auto* holder = static_cast<const NativeShape*>(body.get());
    EngineShape out = applyNativeTransform(*holder, nmath::Transform{L, t});
    if (!out) return make_error("mirror_shape: native placement declined");
    return track(std::move(out));
}
ShapeResult NativeEngine::translate_shape(EngineShape body, double tx, double ty, double tz) {
    if (!isNative(body)) return fallback().translate_shape(body, tx, ty, tz);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    EngineShape out =
        applyNativeTransform(*holder, nmath::Transform::translationOf(nmath::Vec3{tx, ty, tz}));
    if (!out) return make_error("translate_shape: native placement declined");
    return track(std::move(out));
}
ShapeResult NativeEngine::place_on_frame(EngineShape body, double ox, double oy, double oz,
                                         double ux, double uy, double uz, double vx, double vy,
                                         double vz) {
    if (!isNative(body))
        return fallback().place_on_frame(body, ox, oy, oz, ux, uy, uz, vx, vy, vz);
    const nmath::Vec3 u{ux, uy, uz}, v{vx, vy, vz};
    const nmath::Vec3 n = nmath::cross(u, v);
    if (nmath::norm(u) < 1.0e-9 || nmath::norm(v) < 1.0e-9 || nmath::norm(n) < 1.0e-9)
        return make_error("place_on_frame: degenerate frame");
    // Rigid motion relocating the global XOY frame onto the destination frame
    // (origin, x-dir = u, main/z-dir = u×v, y-dir = z×x) — a pure rotation+translation,
    // sketch dimensions preserved. Matches OCCT gp_Ax3(o, dir(n), dir(u)) +
    // SetDisplacement(gp_Ax3(), dst). L's columns are the destination axes.
    const nmath::Dir3 ex(u), ez(n);
    const nmath::Dir3 ey(nmath::cross(ez.vec(), ex.vec()));
    const nmath::Mat3 L(ex.x(), ey.x(), ez.x(), ex.y(), ey.y(), ez.y(), ex.z(), ey.z(), ez.z());
    const nmath::Vec3 t{ox, oy, oz};
    const auto* holder = static_cast<const NativeShape*>(body.get());
    EngineShape out = applyNativeTransform(*holder, nmath::Transform{L, t});
    if (!out) return make_error("place_on_frame: native placement declined");
    return track(std::move(out));
}

// ── exchange: NATIVE STEP export for a native-representable body, else OCCT ──────
//
// step_export is NATIVE (Phase 4 #7) for a native body whose every face surface +
// edge curve is a kind the writer serialises (plane/cylinder/cone/sphere/bspline
// surfaces; line/circle/bspline curves): src/native/exchange emits a valid
// ISO-10303-21 AP203 file in true mm. The honesty gate (NATIVE-REWRITE.md #7): the
// file must re-read through OCCT STEPControl_Reader to the SAME solid.
//   * A native body OUT of the writer's scope (an unsupported geometry kind) has no
//     native serialisation AND must not be handed to OCCT (which cannot read a
//     native void) → clean honest error, never a wrong/invalid file.
//   * A NON-native (OCCT-built) body forwards to the OCCT STEPControl_Writer.
// step_import + iges_* stay OCCT (parsing arbitrary STEP/IGES is out of scope).
Result<void> NativeEngine::step_export(EngineShape body, const char* path) {
    if (isNative(body)) {
        const auto* holder = static_cast<const NativeShape*>(body.get());
        if (cybercad::native::exchange::step_can_export_native(holder->shape)) {
            const std::string p = path ? path : "";
            if (cybercad::native::exchange::step_export_native(holder->shape, p))
                return Result<void>{};
            return make_error("native STEP export failed to write file: " + p);
        }
        return make_error(
            "native STEP export unsupported for this body's geometry "
            "(scope: plane/cylinder/cone/sphere/bspline faces, line/circle/bspline edges)");
    }
    return fallback().step_export(body, path);
}
// NATIVE STEP IMPORT (first slice). The reader is OCCT-free: parse the ISO-10303-21
// AP203 file and reconstruct a native B-rep Solid the writer's entity set describes
// (plane/cylinder/cone/sphere/bspline faces, line/circle/bspline edges, one
// MANIFOLD_SOLID_BREP, mm units). We keep the result native ONLY when it self-
// verifies robustly watertight with positive volume; a NULL parse (out of scope /
// malformed / unhealable) or a failed self-verify falls through to the OCCT
// STEPControl_Reader (labelled, honest — src/native stays OCCT-free, the fallback
// is engine-side only). Never fabricates a solid the file did not describe.
ShapeResult NativeEngine::step_import(const char* path) {
    if (path) {
        ntopo::Shape solid = cybercad::native::exchange::step_import_native(path);
        // A single Solid or a multi-solid Compound (T2): every member must self-verify
        // robustly watertight, else the whole import declines to the OCCT reader.
        if (!solid.isNull() && robustlyWatertightImport(solid))
            return track(wrapNative(std::move(solid)));
    }
    return fallback().step_import(path);
}
// NATIVE AP242 PMI SCAN (additive, read-only). A SEPARATE pass over the parsed
// Part-21 record table — it does NOT touch step_import's geometry path, so the
// imported solid is byte-identical whether or not this is called. Recognise /
// classify / count only (no GD&T semantic model); OCCT-free.
Result<PmiData> NativeEngine::pmi_scan(const char* path) {
    if (!path) return make_error("pmi_scan: null path");
    // An unreadable file is a FAILURE (0 + error); a readable file with no PMI is a
    // SUCCESS with total 0. Probe openability to keep those cases distinct.
    if (!std::ifstream(path, std::ios::binary))
        return make_error(std::string("pmi_scan: cannot open file: ") + path);
    const cybercad::native::exchange::PmiSummary s =
        cybercad::native::exchange::step_scan_pmi(path);
    PmiData out;
    out.dimensions = static_cast<int>(s.dimensions);
    out.tolerances = static_cast<int>(s.tolerances);
    out.datums = static_cast<int>(s.datums);
    out.datumTargets = static_cast<int>(s.datumTargets);
    out.notes = static_cast<int>(s.notes);
    out.annotationGeometry = static_cast<int>(s.annotationGeometry);
    out.unknown = static_cast<int>(s.unknown);
    out.total = static_cast<int>(s.total);
    out.anyPmi = s.anyPmi;
    return out;
}
Result<void> NativeEngine::iges_export(EngineShape body, const char* path) {
    CC_NATIVE_BODY_UNSUPPORTED("iges_export", body);
    return fallback().iges_export(body, path);
}
ShapeResult NativeEngine::iges_import(const char* path) { return fallback().iges_import(path); }

// NATIVE STL import (issue #5). The reader is OCCT-free, so this builds a mesh-backed
// native body directly (no fallthrough). A malformed file yields no mesh → clean
// error, and nothing is registered (the facade only records a valid ShapeResult).
ShapeResult NativeEngine::stl_import(const char* path) {
    if (!path) return make_error("stl_import: null path");
    std::string err;
    std::optional<ntess::Mesh> mesh = cybercad::native::exchange::stl_read(path, err);
    if (!mesh) return make_error("stl_import: " + err);
    return track(wrapNativeMesh(std::move(*mesh)));
}

// NATIVE bounded N-sided fill (MOAT surface). Build the analytic boundary loop from
// the POD arrays and evaluate the tessellated Coons/Gregory patch (surface::fillNGon).
// SCOPE BOUND: 3–6 straight/arc sides; the patch is a MESH interpolant, NOT a NURBS
// surface. Returns a MESH-BACKED body (the patch surface, like an imported STL soup —
// served by mass_properties/bounding_box/tessellate/face_meshes). A non-analytic /
// >6-sided / degenerate / self-intersecting boundary HONEST-DECLINES: on the OCCT-free
// host it errors; where OCCT is linked the facade routes to the BRepFill_Filling
// oracle. A native void is NEVER handed to OCCT.
ShapeResult NativeEngine::fill_ngon(const double* boundaryXYZ, int cornerCount,
                                    const int* edgeKinds, const double* arcMids, int gridN) {
    namespace nsf = cybercad::native::surface;
    if (boundaryXYZ == nullptr || cornerCount < 3 || cornerCount > 6)
        return fallback().fill_ngon(boundaryXYZ, cornerCount, edgeKinds, arcMids, gridN);

    auto pt = [](const double* a, int k) {
        return nmath::Point3{a[k * 3], a[k * 3 + 1], a[k * 3 + 2]};
    };
    nsf::Boundary boundary;
    boundary.sides.reserve(cornerCount);
    int arcSeen = 0;
    for (int i = 0; i < cornerCount; ++i) {
        const int j = (i + 1) % cornerCount;
        nsf::BoundarySide s;
        s.start = pt(boundaryXYZ, i);
        s.end = pt(boundaryXYZ, j);
        s.arc = (edgeKinds != nullptr && edgeKinds[i] == 1);
        if (s.arc) {
            if (arcMids == nullptr)
                return fallback().fill_ngon(boundaryXYZ, cornerCount, edgeKinds, arcMids, gridN);
            s.mid = pt(arcMids, arcSeen++);
        }
        boundary.sides.push_back(s);
    }

    nsf::NGonOptions opts;
    opts.gridN = gridN >= 2 ? gridN : 12;
    nsf::NGonDecline why = nsf::NGonDecline::Ok;
    const nsf::NGonPatch patch = nsf::fillNGon(boundary, opts, &why);
    // Self-verify boundary-coincidence + a non-degenerate patch, else honest decline
    // (never a wrong patch): the patch must have triangles, positive area, and every
    // boundary sample must lie on its analytic curve (residual ~0 by construction).
    if (!patch.valid || patch.mesh.triangleCount() == 0 ||
        !(ntess::surfaceArea(patch.mesh) > 0.0) || patch.onBoundaryResidual > 1e-6)
        return fallback().fill_ngon(boundaryXYZ, cornerCount, edgeKinds, arcMids, gridN);
    return track(wrapNativeMesh(patch.mesh));
}

}  // namespace cyber
