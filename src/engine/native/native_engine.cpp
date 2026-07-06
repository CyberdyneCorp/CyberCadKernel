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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "engine/native/native_heal_hook.h"
#include "native/blend/native_blend.h"
#include "native/boolean/native_boolean.h"
#include "native/construct/native_construct.h"
#include "native/exchange/native_exchange.h"
#include "native/feature/wrap_emboss.h"
#include "native/heal/native_heal.h"
#include "native/tessellate/native_tessellate.h"

#ifdef CYBERCAD_HAS_OCCT
#include "engine/occt/occt_engine.h"
#endif

namespace cyber {

namespace {

namespace ntopo = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;

// ── Native shape holder type-erased behind the registry's EngineShape ──────────
// Distinct from occt::OcctShape. A native void MUST NEVER be handed to the OCCT
// adapter, whose unwrap() does an UNCHECKED static_pointer_cast<OcctShape> and would
// read a NativeShape as garbage (a null/garbage TopoDS_Shape → BRepBndLib::Add
// crashes). Identifying a native body therefore CANNOT depend on any single engine
// instance's bookkeeping: cc_set_engine(1) builds a fresh NativeEngine each time, so
// a body built under one instance would look "unknown" (→ OCCT) to the next.
//
// PROCESS-WIDE IDENTITY. Every live NativeShape registers its own address in a
// static set on construction and removes it on destruction, so isNative() is a
// stable, instance-independent fact carried by the shape itself. This is the robust
// fix for the boolean-parity crash (build under engine 1, run boolean under a second
// engine-1 instance → the second instance no longer misclassifies the operands as
// OCCT bodies and never forwards a native void to OCCT).
class NativeShapeRegistry {
public:
    static void add(const void* p) {
        std::lock_guard<std::mutex> lock(mutex());
        live().insert(p);
    }
    static void remove(const void* p) {
        std::lock_guard<std::mutex> lock(mutex());
        live().erase(p);
    }
    static bool contains(const void* p) {
        if (p == nullptr) return false;
        std::lock_guard<std::mutex> lock(mutex());
        return live().find(p) != live().end();
    }

private:
    // Meyers singletons: guaranteed initialised on first use, no static-init-order
    // dependency (this TU may build shapes during another TU's static setup).
    static std::mutex& mutex() {
        static std::mutex m;
        return m;
    }
    static std::unordered_set<const void*>& live() {
        static std::unordered_set<const void*> s;
        return s;
    }
};

struct NativeShape {
    ntopo::Shape shape;       // B-rep bodies; left default/empty for mesh bodies
    ntess::Mesh mesh;         // populated for imported STL mesh bodies (issue #5)
    bool isMesh = false;      // true => serve the mesh directly (no B-rep)
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
    if (op != 2) return {};  // only COMMON has the analytic Steinmetz oracle in S5-a
    namespace nb = cybercad::native::boolean;
    const auto csA = nb::ssidetail::recogniseCurvedSolid(a);
    const auto csB = nb::ssidetail::recogniseCurvedSolid(b);
    if (!csA || !csB) return {};
    using CK = nb::ssidetail::CurvedKind;
    if (csA->kind != CK::Cylinder || csB->kind != CK::Cylinder) return {};
    // Equal radii and perpendicular, colinear-origin axes → the Steinmetz configuration.
    if (std::fabs(csA->radius - csB->radius) > 1e-6) return {};
    const double axisDot = std::fabs(cybercad::native::math::dot(csA->frame.z.vec(),
                                                                 csB->frame.z.vec()));
    if (axisDot > 1e-4) return {};  // not perpendicular → no closed form here
    const double r = csA->radius;
    const double expected = 16.0 * r * r * r / 3.0;  // Steinmetz bicylinder common volume
    const double vr = watertightVolume(result);
    if (vr < 0.0) return {true, false};  // not watertight
    const double tol = std::max(1e-2 * expected, 1e-6);  // deflection-bounded curved mesh
    return {true, std::fabs(vr - expected) <= tol};
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
    const double tol = std::max(1e-6 * expected, 1e-9);
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
// guard mirroring the blend guard with wantGrow=true, but the growth is ANALYTIC: an
// emboss (boss) RAISES a pad, so the result must be a valid watertight solid whose
// volume EXCEEDS the base by the wrapped footprint area × height. For a rectangular
// footprint on a cylinder the wrapped footprint area is the arc-length span × axial
// span, and because the profile's px is ALREADY arc-length that equals the flat profile
// area |pxMax-pxMin|·|pyMax-pyMin| (independent of R). The result carries TRUE curved
// (cylindrical) faces, so its watertight mesh volume only approximates the analytic
// target — deflection-bounded tolerance (1% relative + a small floor), like the curved
// boolean guards. A NULL or failing candidate is DISCARDED -> OCCT cc_wrap_emboss.
bool wrapEmbossVerified(const ntopo::Shape& result, const ntopo::Shape& original,
                        const double* profileXY, int count, double height) {
    const double vr = watertightVolume(result);
    if (vr <= 0.0) return false;  // not watertight or empty → reject
    const double vo = watertightVolume(original);
    if (vo <= 0.0) return true;   // original not measurable → trust watertight+positive
    if (profileXY == nullptr || count < 3 || !(height > 0.0)) return false;
    double xlo = profileXY[0], xhi = profileXY[0], ylo = profileXY[1], yhi = profileXY[1];
    for (int i = 1; i < count; ++i) {
        xlo = std::min(xlo, profileXY[i * 2]);
        xhi = std::max(xhi, profileXY[i * 2]);
        ylo = std::min(ylo, profileXY[i * 2 + 1]);
        yhi = std::max(yhi, profileXY[i * 2 + 1]);
    }
    const double footArea = (xhi - xlo) * (yhi - ylo);
    if (!(footArea > 0.0)) return false;
    const double expected = vo + footArea * height;
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

Result<MeshData> NativeEngine::extrude_mesh(const double* p, int n, double d) {
    return fallback().extrude_mesh(p, n, d);
}
// ── Tier-B (#4b) NATIVE 2-section ruled loft (fall through for the deferred
// cases: mismatched section counts, a non-planar section, a degenerate/point
// section, or 3+/guided/rail lofts — a NULL Shape → forward the SAME args). ──────
ShapeResult NativeEngine::solid_loft(const double* b, int bc, const double* t, int tc, double d) {
    ntopo::Shape solid = ncst::build_loft(b, bc, t, tc, d);
    if (solid.isNull()) return fallback().solid_loft(b, bc, t, tc, d);
    return track(wrapNative(std::move(solid)));
}
ShapeResult NativeEngine::solid_loft_wires(const double* a, int ac, const double* b, int bc) {
    ntopo::Shape solid = ncst::build_loft_wires(a, ac, b, bc);
    if (solid.isNull()) return fallback().solid_loft_wires(a, ac, b, bc);
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
// twisted_sweep: NATIVE only when it reduces to the plain sweep (twist ≈ 0, scale ≈
// 1); any real twist/scale (an extra per-section rotation the RMF sweep does not
// model) → NULL → OCCT twisted_sweep.
ShapeResult NativeEngine::twisted_sweep(const double* p, int pc, const double* path, int pathc,
                                        double tw, double se) {
    ntopo::Shape solid = ncst::build_twisted_sweep(p, pc, path, pathc, tw, se);
    if (solid.isNull()) return fallback().twisted_sweep(p, pc, path, pathc, tw, se);
    return track(wrapNative(std::move(solid)));
}
// ── Tier-2#4 (#4b) NATIVE general sweep: loft_along_rail + guided_sweep ──────────
// loft_along_rail is NATIVE for a STRAIGHT rail (a ruled loft between the two
// equal-count sections placed perpendicular to the rail tangent, matching
// MakePipeShell on a straight rail); a CURVED / kinked rail (genuine pipe-shell
// morph) or mismatched section counts → build_loft_along_rail returns NULL → OCCT.
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
// ── NATIVE wrap-emboss (Phase 4 #7 native-wrap-emboss) with mandatory self-verify ──
// First native slice: a RECTANGULAR pad embossed (boss=1) on a native body's CYLINDER
// lateral face. The OCCT-free builder (src/native/feature/wrap_emboss.h) wraps the
// footprint onto the cylinder and rebuilds the whole embossed solid as a deflection-
// bounded planar-facet soup welded watertight via the boolean assembleSolid (mirroring
// the curved-fillet slice — the pad-wall∩cylinder seam is expressed by shared footprint
// vertices, not a fragile boolean). The result is SELF-VERIFIED (watertight + volume
// GROWS by ≈ footprint area × height) and DISCARDED on failure. A native body the slice
// declines (deboss, non-rectangular / non-cylindrical, off-end footprint) canNOT be
// forwarded to OCCT (OCCT would misread the native void) → honest error. An OCCT body
// forwards to the Phase-3 OCCT cc_wrap_emboss oracle unconditionally.
ShapeResult NativeEngine::wrap_emboss(EngineShape body, int faceId, const double* p, int c, double d,
                                      int boss) {
    if (!isNative(body)) return fallback().wrap_emboss(body, faceId, p, c, d, boss);
    const auto* h = static_cast<const NativeShape*>(body.get());
    ntopo::Shape result =
        cybercad::native::feature::wrap_emboss(h->shape, faceId, p, c, d, boss);
    if (result.isNull() || !wrapEmbossVerified(result, h->shape, p, c, d))
        return make_error(
            "native wrap_emboss: no verified watertight result for this native body "
            "(deboss / non-rectangular profile / non-cylindrical face / off-end or "
            ">2π footprint → OCCT-only)");
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

    return make_error(
        "native fillet_edges: no verified watertight result for this native body "
        "(non-circular curved crease / blind-hole rim / Rc<2r convex / ≠cyl-plane rim / "
        "variable / cyl-cyl canal / interference → OCCT-only)");
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
    ntopo::Shape result = nblend::chamfer_edges(h->shape, e, ec, d);
    if (result.isNull() || !blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return make_error(
            "native chamfer_edges: no verified watertight result for this native body "
            "(curved face / concave edge / ≠2-face edge → OCCT-only)");
    return track(wrapNative(std::move(result)));
}
ShapeResult NativeEngine::shell(EngineShape body, const int* f, int fc, double t) {
    if (!isNative(body)) return fallback().shell(body, f, fc, t);
    const auto* h = static_cast<const NativeShape*>(body.get());
    ntopo::Shape result = nblend::shell(h->shape, f, fc, t);
    if (result.isNull() || !blendResultVerified(result, h->shape, /*wantGrow=*/false))
        return make_error(
            "native shell: no verified watertight wall for this native body "
            "(curved/non-convex solid or thickness too large → OCCT-only)");
    return track(wrapNative(std::move(result)));
}
ShapeResult NativeEngine::offset_face(EngineShape body, int f, double d) {
    if (!isNative(body)) return fallback().offset_face(body, f, d);
    const auto* h = static_cast<const NativeShape*>(body.get());
    ntopo::Shape result = nblend::offset_face(h->shape, f, d);
    // Grow (d>0) must increase volume; shrink (d<0) must decrease it.
    if (result.isNull() || !blendResultVerified(result, h->shape, /*wantGrow=*/d > 0.0))
        return make_error(
            "native offset_face: no verified watertight result for this native body "
            "(curved solid / non-planar face or degenerate offset → OCCT-only)");
    return track(wrapNative(std::move(result)));
}
ShapeResult NativeEngine::replace_face(EngineShape body, int f, double o, double t) {
    CC_NATIVE_BODY_UNSUPPORTED("replace_face", body);
    return fallback().replace_face(body, f, o, t);
}
ShapeResult NativeEngine::replace_face_to_plane(EngineShape body, int f, double px, double py,
                                                double pz, double nx, double ny, double nz) {
    CC_NATIVE_BODY_UNSUPPORTED("replace_face_to_plane", body);
    return fallback().replace_face_to_plane(body, f, px, py, pz, nx, ny, nz);
}
ShapeResult NativeEngine::fillet_face(EngineShape body, int f, double r) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_face", body);
    return fallback().fillet_face(body, f, r);
}
ShapeResult NativeEngine::split_plane(EngineShape body, double ox, double oy, double oz, double nx,
                                      double ny, double nz, int keep) {
    CC_NATIVE_BODY_UNSUPPORTED("split_plane", body);
    return fallback().split_plane(body, ox, oy, oz, nx, ny, nz, keep);
}
ShapeResult NativeEngine::full_round_fillet(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("full_round_fillet", body);
    return fallback().full_round_fillet(body, f);
}
ShapeResult NativeEngine::full_round_fillet_faces(EngineShape body, int l, int m, int r) {
    CC_NATIVE_BODY_UNSUPPORTED("full_round_fillet_faces", body);
    return fallback().full_round_fillet_faces(body, l, m, r);
}
ShapeResult NativeEngine::fillet_edges_g2(EngineShape body, const int* e, int ec, double r) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_edges_g2", body);
    return fallback().fillet_edges_g2(body, e, ec, r);
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
ShapeResult NativeEngine::thread_apply(EngineShape shaft, EngineShape thread, int op) {
    CC_NATIVE_BODY_UNSUPPORTED("thread_apply", shaft);
    CC_NATIVE_BODY_UNSUPPORTED("thread_apply", thread);
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
Result<std::vector<double>> NativeEngine::principal_moments(EngineShape body) {
    CC_NATIVE_BODY_UNSUPPORTED("principal_moments", body);
    return fallback().principal_moments(body);
}
Result<std::vector<double>> NativeEngine::face_axis(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("face_axis", body);
    return fallback().face_axis(body, f);
}
Result<std::vector<double>> NativeEngine::ref_plane_from_face(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_plane_from_face", body);
    return fallback().ref_plane_from_face(body, f);
}
Result<std::vector<double>> NativeEngine::ref_axis_from_edge(EngineShape body, int e) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_axis_from_edge", body);
    return fallback().ref_axis_from_edge(body, e);
}
Result<std::vector<double>> NativeEngine::ref_axis_from_face(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_axis_from_face", body);
    return fallback().ref_axis_from_face(body, f);
}
Result<std::vector<int>> NativeEngine::tangent_chain(EngineShape body, const int* e, int ec) {
    CC_NATIVE_BODY_UNSUPPORTED("tangent_chain", body);
    return fallback().tangent_chain(body, e, ec);
}
Result<std::vector<int>> NativeEngine::outer_rim_chain(EngineShape body, const int* e, int ec) {
    CC_NATIVE_BODY_UNSUPPORTED("outer_rim_chain", body);
    return fallback().outer_rim_chain(body, e, ec);
}
Result<std::vector<double>> NativeEngine::offset_face_boundary(EngineShape body, int f, double d) {
    CC_NATIVE_BODY_UNSUPPORTED("offset_face_boundary", body);
    return fallback().offset_face_boundary(body, f, d);
}

// ── transform fallthrough ─────────────────────────────────────────────────────────

ShapeResult NativeEngine::scale_shape(EngineShape body, double f) {
    CC_NATIVE_BODY_UNSUPPORTED("scale_shape", body);
    return fallback().scale_shape(body, f);
}
ShapeResult NativeEngine::scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                            double f) {
    CC_NATIVE_BODY_UNSUPPORTED("scale_shape_about", body);
    return fallback().scale_shape_about(body, cx, cy, cz, f);
}
ShapeResult NativeEngine::rotate_shape_about(EngineShape body, double cx, double cy, double cz,
                                             double ax, double ay, double az, double a) {
    CC_NATIVE_BODY_UNSUPPORTED("rotate_shape_about", body);
    return fallback().rotate_shape_about(body, cx, cy, cz, ax, ay, az, a);
}
ShapeResult NativeEngine::mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                                       double ny, double nz) {
    CC_NATIVE_BODY_UNSUPPORTED("mirror_shape", body);
    return fallback().mirror_shape(body, px, py, pz, nx, ny, nz);
}
ShapeResult NativeEngine::translate_shape(EngineShape body, double tx, double ty, double tz) {
    CC_NATIVE_BODY_UNSUPPORTED("translate_shape", body);
    return fallback().translate_shape(body, tx, ty, tz);
}
ShapeResult NativeEngine::place_on_frame(EngineShape body, double ox, double oy, double oz,
                                         double ux, double uy, double uz, double vx, double vy,
                                         double vz) {
    CC_NATIVE_BODY_UNSUPPORTED("place_on_frame", body);
    return fallback().place_on_frame(body, ox, oy, oz, ux, uy, uz, vx, vy, vz);
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

}  // namespace cyber
