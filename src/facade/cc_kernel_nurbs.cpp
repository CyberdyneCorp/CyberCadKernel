// Public C facade — additive NURBS geometry surface (cc_curve / cc_surface).
//
// This is the BRIDGE layer for the Wave D–I exact-NURBS geometry. Like the rest of
// src/facade it is allowed to call src/native directly (src/native itself stays
// OCCT-free; the bridging lives here). Each entry point is a thin, guarded
// delegation: it validates its raw C input, drives the OCCT-free native NURBS
// representation (cybercad::native::math::Bspline{Curve,Surface}Data + the
// bspline.h evaluators), and NEVER leaks a C++ type across the boundary. Honest
// declines collapse to a 0 (invalid) handle + cc_last_error — never a
// plausible-but-wrong handle (design.md §3).
//
// The NURBS curves/surfaces live in a process-wide registry independent of the
// CCShapeId ShapeRegistry (curves/surfaces are not engine shapes), but it mirrors
// that registry's model exactly: monotonic non-zero ids, id 0 = invalid, a mutex
// for thread-safety, idempotent crash-free release of unknown/stale handles.

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"
#include "native/math/bspline.h"
#include "native/math/bspline_ops.h"

namespace {

using cyber::guard;
using cyber::set_last_error;
using cybercad::native::math::BsplineCurveData;
using cybercad::native::math::BsplineSurfaceData;
using cybercad::native::math::nurbsCurvePoint;
using cybercad::native::math::nurbsSurfacePoint;
using cybercad::native::math::Point3;
using cybercad::native::math::SurfaceGrid;

// ── registries (process-wide, mirror ShapeRegistry's model) ───────────────────
//
// Intentionally leaked (like the shape registry): a static-destructor free would
// race process teardown for no benefit; the OS reclaims memory at exit.

template <class Payload>
class NurbsRegistry {
public:
    int32_t add(Payload payload) {
        std::lock_guard<std::mutex> lk(mutex_);
        const int32_t id = next_id_++;
        items_.emplace(id, std::move(payload));
        return id;
    }

    // Returns nullptr for id 0 or an unknown / released id.
    const Payload* get(int32_t id) const {
        if (id == 0) {
            return nullptr;
        }
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = items_.find(id);
        return it == items_.end() ? nullptr : &it->second;
    }

    // Idempotent: releasing id 0 or an unknown id is a no-op (never throws).
    void release(int32_t id) {
        if (id == 0) {
            return;
        }
        std::lock_guard<std::mutex> lk(mutex_);
        items_.erase(id);
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<int32_t, Payload> items_;
    int32_t next_id_ = 1;  // 0 is the invalid sentinel.
};

NurbsRegistry<BsplineCurveData>& curveRegistry() {
    static auto* reg = new NurbsRegistry<BsplineCurveData>();
    return *reg;
}

NurbsRegistry<BsplineSurfaceData>& surfaceRegistry() {
    static auto* reg = new NurbsRegistry<BsplineSurfaceData>();
    return *reg;
}

// ── validation / build helpers ────────────────────────────────────────────────

// True when the knot vector is non-decreasing and of the exact expected length.
bool validKnots(const double* knots, int n_knots, int n_ctrl, int degree) {
    if (knots == nullptr || n_knots != n_ctrl + degree + 1) {
        return false;
    }
    for (int i = 1; i < n_knots; ++i) {
        if (knots[i] < knots[i - 1]) {
            return false;
        }
    }
    return true;
}

// Split an interleaved (x,y,z,w) homogeneous pole stream into poles + weights, with
// every weight strictly positive. Returns false on a non-positive or non-finite w.
bool splitHomogeneous(const double* polesXYZW, int count, std::vector<Point3>& poles,
                      std::vector<double>& weights) {
    poles.resize(static_cast<std::size_t>(count));
    weights.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double w = polesXYZW[4 * i + 3];
        if (!(w > 0.0)) {  // rejects 0, negative, NaN
            return false;
        }
        poles[static_cast<std::size_t>(i)] =
            Point3{polesXYZW[4 * i + 0], polesXYZW[4 * i + 1], polesXYZW[4 * i + 2]};
        weights[static_cast<std::size_t>(i)] = w;
    }
    return true;
}

// A curve/surface is stored non-rational (empty weights) iff every weight is 1.
bool allUnitWeights(const std::vector<double>& weights) {
    for (double w : weights) {
        if (w != 1.0) {
            return false;
        }
    }
    return true;
}

// The domain of a clamped/unclamped flat knot vector for eval clamping.
double knotLo(const std::vector<double>& k, int degree) {
    return k[static_cast<std::size_t>(degree)];
}
double knotHi(const std::vector<double>& k, int degree, int nCtrl) {
    return k[static_cast<std::size_t>(nCtrl)];  // U[n+1] with n = nCtrl-1
}
double clampDomain(double t, double lo, double hi) {
    return t < lo ? lo : (t > hi ? hi : t);
}

// Evaluate a stored curve at t (uniform rational path — all-1 weights reduce to the
// non-rational point exactly).
Point3 evalCurve(const BsplineCurveData& c, double t) {
    const int nCtrl = static_cast<int>(c.poles.size());
    const double lo = knotLo(c.knots, c.degree);
    const double hi = knotHi(c.knots, c.degree, nCtrl);
    const std::vector<double> ones(c.poles.size(), 1.0);
    const std::vector<double>& w = c.weights.empty() ? ones : c.weights;
    return nurbsCurvePoint(c.degree, c.poles, w, c.knots, clampDomain(t, lo, hi));
}

// Evaluate a stored surface at (u,v).
Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
    SurfaceGrid grid{s.poles, s.nPolesU, s.nPolesV};
    const double uLo = knotLo(s.knotsU, s.degreeU);
    const double uHi = knotHi(s.knotsU, s.degreeU, s.nPolesU);
    const double vLo = knotLo(s.knotsV, s.degreeV);
    const double vHi = knotHi(s.knotsV, s.degreeV, s.nPolesV);
    const std::vector<double> ones(s.poles.size(), 1.0);
    const std::vector<double>& w = s.weights.empty() ? ones : s.weights;
    return nurbsSurfacePoint(s.degreeU, s.degreeV, grid, w, s.knotsU, s.knotsV,
                             clampDomain(u, uLo, uHi), clampDomain(v, vLo, vHi));
}

// Copy a std::vector<double> into a caller buffer of capacity `cap`. Returns the
// count written, or < 0 (writing NOTHING) when the buffer is too small.
int fillBuffer(const std::vector<double>& src, double* out, int cap) {
    const int n = static_cast<int>(src.size());
    if (out == nullptr || cap < n) {
        return -1;
    }
    std::memcpy(out, src.data(), static_cast<std::size_t>(n) * sizeof(double));
    return n;
}

// Homogeneous (x,y,z,w) flatten of a pole/weight net (weights empty ⇒ all 1).
std::vector<double> homogeneousPoles(const std::vector<Point3>& poles,
                                     const std::vector<double>& weights) {
    std::vector<double> out(poles.size() * 4);
    for (std::size_t i = 0; i < poles.size(); ++i) {
        const double w = weights.empty() ? 1.0 : weights[i];
        out[4 * i + 0] = poles[i].x;
        out[4 * i + 1] = poles[i].y;
        out[4 * i + 2] = poles[i].z;
        out[4 * i + 3] = w;
    }
    return out;
}

int clampSamples(int n, int fallback) {
    if (n <= 0) {
        return fallback;
    }
    return n < 2 ? 2 : n;
}

}  // namespace

// ── construction ──────────────────────────────────────────────────────────────

cc_curve cc_curve_create(int degree, const double* polesXYZW, int n_ctrl,
                         const double* knots, int n_knots) {
    return guard(
        [&]() -> cc_curve {
            if (degree < 1 || polesXYZW == nullptr || n_ctrl <= degree) {
                set_last_error("cc_curve_create: invalid degree / control-point count");
                return cc_curve{0};
            }
            if (!validKnots(knots, n_knots, n_ctrl, degree)) {
                set_last_error(
                    "cc_curve_create: knot vector must be non-decreasing of length "
                    "n_ctrl + degree + 1");
                return cc_curve{0};
            }
            BsplineCurveData c;
            c.degree = degree;
            if (!splitHomogeneous(polesXYZW, n_ctrl, c.poles, c.weights)) {
                set_last_error("cc_curve_create: every homogeneous weight must be > 0");
                return cc_curve{0};
            }
            if (allUnitWeights(c.weights)) {
                c.weights.clear();  // store non-rational
            }
            c.knots.assign(knots, knots + n_knots);
            return cc_curve{curveRegistry().add(std::move(c))};
        },
        cc_curve{0});
}

cc_surface cc_surface_create(int degreeU, int degreeV, const double* polesXYZW,
                             int n_ctrl_u, int n_ctrl_v, const double* knotsU,
                             int n_knots_u, const double* knotsV, int n_knots_v) {
    return guard(
        [&]() -> cc_surface {
            if (degreeU < 1 || degreeV < 1 || polesXYZW == nullptr ||
                n_ctrl_u <= degreeU || n_ctrl_v <= degreeV) {
                set_last_error(
                    "cc_surface_create: invalid degree / control-point count");
                return cc_surface{0};
            }
            if (!validKnots(knotsU, n_knots_u, n_ctrl_u, degreeU) ||
                !validKnots(knotsV, n_knots_v, n_ctrl_v, degreeV)) {
                set_last_error(
                    "cc_surface_create: each knot vector must be non-decreasing of "
                    "length n_ctrl + degree + 1");
                return cc_surface{0};
            }
            BsplineSurfaceData s;
            s.degreeU = degreeU;
            s.degreeV = degreeV;
            s.nPolesU = n_ctrl_u;
            s.nPolesV = n_ctrl_v;
            const int total = n_ctrl_u * n_ctrl_v;
            if (!splitHomogeneous(polesXYZW, total, s.poles, s.weights)) {
                set_last_error(
                    "cc_surface_create: every homogeneous weight must be > 0");
                return cc_surface{0};
            }
            if (allUnitWeights(s.weights)) {
                s.weights.clear();  // store non-rational
            }
            s.knotsU.assign(knotsU, knotsU + n_knots_u);
            s.knotsV.assign(knotsV, knotsV + n_knots_v);
            return cc_surface{surfaceRegistry().add(std::move(s))};
        },
        cc_surface{0});
}

void cc_curve_release(cc_curve h) {
    curveRegistry().release(h.id);  // idempotent, never throws
}

void cc_surface_release(cc_surface h) {
    surfaceRegistry().release(h.id);
}

// ── accessors ─────────────────────────────────────────────────────────────────

int cc_curve_info(cc_curve h, CCCurveInfo* out) {
    return guard(
        [&]() -> int {
            const BsplineCurveData* c = curveRegistry().get(h.id);
            if (c == nullptr || out == nullptr) {
                set_last_error("cc_curve_info: unknown handle or null out");
                return 0;
            }
            out->degree = c->degree;
            out->n_ctrl = static_cast<int32_t>(c->poles.size());
            out->n_knots = static_cast<int32_t>(c->knots.size());
            out->rational = c->weights.empty() ? 0 : 1;
            return 1;
        },
        0);
}

int cc_surface_info(cc_surface h, CCSurfaceInfo* out) {
    return guard(
        [&]() -> int {
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr || out == nullptr) {
                set_last_error("cc_surface_info: unknown handle or null out");
                return 0;
            }
            out->degree_u = s->degreeU;
            out->degree_v = s->degreeV;
            out->n_ctrl_u = s->nPolesU;
            out->n_ctrl_v = s->nPolesV;
            out->n_knots_u = static_cast<int32_t>(s->knotsU.size());
            out->n_knots_v = static_cast<int32_t>(s->knotsV.size());
            out->rational = s->weights.empty() ? 0 : 1;
            return 1;
        },
        0);
}

int cc_curve_knots(cc_curve h, double* out, int cap) {
    return guard(
        [&]() -> int {
            const BsplineCurveData* c = curveRegistry().get(h.id);
            if (c == nullptr) {
                set_last_error("cc_curve_knots: unknown handle");
                return -1;
            }
            return fillBuffer(c->knots, out, cap);
        },
        -1);
}

int cc_curve_poles(cc_curve h, double* out, int cap) {
    return guard(
        [&]() -> int {
            const BsplineCurveData* c = curveRegistry().get(h.id);
            if (c == nullptr) {
                set_last_error("cc_curve_poles: unknown handle");
                return -1;
            }
            return fillBuffer(homogeneousPoles(c->poles, c->weights), out, cap);
        },
        -1);
}

int cc_surface_knots_u(cc_surface h, double* out, int cap) {
    return guard(
        [&]() -> int {
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr) {
                set_last_error("cc_surface_knots_u: unknown handle");
                return -1;
            }
            return fillBuffer(s->knotsU, out, cap);
        },
        -1);
}

int cc_surface_knots_v(cc_surface h, double* out, int cap) {
    return guard(
        [&]() -> int {
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr) {
                set_last_error("cc_surface_knots_v: unknown handle");
                return -1;
            }
            return fillBuffer(s->knotsV, out, cap);
        },
        -1);
}

int cc_surface_poles(cc_surface h, double* out, int cap) {
    return guard(
        [&]() -> int {
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr) {
                set_last_error("cc_surface_poles: unknown handle");
                return -1;
            }
            return fillBuffer(homogeneousPoles(s->poles, s->weights), out, cap);
        },
        -1);
}

// ── evaluators ────────────────────────────────────────────────────────────────

int cc_curve_eval(cc_curve h, double t, double* xyz) {
    return guard(
        [&]() -> int {
            const BsplineCurveData* c = curveRegistry().get(h.id);
            if (c == nullptr || xyz == nullptr) {
                set_last_error("cc_curve_eval: unknown handle or null out");
                return 0;
            }
            const Point3 p = evalCurve(*c, t);
            xyz[0] = p.x;
            xyz[1] = p.y;
            xyz[2] = p.z;
            return 1;
        },
        0);
}

int cc_surface_eval(cc_surface h, double u, double v, double* xyz) {
    return guard(
        [&]() -> int {
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr || xyz == nullptr) {
                set_last_error("cc_surface_eval: unknown handle or null out");
                return 0;
            }
            const Point3 p = evalSurface(*s, u, v);
            xyz[0] = p.x;
            xyz[1] = p.y;
            xyz[2] = p.z;
            return 1;
        },
        0);
}

// ── display tessellation bridge (single-surface DISPLAY mesh only) ────────────

int cc_surface_tessellate(cc_surface h, const CCTessOptions* opt, CCMesh* out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) {
                *out = CCMesh{nullptr, 0, nullptr, 0};
            }
            const BsplineSurfaceData* s = surfaceRegistry().get(h.id);
            if (s == nullptr || out == nullptr) {
                set_last_error("cc_surface_tessellate: unknown handle or null out");
                return 0;
            }
            constexpr int kDefault = 16;
            const int nu = clampSamples(opt ? opt->n_u : 0, kDefault);
            const int nv = clampSamples(opt ? opt->n_v : 0, kDefault);

            const double uLo = knotLo(s->knotsU, s->degreeU);
            const double uHi = knotHi(s->knotsU, s->degreeU, s->nPolesU);
            const double vLo = knotLo(s->knotsV, s->degreeV);
            const double vHi = knotHi(s->knotsV, s->degreeV, s->nPolesV);

            const int nVerts = nu * nv;
            const int nTris = (nu - 1) * (nv - 1) * 2;
            auto* verts = static_cast<double*>(
                std::malloc(static_cast<std::size_t>(nVerts) * 3 * sizeof(double)));
            auto* tris = static_cast<int*>(
                std::malloc(static_cast<std::size_t>(nTris) * 3 * sizeof(int)));
            if (verts == nullptr || tris == nullptr) {
                std::free(verts);
                std::free(tris);
                set_last_error("cc_surface_tessellate: out of memory");
                return 0;
            }

            for (int i = 0; i < nu; ++i) {
                const double u =
                    (nu == 1) ? uLo : uLo + (uHi - uLo) * (double(i) / double(nu - 1));
                for (int j = 0; j < nv; ++j) {
                    const double v = (nv == 1)
                                         ? vLo
                                         : vLo + (vHi - vLo) * (double(j) / double(nv - 1));
                    const Point3 p = evalSurface(*s, u, v);
                    const int idx = (i * nv + j) * 3;
                    verts[idx + 0] = p.x;
                    verts[idx + 1] = p.y;
                    verts[idx + 2] = p.z;
                }
            }

            int t = 0;
            for (int i = 0; i < nu - 1; ++i) {
                for (int j = 0; j < nv - 1; ++j) {
                    const int a = i * nv + j;
                    const int b = i * nv + (j + 1);
                    const int c = (i + 1) * nv + j;
                    const int d = (i + 1) * nv + (j + 1);
                    tris[t++] = a;
                    tris[t++] = c;
                    tris[t++] = b;
                    tris[t++] = b;
                    tris[t++] = c;
                    tris[t++] = d;
                }
            }

            out->vertices = verts;
            out->vertexCount = nVerts;
            out->triangles = tris;
            out->triangleCount = nTris;
            return 1;
        },
        0);
}

int cc_curve_polyline(cc_curve h, int n_samples, CCEdgePolyline* out) {
    return guard(
        [&]() -> int {
            if (out != nullptr) {
                out->edgeId = 0;
                out->points = nullptr;
                out->pointCount = 0;
            }
            const BsplineCurveData* c = curveRegistry().get(h.id);
            if (c == nullptr || out == nullptr) {
                set_last_error("cc_curve_polyline: unknown handle or null out");
                return 0;
            }
            const int n = clampSamples(n_samples, 16);
            const int nCtrl = static_cast<int>(c->poles.size());
            const double lo = knotLo(c->knots, c->degree);
            const double hi = knotHi(c->knots, c->degree, nCtrl);
            auto* pts = static_cast<double*>(
                std::malloc(static_cast<std::size_t>(n) * 3 * sizeof(double)));
            if (pts == nullptr) {
                set_last_error("cc_curve_polyline: out of memory");
                return 0;
            }
            for (int i = 0; i < n; ++i) {
                const double t =
                    (n == 1) ? lo : lo + (hi - lo) * (double(i) / double(n - 1));
                const Point3 p = evalCurve(*c, t);
                pts[3 * i + 0] = p.x;
                pts[3 * i + 1] = p.y;
                pts[3 * i + 2] = p.z;
            }
            out->edgeId = 0;
            out->points = pts;
            out->pointCount = n;
            return 1;
        },
        0);
}
