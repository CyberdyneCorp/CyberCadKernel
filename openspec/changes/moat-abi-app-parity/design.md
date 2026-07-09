# Design — moat-abi-app-parity

## Context

The app defines the ABI shape (its `KernelBridgeAPI.h` is the contract the kernel must satisfy
to be link-compatible). This is the inverse of the usual "kernel owns the ABI" rule, and it is
intentional: the whole point of the drop-OCCT campaign is that the app links `libcybercadkernel.a`
with its existing call-sites UNCHANGED. So the six signatures are copied byte-for-byte from the
app header, not designed fresh.

## Signature reconciliation (app header → kernel header)

All six are added EXACTLY as the app declares them — no reconciliation was needed; each is a
reasonable, additive-safe C signature:

| Function | Signature (as added, == app) |
|---|---|
| `cc_loft_circles` | `CCShapeId cc_loft_circles(const double *c1, const double *n1, double r1, const double *c2, const double *n2, double r2)` |
| `cc_loft_circle_wire` | `CCShapeId cc_loft_circle_wire(const double *cc, const double *cn, double cr, const double *wXYZ, int wCount)` |
| `cc_loft_typed` | `CCShapeId cc_loft_typed(const CCProfileSeg *segsA, int countA, const double *splineA, int splineACount, const double *frameA, const CCProfileSeg *segsB, int countB, const double *splineB, int splineBCount, const double *frameB)` |
| `cc_loft_along_rails` | `CCShapeId cc_loft_along_rails(const double *railXYZ, int railCount, const double *guideXYZ, int guideCount, const double *profileA_XY, int aCount, const double *profileB_XY, int bCount)` |
| `cc_shape_solid_count` | `int cc_shape_solid_count(CCShapeId body)` |
| `cc_shape_solid_at` | `CCShapeId cc_shape_solid_at(CCShapeId body, int index)` |

`CCProfileSeg` already exists in `cc_kernel.h` with the identical layout to the app's, so
`cc_loft_typed` needs no new struct.

## Routing (the standard cc_* seam)

Each facade entry is `cyber::guard([&]{ … active_engine()->OP(...); finish_shape(r); }, 0)`,
identical to every existing loft/query op. Six new `IEngine` virtuals carry the calls; their
default is `engine_unsupported(...)` so the stub inherits an honest decline.

## Semantics matched to the app's reference (`KernelBridge.mm`)

- **`cc_loft_circles`** — two `gp_Circ` section wires (centre, unit normal, radius) →
  `ThruSections(solid, ruled)`. A TRUE circular B-rep (one side face, two circular edges), not
  a faceted polygon. Guards: null pointers or `r ≤ 1e-6` → 0.
- **`cc_loft_circle_wire`** — one `gp_Circ` wire + one `MakePolygon` wire (the polygon
  `wXYZ`) → `ThruSections`. Smooth circle rim, exact polygon side. Guards: nulls / `cr ≤ 1e-6`
  / `wCount < 3` → 0.
- **`cc_loft_typed`** — two typed section loops built by the existing `buildTypedWire`
  (line/arc/circle/spline, spline side-channel), each placed on its own plane frame
  (origin(3)+u(3)+v(3)) by a rigid `SetDisplacement`, → `ThruSections`. Curved boundaries stay
  true B-rep curve edges.
- **`cc_loft_along_rails`** — spine from `railXYZ`, sections framed perpendicular to the rail
  ends, `MakePipeShell` with the `guideXYZ` wire added as an auxiliary spine
  (`SetMode(guide, curvilinear)`); if the guided build fails it retries WITHOUT the guide
  (single-rail sweep), returning 0 only if both fail. This mirrors the app's two-pass loop.
- **`cc_shape_solid_count`** — `TopExp_Explorer(shape, TopAbs_SOLID)` count.
- **`cc_shape_solid_at`** — the `index`-th solid (0-based) from the same explorer, registered
  as its own shape. `index < 0` or out-of-range → 0.

## Native path

- **`shape_solid_count` / `shape_solid_at`:** NATIVE for a native body. The native
  `Explorer(root, ShapeType::Solid)` emits the root itself when the root IS a solid and dedups
  shared solids by `isSame` — identical to OCCT's `TopExp_Explorer(TopAbs_SOLID)` for disjoint
  lumps (the app's only use: split a boolean union into its connected components). `shape_solid_at`
  wraps the extracted `ntopo::Shape` via the existing `wrapNative`, giving an independent body.
  An OCCT body forwards to the fallback (which runs the OCCT explorer).
- **Four loft variants:** the native engine leaves these on the `IEngine` default (honest
  decline) → OCCT during the transition. A native ruled loft cannot reproduce the true circular
  / spline rim topology the app's contract promises, so faking it would be a WRONG result; an
  honest decline that falls to the OCCT oracle is the disciplined choice.

## Two-gate verification (OCCT is the oracle)

- **Gate A (host, OCCT-free):**
  - `cc_loft_circles` of coaxial circles → closed-form frustum volume `π h (r²+rR+R²)/3`
    (and the cylinder degenerate `π r² h` when r==R). On the OCCT-free host this exercises the
    OCCT-free arms: the loft wrappers themselves return 0 (no OCCT) — so the host analytic gate
    for the loft is the SIM gate; on host we assert the honest 0 + error. Solid enumeration is
    fully native on host: a 2-lump compound → count 2, each extracted solid → count 1, volumes
    sum to the whole.
  - Actually: the host gate for the loft closed-form is run under the OCCT engine when the host
    build links OCCT; when it does not, the loft host test asserts the honest decline and the
    frustum closed-form is verified on the SIM gate. Solid_count/at is asserted on host natively.
- **Gate B (sim, OCCT linked):** native-vs-OCCT parity for solid enumeration (count + per-solid
  volume) and OCCT-vs-closed-form for the loft wrappers (volume/area within the deflection
  bound), on a booted simulator, via a dedicated harness with its own `main()` + runner and a
  `run-sim-suite.sh` SKIP entry.

## Alternatives considered

- *Implement the loft variants natively now.* Rejected: the true-circle / spline rim is out of
  the native ruled-loft envelope; a faceted approximation violates the app's topology contract
  (one circular edge, not a polygon). Honest decline → OCCT is the campaign discipline.
- *Facade-only composition (build circle sample wires, call `cc_solid_loft_wires`).* Rejected
  for the same reason — it would face the circle and silently change topology.
