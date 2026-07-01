# CyberCadKernel — Project Context

## Purpose
CyberCadKernel is a portable, modern **C++20 geometry kernel** for precision CAD,
built to power CyberCad (iPadOS-first) and future desktop/Android targets. It is
introduced behind CyberCad's existing plain-C facade (`cc_*`) and **wraps
OpenCASCADE (OCCT)** as the initial exact B-rep engine, then migrates capability
by capability toward a **fully native C++20 implementation** that eventually
removes the OCCT dependency.

## Strategic decisions (locked)
- **Stable C ABI seam.** The public boundary is a plain-C facade (`cc_*`, integer
  shape handles, POD structs) — identical to CyberCad's current
  `KernelBridgeAPI.h`. No C++ or engine type crosses it. The app never changes
  as the engine behind it evolves.
- **Wrap → accelerate → rewrite.** OCCT is the initial engine (proven, already
  cross-compiled for arm64 iOS). Capabilities migrate to native C++20 on a
  roadmap; the **end state is a full C++20 kernel with no OCCT**. See
  [ROADMAP.md](ROADMAP.md).
- **License trajectory.** CyberCadKernel is **MIT**. Wrapping OCCT (LGPL-2.1 +
  exception) imposes a static-relink obligation on shipped apps; completing the
  native rewrite makes the kernel cleanly MIT with no LGPL obligation. This is a
  primary motivation for the rewrite endgame, not only modernization.
- **Compute-backend abstraction from day one.** Desktop and Android are planned,
  so GPU compute is behind an interface: **Metal** first (the only GPU path on
  iOS), with **CUDA / OpenCL / Vulkan** backends addable for non-Apple targets.
- **CPU is the source of truth; GPU is throughput.** Exact modeling is
  double-precision on CPU. GPU handles only fp32-tolerant, data-parallel work
  (tessellation, BVH, picking, mesh post-processing).

## Tech stack
- **Language**: C++20 (library core). Public ABI is plain C.
- **Initial engine**: OCCT (C++17 static libs), wrapped behind an engine adapter.
- **Async**: C++20 coroutines + `std::stop_token` for cancellable,
  progress-reporting operations.
- **GPU compute**: Metal (mobile) via a backend interface; CUDA/OpenCL/Vulkan
  later for desktop/Android.
- **Build**: CMake; cross-compiled for arm64 iOS device + simulator today.
- **Precision / units**: internal millimetres, double precision, on CPU.

## Architecture (initial)
```
host app ── cc_* C facade ──► CyberCadKernel (C++20)
                              ├─ shape registry + error/guard model
                              ├─ operation scheduler (coroutines, cancel, progress)
                              ├─ engine adapter  ──► OCCT (exact B-rep, fp64, CPU)
                              │                       [replaced natively over time]
                              └─ compute backend ──► Metal | CUDA | OpenCL | Vulkan
                                                     (fp32: tessellation, BVH, pick)
```

## Conventions
- **ABI stability is sacred.** Additive changes to `cc_*` only; never break a
  signature the app depends on. The contract is mirrored from CyberCad's
  `openspec/specs/occt-usage/spec.md`.
- **Every engine capability is pluggable** so OCCT-backed and native
  implementations can coexist and be compared behind the same facade call.
- **No engine type in public headers.** OCCT (and any future engine) is an
  internal implementation detail.
- **Determinism by default**; parallelism must preserve reproducible results.
- Cognitive complexity: services/orchestration ≤ 15; irreducible geometry
  algorithms may reach the systems band (≤ 25–35), flagged and documented.

## Related specs
- OCCT baseline & acceleration analysis:
  `/Users/leonardoaraujo/work/OCCT/openspec/` (`acceleration.md`, capability specs).
- CyberCad dependency contract & pain points:
  `/Users/leonardoaraujo/work/cybercad/openspec/specs/occt-usage/spec.md`.
- Feasibility study that motivated this repo:
  `/Users/leonardoaraujo/work/cybercad/openspec/research-modern-kernel.md`.
