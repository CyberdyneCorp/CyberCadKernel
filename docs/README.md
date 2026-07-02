# CyberCadKernel documentation

Start at the project [README](../README.md). This folder holds the detail.

| Doc | What's in it |
|---|---|
| [ROADMAP.md](ROADMAP.md) | Phase plan (wrap → accelerate → rewrite) and where each phase stands. |
| [FEATURES.md](FEATURES.md) | Capability catalogue — the 72 `cc_*` functions, grouped, with per-feature notes. |
| [STATUS.md](STATUS.md) | Consolidated, verified status + one-command reproduce for every suite. |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Layers, boundaries, seams, and design decisions. |
| [build.md](build.md) | Toolchain and build/link instructions (host + iOS). |
| [python.md](python.md) | Desktop Python binding (`cybercadkernel`) — build, usage, viz, and pytest. |
| STATUS-phase-0-1.md · [-2](STATUS-phase-2.md) · [-3](STATUS-phase-3.md) · [-4](STATUS-phase-4.md) | Deep per-phase verification detail. |

**Specs live in [`../openspec/`](../openspec/):** the canonical change-level
[roadmap](../openspec/ROADMAP.md), the per-capability specs, and the change
proposals that drove each phase (`openspec list`, `openspec validate --all`).
