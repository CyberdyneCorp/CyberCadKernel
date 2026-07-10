"""Build every gallery piece, collect artifacts + captions, regenerate README.

Imports each ``NN_<name>.py`` example module, runs its ``build(kernel)`` through
the shared :func:`_gallery.emit`, and then renders ``examples/README.md`` as a
gallery: one row per piece (PNG thumbnail, description, features, volume/bbox).

Deterministic: the output contains no timestamps, so re-running produces a
byte-identical README when the geometry is unchanged.

Usage::

    CYBERCADKERNEL_DYLIB=.../build-mac/libcybercadkernel.dylib \\
        python3 examples/run_all.py
"""

from __future__ import annotations

import importlib.util
import os
import sys

import _gallery
from _gallery import PieceResult, emit

_HERE = os.path.dirname(os.path.abspath(__file__))

# The pieces, in gallery order. Each entry names the module file (sans .py).
PIECE_MODULES = [
    "01_pipe_flange",
    "02_l_bracket",
    "03_bearing_block",
    "04_v_pulley",
    "05_enclosure",
    "06_spur_gear_simplified",
    "07_manifold_block",
]

# Pieces deliberately left out of this gallery, each with the reason. Two kinds:
#
#  * "reserved" — the feature IS bound and working today (loft / sweep / thread),
#    but these pieces are owned by the concurrent binding-completion track, so
#    they are left for that track to add rather than duplicated here.
#  * "unbound"  — the feature is genuinely not exposed through the Python facade
#    yet (no method in api.py, no cc_* symbol): sheet-metal, draft, section/HLR.
COMING_LATER = [
    (
        "Round-to-square transition (loft adapter)",
        "reserved",
        "`Kernel.loft` / `Kernel.loft_wires` (bound and working) — owned by the "
        "binding-completion track.",
    ),
    (
        "Swept coolant tube",
        "reserved",
        "`Kernel.sweep` / `Kernel.twisted_sweep` (bound and working) — owned by "
        "the binding-completion track.",
    ),
    (
        "Threaded hex bolt",
        "reserved",
        "`Kernel.helical_thread` + `Shape.thread_apply` (bound and working) — "
        "owned by the binding-completion track.",
    ),
    (
        "Sheet-metal bracket (bends + flanges)",
        "unbound",
        "sheet-metal ops (flange / bend / unfold) — no method in `api.py` yet.",
    ),
    (
        "Draft-moulded housing",
        "unbound",
        "a draft-face op (pull-direction draft angle on side walls) — not exposed "
        "in `api.py` yet.",
    ),
    (
        "2-D section / HLR drawing view",
        "unbound",
        "`cc_hlr_project` + a section op — no hidden-line / planar-section method "
        "in `api.py` yet.",
    ),
]


def _load_module(name: str):
    """Import an example module by file path (names start with a digit)."""
    path = os.path.join(_HERE, f"{name}.py")
    spec = importlib.util.spec_from_file_location(f"example_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _build_all() -> list[PieceResult]:
    from cybercadkernel import Kernel

    kernel = Kernel()
    results: list[PieceResult] = []
    for name in PIECE_MODULES:
        module = _load_module(name)
        shape = module.build(kernel)
        result = emit(
            shape,
            module.NAME,
            _module_title(module),
            _module_desc(module),
            _module_features(module),
        )
        results.append(result)
        _print_caption(result)
    return results


def _module_title(module) -> str:
    return getattr(module, "TITLE", module.NAME)


def _module_desc(module) -> str:
    return getattr(module, "DESCRIPTION", "")


def _module_features(module) -> list[str]:
    return list(getattr(module, "FEATURES", []))


def _print_caption(result: PieceResult) -> None:
    sz = result.bbox_size
    print(f"[{result.name}] {result.title}")
    print(
        f"    volume = {result.volume_mm3:.3f} mm^3   "
        f"bbox = {sz[0]:.1f} x {sz[1]:.1f} x {sz[2]:.1f} mm   "
        f"render = {result.render_backend or 'none'}"
    )


def _fmt_num(x: float) -> str:
    return f"{x:,.1f}"


def _fmt_vec(v) -> str:
    return " × ".join(f"{c:.1f}" for c in v)


def _render_readme(results: list[PieceResult]) -> str:
    lines: list[str] = []
    lines.append("# CyberCadKernel — mechanical example gallery")
    lines.append("")
    lines.append(
        "Parametric mechanical CAD pieces built through the CyberCadKernel Python "
        "binding (`python/cybercadkernel`), on the real OCCT-backed engine. Every "
        "piece is a genuine B-rep solid: the volumes and bounding boxes below come "
        "from the kernel's exact mass-property query, not the mesh."
    )
    lines.append("")
    lines.append("## How to build")
    lines.append("")
    lines.append("```sh")
    lines.append("# 1. Build the real-engine dylib (Homebrew OCCT at /opt/homebrew/opt/opencascade)")
    lines.append("CLEAN=1 bash scripts/build-macos-dylib.sh")
    lines.append("")
    lines.append("# 2. Point the binding at it and regenerate the whole gallery")
    lines.append("export CYBERCADKERNEL_DYLIB=\"$PWD/build-mac/libcybercadkernel.dylib\"")
    lines.append("python3 examples/run_all.py")
    lines.append("```")
    lines.append("")
    lines.append(
        "Each script is self-contained (`python3 examples/01_pipe_flange.py`) with "
        "its parametric constants at the top and a `build(kernel)` returning the "
        "`Shape`. Artifacts land in `examples/out/<name>/`: a STEP model, a binary "
        "STL, a glTF (`.glb`) and a PNG thumbnail."
    )
    lines.append("")
    lines.append("## Pieces")
    lines.append("")

    for r in results:
        lines.append(f"### {r.title}")
        lines.append("")
        if r.png_rel:
            lines.append(f'<img src="{r.png_rel}" width="360" alt="{r.title}">')
            lines.append("")
        lines.append(r.description)
        lines.append("")
        lines.append(f"- **Features:** {', '.join(r.features)}")
        lines.append(
            f"- **Volume:** {_fmt_num(r.volume_mm3)} mm³ · "
            f"**Surface area:** {_fmt_num(r.area_mm2)} mm²"
        )
        lines.append(f"- **Bounding box:** {_fmt_vec(r.bbox_size)} mm")
        arts = []
        for kind in ("step", "stl", "glb", "png"):
            if kind in r.artifacts:
                arts.append(f"[{kind.upper()}]({r.artifacts[kind]})")
        if arts:
            lines.append(f"- **Artifacts:** {' · '.join(arts)}")
        lines.append(f"- **Script:** [`{r.name}.py`]({r.name}.py)")
        lines.append("")

    lines.append("## Rendering")
    lines.append("")
    backends = sorted({r.render_backend for r in results if r.render_backend})
    backend_str = ", ".join(backends) if backends else "none available"
    lines.append(
        "Thumbnails are rendered offscreen by `cybercadkernel.viz.render_png`, "
        "which tries the trimesh OpenGL path first and falls back to a headless "
        f"matplotlib rasterizer. In this run the renders resolved via: "
        f"**{backend_str}**."
    )
    lines.append("")
    lines.append("## Coming when the binding completes")
    lines.append("")
    lines.append(
        "These parts are intentionally *not* built here (and not faked). Two "
        "reasons appear below:"
    )
    lines.append("")
    reserved = [c for c in COMING_LATER if c[1] == "reserved"]
    unbound = [c for c in COMING_LATER if c[1] == "unbound"]
    lines.append(
        "**Reserved for the binding-completion track** — the feature is already "
        "bound and working in the current dylib, but the piece belongs to that "
        "concurrent track and is left for it to add rather than duplicated here:"
    )
    lines.append("")
    for title, _kind, need in reserved:
        lines.append(f"- **{title}** — {need}")
    lines.append("")
    lines.append(
        "**Not yet exposed through the Python facade** — no method in `api.py` "
        "yet, so genuinely unbuildable through the binding today:"
    )
    lines.append("")
    for title, _kind, need in unbound:
        lines.append(f"- **{title}** — {need}")
    lines.append("")

    return "\n".join(lines) + "\n"


def main() -> int:
    results = _build_all()
    readme = _render_readme(results)
    readme_path = os.path.join(_HERE, "README.md")
    with open(readme_path, "w") as fh:
        fh.write(readme)
    print()
    print(f"Wrote {readme_path} ({len(results)} pieces).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
