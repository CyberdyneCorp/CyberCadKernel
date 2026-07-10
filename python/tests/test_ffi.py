"""Low-level FFI contract: the library loads, the engine is real, the POD
struct layouts match the ABI header, and a raw ``cc_solid_extrude`` builds a box
whose exact volume the engine reports."""

import ctypes

from cybercadkernel import _cffi


def test_library_loads_and_brep_available():
    lib = _cffi.lib()
    assert lib.cc_brep_available() == 1, "expected the real OCCT desktop build"


def test_struct_layouts_match_abi():
    # Cross-checked against a C `sizeof` of cc_kernel.h on this platform
    # (LP64, arm64 macOS); guards a ctypes field declaration from drifting.
    assert ctypes.sizeof(_cffi.CCMesh) == 32
    assert ctypes.sizeof(_cffi.CCMassProps) == 48
    assert ctypes.sizeof(_cffi.CCProfileSeg) == 88
    assert ctypes.sizeof(_cffi.CCEdgePolyline) == 24
    assert ctypes.sizeof(_cffi.CCFaceMesh) == 40
    assert ctypes.sizeof(_cffi.CCShapeId) == 8


def test_new_struct_layouts_match_abi():
    # Sizes cross-checked against a C `sizeof` compiled from cc_kernel.h
    # (CC_KERNEL_NO_PROTOTYPES) on this platform (LP64, arm64 macOS). These guard
    # the POD structs used by the full-coverage additions (projection, clash,
    # validity, display/tet mesh, quality, PMI, drawing/HLR, section) from drifting.
    assert ctypes.sizeof(_cffi.CCProjection) == 40
    assert ctypes.sizeof(_cffi.CCInterference) == 112
    assert ctypes.sizeof(_cffi.CCValidityReport) == 32
    assert ctypes.sizeof(_cffi.CCDisplayMesh) == 48
    assert ctypes.sizeof(_cffi.CCTetMesh) == 40
    assert ctypes.sizeof(_cffi.CCVolumeMeshOptions) == 32
    assert ctypes.sizeof(_cffi.CCQualityReport) == 64
    assert ctypes.sizeof(_cffi.CCPmiSummary) == 32
    assert ctypes.sizeof(_cffi.CCDrawingSegment) == 32
    assert ctypes.sizeof(_cffi.CCDrawing) == 32
    assert ctypes.sizeof(_cffi.CCHlrOptions) == 24
    assert ctypes.sizeof(_cffi.CCSectionLoop) == 32
    assert ctypes.sizeof(_cffi.CCSection) == 32


def test_all_abi_symbols_are_bound():
    # 100% coverage guard: every cc_* symbol declared in the header must have a
    # signature in the _cffi table (kept in lockstep with _ffi).
    lib = _cffi.lib()
    for name in _cffi._SIGS:
        assert callable(getattr(lib, name)), name
    assert len(_cffi._SIGS) == 111


def test_raw_extrude_box_volume():
    lib = _cffi.lib()
    prof = (ctypes.c_double * 8)(0, 0, 10, 0, 10, 10, 0, 10)
    sid = lib.cc_solid_extrude(prof, 4, 10.0)
    assert sid != 0
    try:
        mp = lib.cc_mass_properties(sid)
        assert mp.valid == 1
        assert abs(mp.volume - 1000.0) < 1e-6
        assert abs(mp.area - 600.0) < 1e-6
    finally:
        lib.cc_shape_release(sid)
