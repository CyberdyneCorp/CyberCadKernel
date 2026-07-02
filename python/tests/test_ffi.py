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
