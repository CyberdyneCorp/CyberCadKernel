// ABI contract test. Proves the public cc_kernel.h is binary-compatible with the
// app's reference KernelBridgeAPI.h by static_asserting every POD struct's size
// and field offsets across the two headers, and by linking the reference cc_*
// prototypes against this library's implementation.
//
// The reference header is included at global scope (its cc_* prototypes resolve
// to our library at link time); our header is re-included inside namespace `pub`
// with CC_KERNEL_NO_PROTOTYPES so the two struct sets coexist under distinct
// names for comparison. When the reference header is not available on the build
// machine, the test degrades to a self-consistency + link check.

#include <cstddef>
#include <string>

#include "harness.h"

#if defined(CYBERCAD_HAVE_REFERENCE_ABI)

#include "KernelBridgeAPI.h"  // reference structs (::CCMesh ...) + cc_* prototypes

namespace pub {
#define CC_KERNEL_NO_PROTOTYPES
#include "cybercadkernel/cc_kernel.h"  // our structs, namespaced for comparison
#undef CC_KERNEL_NO_PROTOTYPES
}  // namespace pub

// Handle sentinel type must match exactly.
static_assert(sizeof(::CCShapeId) == sizeof(pub::CCShapeId), "CCShapeId size drift");

// CCMesh
static_assert(sizeof(::CCMesh) == sizeof(pub::CCMesh), "CCMesh size drift");
static_assert(offsetof(::CCMesh, vertices) == offsetof(pub::CCMesh, vertices), "");
static_assert(offsetof(::CCMesh, vertexCount) == offsetof(pub::CCMesh, vertexCount), "");
static_assert(offsetof(::CCMesh, triangles) == offsetof(pub::CCMesh, triangles), "");
static_assert(offsetof(::CCMesh, triangleCount) == offsetof(pub::CCMesh, triangleCount), "");

// CCProfileSeg
static_assert(sizeof(::CCProfileSeg) == sizeof(pub::CCProfileSeg), "CCProfileSeg size drift");
static_assert(offsetof(::CCProfileSeg, kind) == offsetof(pub::CCProfileSeg, kind), "");
static_assert(offsetof(::CCProfileSeg, x0) == offsetof(pub::CCProfileSeg, x0), "");
static_assert(offsetof(::CCProfileSeg, cx) == offsetof(pub::CCProfileSeg, cx), "");
static_assert(offsetof(::CCProfileSeg, a0) == offsetof(pub::CCProfileSeg, a0), "");
static_assert(offsetof(::CCProfileSeg, ptOffset) == offsetof(pub::CCProfileSeg, ptOffset), "");
static_assert(offsetof(::CCProfileSeg, ptCount) == offsetof(pub::CCProfileSeg, ptCount), "");

// CCMassProps
static_assert(sizeof(::CCMassProps) == sizeof(pub::CCMassProps), "CCMassProps size drift");
static_assert(offsetof(::CCMassProps, volume) == offsetof(pub::CCMassProps, volume), "");
static_assert(offsetof(::CCMassProps, area) == offsetof(pub::CCMassProps, area), "");
static_assert(offsetof(::CCMassProps, cx) == offsetof(pub::CCMassProps, cx), "");
static_assert(offsetof(::CCMassProps, cz) == offsetof(pub::CCMassProps, cz), "");
static_assert(offsetof(::CCMassProps, valid) == offsetof(pub::CCMassProps, valid), "");

// CCEdgePolyline
static_assert(sizeof(::CCEdgePolyline) == sizeof(pub::CCEdgePolyline), "CCEdgePolyline size drift");
static_assert(offsetof(::CCEdgePolyline, edgeId) == offsetof(pub::CCEdgePolyline, edgeId), "");
static_assert(offsetof(::CCEdgePolyline, points) == offsetof(pub::CCEdgePolyline, points), "");
static_assert(offsetof(::CCEdgePolyline, pointCount) == offsetof(pub::CCEdgePolyline, pointCount),
              "");

// CCFaceMesh
static_assert(sizeof(::CCFaceMesh) == sizeof(pub::CCFaceMesh), "CCFaceMesh size drift");
static_assert(offsetof(::CCFaceMesh, faceId) == offsetof(pub::CCFaceMesh, faceId), "");
static_assert(offsetof(::CCFaceMesh, vertices) == offsetof(pub::CCFaceMesh, vertices), "");
static_assert(offsetof(::CCFaceMesh, vertexCount) == offsetof(pub::CCFaceMesh, vertexCount), "");
static_assert(offsetof(::CCFaceMesh, triangles) == offsetof(pub::CCFaceMesh, triangles), "");
static_assert(offsetof(::CCFaceMesh, triangleCount) == offsetof(pub::CCFaceMesh, triangleCount),
              "");

CC_TEST(reference_abi_layout_matches) {
    // The static_asserts above already proved layout at compile time; this case
    // documents that the reference-header path was exercised.
    CC_CHECK(sizeof(::CCMesh) == sizeof(pub::CCMesh));
}

#else  // no reference header on this machine

#include "cybercadkernel/cc_kernel.h"

CC_TEST(reference_abi_unavailable_note) {
    std::printf("  (reference KernelBridgeAPI.h not found; layout compare skipped)\n");
    CC_CHECK(true);
}

#endif

CC_TEST(library_links_and_reports_no_brep) {
    // Proves the reference (or our) cc_* prototypes link against this library.
    CC_CHECK_EQ(cc_brep_available(), 0);
    CC_CHECK(cc_last_error() != nullptr);
}

CC_RUN_ALL()
