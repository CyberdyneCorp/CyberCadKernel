// Parallel-acceleration policy — OCCT-touching translation unit.
//
// Everything policy-shaped (toggle, worker-cap resolution, fine-thread gate,
// scheduler routing) lives header-only in parallel_policy.h so it is
// host-testable. This TU holds the ONE piece that needs an OpenCASCADE type:
// applying the resolved worker cap to OCCT's process-wide OSD_ThreadPool. Like
// every occt_*.cpp it is iOS-only (CYBERCAD_HAS_OCCT=ON) — there is no host OCCT,
// so this file is simply omitted from the host build and the pure policy above
// still links and runs.

#include "engine/occt/parallel_policy.h"

// ── OCCT headers (adapter TU only) ────────────────────────────────────────────
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>

namespace cyber {
namespace occt {

void applyOcctWorkerCap() {
    const int workers = ParallelPolicy::instance().resolvedWorkerCount();
    // Prefer OCCT's own thread pool (rather than TBB, if OCCT was built with it)
    // so SetNbDefaultThreadsToLaunch is the authoritative bound; this keeps the
    // cap honoured on mobile regardless of how the trimmed OCCT was configured.
    OSD_Parallel::SetUseOcctThreads(Standard_True);
    const Handle(OSD_ThreadPool)& pool = OSD_ThreadPool::DefaultPool();
    if (!pool.IsNull()) {
        pool->SetNbDefaultThreadsToLaunch(workers);
    }
}

}  // namespace occt
}  // namespace cyber
