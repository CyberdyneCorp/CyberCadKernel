# add-metal-compute-backend

A Metal implementation of the Phase-0 compute-backend interface (device init,
unified-memory buffers, runtime MSL pipeline compilation, compute dispatch,
registry integration, fp32 precision guard), iOS-only behind CYBERCAD_HAS_METAL
and verifiable on the iOS simulator GPU
