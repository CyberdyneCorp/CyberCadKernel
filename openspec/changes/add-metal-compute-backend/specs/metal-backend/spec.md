# metal-backend

## ADDED Requirements

### Requirement: Metal backend implements the compute-backend interface
The library SHALL provide a Metal-based backend that implements the Phase-0
`IComputeBackend` interface (`name`, `supports_fp64`, `parallel_for`) so it is
interchangeable with the CPU backend without any change to callers, the `cc_*`
facade, or the engine adapter. The backend SHALL be iOS-only and built only when
`CYBERCAD_HAS_METAL` is enabled.

#### Scenario: Metal backend is usable through the compute-backend interface
- GIVEN a booted iOS simulator whose default Metal device is "Apple iOS simulator GPU"
- WHEN the Metal backend is constructed and referenced as an `IComputeBackend`
- THEN it SHALL report a stable `name()` and SHALL execute a data-parallel task
  through the same interface the CPU backend implements

#### Scenario: Host build stays CPU-only
- GIVEN a host build with `CYBERCAD_HAS_OCCT=OFF` and `CYBERCAD_HAS_METAL=OFF`
- WHEN the library is built and its CTest is run
- THEN the Metal backend SHALL NOT be compiled or linked AND the CPU-only build
  and its tests SHALL keep passing

### Requirement: Metal device initialization
The Metal backend SHALL acquire the system default Metal device and a command
queue when it is created. When no Metal device is available, backend creation
SHALL fail cleanly (the backend is not registered) so the CPU path is unaffected.

#### Scenario: Device init succeeds on the simulator GPU
- GIVEN a booted iOS simulator with an available Metal device
- WHEN the Metal backend is created
- THEN it SHALL obtain the default device and a command queue and become ready to
  dispatch compute work

#### Scenario: No Metal device present
- GIVEN an environment where `MTLCreateSystemDefaultDevice()` returns none
- WHEN Metal backend creation is attempted
- THEN creation SHALL fail cleanly without crashing AND the backend SHALL NOT be
  registered, leaving the CPU backend as the active/fallback backend

### Requirement: Unified-memory shared buffers
The Metal backend SHALL allocate shared (unified-memory,
`MTLResourceStorageModeShared`) buffers on Apple Silicon so the CPU and GPU
address the same memory without an explicit copy on the data path.

#### Scenario: CPU/GPU shared-buffer round-trip
- GIVEN a shared buffer allocated by the Metal backend and filled by the CPU
- WHEN a GPU kernel reads and writes that buffer and control returns to the CPU
- THEN the CPU SHALL observe the GPU's writes in the same allocation without an
  explicit copy step

### Requirement: Runtime MSL pipeline compilation
The Metal backend SHALL compile MSL kernels embedded as source strings at runtime
(via `newLibraryWithSource:`) into compute pipeline states, with no precompiled
`.metallib` step, and SHALL cache each pipeline by its kernel-function name so
repeated dispatches do not recompile.

#### Scenario: Kernel compiles at runtime and is cached
- GIVEN an MSL kernel embedded as a source string
- WHEN the backend compiles it into a compute pipeline state and then dispatches
  the same kernel again
- THEN the first call SHALL build the pipeline from source at runtime AND the
  second call SHALL reuse the cached pipeline without recompiling

#### Scenario: Malformed kernel source is reported, not fatal
- GIVEN an MSL source string that fails to compile
- WHEN the backend attempts to build a pipeline from it
- THEN it SHALL return an error result rather than crashing the process

### Requirement: Compute dispatch over an index range
The Metal backend SHALL dispatch a compute kernel over an index range `[0, count)`
— encoding the command, binding buffers, sizing threadgroups, committing, and
waiting for completion — and produce results equal to a CPU reference within an
fp32 tolerance.

#### Scenario: Elementwise kernel matches a CPU reference
- GIVEN an fp32 elementwise kernel (e.g. `y = a*x + y`) and input buffers of length N
- WHEN the Metal backend dispatches it over `[0, N)` on the simulator GPU
- THEN every output element SHALL equal the CPU reference result within the
  documented fp32 tolerance

### Requirement: Registry integration
The Metal backend SHALL register with the Phase-0 `ComputeRegistry` when a Metal
device is available, while the CPU backend remains the default and the fallback.

#### Scenario: Metal registers alongside the CPU default
- GIVEN a build with `CYBERCAD_HAS_METAL` enabled and an available Metal device
- WHEN the backends initialize
- THEN the Metal backend SHALL be registered in the `ComputeRegistry` AND the CPU
  backend SHALL remain registered as the fallback

### Requirement: fp32 precision boundary
The Metal backend SHALL report `supports_fp64()` as false and SHALL never accept
exact double-precision (fp64) work, so the Phase-0 precision guard
(`backend_for(Precision::Fp64)`) always keeps exact modeling on the CPU.

#### Scenario: fp64 work is never routed to Metal
- GIVEN the Metal backend is registered and active for fp32 work
- WHEN an fp64 (exact modeling) job is requested through the precision guard
- THEN the guard SHALL route it to the fp64-capable CPU backend AND the Metal
  backend SHALL NOT receive it

#### Scenario: Metal refuses a direct fp64 request
- GIVEN the Metal backend
- WHEN it is asked directly to run an explicitly fp64-typed job
- THEN it SHALL refuse the request rather than silently downcasting to fp32
