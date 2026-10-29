# ground-contact/wasm

Rust kernels for the fixed ground-contact step, built for
`wasm32-unknown-unknown` with `simd128` and no wasm-bindgen. Each kernel is
the arithmetic of a TypeScript function in `../src`, in the same order. The
host keeps everything that touches JSBSim or the surface:

| export | does | reference |
| --- | --- | --- |
| `gc_hull_place` | the pose from the 22 properties, each hull probe's swept point in ECEF and its sweep this step | `visibleMeshCollision.ts` `place` |
| `gc_hull_resolve` | the cast hits validated and ordered by step fraction; per hit a normal impulse with restitution and a Coulomb friction impulse; the CG moved clear; the state JSBSim is restored to | `visibleMeshCollision.ts` `sweep`, `resolve`, `update` |
| `gc_support_reset`, `gc_support_plan`, `gc_support_finish` | `measure` split at its surface samples: the footprints and the samples to take, then the filtered ground per contact and the support plane | `terrainContact.ts` `createContactSupport`, `wheelGroundFilter.ts` |
| `gc_solve_wheels` | `solveCoupledRigidWheels` | `coupledRigidWheels.ts`, `wheelContact.ts` |
| `gc_step_wheel_spin` | `stepWheelSpin` | `wheelSpin.ts` |
| `gc_sin`, `gc_cos`, `gc_atan2` | the core's libm trig | for checks that need the core's exact results |
| `gc_alloc`, `gc_free`, `gc_last_error`, `gc_heap_allocations`, `gc_abi_version` | buffers, errors, the allocation count, ABI 2 | |

`../src/groundContactCore.ts` declares the parts of the core the modules
call. `createVisibleMeshCollision`, `createTerrainContact`,
`createCoupledWheelSolver` and `createWheelSpinStepper` take a `core` (or its
load) and run their hot path on a kernel. If the kernel throws, the module
goes back to the TypeScript reference for good and `status()` says why.

## Use

```js
import { loadGroundContact, SPIN } from "./wasm/index.js"; // not in package.json exports yet

const core = await loadGroundContact();            // or instantiateGroundContact(bytes)
const wheels = core.createWheelSolver(3);
const spin = core.createWheelSpin(3);
// per step: fill the input views, run, read the outputs
wheels.body.set(body); wheels.wheels.set(rows); wheels.contacts.set(contacts);
wheels.run(1 / 120, contactCount);
spin.run(1 / 120, SPIN.INERTIA);
core.dispose();
```

Each `create*` call allocates its buffers in WASM memory once. Its views are
`Float64Array`s over that memory, cached and replaced only if the memory
grows, which stepping never does. A step writes the inputs, calls the kernel
and reads the outputs: neither JS nor Rust allocates, and
`heapAllocations()` lets a test prove it. Errors throw with `.status` set:
1 invalid buffer or count, 2 allocation, 3 a non-finite or out-of-range
input. The row layouts are in `index.js` (`HULL`, `SUPPORT`, `WHEELS`,
`SPIN`) and are documented field by field at the top of each Rust module in
`crates/ground-contact/src/`.

There is no worker guard. The kernels are synchronous, bounded and take
microseconds, so they run wherever the fixed step runs, page main thread
included.

## Hull collision

`createHullCollision(probeCount)` has 1 to 64 probes, each
`[xIn, yIn, zIn, insetIn]`. A probe's lever is the swept point in body feet:
`((cgX - x) / 12, (y - cgY) / 12, (cgZ - z - inset) / 12)`. Inertia is
JSBSim's tensor from the property getters (`-ixy`, `+ixz`, `-iyz`), and the
impulses are in body axes, feet, slugs and radians. Per step, as
`createVisibleMeshCollision`'s `update`:

1. Read the 22 `PATHS` into `values` and call `place()`. If that returns
   false (a value is not finite), forget the previous points and stop.
2. Sample the surface under the CG. On the first step, without a sample, or
   more than `MAX_BODY_COLLISION_ALTITUDE_METERS` above it: `swap()` and stop.
3. For each probe whose `segments` distance is finite and at least
   `HULL.minSweepMeters`, raycast from `from` along `movement` for
   `distance` and write the hit as returned. Write a NaN distance for no hit
   or no cast.
4. If no cast hit anything, `swap()` and stop. Otherwise call
   `resolve(getRestitution?.() ?? NaN, friction ?? NaN)`. It validates each
   hit as `sweep` does, faces each normal against the motion and resolves
   the hits in step-fraction order. If it returns 0, `swap()` and stop.
5. Restore JSBSim with `ic/lat-geod-deg`, `ic/long-gc-deg`, `ic/h-sl-ft`,
   `ic/vn-fps`, `ic/ve-fps`, `ic/vd-fps`, `ic/p-rad_sec`, `ic/q-rad_sec` and
   `ic/r-rad_sec` from `output[1..9]`. Read the properties again, `place()`,
   and `swap()` if that succeeded.

The output also has the CG shift, the body velocity and per hit
`[probe, fraction, normal, approach, normal impulse, friction impulse, lift]`.
`getRestitution` is called only on steps with a hit, so it must not have
side effects the rest of the step depends on.

## Contact support

`createContactSupport(units, previous)` takes the model's contact units,
`{ retractable, prefix }` from `groundContactUnits` or
`{ retractable, wheel }`. A unit is a wheel when its prefix is `gear/`
(a BOGEY) rather than `contact/` (a STRUCTURE). The kernel keeps every
contact's state between steps, including its ground filter. Per step, as
`createContactSupport`'s `measure`:

1. Fill `input`: lat, lon (deg), altitude (m), roll, pitch, heading (rad),
   cgGround (m), revision, revisionChanged, cgDelta (m), urgent, snap,
   samples, previousElevation (m).
2. If `wantsGeometry()`, read `[gear/num-units, gear/gear-pos-norm,
   cg x, y, z in]` and each unit's `x, y, z-position` into `values`.
   Pass the outcome to the plan: `GEOMETRY_READ`, `GEOMETRY_FAILED` (the
   read threw, which forgets the contacts) or `GEOMETRY_UNAVAILABLE` (no
   units). A changed unit count also forgets them, and `discovered` turns
   false. Discover the units again on the next step that wants geometry.
3. `plan(geometry)` returns the request count. Sample the surface at each
   request's `[latDeg, lonDeg]` in order and write `heightMeters` and
   `revision` (NaN for no sample). Which spots are requested never depends
   on what a sample returns.
4. `finish()` returns `[elevation (NaN: the centre's ground stands in),
   refinement, lowest, inRange, requests, applied]`.

`measure(geometry, sample)` does steps 3 and 4 with a callback. If the
callback throws, the step is abandoned and the kernel reset. `reset()` is the
module's `reset`. If rediscovery finds units with other flags, create a
kernel for them and pass the old one as `previous`: the reference forgets the
contacts but keeps its step count and revalidation countdown, and `previous`
hands them on. Before any kernel exists and with no units, the module returns
`{ elevation: NaN, refinement: revisionChanged && finite(cgDelta) ? cgDelta : 0 }`
without one.

## Build, check, bench

Node 26 is required: `check.mjs` imports the TypeScript sources through
type stripping and a `module.registerHooks` resolver. `visibleMeshCollision.ts`
imports FOSS Earth's `cameraMath.ts`, found beside the 0SFS checkout or at
`$FOSS_EARTH_ENGINE/src/camera/cameraMath.ts`.

```sh
node scripts/build.mjs   # single/ground_contact.wasm and build-metadata.json, offline, --locked
node scripts/check.mjs   # parity against ../src; exit 1 on any failure
node scripts/bench.mjs   # per-kernel and per-step times for the C172
CARGO_TARGET_DIR=build/target-host cargo test --offline --locked
```

`build.mjs` refuses toolchains that differ from `toolchain.lock.json` and
installs nothing. Its receipt hashes every crate file, `scripts/build.mjs`,
and the files next to the binary. `check.mjs` refuses a stale receipt, so
rebuild after editing any of them.

`check.mjs` compares against the working tree's `../src`, so its result holds
for that tree. With a seeded PRNG it checks:

- coupled wheels and wheel spin, identical with `Object.is` in every output
  value, cold retries and rejected contacts included
- `createVisibleMeshCollision` over 300 sequences of 10 steps. The probe sets
  are the C172, SF50 and F-35B, plus random sets of 1 to 64 probes. The
  surfaces are two-sided planes and walls. One cast in ten returns junk: no
  hit, a NaN, negative or overlong distance, a zero, tiny or NaN normal, a NaN
  point, or a valid hit at either end of the sweep or tied with another.
  Resets and non-finite properties happen at random. After every step, every
  property the mock JSBSim holds is compared.
- `createContactSupport` over 160 sequences of 60 steps on the C172,
  retractable, random, unit-less and 64-unit models. The steps include
  resets, failing reads, unit count changes, units added, gear moving, world
  revisions, teleports, heights out of range and sample budgets from 0 to
  infinite. The elevation, the refinement and every sampled spot are
  compared.
- each of those against both the host flow above, on the kernel, and the
  module's own WASM path (given this core), which must never fall back
- zero heap allocations over 20,000 full steps, and the raw ABI error codes

Rust's libm `sin` and `cos` differ from V8's on about 1.4% of inputs in
[-7, 7], and `atan2` on 0.15%, in the last bits. So the hull and the support
run twice. With the
TypeScript's trig replaced by `wasmMath` (in memory, for modules imported
with `?wasm-trig`), every value must match bit for bit, because the rest
of the arithmetic is the TypeScript's in its order. With V8's trig, values
must agree within a bound per step, and the report gives the largest
difference.

On the tree this was last run on, the check had 0 failures:

- hull, both comparisons: all 124,142 values bit for bit with the core's
  trig. With V8's trig, 123,328 bit for bit and the largest difference
  9.2e-9 ft of `h-sl-ft`.
- support, both comparisons: all 19,200 results and every sampled spot bit
  for bit in both modes
- wheels and spin: 663,000 and 466,200 values bit for bit

`bench.mjs` prints the host CPU and `pmset` power source with its numbers.
The step it times is the C172's hull (five probes, clear or with two wingtip
strikes), its support (nine units, two samples) and its three wheels; the
surface queries are the host's and are not timed. On an Apple M5 on AC power
(charging), the medians were:

- hull `place` 68 ns, `resolve` 79 ns clear and 204 ns with two strikes
- support 400 ns, wheels 451 ns, spin 55 ns
- the host's writes into all the views, 125 ns
- a full step 1.08 µs clear and 1.20 µs with the strikes, against a 5 µs
  budget

A laptop's times move with power source, thermal state and load. Compare
runs only on the same machine in the same state.
