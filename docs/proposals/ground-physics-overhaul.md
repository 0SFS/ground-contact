# Ground Physics & Near-Ground Collision Overhaul

Status: Stages 1–3 built, 2026-10-27; Stage 4 proposed. See [Implementation record](#5-implementation-record).
Date: 2026-10-27
Scope: `@0sfs/ground-contact`, `fly/src/flight/physics`, `@felipegalind0/jsbsim` WASM bridge, and `foss-earth/engine` (`surfaceQuery.ts`, `groundAcceleration.ts`, `ground-WASM`)

---

## 1. Executive Summary & Hardware Architecture Choice

Near-ground flight, takeoff, and landing currently suffer from two severe issues:
1. **Performance:** Frame rates drop by ~50% within 150 m of the ground (`MAX_BODY_COLLISION_ALTITUDE_METERS`), accompanied by frame-time spikes when Google 3D Tiles stream or refine.
2. **Physical Realism & Underground Clipping:** The aircraft nose, wingtips, tail, and belly frequently clip underground on sloped runways or uneven terrain, and contact impacts either teleport/bounce the aircraft without angular torque or trip a hard simulation reset (`resetToInitialConditions(2)` + double `runIc()`).

### Why CPU WASM (SIMD + Worker) Beats WebGPU Here
As already measured on Apple M5 in [`collision-compute-comparison.md`](../collision-compute-comparison.md):
- **5-ray local hit batch:** CPU BVH takes **0.0008 ms (0.8 µs)** vs. WebGPU BVH at **0.4 ms** (a **497× latency advantage** for CPU BVH) and Production CPU picking at **2.25 ms** (**2,792× slower** than CPU BVH).
- A single aircraft has 3–5 landing gear units and 6–16 structural hull contact proxies. Dispatching 10–20 rays to a WebGPU compute shader leaves >99% of the GPU idle while paying 0.4–2.0 ms of command-buffer submission and `mapAsync` readback latency every physics sub-step.
- Because JSBSim runs on the CPU in WebAssembly at 60–240 Hz, **keeping spatial queries, per-wheel terrain plane evaluation, and impulse contact resolution in CPU WASM / typed-array memory** avoids all CPU↔GPU synchronization stalls and executes in **< 25 microseconds per frame**.

---

## 2. Root-Cause Diagnosis of the Current Stack

### A. Why FPS Drops by 50% Near the Ground

1. **Unbatched Full-Scene Raycasts Every Physics Sub-Step (`surfaceQuery.ts`, `visibleMeshCollision.ts`):**
   - Within 150 m AGL, each fixed step runs [`terrainContact.update()`](../../src/terrainContact.ts) (1–3 vertical rays due to coarse-tile loop in [`surfaceQuery.ts`](../../../../foss-earth/engine/src/babylon/surfaceQuery.ts)) plus [`visibleMeshCollision.update()`](../../src/visibleMeshCollision.ts) (5–6 swept rays).
   - With 2–4 physics sub-steps per render frame, the main thread executes **12 to 30 full-scene raycasts per frame**.
   - Every single `cast()` in [`surfaceQuery.ts`](../../../../foss-earth/engine/src/babylon/surfaceQuery.ts) recomputes `getWorldRoot()?.computeWorldMatrix(true)`, inverts the matrix, and scans `scene.meshes`, calling `mesh.computeWorldMatrix()` inside the filter predicate for every visible surface mesh.

2. **First-Query Brute-Force Parity Check & Unprepared Mesh Fallbacks (`groundAcceleration.ts`):**
   - In [`groundAcceleration.ts`](../../../../foss-earth/engine/src/babylon/groundAcceleration.ts), the first time a ray hits a newly built mesh BVH (`!entry.validated`), production code executes **both** the BVH query **and** an unaccelerated Babylon `scene.pickWithRay()` triangle scan to verify parity (`samePick(pick, reference)`). During landing/takeoff, as Google tiles refine, this causes severe main-thread stalls.
   - Any visible mesh whose worker BVH build is still `pending` (`!entry?.snapshot`) falls back to `scene.pickWithRay()` across raw triangles on the main thread.

3. **Full JSBSim Snapshot Allocation Before Every Single Sub-Step (`fixedStepLoop.ts`):**
   - [`fixedStepLoop.ts`](../../../fly/src/flight/physics/fixedStepLoop.ts) calls [`captureSimulation(sdk)`](../../src/safeFlightState.ts) before **every** `sdk.run()` step (60–120+ times per second), reading dozens of properties across the WASM boundary and allocating fresh JS objects (`initial`, `controls`, `readFlightState`) even when no fault occurs.

### B. Why the Aircraft Goes Underground & Feels Unrealistic

1. **Single Horizontal Ground Plane Under the CG (`terrainContact.ts`):**
   - [`terrainContact.ts`](../../src/terrainContact.ts) samples **one** `(lat, lon)` coordinate under the CG and writes a single scalar `position/terrain-elevation-asl-ft` into JSBSim.
   - JSBSim treats the entire world as a flat horizontal plane at that elevation. On sloped runways, crowned taxiways, or hills, individual wheels and wingtips do not see the terrain height or surface normal beneath their own coordinates—allowing the nose, tail, or wingtips to pass straight underground.

2. **Sparse Zero-Radius Point Probes (`collisionGeometry.ts`):**
   - [`BODY_COLLISION_PROBES`](../../src/collisionGeometry.ts) uses only 5 points (`nose`, `center`, `left-wing`, `right-wing`, `tail`) with `center` at `up: -0.2 m`, leaving the lower belly, nose gear pod, main gear sponsons, and wing leading edges unguarded between points.
   - Fixed CG-relative offsets shift when fuel/payload changes the CG.

3. **State-Destroying `restoreSimulation` Bounce Hack (`visibleMeshCollision.ts`, `safeFlightState.ts`):**
   - When a body probe hits in [`visibleMeshCollision.ts`](../../src/visibleMeshCollision.ts), it reflects only the **linear** velocity vector at the CG, ignores angular momentum (`r × J` torque—so a wingtip strike produces zero roll or yaw impulse), and calls [`restoreSimulation()`](../../src/safeFlightState.ts).
   - `restoreSimulation()` calls `sdk.resetToInitialConditions(2)` and `sdk.runIc()` **twice**, wiping out JSBSim's integrator history, aerodynamic state derivatives, and oleo strut compression.
   - [`terrainContact.ts`](../../src/terrainContact.ts) performs the same destructive `restoreSimulation()` reset on tile refinement (`liftedThroughPlane`, `followGround`), snapping the aircraft to `ground + aircraftClearanceMeters()` (uncompressed gear height) so it repeatedly drops back onto its springs.

4. **Disconnected Coupled Wheel Solver (`coupledRigidWheels.ts`, `wheelContact.ts`):**
   - [`coupledRigidWheels.ts`](../../src/coupledRigidWheels.ts) implements an energy-conserving sequential-impulse tire/brake solver, but it is unwired because `@felipegalind0/jsbsim` lacks the `NativeWheelContactBridge` ([`wheelContact.ts`](../../src/wheelContact.ts)).

---

## 3. Target Architecture & Staged Implementation Plan

### Stage 1: Eliminate Main-Thread Near-Ground FPS Spikes (`foss-earth/engine`, `fly`, `ground-contact`)

1. **Remove Runtime Parity Re-Picking in Production (`groundAcceleration.ts`):**
   - Gate the `!entry.validated` brute-force `scene.pickWithRay()` check in [`groundAcceleration.ts`](../../../../foss-earth/engine/src/babylon/groundAcceleration.ts) behind an explicit test/diagnostic flag (off in normal flight). The worker already runs `validateGroundBvh()` against mesh geometry and verifies bit-for-bit parity on the admission fixture.
2. **Batch Multi-Ray Surface Queries (`surfaceQuery.ts` / `groundAcceleration.ts`):**
   - Add `raycastBatch(rays: readonly EcefRay[]): (SurfaceHit | null)[]` (or hoist world-root matrix inversion and active surface mesh collection once per physics step in `createFrameSurfaceQuery`).
   - Pre-filter active surface meshes once per step and cache their world-to-local inverse matrices for the duration of the frame's physics sub-steps instead of calling `mesh.computeWorldMatrix()` and `Matrix.Invert()` inside every ray × mesh test.
3. **Bound Unindexed Mesh Fallback During Tile Streaming:**
   - Increase the worker preparation queue from a single slot (`busy` boolean dropping concurrent requests in [`groundBvhWorker.ts`](../../../../foss-earth/engine/src/browser/groundBvhWorker.ts)) to a bounded FIFO/LIFO queue prioritized by distance to the focus point (aircraft).
   - Never fall back to brute-force triangle scanning on high-poly meshes (> 2,048 triangles) while their BVH is building; retain the parent/previous tile's accelerated BVH until the child's BVH snapshot is ready.
4. **Replace Per-Step JS `captureSimulation` with Lazy / Ring-Buffer Recovery (`fixedStepLoop.ts`, `safeFlightState.ts`):**
   - Stop allocating a full `SimulationSnapshot` object with control catalog maps before every `sdk.run()` in [`fixedStepLoop.ts`](../../../fly/src/flight/physics/fixedStepLoop.ts).
   - Reuse a pre-allocated flat `Float64Array` ring buffer of batch-read values per sub-step, and only materialize the `SimulationSnapshot` record when a step actually trips a fault guard (`tripped.length > 0`).

### Stage 2: Per-Contact Ground Sampling & Smooth Tile Refinement (`ground-contact`, `fly`)

1. **Multi-Point Terrain Sampling Under Each Gear & Structure Contact:**
   - Compute the world `(lat, lon, alt)` of each active landing gear (`BOGEY`) and structural (`STRUCTURE`) contact point from its structural inches (`x-position`, `y-position`, `z-position`) and current CG (`inertia/cg-*-in`), using [`groundContactBodyPosition`](../../src/collisionGeometry.ts).
   - Sample `surface.sample(lat, lon)` at the nose wheel, left main, and right main (and wingtips/tail when near the ground).
   - Fit an oriented local support plane (or pass per-contact elevations + normals into JSBSim) so that:
     - Runway slope pitches the aircraft naturally and loads the nose/main struts accurately.
     - Cross-slope rolls the aircraft onto the downhill main gear.
     - No wheel or structural contact clips below a sloped runway or hill while the CG is over lower ground.
2. **Non-Destructive Tile Refinement Adaptation (`terrainContact.ts`):**
   - When a tile refines under a resting or taxiing aircraft (`changedHere`), **do not** call `resetToInitialConditions(2)` / `runIc()` and do not reset altitude to uncompressed `aircraftClearanceMeters()`.
   - Instead, shift `position/h-sl-ft` and `position/terrain-elevation-asl-ft` by the exact `refinementDelta` (preserving the current oleo strut compression and velocity state), or slew the support elevation over a short distance/time window so the oleo dampers absorb sub-decimetre photogrammetry steps without resetting the FDM executive.

### Stage 3: True 6-DOF Rigid-Body Hull Collision & Structural Contacts (`ground-contact`, `aircraft`)

1. **Mesh-Derived Structural Contacts & Swept Hull Proxies:**
   - Ensure all aircraft (C172, SF-50, F-35B) declare accurate `STRUCTURE` contacts in structural inches (nose, belly fore/aft, tail skid, left/right wingtips, nacelle/pod bottoms) so normal runway scrapes (tailstrike, gear-up belly landing, wingtip scrape) are handled continuously by JSBSim's spring/damper + Coulomb friction model rather than triggering discontinuous rewinds.
   - Upgrade [`collisionGeometry.ts`](../../src/collisionGeometry.ts) so all hull probes are stored in **structural inches** (converted live with the current CG) and cover the belly, nose, tail, wingtips, and leading-edge segments.
2. **Replace `restoreSimulation` Velocity Reflection with 6-DOF Impulse + Position Projection (`visibleMeshCollision.ts`):**
   - When a swept hull probe hits an obstacle or terrain mesh:
     - Compute the lever arm $\mathbf{r} = \mathbf{p}_{\text{contact}} - \mathbf{p}_{\text{CG}}$ and contact normal $\mathbf{n}$.
     - Compute the 6-DOF effective normal mass:
       $$m_{\text{eff}, n}^{-1} = \frac{1}{m} + (\mathbf{r} \times \mathbf{n})^\top \mathbf{I}_{\text{world}}^{-1} (\mathbf{r} \times \mathbf{n})$$
     - Compute normal impulse $j_n = \max\bigl(0, -(1 + e)\,(\mathbf{v}_{\text{contact}} \cdot \mathbf{n})\, m_{\text{eff}, n}\bigr)$ and tangential friction impulse $\mathbf{j}_t$ (clamped to $\mu j_n$).
     - Update **both** linear velocity $(\Delta \mathbf{v} = \mathbf{j} / m)$ and angular velocity $(\Delta \boldsymbol{\omega} = \mathbf{I}_{\text{world}}^{-1}(\mathbf{r} \times \mathbf{j}))$, and project position out of penetration along $\mathbf{n}$ without zeroing unrelated aerodynamic/engine/gear states where possible.

### Stage 4: Wire `NativeWheelContactBridge` & `coupledRigidWheels.ts` (`@felipegalind0/jsbsim`, `ground-contact`)

1. **Expose `NativeWheelContactBridge` in the JSBSim WASM Build:**
   - Implement the contract already specified in [`wheelContact.ts`](../../src/wheelContact.ts) (`readContacts`, `applyAcceptedImpulses`, `exclusiveLongitudinalFriction: true`, plus per-wheel ground height/normal inputs).
2. **Activate `solveCoupledRigidWheels` (`coupledRigidWheels.ts`):**
   - Connect [`solveCoupledRigidWheels()`](../../src/coupledRigidWheels.ts) inside the fixed-step loop so wheel rotational inertia, touchdown spin-up drag, brake torque saturation, and energy dissipation drive actual aircraft forces as well as tire audio and haptics.

---

## 4. Verification & Acceptance Criteria

1. **Performance:**
   - Zero brute-force `scene.pickWithRay` parity re-picks during normal flight (`entry.validated` check off in production).
   - `flight/physics/terrain contact` + `flight/physics/collision` combined frame budget **< 0.25 ms median / < 0.5 ms P95** at 120 Hz during landing and rollout on both Raster and Google 3D Tiles.
   - Zero per-step heap object allocations in `fixedStepLoop.ts` when no fault occurs.
2. **Physical Accuracy & Non-Penetration:**
   - Aircraft taxiing, taking off, or landing on a sloped runway pitches to match the runway grade without the nose wheel or tail sinking underground.
   - Tile refinement while parked or taxiing preserves oleo compression and does not drop the aircraft from uncompressed clearance height.
   - Off-angle wingtip or nose contact produces physically consistent roll/pitch/yaw angular impulses and friction slowdown, never clipping through the ground mesh.
   - All existing unit and integration suites in `ground-contact` and `fly/src/flight/physics` pass, plus new tests covering sloped surfaces, multi-point contact planes, and 6-DOF hull impulse response.

---

## 5. Implementation Record

Where the build differs from the plan above, or stops short of it.

- **Stage 1.** The first-query comparison with Babylon is the
  `compareFirstQueries` diagnostic, off by default. Instead of `raycastBatch`,
  `SurfaceQuery` has optional `beginFrame()`/`endFrame()`: one physics tick's
  queries share the world transform, the candidate meshes and their bounds, and
  each mesh's inverse is kept while its world matrix is unchanged. Builds queue
  (eight in flight on the page, sixteen in the preparer and worker) rather than
  dropping. A mesh without an index is still picked by Babylon while its index
  builds: skipping it would leave the aircraft without ground on that tile. The
  fixed-step loop records the pre-step state into a reused array and builds the
  snapshot only when a guard trips; a test holds `captureSimulation`'s
  replacement to zero allocations.
- **Stage 2.** JSBSim still takes one horizontal plane. The plane is chosen from
  samples under every contact ([ground-contact.md](../ground-contact.md), "One
  plane for every contact"), so no contact sinks deeper than the lowest one is
  compressed, but a slope does not pitch the aircraft: that needs a ground per
  contact in the native gear model. Refinement moves only `position/h-sl-ft` by
  the change.
- **Stage 3.** Probes are hand-placed per aircraft in structural inches, not
  generated from the mesh, and do not cover leading edges
  ([generated-collision-geometry.md](generated-collision-geometry.md)). The
  impulse solver is as specified, with sequential resolution of several hits
  in crossing order. The result is still written through `ic/*` and RunIC,
  because the WASM SDK sets velocities and rates no other way, so a hull impact
  resets the integrators' history. The C172's contacts moved to their surveyed
  positions and gained one at the aft end of the fuselage, which meets the
  runway at 14.5° nose-up on the main wheels: on a grade the tail carries the
  aircraft from about 25%, where on 20% the main wheels still do. The F-35B
  packages gained contacts at the stabilators' trailing edges, which touch at
  about 22°, before the tail skid at 26°. They have no hull probes: six is the
  budget per airframe.
- **Tests.** Required test 3 is
  `fly/src/flight/physics/hullImpact.integration.test.ts`, against real JSBSim:
  restitution at the struck point, the signs of p, q and r, friction slowdown,
  no energy gained, and a second of scraping with every probe clear. The same
  file checks that the structure contacts meet level ground before any inset
  probe at any attitude, and that the SF-50 and F-35B rest gear-up on them.
