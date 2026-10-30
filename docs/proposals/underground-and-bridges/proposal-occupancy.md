# Underground and bridges: geometry, support, and recovery

Design proposal responding to [the brief](brief.md). See the [README](README.md) for its status and review.

## 1. Definition and design boundaries

### Recommendation

Separate four questions:

1. **Occupancy:** does the aircraft overlap solid matter?
2. **Collision:** did its motion cross a surface?
3. **Support:** which surface below its contacts should support it?
4. **Recovery:** is there a sufficiently certain, short correction that may run automatically?

Today, the first surface visible from the sky answers all four. That is the central error.

Use three occupancy states: `OUTSIDE`, `INSIDE`, and `UNKNOWN`. Missing geometry, conflicting LODs, and unverified orientation must be capable of producing `UNKNOWN`. Uncertainty must never, by itself, authorize a relocation or an underground card.

### Important corrections to the brief

Inspection of the checkout revealed:

- **Current normals are ray-facing.** `surfaceQuery.ts` calls Babylon's `pick.getNormal(true, false)`. The installed Babylon implementation flips that normal against the ray. The accelerated Rust path mirrors this behavior. The existing `SurfaceHit.normal` therefore cannot distinguish entry from exit.
- **A BVH already exists.** Extend the existing per-mesh acceleration system, including its TypeScript and Rust implementations, worker preparation, generation checks, and disposal.
- **Physics supports 30/60/120 Hz.** The general default is 60 Hz; the drone controller currently requires 120 Hz. Express recovery and missing-surface timers in seconds. Report costs at the requested 120 Hz.
- **There are real JSBSim regression harnesses.** Existing refinement tests already exercise paired altitude/floor translations. Some automatically accept relocation proposals, so they cannot establish that a new design produces no questions.

The relevant current behavior is in [terrainContact.ts](../../../src/terrainContact.ts), [surfaceQuery.ts](https://github.com/foss-earth/engine/blob/main/src/babylon/surfaceQuery.ts), and [fixedStepLoop.ts](https://github.com/0SFS/fly/blob/main/src/flight/physics/fixedStepLoop.ts).

### Ideal solid model

Let S denote the reconstructed solid.

For raster terrain with height function h(x, y):

```text
S = { (x, y, z) : z < h(x, y) }.
```

For photogrammetry, S consists of terrain extending downward and reconstructed object volumes. Open passages remain free space. A bridge deck is a bounded volume above a separate road surface; it does not extend downward to the road.

Let B(q) be the aircraft's posed collision envelope, including deployed gear. Allow ordinary suspension compression separately; a wheel settling onto its spring is not an underground event.

The collision-free configuration set is:

```text
C_free(S) = { q : interior(B(q)) intersects interior(S) nowhere }.
```

A point being outside is weaker than the entire aircraft being clear.

### Incomplete observations

An arbitrary open triangle soup does not uniquely define S. Introduce a family of admissible solid interpretations consistent with:

- the selected LOD representation;
- trusted surface orientation;
- known free-space observations;
- observed passages;
- explicitly bounded seam repairs;
- unknown regions where coverage is absent.

For a configuration q:

- `OUTSIDE` means q is collision-free in every admissible solid interpretation.
- `INSIDE` means q overlaps solid matter in every admissible interpretation.
- Otherwise, the result is `UNKNOWN`.

This is an ideal specification, not a claim that a few rays enumerate all admissible worlds. The real-time implementation provides evidence under stated geometry assumptions and preserves uncertainty where those assumptions fail.

A stable mesh is not necessarily a correct mesh. Waiting cannot distinguish a real wall from a photogrammetry curtain if their triangles are identical.

### Intended interpretation

| Situation | Definition |
|---|---|
| Under a bridge, awning, or parking deck | Outside when the open passage is represented; overhead enclosure in some directions is irrelevant. |
| Below a road surface | Inside the terrain solid, subject to coverage and surface confidence. |
| Inside a closed building blob | Inside the represented solid. We cannot infer an unobserved usable interior. |
| Inside a closed tree-canopy blob | Inside the represented solid. Without semantic data, inventing vegetation-specific free space is unjustified. |
| Beneath an open canopy | Outside. |
| Tunnel mouth or underpass | Outside through a represented open passage. A sealed or unresolved mouth is unknown. |
| Tile crack | Preserve the previous contact/free-space interpretation; the crack does not establish a traversable passage. |
| Photogrammetry curtain | Collision geometry may exist, but its meaning as a solid boundary can remain unknown. No automatic climb through the bridge. |

Crack repair must use a small, separate seam tolerance. The approximately 1 m recovery threshold must never double as a geometry-closing radius.

### Why not choose one standard classifier?

- **Generalized winding number** supplies useful occupancy evidence for reasonably consistently oriented meshes and supports segmentation of defective surfaces. It does not establish missing real-world topology. Summing parent and child LODs also corrupts the interpretation. [Jacobson et al., 2013](https://users.cs.utah.edu/~ladislav/jacobson13robust/jacobson13robust.html)
- **Fast winding numbers** accelerate evaluation of that field. They improve cost, not observation quality. [Barill et al., 2018](https://doi.org/10.1145/3197517.3201337)
- **Ray-parity voting** is useful evidence after duplicate crossings are removed. It requires known exterior endpoints and complete crossing lists; a finite ray ending in unknown space has no valid parity interpretation. Volumetric repair is a relevant precedent, rather than a proof that arbitrary streamed meshes have a unique interior. [Nooruddin and Turk, 2003](https://faculty.cc.gatech.edu/~turk/my_papers/volumetric_simp.pdf)
- **Nearest-surface sign** is cheap when orientation and topology are trustworthy. The pseudonormal result for closed meshes does not automatically extend to tile seams, open sheets, or inconsistent winding. [Bærentzen and Aanæs, 2005](https://orbit.dtu.dk/en/publications/signed-distance-computation-using-the-angle-weighted-pseudonormal/)
- **Free-space reachability** handles bridges and tunnels naturally, but unrestricted flood fill leaks through cracks. Acquisition visibility would help, but Google's original sensor rays are absent from this interface. Aircraft motion provides a limited additional observation, under the collision model's assumptions. [Labatut et al., 2009](https://imagine.enpc.fr/publications/papers/CGF09.pdf)

Use temporal collision tracking as the ordinary flight mechanism, local geometric classification for initialization and map changes, and explicit uncertainty throughout.

## 2. Ideal and real-time geometry algorithms

### Ideal algorithm, ignoring cost

1. Construct a spatially coherent LOD representation. Never treat simultaneous parent and child surfaces as independent solid boundaries.
2. Build an arrangement of the selected surfaces and possible bounded seam repairs.
3. Establish free-space seeds from trusted exterior regions and previously validated aircraft configurations.
4. Infer candidate solids using orientation, winding, surface agreement, and free-space connectivity. Keep alternative interpretations where evidence is insufficient.
5. Compute occupancy for the complete aircraft envelope against those interpretations.
6. Compute support and recovery in configuration space.
7. Return an occupancy verdict, support candidates, uncertainty, and a validated recovery path.

Do not close the boundary of a resident tile neighborhood and call its interior solid. Residency boundaries are observation limits.

### Required query extensions

Preserve existing `sample()` semantics for consumers that explicitly want the topmost displayed surface. Add aircraft-aware queries rather than silently changing that API.

```ts
interface SurfaceCrossing extends SurfaceHit {
  faceId: number;
  rawNormal: Ecef;       // geometric winding normal; never ray-faced
  orientation: "verified" | "unknown";
  tileId: string;
  meshGeneration: number;
}

interface CompleteResult {
  complete: boolean;    // false on traversal/output exhaustion
  localGeneration: number;
}

interface AircraftSurfaceQueries {
  raycastAll(
    origin: Ecef,
    direction: Ecef,
    lengthMeters: number,
    output: SurfaceCrossing[],
    maxHits: number,
    budget: QueryBudget,
  ): CompleteResult;

  trianglesNear(
    bounds: EcefBounds,
    output: TriangleRef[],
    budget: QueryBudget,
  ): CompleteResult;

  localCoverage(bounds: EcefBounds): {
    localGeneration: number;
    status: "ready" | "pending" | "missing" | "conflicting";
  };
}
```

Internally expose tile ancestry, replacement coverage, and local geometry generations. Add closest-point and swept-envelope traversal to the existing BVH as the collision-envelope stage lands.

Define `rawNormal` from transformed triangle vertices in a documented coordinate convention. Calibrate its outward sign separately. Test reflected transforms and nonuniform scales. Preserve the existing ray-facing normal for impact consumers.

An all-hit overflow returns incomplete evidence. It must not silently become a shorter valid crossing list.

### A coherent LOD representation

Use tile replacement relationships and actual replacement coverage:

- Exclude an ancestor only where replacement children provide the required coverage.
- Retain the parent in uncovered regions, with appropriate uncertainty.
- Do not infer replacement from overlapping bounding boxes.
- Do not merge surfaces merely because their heights differ by less than geometric error.
- Preserve distinct road and deck surfaces.

Where replacement coverage cannot be established, classify the overlap as unresolved. Geometric error can help prioritize detail requests; it is not a calibrated vertical-error bound.

Maintain a local generation for the aircraft neighborhood. Global revision churn elsewhere must not invalidate every local result.

### Temporal coherence

Store:

- the last validated aircraft pose and swept free-space corridor;
- selected support identities and contact patches;
- local geometry generation;
- pending classification work;
- recovery episode and correction ledger.

On unchanged local geometry, valid continuous collision detection explains whether aircraft motion crossed a surface. Reclassifying from scratch every tick is unnecessary.

On a local map change, distinguish:

1. **Matched support refinement:** the existing support patch moved slightly without changing the local layer topology.
2. **New containment:** new geometry encloses previously free space, adds a curtain, or changes the road/deck relationship.

The first may permit bounded contact recovery. The second requires more evidence and does not authorize movement merely because it persists.

Historical free space is a veto against unquestioned adoption of a newly enclosing map. It is not permanent authority to ignore future genuine collisions.

### Practical reclassification

Use a deterministic set of 12 directions, such as normalized icosahedron vertices. Evaluate witnesses needed by the aircraft envelope, rather than the CG alone.

For a verified outward normal n, the first nontangent crossing in direction d supports:

```text
dot(n, d) < 0: origin outside
dot(n, d) > 0: origin inside
```

Unknown orientation, tangencies, inconsistent crossings, incomplete coverage, and truncated queries abstain.

```ts
function reclassify(snapshot: LocalSnapshot): Classification {
  if (!snapshot.hasCoherentLodFrontier)
    return unknown("LOD coverage unresolved");

  // Matched support refinements take their separately validated contact path.
  if (snapshot.hasNewEnclosureOfHistoricalFreeSpace)
    return unknown("new geometry conflicts with prior free space");

  const evidence = classifyEnvelopeWitnesses({
    snapshot,
    directions: ICOSAHEDRON_DIRECTIONS,
    maxHitsPerRay: 32,
    requireKnownExteriorEndpointForParity: true,
    allowVerifiedLocalSurfaceSign: true,
  });

  if (!evidence.complete || evidence.hasContradictions)
    return unknown("insufficient geometric evidence");

  // Agreement is conditional on verified coverage/orientation assumptions.
  const occupancy = resolveEvidence(evidence);

  if (occupancy !== "INSIDE")
    return { occupancy };

  return {
    occupancy,
    escape: planWholeEnvelopeEscape(snapshot),
  };
}
```

A locally verified terrain patch may establish the occupied side of a road without requiring rays to traverse the whole Earth. Its validity is restricted to that patch's supported neighborhood.

Classification jobs are bound to a geometry snapshot. Before acting, revalidate the current pose, support identity, and correction corridor. Results for an earlier pose cannot directly authorize a later relocation.

### Continuous collision

Retain existing probe sweeps initially, but identify them as an approximation.

The complete design adds conservative swept capsules or another small finite envelope for:

- drone bodies and arms;
- aircraft fuselages;
- wings and tail;
- deployed gear where necessary.

Include rotation in the swept bound. A rotating wing can hit a pole while the CG barely moves.

A 60 m/s aircraft travels 0.5 m per step at 120 Hz. Collision queries must cover the entire motion, not just test endpoints. Thin geometry is primarily a continuous-collision problem, not an occupancy-voting problem.

## 3. Support, depth, and recovery

### Selecting support

Use local downward queries from the aircraft's current free-space layer.

For each active contact:

1. Start a downward ray just above the contact, using a small numerical skin.
2. Select a support-compatible surface reached from that layer.
3. Reject a deck underside as support.
4. Preserve the previous support patch through small contact compression and validated refinement.

Do not raise every ray origin by T. That would reintroduce low ceilings into the support selection.

A slightly embedded contact cannot find its road by casting farther downward. Recover that support from its previous patch or a separately validated local bracket. The bracket must identify the same support layer; an arbitrary overhead surface is insufficient.

The centre query helps establish the local layer and provides fallback information. It must not impose a topmost-surface lower bound on the floor.

### Reducing contacts to one horizontal plane

Let:

- z_i be contact i's vertical offset from the CG;
- z_min = min(z_i);
- h_i be the selected support height beneath contact i.

The existing conservative reduction can be expressed as:

```text
F_target = max_i(h_i + z_min - z_i).
```

This makes the lowest contact's plane penetration at least as large as each participating contact's penetration into its own support.

Only eligible contact supports participate. A cliff face or overhead roof does not become support because it intersects a contact footprint.

The single plane remains an approximation:

- It cannot provide independent wheel heights or a tilted normal.
- It may support one part of a large aircraft too early on slopes.
- Mixed road/deck contact cannot be represented exactly.

Measure the discrepancy between actual contact clearances and plane clearances. Keep steep faces in the 3D collision system rather than turning them into a horizontal floor.

### Transitions

- **Under a deck:** local contact rays reach the road; the deck is ceiling/collision geometry.
- **Landing on a deck:** approaching from above selects the deck top.
- **Flying off a deck:** support changes to the lower surface after the contact footprint leaves the deck. Gravity provides descent.
- **Flying into the side of a deck:** resolve an impact; do not lift onto it.
- **Slopes:** refresh footprints using travel, rotation, and patch boundaries.
- **Large aircraft:** two lazy footprint refreshes alone are insufficient at speed. Reuse valid patches and sweep the footprint/envelope across their boundaries.

### Defining depth

Distinguish two quantities:

```text
d_min = inf { length(t) : B(q) + t is clear }
d_plan = length of the actual proposed recovery path
```

For upward recovery, with local up direction u:

```text
d_up = inf { s >= 0 : B(q) + s*u is clear }.
```

The candidate path must also avoid entering additional obstacles. Being clear eventually above a bridge is not enough.

A shallow wall intersection can have d_min = 0.1 m but require 20 m of upward travel. It must not receive an automatic 20 m climb.

Apply the threshold to the correction actually executed, including clearance margin and accumulated correction:

- Automatic recovery requires a trusted path whose upper displacement bound is at most T.
- A confidently deep overlap permits the underground card.
- An uncertainty interval straddling the threshold permits neither an automatic move nor a confident deep verdict.

Default T = 1.0 m for all aircraft. Keep it tunable. Introduce size scaling only after evidence supports a specific policy.

### Recovery eligibility

Automatic recovery is for:

- validated shallow initial overlap;
- matched support refinement;
- small numerical penetration after ordinary contact resolution.

It is not a mechanism for turning a high-speed collision into flight through a hill. Such crossings remain impacts or existing crash/fault behavior.

Open a recovery episode with a fixed source geometry, target, and correction budget. Do not repeatedly rebase the budget after small movements. Otherwise a 10 m change could be laundered into ten unconfirmed 1 m recoveries.

### Floating without exciting the springs

Maintain three heights:

- observed support;
- validated target support H;
- effective JSBSim floor F.

For a standing aircraft, move altitude A and effective floor upward together:

```ts
const dz = boundedRecoveryStep({
  remainingMeters: H - F,
  dt,
  maxSpeedMps: 0.25,
  maxAccelerationMps2: 1.0,
});

writeAltitude(A + dz);
writeTerrainFloor(F + dz);
recordRecoveryCorrection(dz);
```

The contact compression remains unchanged:

```text
(F + dz) - (A + dz + z_i) = F - (A + z_i).
```

Never install H first and let the springs lift the aircraft.

For a nonstanding embedded placement:

1. Establish a temporary floor at or below the active contacts, with safe native penetration.
2. If necessary, use one captured-state IC restoration to remove inward vertical velocity and stale integrator history.
3. Advance altitude and effective floor together toward the validated support.
4. Finish using ordinary damped contact.

The existing loop rejects native ground penetration over 0.5 m before JSBSim runs. A 1 m recovery must therefore keep the effective floor shallow throughout; the threshold does not relax that guard.

Routine standing refinements must not call `runIc()` each tick.

### Controls and history

Each correction must:

- rebase hull sweep history so artificial motion is not treated as a flight collision;
- rebase interpolation;
- shift active drone altitude-hold anchors by the same accepted correction;
- preserve pilot inputs, horizontal motion, engine state, fuel, and simulation time.

Use the existing per-step `"reset"` result to rebase interpolation while continuing physics. Do not reset the whole fixed-step loop each recovery tick and discard its accumulated time.

If the pilot commands downward motion, ordinary unilateral contact should oppose penetration. Do not repeatedly erase velocity or renew the recovery budget. Excessive inward motion follows collision/fault policy.

Paired writes prevent a floor-height jump from adding spring energy. They do not prove that every aircraft and control configuration is bounce-free; that requires real JSBSim validation.

### Walls and cliffs

The ideal escape direction is often lateral near a wall. Altitude writes cannot execute it.

Under the current interface, smooth lateral correction would require repeated IC restorations. That has material cost and state-continuity risks.

For the first implementation:

- resolve lateral penetration through the existing rigid-body collision path;
- allow automatic floating only along a validated upward route within T;
- do not substitute an excessive climb for a short lateral exit;
- retain unresolved cases without movement and identify the overlap accurately.

General smooth lateral recovery is a separate extension requiring either a validated IC-based controller or a richer pose/velocity correction interface. The current primitives do not support promising arbitrary-direction smooth recovery at the cost of a few altitude writes.

### Per-step loop

```ts
function physicsTick(dt: number): StepResult {
  surface.beginFrame?.();

  try {
    const pose = readAircraftPose();
    const local = neighborhood.observe(pose);

    // Includes actual flight motion; correction history is rebased separately.
    resolveSweptCollisions(previousAcceptedPose, pose, local);

    if (local.changed || certificates.expiredFor(pose))
      classification.schedule(local.snapshot(), pose);

    classification.advance({
      maxRays: 2,
      maxTriangleTests: 4096,
    });

    const support = supportTracker.update({
      pose,
      local,
      maxOrdinaryFootprintQueries: 2,
    });

    const verdict = classification.resultValidFor(pose, local);
    const action = recoveryPolicy.evaluate({
      pose, support, verdict, episode, dt,
    });

    if (action.kind === "float") {
      const correction = applyPairedAltitudeAndFloorShift(action, dt);
      rebaseCollisionAndControllerHistory(correction);
      return "reset"; // Native physics still advances.
    }

    if (action.kind === "deep-overlap") {
      publishExactRelocationProposal(action);
      preserveStandingFloorOrWithholdUnsafeNewSupport();
      return standingOnKeptFloor() ? true : false;
    }

    applySelectedOrHistoricalSafeFloor(support);
    return true;
  } finally {
    surface.endFrame?.();
  }
}
```

Classification is advisory until current-pose validation succeeds. A missing classification result is not permission to install a high floor.

## 4. Streaming behavior, cost, and validation

### Map and consent state transitions

| Event | Behavior |
|---|---|
| Centimetre refinement under a parked aircraft | Match the existing support patch, validate the short route, and float silently. Preserve compression and physics stepping. |
| Refinement below a drone hovering 5 cm above ground | No aircraft correction unless actual permitted clearance is exceeded. Never lift merely because the map changed. |
| Coarse tile fills a previously open underpass | Quarantine the new containment. Keep trusted local support and request detail. No underground card or lift. |
| Fine geometry opens the passage | Adopt the coherent finer representation and retire obsolete evidence. No relocation. |
| Dropout | Preserve standing support at its anchored footprint. Mark occupancy unknown. |
| Regain with another surface | Treat as new evidence, not authoritative placement after a fixed timeout. Match layers or wait. |
| First placement | Validate the intended pose/layer. Resolve shallow confirmed overlap; propose a deep correction with consent. Initial preparation may wait for coverage. |
| Downward support refinement | No underground question or automatic downward teleport. Update support safely and allow ordinary contact/gravity to settle the aircraft. |
| Unrelated revision elsewhere | Keep local certificates. |

A historical floor is a temporary support constraint, not current map evidence. Retain it for a stationary standing aircraft, but do not carry an old flat floor across new geography. Its spatial validity ends when the relevant footprint leaves its supported neighborhood.

For a missing resident patch, use a bounded safety neighborhood around the last supported footprint. Continued stationary support may remain while detail is unavailable; moving beyond it requires fresh support evidence.

Waiting should end when evidence improves, not merely when a timer expires.

### Other ground consumers

Provide one shared aircraft-environment result containing:

- selected support below the aircraft;
- physical support status;
- optional overhead clearance;
- occupancy and confidence;
- optional validated recovery destination.

Use it for contact, drone AGL, landing/disarm decisions, and HUD AGL.

Do not expose the temporary JSBSim floor as measured map height. When only historical support remains, distinguish that from a current measurement; unknown AGL must not trigger disarming.

Teleport targets come from validated whole-aircraft free configurations. A topmost sample may help search, but cannot certify the destination by itself.

Spawn has two explicit meanings:

- An altitude-specified spawn chooses the free-space layer around that altitude.
- A requested surface spawn chooses its prepared surface; topmost placement is appropriate only when that is the intended operation.

For a deep overlap, preserve the current two actions and the separate Yes/No confirmation. Bind confirmation to the shown absolute destination, altitude, reason, and local geometry generation. Revalidate before committing; changed destinations require fresh consent.

After Resume, suppress only the containing overlap episode/geometry. Preserve unrelated collisions and valid support below the aircraft. Do not turn a local waiver into global terrain-collision disablement.

### Cost at 120 Hz

Let P be existing point probes and K the eventual finite-envelope primitives.

| Work | Ordinary tick | Invalidation/recovery work |
|---|---:|---:|
| Centre support query | 1 local downward ray | Reuse within tick |
| Contact support | Up to 2 refreshed rays | Extra urgent validation when certificates expire |
| Existing hull collision | Up to P swept segments | Every moving tick |
| Occupancy classification | 0 when valid | Up to 2 all-hit rays/tick |
| Finite-envelope collision | Up to K BVH shape traversals | Replaces/complements sparse probes |
| Recovery clearance | None normally | Whole-envelope corridor validation |
| BVH construction | None | Resident geometry arrival/change, prepared off-page |

A full 12-direction classification costs up to 12W rays for W required witnesses. At two rays per tick it can span 6W ticks. Keep that latency out of normal flight and do not move the aircraft while waiting.

Start with:

- 32 crossings per classification ray;
- at most two classification rays per tick;
- at most 4,096 classification triangle tests per tick;
- `UNKNOWN` on budget exhaustion;
- no per-tick geometry allocations.

Those are initial work limits, not measured device-performance claims.

With a BVH, report visited nodes and candidate triangles. "One ray" is not constant cost. Brute-force fallback can scan every triangle in every intersected mesh; adding classification rays to that path can be expensive.

Do not silently truncate required collision queries and call the result clear. Optional occupancy work can stop at its budget. Collision overload needs an explicit degraded/fault policy and measurement; no algorithm offers both exact arbitrary-soup collision and a universal fixed phone-time bound.

Use an initial contact/query performance goal of p95 below 0.3 ms and p99 below 0.75 ms per 120 Hz tick on target devices. These are acceptance targets to measure, not predicted results.

### Failure-mode table

| Scenario | Algorithm behavior | User sees |
|---|---|---|
| Passing under a bridge | Road support; overhead deck excluded | Normal flight |
| Landing under a bridge | Road supports contacts | Normal landing |
| Landing on a bridge | Deck top supports contacts | Normal landing |
| Flying onto a deck from the side | Swept impact, not floor adoption | Collision |
| Flying off a deck | Lower support selected after edge crossing | Normal descent |
| Shallow validated road overlap | Bounded paired recovery | Smooth silent rise |
| Deep confirmed overlap | Safe floor handling and proposal | Underground card; no unconfirmed move |
| Inside building blob | Confirm represented solid; search escape | Float if valid and within T, otherwise card |
| Open tunnel/underpass | Preserve connected passage | Normal flight |
| Sealed tunnel mouth/curtain | No invented free passage or upward rescue | Collision or unresolved geometry status |
| Tile crack | Preserve local support continuity | No fall or question |
| Parent/child overlap | One coherent representation or unknown | No duplicate-solid verdict |
| Half-second dropout | Keep anchored standing support | Flight continues; no regain card |
| Fast jet | Sweep full translation and rotation | Impact rather than tunnelling, within envelope fidelity |
| Drone hovering 5 cm above ground | Compare actual clearance | No question; lift only if required |
| Tree canopy | Closed blob occupied; represented opening free | Geometry-consistent contact |
| Pole or railing | Finite-envelope sweep | Collision; sparse-probe stage remains limited |
| Steep cliff/building wall | Lateral collision; no long automatic climb | Impact or unresolved overlap |
| Pilot pushes down during recovery | Normal contact; fixed episode budget | No repeated upward ratchet |

### Headless tests and pass criteria

Extend the existing real JSBSim harness and scripted geometry queries. New no-question tests must instantiate raw `createTerrainContact`, without an automatic-consent wrapper.

**Bridge and layer tests**

- Road at 0 m, deck top at 9 m, aircraft below it: road remains support; zero proposals and recovery corrections.
- Land below and above the deck: correct support and contact state.
- Cross deck edges in both directions: no ceiling adoption or premature underside support.
- Low ceiling within T: a shallow road recovery cannot pass through it.
- Partially sunk contact: recover its prior road patch without selecting a lower layer or overhead deck.

**Recovery tests**

- Confirmed upward corrections of 0.01, 0.05, 0.5, and 1.0 m: silent recovery within the configured speed/acceleration limits.
- Correction above 1.0 m: no automatic movement; exact proposal required.
- Repeated revisions totaling more than 1.0 m: cannot renew the same episode's budget.
- Parked aircraft: every accepted tick retains a safe floor; no relocation-induced physics hold.
- Native velocity remains unchanged by paired writes, apart from an explicitly tested entry projection.
- Engine running state, RPM, fuel, controls, and simulation time remain continuous.
- Compression and upward velocity remain within existing aircraft-specific nonlaunch bounds.
- Held downward input: bounded contact response, no repeated IC resets or unlimited recovery.

**Streaming and classification tests**

- Parent closed slab plus finer open bridge, in several load orders.
- Partial child coverage: no wholesale parent removal.
- Cracks, contradictory winding, tangent rays, incomplete rays, and hit-buffer overflow.
- Dropout followed by identical road, raised road, or overhead deck.
- Unrelated revision churn.
- Unknown and conflicting cases produce neither an automatic move nor a confident underground card.

**Collision and consumer tests**

- Thin obstacle crossed between tick endpoints.
- Wingtip rotation into an obstacle.
- Drone body collision despite no legacy probe.
- Controller AGL, HUD AGL, and disarm behavior below a deck.
- Artificial correction does not create a swept impact.
- Altitude-hold anchors do not command descent after recovery.
- Resume preserves unrelated obstacle collisions.
- Destination change invalidates prior consent.

**Parity and rates**

Run representative cases at supported physics rates using elapsed-time policies. Run drone cases at their supported rate.

Give TypeScript and Rust identical ordered inputs, tie-breaking, completion flags, and failure behavior. Test translation, rotation, reflected transforms, large ECEF coordinates, and mesh-generation changes. Compare decisions exactly; use the project's established numerical parity rules for floating-point outputs.

## 5. Migration, assumptions, and first measurements

### Stage 1: smallest safe bridge and nuisance-card fix

1. Introduce shared local support queries and replace both centre and footprint sky sampling.
2. Expose raw triangle normals separately so undersides can be excluded correctly.
3. Update overlap-clearance checks and aircraft ground consumers together.
4. Remove "revision changed by more than 0.00001 m" as sufficient grounds for a relocation proposal.
5. Preserve standing support during unresolved changes.
6. Before enabling automatic lifts, add a conservative whole-body recovery-corridor check using a simple aircraft envelope. Exempt only the validated original penetration; reject newly encountered ceilings or obstacles. Then add bounded paired altitude/floor recovery for validated shallow refinements of an existing support patch.
7. Preserve deep-overlap consent and known-obstacle collision behavior.

This fixes ordinary represented bridges and the centimetre card. It does not claim to solve curtains or arbitrary solid reconstruction.

Do not replace the old trigger with only a 1 m cutoff on topmost height difference. That would retain the bridge bug and could silently lift aircraft toward ceilings.

### Stage 2: streaming-aware occupancy

Add coherent LOD selection, local generations, all-hit queries, three-state classification, and correction-episode accounting. Replace fixed missing-step counts with elapsed seconds.

Make the distinction between matched support movement and new containment explicit.

### Stage 3: complete collision envelope

Add drone and aircraft swept envelopes and footprint-boundary checks. Improve the conservative recovery envelope introduced in Stage 1 with model-specific geometry. Retire assumptions that sparse points establish body clearance.

Extend the existing TypeScript/Rust kernels rather than introducing another geometry engine.

### Stage 4: optional stronger reconstruction

Evaluate fast winding numbers or local volumetric segmentation only if measured unresolved cases justify their preparation and memory cost.

Keep these structures in memory, tied to resident mesh generations. Dispose them with their source geometry. Do not persist geometry-derived indices, occupancy fields, or replay bundles containing Google geometry.

### Explicit assumptions and defaults

- T = 1.0 m; no initial aircraft-size scaling.
- Upward recovery speed is 0.25 m/s with a 1.0 m/s² correction ramp.
- Ordinary contact tolerances and suspension compression are distinct from T.
- Aircraft geometry is represented by declared collision envelopes, not rendering triangles.
- Heights are converted consistently through the existing flight/map coordinate bridge. Do not mix ellipsoidal heights with a different ASL datum without an explicit conversion.
- Unknown topology can remain unknown indefinitely.
- A sufficiently detailed curtain can still be observationally indistinguishable from a wall.
- General smooth lateral recovery is outside the first implementation's guarantees.
- No simulator implementation or new real-app reproduction was performed for this proposal.

### Measure first

Before broad implementation, collect short diagnostic runs showing:

1. Raw winding consistency after all transforms, including TypeScript/Rust agreement.
2. Local revision frequency versus global revision frequency.
3. Parent/child replacement coverage around representative bridges.
4. Query rays, BVH nodes, triangle tests, allocations, and worst ticks by query purpose.
5. Effective floor, target support, contact compression, and correction ledger during refinement.
6. Hold-controller behavior and IC restoration count during recovery.
7. Real curtain prevalence and finite-envelope cost on the laptop and a phone.

The principal acceptance result is concrete: an aircraft can fly and land below a represented bridge, land on its deck, and silently follow a small validated support refinement without a floor jump, launch, freeze, or relocation question.
