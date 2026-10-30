# Design brief: telling "inside the ground" from "outside" in a flight simulator that flies over streamed photogrammetry

You are designing an algorithm. Think from first principles and be rigorous. I want the ideal definition first, then a real-time version, then the path from today's code to it. Where you must assume something, state the assumption and keep going rather than stopping to ask.

## The product

0SFS is a browser flight simulator (TypeScript, Babylon.js). It flies a DJI Mini 2 and Air 2S drone, a Cessna 172, an SF-50 jet and an F-35B over the real world. The world is either:

- **Google Photorealistic 3D Tiles**: streamed glTF photogrammetry meshes in a 3D Tiles hierarchy. They are not watertight and are full of cracks between tiles and noisy at the centimetre level. Bridges, trees and buildings are part of the same mesh. Coarse tiles stay drawn while finer ones load, and tile bounds overlap, so for a while two levels of detail can coexist at different heights.
- **Raster terrain**: a heightfield with no overhangs.

Physics is JSBSim compiled to WASM, stepped at a fixed 120 Hz in the browser. Targets are a fanless laptop and phones, so per-step cost matters.

## What the user reports

1. After landing a drone, a card still appears for no good reason: "Terrain overlaps your aircraft. The current map suggests moving you 0.0 m (surface refinement). Your position has been kept." It offers **Resume flight here** and **Teleport above ground**.
2. They want a new rule:
   - If the aircraft is up to a threshold underground, it floats up automatically.
   - If it is deeper than the threshold (say 1 m), it gets the "you are underground" card.
3. Flying under a bridge, the simulator thinks the aircraft is underground.
4. In their words: "I wish we could magically segment 'underground' as the stuff inside the meshes and 'outside' as the stuff outside." How would that work, and how could it run in real time?

## How the system works today

### Physics only knows one floor

JSBSim has exactly one notion of terrain: a single horizontal plane at `position/terrain-elevation-asl-ft`.

- **Contact force:** each contact point (wheels; skids or leg tips; structure points at nose, belly, tail and wingtips) gets a spring/damper force proportional to its penetration below that plane.
- **Contact state:** each contact's state is readable (`contact/unit[i]/WOW`, positions in body inches).
- **No per-wheel heights:** there is no per-wheel ground and no ground callback, and the plane cannot tilt.
- **Moving the aircraft:** the only ways to move it nonphysically are writing its altitude directly (`position/h-sl-ft`; velocities and integrator history carry on) or re-running initial conditions (which resets integrator history).
- **What a deep penetration does:** a floor set metres above the contacts makes the springs fire with huge force; a few metres of penetration is a launch.

So whatever "inside" means in 3D, the final output each step must be **one floor height** (plus optionally an altitude correction). Any other effect must come from our own impulses: the hull-probe system below applies rigid-body impacts through re-initialisation.

### The map query (all we can ask the map)

The map exposes this interface. Coordinates are ECEF metres, and heights are WGS84 ellipsoidal.

```ts
interface SurfaceHit {
  point: Ecef; normal: Ecef;          // triangle face normal (from vertex winding)
  distanceMeters: number; heightMeters: number;
  meshId: string; revision: number;  // revision changes whenever queryable geometry changes
  quality: number; geometricErrorMeters?: number; // Google tile's advertised geometric error
}
interface SurfaceQuery {
  raycast(origin: Ecef, direction: Ecef, lengthMeters: number): SurfaceHit | null; // nearest hit only
  sample(latDeg: number, lonDeg: number): SurfaceHit | null;
  revision(): number;
  beginFrame(): void; endFrame(): void; // one physics tick may reuse transforms and mesh bounds
}
```

How these behave:

- **`raycast`:**
  - Babylon picking: a world-bounds broadphase per mesh, then a triangle scan. An optional acceleration structure may replace the triangle scan.
  - Only enabled, visible map meshes count; an aircraft-local loading region keeps tiles near the aircraft resident.
  - It returns only the nearest hit. Triangle tests are two-sided as far as we know, and the normal comes from vertex winding, not flipped toward the ray.
  - Whether winding is reliably "outward" for Google tiles after the glTF → Babylon → ECEF transforms is unverified. Treat it as unverified, or design for both cases.
- **`sample(lat, lon)`:**
  - Casts straight **down from 20 km above the ellipsoid**, so it returns the **topmost** drawn surface.
  - LOD rule: if the hit belongs to a coarse Google tile and a finer tile's surface lies within the coarse tile's geometric error below it, the finer one wins. Farther below, the upper surface "is a feature of its own, such as a bridge, and stays the ground."
  - Raster mode answers from decoded heightfield data instead.
- **Possible extensions:** we can add query primitives if the design needs them, for example all hits along a ray with entry/exit sign, or triangles near a point. Say what you need and why; the cost is a CPU triangle/BVH query against the resident tile meshes.

### The contact layer (`terrainContact.update()`, once per physics step)

1. **Sample under the aircraft:**
   - Call `sample(lat, lon)` under the centre of gravity.
   - Reject samples more than 1 km above the aircraft or 1 km above the previous sample.
   - Smooth the sample with a wheel filter: low-pass over distance travelled, with steps taller than the wheel passing through.
2. **Sample under each contact:** within 10 m of the ground, also `sample` under each contact point at its footprint. On Google tiles this is limited to 2 contact samples per step, refreshed lazily.
3. **Choose the floor:** build one horizontal "support plane": the lowest plane at which no contact is deeper in its own ground than the lowest contact is in the plane. The ground under the centre stays a floor under the plane. Ground more than 1 m above where a structure contact would rest is not support; that case is left to hull collision.
4. **Write the floor** to JSBSim.
5. **Detect map changes:** when the map revision changes, the same coordinate is re-sampled to measure how much the world moved there (the "refinement").
6. **Decide whether to propose a relocation.** Relocations are always proposals the pilot answers. There are three causes:
   - **placement**: first frame at a new location, aircraft below its support.
   - **surface regained above the aircraft**: after 0.5 s without any surface (60 missing samples), the first sample is above the aircraft.
   - **surface refinement**: a revision moved the ground under an aircraft that is **standing**, or raised it through a level aircraft's lowest contact. Standing means its lowest contact is within 2 cm of the floor or pressed into it, with the centre of gravity above the floor. The proposal is created for **any** change larger than 0.00001 m, which is why the user sees "moving you 0.0 m": photogrammetry refines by centimetres, and the card prints one decimal.
7. **While a proposal is pending:**
   - A standing aircraft keeps its floor and keeps flying.
   - Any other aircraft is held: physics stops stepping and its floor is withheld (moved 10 km down).
8. **The pilot's answer:**
   - **Resume flight here** keeps the position. Contact with the "containing" world stays suppressed until either the aircraft's whole footprint is clear above the surface or the surface comes back down to the floor it had.
   - **Teleport above ground** asks a second Yes/No, then moves the aircraft.
9. **Flying into a hill:** an aircraft that flies into a hill in full view of the surface is deliberately not lifted out, because lifting it would fly it through terrain. That ends in a physics fault.

### Hull collision (planes only)

The C172, SF-50 and F-35B carry a few body probe points. Each step, the segment each point swept since the last step is raycast against the drawn mesh. A crossing is answered as a rigid-body impact (restitution, Coulomb friction, through the mass and inertia tensor), and the aircraft is pushed out along the hit normal. The probe is treated as hitting from outside: the normal is faced against the probe's motion. Drones have no probes.

### Other consumers of "ground height here"

All of these use the topmost `sample`, so all of them are wrong under a bridge:

- the drone controller (above-ground altitude for hold and landing/disarm decisions)
- the HUD's above-ground readout
- the overlap card's "Teleport above ground" target
- spawn placement

### Why a bridge breaks it

Picture a drone at 3 m above a road, passing under a deck whose top is 9 m above the road:

1. `sample()` returns the deck top. That is the "ground", 6 m above the aircraft.
2. The support plane keeps the ground under the centre as a floor, so JSBSim's floor jumps above the drone.
3. Its springs see 6 m of penetration and fire hard.
4. If a revision change or a tile dropout happens while under the deck, the relocation logic sees an aircraft 6 m "below the surface" and puts up the underground card.

This mechanism was derived from the code; it was not run as a headless reproduction. Either way, the root cause is that **"the ground" is defined as the first surface seen from the sky, not the surface below the aircraft**, and nothing in the system knows whether a point is inside solid matter.

## Requirements for the new design

1. **Under is not inside:** under a bridge, an overpass, an awning, a tree canopy's open underside or a parking deck, the aircraft is **outside**:
   - It flies normally.
   - Its floor is the surface below it (the road), not the one overhead.
   - Nothing proposes moving it.
2. **Above:** on top of the deck, the deck is the floor; landing on a bridge works.
3. **Sunk in slightly:** when the aircraft is inside solid matter by at most a threshold T (tunable, about 1 m; possibly scaled by aircraft size), it **floats out automatically** with no question:
   - It rises smoothly (velocity-limited) to stand on or clear the surface, without launching or bouncing.
   - The user treats this as ordinary contact response, which our consent rules exempt from confirmation.
4. **Sunk in deeply:** deeper than T, show the underground card (Resume flight here / Teleport above ground) as today. Every nonphysical move above the threshold still needs a fresh Yes/No showing destination, altitude and reason.
5. **Small map changes are silent:** centimetre-level refinements under a parked aircraft never produce a question.
6. **Standing aircraft keep moving:** physics never freezes for a standing aircraft, and a parked aircraft is never left without a floor.
7. **No false "inside" from map changes:** map changes must not create false verdicts that move the aircraft. Watch for:
   - a coarse tile that fills the space under a bridge until the finer one arrives
   - LOD overlap
   - tile cracks
   - a half-second dropout followed by a different surface
8. **Both worlds:** it must work for Google tiles and raster terrain (raster is a heightfield, so the simple answer is probably right there).
9. **Real time:** a per-step budget at 120 Hz on a fanless laptop or a phone. Today we spend roughly one centre sample plus up to two contact samples per step near the ground, plus the planes' probe sweeps. Give cost estimates in rays or triangle tests per step, and say what may be amortised across steps or recomputed only when the revision changes.
10. **Kernels in TypeScript and Rust:** numeric kernels must exist as a TypeScript reference with a Rust/WASM twin, so prefer algorithms that are simple to state exactly.
11. **No persistent derived data:** assume we may not persistently cache or store data derived from Google tiles. In-memory, per-session structures built from the currently resident meshes are fine.

## What I want from you

1. **Definition.** What should "inside" mean for a non-watertight, multi-LOD, streaming triangle soup plus a heightfield? Give a principled definition that matches intuition in these cases:
   - under a bridge: outside
   - sunk into a road: inside
   - inside a building's photogrammetry blob: inside
   - in a tree canopy: decide and justify
   - in a tunnel mouth or underpass
   - in a crack between tiles

   Discuss where any definition is inherently ambiguous and how the system should behave there. Candidates include generalized winding number, ray-parity voting, reachability from the sky through free space, and front/back-face of the first hit; use any of them or something better.
2. **The ideal algorithm,** ignoring cost.
3. **A real-time algorithm.**
   - What runs every step, and what runs only when the map revision changes or the aircraft moves far.
   - How to use temporal coherence: the aircraft starts outside, and it can only become inside if its own motion crosses a surface or the map changes around it.
   - How to compute the **depth** that the threshold rule needs, and the exit direction (usually up, but not always: consider walls and cliffs).
4. **Choosing the floor.** Which surface becomes JSBSim's single floor each step (for example, the first surface below the contacts, cast from just above the aircraft rather than from the sky). Cover transitions:
   - flying onto and off a deck
   - passing under one
   - slopes
   - an aircraft as small as a drone versus one as large as a jet
5. **Floating out.** How to float the aircraft out using only what JSBSim allows (floor height, direct altitude writes, initial conditions) without fighting its springs or freezing it. Also what happens if the pilot pushes back down while floating.
6. **Map changes.** Handle:
   - a refinement under a parked aircraft
   - an LOD swap that temporarily fills or opens the space under a bridge
   - a tile dropout and regain
   - first placement at a new location

   When should the system wait for more detail before deciding?
7. **Failure-mode table.** Each scenario above, plus:
   - landing under a bridge
   - landing on a bridge
   - a fast jet at low level (0.5 m per step at 60 m/s)
   - a drone hovering 5 cm over the ground while a tile refines
   - photogrammetry "curtains" that fill the space under some bridges
   - thin geometry (poles, railings)
   - steep cliffs and building walls

   For each: what your algorithm does, and what the user sees.
8. **Tests.** We have a headless harness: real JSBSim plus a mock `SurfaceQuery` whose geometry we script (planes, slabs, bridges, refinements, dropouts). List scenarios with pass/fail criteria.
9. **Migration path from the current code.** What is the smallest safe first step that fixes bridges and the 0.0 m question, and what comes after?

Format: a design document with:

- exact definitions
- TypeScript-like pseudocode for the per-step loop and the reclassification step
- cost per step
- explicit assumptions
- risks and what to measure first in the real app

Starting points, not requirements:

- generalized winding numbers (Jacobson et al. 2013) and fast winding numbers (Barill et al. 2018)
- ray-parity voting (Nooruddin and Turk 2003)
- angle-weighted pseudo-normals for sign (Bærentzen and Aanæs 2005)
- visibility-based inside/outside labelling from surface reconstruction (free space is what a sensor saw through; Labatut et al. 2009)
- swept continuous collision and depenetration ("stuck inside geometry" recovery) as game character controllers do it against two-sided triangle soups
