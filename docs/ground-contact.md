# Ground Contact

How the simulator decides where the ground is, what it does when it cannot tell,
and why the aircraft stands where it does.

For collision-query costs, the current optimizations, GPU/BVH options and the
profiling procedure, see [Near-ground CPU performance](https://github.com/0SFS/research/blob/main/near-ground-performance.md).

## The problem

JSBSim computes gear forces against a ground plane it is told about through one
property, `position/terrain-elevation-asl-ft`. The map, meanwhile, publishes a
surface made of whatever geometry is currently drawn — raster terrain tiles or
Google photorealistic 3D tiles. Connecting the two is not a matter of copying a
number across once per frame, because the map's answer changes for reasons that
have nothing to do with the aircraft: tiles refine, tiles unload, and
photogrammetry is noisy.

Everything in `src/flight/physics/terrainContact.ts` exists to make that
connection safe. The rules below were each written in response to a specific
failure, and the tests next to them pin those failures down.

## Where the aircraft stands

Dropped onto a known elevation and left to settle, the C172 flight model rests
with its reference point **1.33 m** above the ground, at a 2.5° nose-high stance
with 4–7 cm of oleo compression. That number is measured from the model, not
chosen, and it appears as `STATIC_STANCE_METERS` in `safeFlightState.ts`.

Everything that places the aircraft on the ground has to use it:

- `aircraftClearanceMeters()` returns exactly that stance when level, growing
  only for attitudes the gear does not hold the aircraft at. Its pitch term
  measures deviation *from the static stance*, not from level — absolute pitch
  adds 19 cm of float to an aircraft sitting on its own wheels.
- The visual model is dropped by the same distance (`modelOffset` in
  `aircraftCatalog.ts`), because the exported meshes put their origin on the
  ground between the wheels.

Getting this wrong is quietly awful. A placement target a few centimetres above
the settled stance leaves the gear extended; the aircraft sinks back onto its
springs; the next placement lifts it again. It reads as sinking into quicksand
and being teleported out.

## The gear model, and what it does not include

JSBSim's contacts are not naive point collisions. Measured from a rolling
takeoff run, each main gear reports:

| Modelled | Evidence at 41 kt |
| --- | --- |
| Rolling axis | `wheel-speed-fps` tracks ground speed |
| Slip angle | `slip-angle-deg` non-zero and responsive |
| Slip-dependent side force | `side_friction_coeff` varies with slip angle |
| Anisotropy | rolling friction 0.02 against static 0.8 |
| Oleo | spring 5400 lbs/ft, damper 1600, with compression velocity |
| Steering and brakes | 10° nose steer, left/right brake groups |

What is genuinely missing is **wheel angular state**. There is no wheel
rotational inertia, no spin-up drag at touchdown, no brake torque fighting that
inertia, and no lock-up or skid. Braking is an instantaneous friction
multiplier. `wheel-omega-radsec`, `wheel-slip-norm` and `brake-torque` do not
exist in this build.

Per-wheel ground heights are also out of reach. Every contact point queries one
ground callback, the WASM SDK binds no callback of ours, and the only terrain
property is that single scalar. Three wheels cannot be given three heights
without replacing the gear model outright.

## One plane for every contact

Under the centre of gravity alone, a sloped runway put the nose wheel or the
tail under the surface, and a hillside put a wingtip into it. Within 10 m of
the ground, `terrainContact.ts` samples the ground under every contact JSBSim
reports, wheels and structure, placed from its structural inches and the live
CG. The plane it gives JSBSim is the lowest at which no contact is deeper in
its own ground than JSBSim's lowest contact is in the plane. That contact's
spring then holds every other contact to at most its own compression.

- The ground under the centre stays a floor under the plane, so a sample that
  is deferred or missing never leaves the plane lower than before.
- Ground more than a metre above where a structure contact would rest is not
  support: a wingtip under an overhang or in a wall is for hull collision.
- Google samples are limited to two contacts a step, the most overdue first.
  A contact keeps its sample until it has moved at least 5 cm, and is
  confirmed lazily while the ground under the centre does not change.

A horizontal plane cannot pitch the aircraft onto a slope: the uphill contact
carries it and the others ride clear of their ground. That needs a ground per
contact in the native gear model.

## The ground a wheel rides on

Because the surface is sampled at a point and resampled every physics step,
photogrammetry noise of a few centimetres makes the whole ground plane twitch
under a parked or taxiing aircraft, and the oleos never stop ringing.

A real wheel does not follow that noise. It is a circle roughly 0.29 m in
radius, so it bridges anything shorter than its contact patch and climbs only
what it cannot bridge. `wheelGroundFilter.ts` reproduces that: the sampled
height is filtered over **distance travelled**, with the wheel radius as the
length constant.

- Distance, not time. A parked aircraft must not drift, and a fast taxi must not
  be over-smoothed.
- Steps taller than the wheel pass straight through. Those are kerbs and cliffs;
  smoothing one would drive the aircraft through it.
- It adds no raycasts. It filters the sample already being taken.

Raw samples still drive world-change detection. Only the question of where the
aircraft sits uses the ridden height.

## When the map cannot answer

`terrainContact.update()` returns `false` to mean "do not step at all". The
fixed-step loop then declines to integrate, which stops the aircraft dead
without raising a fault, so this is used as narrowly as possible.

| Situation | Behaviour |
| --- | --- |
| More than 500 m above the best known ground | Step normally. Terrain reaches the physics only through the gear; nothing up there can touch it. |
| Within gear range, no height ever established | Hold. JSBSim's terrain elevation is still unset, and stepping drops the aircraft toward a ground plane that is not there. |
| Within gear range, height established, Google tiles | Step. Google publishes only what is currently drawn, so single-frame misses are routine. |
| Within gear range, height established, raster | Hold. Raster has an independent coverage index; a miss means no surface is safe. |

Half a second of continuous misses counts as having flown somewhere the aircraft
could not see. The first sample found afterwards is trusted the way a fresh
placement is, because the aircraft may have crossed a hill it never saw and the
gear cannot resolve that penetration. One missed frame is not a blind stretch —
treating it as one repositioned parked aircraft off their own springs.

## Samples that are not believed

Two limits reject a sample outright, log it with the mesh id, revision and tile
quality, and carry the previous height for that frame:

- more than a kilometre above the aircraft, or
- more than a kilometre above the previous sample, taken half a metre earlier.

Neither is a refinement; both are a different piece of world. Only upward steps
are rejected — a surface that drops away cannot fire the gear springs, and
refusing one could strand the aircraft on a phantom floor.

This matters because of how the failure presents. JSBSim's gear force is
proportional to penetration depth, and its ground plane is horizontal. A surface
placed above a cruising aircraft fires all three springs straight up, and a few
hundred metres of penetration becomes tens of thousands of knots in one 120 Hz
step. The reported symptom is an airspeed in the tens of thousands with the
entire velocity vertical.

## Repositioning

The aircraft is put onto the surface through JSBSim's initial conditions for two
reasons only:

- **placement** — the first frame at a new location,
- **surface regained above the aircraft** — half a second without a surface
  ended with the aircraft under ground it never saw.

A refinement moves the aircraft by the change itself, in two cases:

- **surface lifted through the aircraft** — a refinement raised the ground into
  a level aircraft,
- **resting aircraft followed the ground** — a refinement moved the ground
  under an aircraft standing on it.

Only the altitude changes. Velocities, rates, oleo compression and the
integrators' history carry on, so a parked aircraft does not drop back onto its
springs each time a tile refines under it.

An aircraft that flew into a hill in full view of the surface is deep below it
too, and is deliberately left there: lifting it out would fly it through
terrain. That case ends in a physics fault, which is the intended outcome.

## Hull collision

JSBSim knows only its plane. Two things keep the airframe out of the rest of the
visible surface:

- **Structure contacts.** Every aircraft package declares `STRUCTURE` contacts
  at the nose, belly, tail and wingtips. A tailstrike, a wingtip scrape or a
  gear-up landing on the plane meets JSBSim's own spring, damper and friction,
  continuously. Gear up, the SF-50 rests on its belly contacts with its
  reference point 0.54 m above the ground and the F-35B 0.12 m, at no more than
  1.2 in and 1.5 in of compression.
- **Hull probes.** `visibleMeshCollision.ts` sweeps a few points per airframe
  (`*_BODY_COLLISION_PROBES` in `collisionGeometry.ts`) from one physics step to
  the next against the drawn surface: buildings, trees, and terrain off the
  plane. The points are in structural inches and placed with the live CG.
  Points on a broad surface are swept a few inches inside the hull, so on the
  plane the structure contacts meet it first.

A swept point that crosses the surface is answered as a rigid-body impact at
that point: restitution along the surface normal, Coulomb friction (μ 0.5)
across it, applied to both the velocity and the body rates through the
aircraft's mass and inertia tensor. A wingtip strike rolls the aircraft and
yaws it toward the drag. Restitution is 0.25 unless the caller supplies its
own, clamped to 0–2: the app uses 1.35 while arcade ground launches are on.
Up to 1 a hit cannot add kinetic energy; above it, the hit adds energy on
purpose. Several points struck in one step are resolved in the order they
crossed. The aircraft is then
moved out along the normal until the hull point clears the surface by a
centimetre.

The new state is written through JSBSim's initial conditions, the only way its
velocities and rates can be set, so an impact still resets the integrators'
history. Only position, velocity and rates change: controls, gear, engine and
the terrain plane are kept.

## Diagnosing a fault

The physics loop reports which guard tripped rather than that one did, and every
automatic state change leaves an entry in the event log (Debug tab, mirrored to
the console so it survives a reload).

A fault carries the contact state read *before* the restore rewinds it:
the terrain elevation the gear was given, height above it before and after the
step, and per-gear compression. The line that separates a gear-spring launch
from an aerodynamic divergence also appears on the centre-screen notice, because
that is the part someone reporting a problem can actually see.

Useful log entries when the ground behaves oddly:

- `First terrain height established` — the height, with coordinates.
- `Ignoring a surface sample …` — a rejected sample and the geometry it came from.
- `Flying without a measured surface` — the AGL at which the hold was skipped.
- `Repositioned onto the surface (…)` — which of the two causes fired.
- `Moved with a surface refinement` — a refinement of more than a metre moved
  the aircraft with it.
