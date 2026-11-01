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

A contact is a spring, and it rests pressed into the floor by its share of the
weight. Contacts stiff enough to hide that are not steady at the physics rate on
a drone of a few hundred grams, so the DJI drones' feet sit below the drawn feet
by that compression instead: 2.1 cm at the front and 1.8 cm at the back on the
Air 2S, 1.2 and 0.7 cm on the Mini 2 (their `dji-*.xml`). Once settled, the
drawn feet stand on the ground. The collision view (Debug → Collision geometry)
draws each contact where the physics has it, with markers sized to the aircraft
and a count of each kind, so a resting foot or other structural contact shows a
little below what is drawn. Wheel contacts add their native strut compression,
so a resting wheel's dot sits at its tread.

## What Collision geometry shows

Amber dots show the loaded JSBSim model's wheel contact patches, including
native suspension compression. They are not wheel centres. The bundled models
use body-Z struts; an external aircraft with an oriented strut needs its
compression axis exposed to draw that displacement correctly. Marker size is
only a visibility aid. Cyan markers show body sweep probes, purple markers
structural contacts, and white the centre of gravity.

Amber wireframe wheels and spokes attach to the actual aircraft wheel meshes.
Each outline is measured once, when the wheels are bound, from its mesh's own
vertices: a circle around the tire and a ring at each tire face. Vertices
nearer the axle than half the tire radius are hub and are left out. AF267's
F-35B hubs and axle stubs reach well past one sidewall, so the mesh's bounding
box would have put one ring beside the tire. A wheel without readable vertices
gets no outline rather than a guessed one. Parenting shares the wheels' spin,
gear movement, model scale and suspension transforms. They remain visible
through the aircraft so rotation is easy to inspect. They are diagnostic lines,
not an additional collision solver. The standalone C172 spin benchmark still
has its original estimated outlines; the flight app uses the mesh-backed view
for every aircraft instead of putting C172 coordinates on an F-35B.

Point contacts are the current ground-force model on every device, not a low-end
fallback. Wheel inertia maintains rotational state for presentation and feedback;
it does not replace these contacts with volumetric tires. Collision geometry
shows both because they answer different questions: where physics applies
contact forces, and where/how the drawn wheel rotates.

F-35B native bogey horizontal positions now follow the asset's extended axle and
tread centres. Moving only a debug marker would have hidden the disagreement;
the flight-model definitions themselves are corrected. Suspension presentation
follows the native compressed contact height rather than a fixed drawn pose.

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

Terrain contact uses the heights returned from the displayed surface directly.
There is no independently smoothed terrain surface or second collision mesh.
The centre's sample and each contact's sample enter the support calculation
without a height filter. Consequently, a rise, dip or changed surface while
parked is not hidden behind an older filtered height.

The existing query cache and contact sampling budgets still apply: matching
coordinates share a query within a tick, and a held contact sample is refreshed
according to travel and tile revision. Refinements are still measured at the
same coordinate before they move the aircraft, and missing or implausible hits
keep their existing guards. Removing height smoothing adds no rays, triangles
or terrain detail.

Raw photogrammetry detail can excite the suspension more than a smoothed height.
A short native C172/F-35B taxi fixture with centimetre-scale spatial noise stayed
finite with raw samples, but this is not a guarantee for every tile or runway.
The native solver still receives one horizontal support plane; exact mesh
sampling alone does not supply an independent ground normal or force per wheel,
and it does not correct an aircraft asset's suspension/contact alignment.

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
  a level aircraft, above its lowest contact,
- **resting aircraft followed the ground** — a refinement moved the ground
  under an aircraft standing on it.

Only the altitude changes. Velocities, rates, oleo compression and the
integrators' history carry on, so a parked aircraft does not drop back onto its
springs each time a tile refines under it.

Standing is judged by the contacts, not the stance: the lowest active contact is
within 2 cm of JSBSim's floor or pressed into it, with the centre of gravity
still above the floor. A drone hovering a few centimetres up to land is flying.
Placement is not proposed for an aircraft standing on the very ground it would
be placed on, whatever its nominal stance says.

Every repositioning is a proposal the caller confirms (`acceptRelocation`) or
declines (`declineRelocation`), unless it is small: a move of
`automaticMoveMeters` or less, up or down (fly's
`osfs.ground.automaticMoveMeters`, 1 m by default, 0 always asks), is applied at
once as an accepted one would be, and logged as `Moved onto the ground without
asking`. That limit is measured against the surface seen from above, so a
contact less than that under a bridge deck is lifted onto the deck
([underground and bridges](proposals/underground-and-bridges/README.md)).
An aircraft standing on its floor keeps that
floor and keeps stepping while asked; any other is held with support withheld.
Declining gives the aircraft back the floor it had (the one it stands on, or the
last one adopted under a flying aircraft) until it climbs clear of the surface or
the surface under its centre and every contact is no higher than that floor.
Only an aircraft without a trusted floor, such as a placement, flies on with
support withheld.

An aircraft that flew into a hill in full view of the surface is deep below it
too, and is deliberately left there: lifting it out would fly it through
terrain. That case ends in a physics fault, which is the intended outcome.

## After a hard impact

fly's step loop stops an aircraft whose lowest contact, by its attitude, is more
than 0.5 m under JSBSim's floor (`groundContactClearanceMeters`;
`lowestGroundContact` also names the contact). Clearing the fault alone does not
help: the aircraft is still that deep, and the next step stops on the same
impact. That was the Resume loop a pilot hit after stalling an F-35B into trees.
Terrain contact offers two ways on, and both stop the fall, which was the
impact, keeping the speed along the ground:

- `flyOnFromImpact()` keeps the aircraft where it stopped, on a floor held under
  its lowest contact (`stayHere`), until it is clear of the surface. It returns
  that floor.
- `liftFromImpact(meters)` lifts it straight up and keeps the ground it
  measured. It does not reset the contact history, since a reset means a new
  placement, and the stance clearance of a banked or pitched aircraft can be
  metres more than its lowest contact needs. A reset would call the lifted
  aircraft misplaced, withhold its support and hold it frozen in the air.

fly's bottom card offers these as Resume and Move up by the depth and 10 cm.
`hardImpact.integration.test.ts` in fly replays that F-35B crash in real JSBSim
from the pilot's bug report. It fails if the aircraft stops on the same impact,
is held, or does not settle onto the trees.

## Hull collision

JSBSim knows only its plane. Two things keep the airframe out of the rest of the
visible surface:

- **Structure contacts.** Every aircraft package declares `STRUCTURE` contacts
  at the nose, belly, tail and wingtips. A tailstrike, a wingtip scrape or a
  gear-up landing on the plane meets JSBSim's own spring, damper and friction,
  continuously. Gear up, the SF-50 rests on its belly contacts with its
  reference point 0.55 m above the ground and the F-35B 0.13 m, at about 0.3 in
  and 1 in of compression. Each package's comment says where its contacts sit
  on the drawn model. A belly contact's damping stays low enough not to chatter
  at a 120 Hz step.
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
that is the part someone reporting a problem can actually see. A hard ground
impact is asked about on the bottom card instead (above), and the bug report's
flight state names the fault, how many there were this visit, the last impact's
depth, floor and contact, and what the card shows.

Useful log entries when the ground behaves oddly:

- `First terrain height established` — the height, with coordinates.
- `Ignoring a surface sample …` — a rejected sample and the geometry it came from.
- `Flying without a measured surface` — the AGL at which the hold was skipped.
- `Repositioned onto the surface (…)` — which of the two causes fired.
- `Moved with a surface refinement` — a refinement of more than a metre moved
  the aircraft with it.
- `Moved onto the ground without asking` — a move within `automaticMoveMeters`.
- `Stopped a deep ground impact before loading the gear springs` (fly), with the
  contact that went deepest, then `Flying on from a hard impact` or
  `Moved up out of a hard impact`.
