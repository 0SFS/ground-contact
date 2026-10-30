# Underground and bridges

Status: Proposed, not implemented. The owner decided on 2026-10-30 not to
implement either proposal yet.
Scope: `@0sfs/ground-contact`, `@foss-earth/engine`'s `SurfaceQuery`, and fly's
drone controller and HUD.

## The problem

Terrain contact takes "the ground" to be the first surface seen from 20 km
above (`SurfaceQuery.sample`). Under a bridge, that is the deck, so an aircraft
flying under it is judged underground: it is asked to move, or it stops on a
hard ground impact. The pilot also wants a shallow overlap to float out by
itself, and a card only for a deep one. Ideally, solid would be what is inside
the meshes and free space what is outside, if that can run in real time.

## Documents

- [brief.md](brief.md): the design brief given to two independent design agents.
- [proposal-occupancy.md](proposal-occupancy.md): separates occupancy, collision,
  support and recovery, with `OUTSIDE`, `INSIDE` and `UNKNOWN` states.
- [proposal-sample-below.md](proposal-sample-below.md): a smaller, concrete plan
  built on a `sampleBelow` query that searches down from near the aircraft, with
  tests T1–T10 and phases.

Both cite code lines as of the morning of 2026-10-30. Some of what they describe
has changed since (below).

## Done since

- **Small corrections move without asking.** A relocation of
  `osfs.ground.automaticMoveMeters` (1 m by default, 0 always asks) or less, up
  or down, is applied at once, as Yes would apply it. The 0.0 m "surface
  refinement" card is gone. The 1 m is still measured against the topmost
  surface: a contact less than 1 m under a bridge deck is lifted onto the deck.
  The move is instant, not a float.
- **Hard impact recovery.** A hard ground impact asks on the bottom card, with
  Resume or Move up by its depth and 10 cm, instead of pausing behind a centred
  message ([ground-contact.md](../../ground-contact.md#after-a-hard-impact)).
- The `0.00001` m trigger for a refinement under a standing aircraft
  (`terrainContact.ts`) is still there. The automatic move answers it without a
  card.

## Review

Where both agree, and the direction to take:

- Search for the ground downward from near the aircraft, not from 20 km up.
  This is what fixes bridges.
- Judge support at each contact point.
- Never show a card for centimetres, or for ground that moved down.
- Float the aircraft up while keeping its gear compression.
- Show a card only for deep overlaps.
- The drone controller and HUD use terrain contact's ground, not their own
  sample from above.

**proposal-occupancy** is the more rigorous one. It has an explicit unknown
state, checks that the path is clear before lifting, and handles fast motion.
But it adds many new interfaces, trusts raw surface normals, and its 0.25 m/s
float is slow.

**proposal-sample-below** is smaller and concrete. Its `sampleBelow` is about
12 lines in FOSS Earth, and it measures before building. Its weak points:

- Shallow overlaps float up without asking why they happened. A drone flown
  into a hillside is lifted out, and drones have no hull probes to catch it.
- The 2 m crawlspace rule and the "coarse grace" timer are guesses.
- A sideways exit teleports the aircraft out of a wall.
- With no ceiling known yet, the downward search can start above a low soffit.

They disagree on mesh winding, that is, whether photogrammetry ground is
invisible from below. Both found that `SurfaceHit.normal` is flipped to face
the ray today, in TypeScript and in the Rust picker, so it cannot tell entry
from exit. Measure the winding on real tiles rather than argue it.

## Suggested first stage

- `sampleBelow` in FOSS Earth, in TypeScript and its Rust twin.
- Support judged per contact.
- A clear-path check before any automatic lift.
- A float of 0.5–1 s instead of an instant move.
- Remove the `0.00001` m trigger.
- The drone controller and HUD read terrain contact's ground.
- Tests T1–T7 from proposal-sample-below, plus a test under a low ceiling.
- First, measure under about five real bridges.
