# Collision tab, collision editor, and a shared 3D editing package

Status: Proposed
Date: 2026-10-30
Scope: `@0sfs/ground-contact`, the fly app's right panel, every aircraft
package's collision data, and a new FOSS Earth package extracted from
`foss-earth/app/src/app/poseEditor/`.

## Why

Collision decides whether an aircraft stands, rolls, scrapes or crashes, and
today it is hard to see and harder to change:

- The drones sank 12–28 mm into the ground. Fixing it meant measuring the
  model's feet with a script, working out each spring's compression, editing
  four numbers per drone in XML, and running the landing test four times. With
  an editor this is one click: put each contact where the drawn foot is.
- The data is in three places. Contacts are in each package's JSBSim XML.
  Body probes are TypeScript constants in `collisionGeometry.ts`, mapped to
  aircraft in fly's `fdmProfiles.ts`. The model offset is in the manifest.
- Until today the only view was a switch in Debug. It drew 24 cm balls on a
  25 cm drone and described the C172 for every aircraft. It also miscounted
  the C172's structural contacts (6, not 4).

## What exists

- **`ground-contact` already holds all the collision code.** It covers terrain
  contact, the body probes, visible-mesh collision, wheel spin, clearance and
  the collision overlay. It does not need a second collision repo.
- **The collision overlay** now sizes its markers to the aircraft and reports
  what it shows. The Debug legend counts the aircraft's own probes and contacts.
- **JSBSim takes contact moves live.** `contact/unit[i]/x|y|z-position` and the
  `gear/unit[i]/` equivalents are writable while flying. Lowering the Mini 2's
  four feet by 5 mm raised the resting drone by 5.0 mm on the next settle.
  Springs, dampers and friction are read at load and need a model reload.
- **The pose editor** in `foss-earth/app/src/app/poseEditor/` is a working 3D
  placement editor. The UMN tour uses it to correct 360 photo placements on a
  phone. It has:
  - **Gizmo math** (`poseMath.ts`): view rays, projection, dragging along an
    axis, dragging around a ring, and a gizmo that keeps a constant on-screen
    size.
  - **An SVG gizmo** (`editor.ts`): drawn over the canvas from projected
    points, with selection, a drag threshold, long-press and number fields.
  - **A local edit store** (`poseStore.ts`): coalesced saves to browser
    storage, conflict detection against the published version, and an export
    file with format and version.
  - **A dev-only save route** (`POST /__foss-earth/pose`): writes into the
    repo while running on localhost.
- [Generated collision geometry](generated-collision-geometry.md) proposes
  generating points from the mesh, so that nobody writes points by hand. An
  editor does not contradict it. The generator seeds the points, and the editor
  checks and adjusts them in the same data.

## Recommendation

1. **No new collision repo. Rename `ground-contact` to `collision`**
   (`@0sfs/collision`). It is already the collision repo; the name is what
   hides it. The rename is mechanical, and GitHub redirects the old URL. If
   you would rather not rename, everything below works under the current name.
2. **A Collision tab**, whose panel the collision package owns, as the
   autopilot and fuel packages own theirs. The Debug section moves there.
3. **The collision editor lives in that tab.** Editing is local to the
   browser, and its output is a file you download or, on localhost, write
   into the package.
4. **A new FOSS Earth package, `@foss-earth/transform-editor`**, extracted
   from the pose editor. Both the tour and the collision editor use it. It
   belongs in FOSS Earth because placing things in 3D is a globe-app need; the
   collision editor is just its second user.

## The Collision tab

**Viewing (always available)**

- Show geometry, with the legend: probes, wheels, structure, CG.
- A table of every point, with name and kind. For contacts it also shows live
  compression in mm, load, and whether the point touches the ground. It
  updates while the tab is open, and nothing runs while the tab is closed.
- Select a row or a marker to highlight it.

**Editing (an Edit switch)**

- **Gizmo:** a selected point gets the shared gizmo, with three arrows along
  the aircraft's forward, left and up axes. Number fields show the same
  position in millimetres in that frame, not structural inches. Positions
  apply live through the writable contact properties, so you watch the
  aircraft settle as you drag.
- **Physical fields:**
  - contacts: spring, damper, static and dynamic friction;
  - probes: inset.
  These apply when the flight model reloads, and the tab says so.
- **Rest on the drawn model:** puts a contact at the model's lowest point near
  it, lowered by its spring's compression under the aircraft's weight. Today's
  drone fix is this action done by hand. It works per contact or for all of
  them.
- **Add and remove:** structure contacts and body probes.
- **Save in this browser:**
  - Edits are kept per aircraft and package version, and applied when that
    aircraft loads. The app patches the XML's `<ground_reactions>` before it
    hands the file to JSBSim.
  - A badge shows the aircraft is running edited collision, with a reset.
  - If the package changes underneath the edits, the tab shows the conflict,
    as the pose store already does.
- **Download:**
  - a JSON file with the edited set, which Import reads back;
  - the `<ground_reactions>` block, ready to commit to the aircraft package.
- **Write to package (localhost only):** a dev-only route, like the pose
  editor's. It may only write the selected aircraft package's XML and
  manifest.

## Data: one place per aircraft

- **Contacts stay in the JSBSim XML.** JSBSim reads them there, as do other
  JSBSim tools. The editor reads and writes that block.
- **Body probes move into the aircraft package**, as a `collision.bodyProbes`
  entry in `0sfs-mod.json`. Today they are code in `ground-contact`. After the
  move, the editor writes data, mods can declare their own probes, and
  `fdmProfiles.ts` stops mapping aircraft to constants.
- **Each point records where it came from:** hand-written, generated, or
  edited. The generator can then refresh its own points without overwriting
  tuned ones.

## The shared 3D editing package

**What moves out of the pose editor**

- **The gizmo math:**
  - view ray, projection, axis travel, ring point, signed angle, screen turn;
  - ring drag, arm length for constant screen size, gizmo polylines, spin arc.
- **The SVG gizmo and its pointer handling:** selection, drag threshold,
  long-press, keyboard nudges and number fields. Its target is any frame: an
  origin and three labelled axes. Today the target is always east, north and
  up at a longitude and latitude.
- **The edit store:** generic over the record it keeps. It handles coalesced
  saves, the published baseline, conflicts, and the export/import envelope
  with the download filename.
- **The dev save route**, as a Vite plugin factory. Each app passes the files
  it may write.

**What stays**

- In the FOSS Earth app: the geographic parts (longitude/latitude frames and
  their wrapping), photo opacity, and the scene-file writer.
- In the tour: mapping tour entities to placements.

**Why an SVG overlay rather than Babylon's gizmos**

- It does not depend on the renderer. The FOSS Earth renderer and the fly
  app's Babylon scene can both use it.
- It already works with touch on a phone.
- It keeps a constant size on screen.
- It adds nothing to the 3D scene and needs no extra render pass. With
  on-demand rendering it redraws only when the camera or the point moves.

Babylon's `GizmoManager` would be faster to wire up, but it ties the editor to
Babylon and duplicates what the tour already relies on.

**Order:** extract first, with the pose editor as the only user and its tests
passing unchanged. That proves the boundary before the collision editor
depends on it.

## Stages

1. **Collision tab, viewing only.**
   - Move the Debug section and add the live contact table.
   - The rename happens here if you want it.
   - No new repo.
2. **Extract `@foss-earth/transform-editor`.**
   - The pose editor moves onto it.
   - Its tests and the tour's phone workflow must behave as before.
3. **Collision editor:**
   - gizmo and number fields, with live positions;
   - physical fields on reload;
   - rest on the drawn model;
   - save in the browser, download and import.
4. **Body probes become package data.** Add the localhost write to package.
5. **Later:** the generator from
   [generated collision geometry](generated-collision-geometry.md) writes
   into the same data.

## Decisions needed

- **Rename `ground-contact` to `collision`?** Recommended.
- **Name of the FOSS Earth package and repo:** `transform-editor` proposed.
- **Multiplayer:**
  - Edited collision changes how an aircraft touches the ground, so a flight
    with local edits should be marked modified for multiplayer and
    anti-cheat.
  - Recommended: edits are always allowed offline, and the marking keeps them
    out of ranked or shared sessions.
