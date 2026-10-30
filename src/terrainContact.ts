import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import type { SurfaceQuery } from "foss-earth/runtime";
import { flightLog } from "@0sfs/logging/flightLog";
import { bodyOffsetLocal, bodyToLocalRotation, groundContactBodyPosition, metresPerDegree, type BodyCollisionProbe } from "./collisionGeometry";
import { GEAR_DOWN_POSITION, groundContactClearanceMeters, groundContactUnits, type GroundContactUnit } from "./groundContactClearance";
import { createKernelSlot, type GroundContactCore, type GroundContactCoreSource, type GroundContactKernelStatus, type WasmContactSupport } from "./groundContactCore";
import { createPropertyReadBatch } from "./propertyReadBatch";
import { aircraftClearanceMeters, type AircraftClearanceStance, captureSimulation, PLACEMENT_TOLERANCE_METERS, restoreSimulation, WITHHELD_SUPPORT_DEPTH_METERS } from "./safeFlightState";
import { createWheelGroundFilter, type WheelGroundFilter } from "./wheelGroundFilter";

/**
 * A surface sample this far above a flying aircraft is not believed at all.
 *
 * Photogrammetry refines by metres, not by kilometres. A hit that puts the
 * ground a kilometre over an aircraft that was in normal flight one frame ago
 * is a bad sample - a stale tile transform, a mesh from another part of the
 * world, geometry picked before the tile group was placed - and acting on it
 * would either launch the aircraft on the gear springs or teleport it into the
 * stratosphere. Neither is better than carrying the previous terrain height
 * for a frame, so the sample is dropped and reported.
 */
const MAX_BELIEVABLE_RISE_METERS = 1000;

/**
 * How far the surface under the aircraft may move between two consecutive
 * samples, roughly half a metre apart at cruise.
 *
 * Refinement moves it by metres and occasionally by a few hundred; a kilometre
 * between frames is a different piece of world, not a better measurement of
 * this one. This catches a bad sample that lands only slightly above the
 * aircraft, which the rise limit alone would let through.
 *
 * Only upward steps are rejected. A surface that drops away cannot fire the
 * gear springs, and refusing one would strand the aircraft on a phantom floor
 * if a bad sample were ever adopted as the first reference.
 */
const MAX_BELIEVABLE_STEP_METERS = 1000;

/**
 * Height above the best known ground at which a missing sample stops mattering.
 *
 * Terrain only participates in the physics through the gear contacts, so an
 * aircraft this far up integrates identically whether or not the surface under
 * it is measurable. Holding the simulation there - as it did, for minutes at a
 * time, whenever the raster tiles under a cruising aircraft had not been
 * fetched - buys nothing and looks like a freeze.
 *
 * "Best known ground" is the last established height, or the elevation JSBSim
 * is already holding (sea level at startup) when there has never been one.
 */
const SURFACE_REQUIRED_AGL_METERS = 500;

/**
 * Consecutive missing samples before the aircraft counts as having flown
 * somewhere it could not see.
 *
 * Google publishes only what is currently drawn, so single-frame misses happen
 * constantly, including while taxiing. Treating one of those as a blind
 * stretch made every miss on the ground end in a reposition: the aircraft sank
 * onto its springs and was teleported back up, over and over. Half a second at
 * 120 Hz is a real gap; one frame is not.
 */
const BLIND_STEPS = 60;

/** Above this the wheels cannot be touching, so the surface is adopted raw and
 * touchdown is never met with a lagging ground height. */
const WHEELS_CLEAR_METERS = 5;

/**
 * The ground under each contact is sampled while the lowest contact is within
 * this height of the ground under the centre of gravity. Higher up, no contact
 * can reach the surface, and the centre alone is sampled.
 */
const FOOTPRINT_RANGE_METERS = 10;

/**
 * A lowest contact this close above JSBSim's floor, or pressed into it, stands
 * on it. A drone hovering a few centimetres up to land is flying: the ground
 * under it is not holding it up, and a finer map under it is not a reason to
 * move it.
 */
const STANDING_GAP_METERS = 0.02;

/**
 * Ground more than this above where a structure would rest is not support.
 * The wingtip or tail is under it, or in the wall of a building the aircraft
 * flew into, which is for hull collision to answer. A slope never gets this far
 * above a contact, because the aircraft rides up it. Wheels are the bottom of
 * the aircraft: ground above a wheel is an impact, as it is under the centre.
 */
const FOOTPRINT_REACH_METERS = 1;

/**
 * A contact's sample is reused until the contact has moved this far, plus its
 * margin over the contact deciding the support, up to the reach. Ground
 * cannot rise that margin under a contact within that travel on any slope
 * the aircraft could ride over.
 */
const FOOTPRINT_RESAMPLE_METERS = 0.05;

/**
 * Footprint samples per physics step on Google tiles. Without the optional
 * ground index those are picked triangle by triangle, at up to about half a
 * millisecond a sample on dense tiles; raster samples are read from decoded
 * grids and are not limited. A contact the limit defers keeps its last
 * sample, and the most overdue go first.
 */
const GOOGLE_FOOTPRINT_SAMPLES_PER_STEP = 2;

/**
 * Google's revision changes whenever any tile changes visibility, nearly
 * every frame while the camera moves. When the surface under the centre did
 * not change, a contact's sample is confirmed lazily, at most one contact per
 * this many steps and the longest unconfirmed first, instead of picking under
 * every contact each frame.
 */
const FOOTPRINT_REVALIDATE_STEPS = 30;

// Why a contact wants a sample, most pressing first: it has none; the world
// changed under it; it moved on from it; or it is due a lazy confirmation.
const SAMPLE_FIRST = 4, SAMPLE_URGENT = 3, SAMPLE_MOVED = 2, SAMPLE_LAZY = 1;

interface SupportInput {
  lat: number; lon: number; altitude: number;
  roll: number; pitch: number; heading: number;
  /** Filtered ground under the centre: the fallback, and the range reference. */
  cgGround: number;
  revision: number;
  revisionChanged: boolean;
  /** The refinement measured under the centre; NaN when it could not be. */
  cgDelta: number;
  /** Confirm every contact sample the revision may have changed now, not lazily. */
  urgent: boolean;
  /** Adopt raw heights outright, as the centre does. */
  snap: boolean;
  /** Footprint samples allowed this step. */
  samples: number;
  /** The support plane from the previous step; NaN without one. */
  previousElevation: number;
}

/**
 * The ground under the aircraft's contact units, given to JSBSim as the one
 * horizontal plane it accepts for all of them.
 *
 * Under the centre of gravity alone, a sloped runway put the nose wheel or
 * the tail under the surface and a hillside put a wingtip into it. The plane
 * chosen here is the lowest at which no contact is deeper in its own ground
 * than JSBSim's lowest contact is in the plane, so that contact's spring
 * holds every other contact to at most its own compression. A structure that
 * decides the plane can sink that compression into its ground, about 5 cm on
 * the C172.
 *
 * The ground under the centre, sampled every step, stays a floor under the
 * plane: a contact whose sample is deferred or missing never leaves the plane
 * lower than the centre alone did, and a wall the aircraft drives into still
 * stops it the step it appears. On flat ground the plane is that ground.
 *
 * A horizontal plane cannot pitch the aircraft onto a slope: the uphill
 * contact carries it and the others ride clear of their ground. That needs a
 * ground per contact in the native gear model.
 */
function createReferenceSupport(sdk: JSBSimSdk, surface: SurfaceQuery) {
  let units: readonly GroundContactUnit[] = [];
  let reader: ReturnType<typeof createPropertyReadBatch> | null = null;
  let values = new Float64Array(0);
  // The model has no contact units or does not report them; the centre's
  // ground stands in until the next reset.
  let unavailable = false;
  let count = 0;
  // Farthest contact from the centre, so a high aircraft skips the read.
  let reachMeters = Number.POSITIVE_INFINITY;
  let live = false;
  // The previous plane's parts: the centre's ground and the lowest depth;
  // and the height of the centre that its reach was judged from.
  let hadSupport = false;
  let prevCgGround = 0, prevLowest = 0, prevLevel = 0;
  let revalidateIn = 0;
  let steps = 0;
  let filters: WheelGroundFilter[] = [];
  // Per contact: the current footprint and its depth below the centre.
  let down = new Float64Array(0), footLat = new Float64Array(0), footLon = new Float64Array(0);
  // The last sample, where it was taken, the revision it was taken in, and
  // the step it was last taken or confirmed.
  let heldLat = new Float64Array(0), heldLon = new Float64Array(0), heldRaw = new Float64Array(0);
  let heldRevision = new Float64Array(0), checked = new Float64Array(0);
  // Refinement measured under the centre and applied to a sample not yet
  // measured again at its own spot.
  let provisional = new Float64Array(0);
  // Filtered ground, NaN without a sample; margin over the deciding contact.
  let ground = new Float64Array(0), margin = new Float64Array(0), moved = new Float64Array(0);
  let shift = new Float64Array(0);
  // Why each contact wants a sample this step, and its order within that.
  let wants = new Uint8Array(0), rank = new Float64Array(0);
  let prevGround = new Float64Array(0), prevDown = new Float64Array(0);
  // Per contact at the previous step: it decided the plane; it had a sample.
  let active = new Uint8Array(0), supported = new Uint8Array(0), prevSupported = new Uint8Array(0), prevSampled = new Uint8Array(0);
  let urgent = new Uint8Array(0), wheel = new Uint8Array(0);
  const rotation = new Float64Array(9);
  const scale = { lat: 0, lon: 0 };
  const cg = { xIn: 0, yIn: 0, zIn: 0 }, point = { xIn: 0, yIn: 0, zIn: 0 };
  const body = { left: 0, up: 0, forward: 0 }, local = { north: 0, east: 0, down: 0 };
  const result = { elevation: Number.NaN, refinement: 0 };

  function clearContacts(): void {
    if (!live) return;
    live = false;
    hadSupport = false;
    for (let i = 0; i < count; i++) {
      filters[i].reset();
      heldRaw[i] = Number.NaN; ground[i] = Number.NaN;
      provisional[i] = 0; margin[i] = 0; urgent[i] = 0; supported[i] = 0; prevSupported[i] = 0; prevSampled[i] = 0;
    }
  }

  function forget(): void {
    clearContacts();
    reader?.dispose();
    reader = null;
    count = 0;
    reachMeters = Number.POSITIVE_INFINITY;
  }

  function discover(): boolean {
    if (reader) return true;
    if (unavailable) return false;
    try { units = groundContactUnits(sdk); }
    catch { unavailable = true; return false; }
    if (units.length === 0) { unavailable = true; return false; }
    const paths = ["gear/num-units", "gear/gear-pos-norm", "inertia/cg-x-in", "inertia/cg-y-in", "inertia/cg-z-in"];
    for (const unit of units) paths.push(`${unit.prefix}x-position`, `${unit.prefix}y-position`, `${unit.prefix}z-position`);
    reader = createPropertyReadBatch(sdk, paths);
    values = new Float64Array(paths.length);
    count = units.length;
    filters = Array.from({ length: count }, () => createWheelGroundFilter());
    const floats = () => new Float64Array(count), flags = () => new Uint8Array(count);
    down = floats(); footLat = floats(); footLon = floats();
    heldLat = floats(); heldLon = floats(); heldRaw = floats().fill(Number.NaN);
    heldRevision = floats(); checked = floats();
    provisional = floats(); ground = floats().fill(Number.NaN); margin = floats(); moved = floats();
    shift = floats(); rank = floats(); prevGround = floats(); prevDown = floats();
    active = flags(); supported = flags(); prevSupported = flags(); prevSampled = flags(); urgent = flags(); wants = flags();
    // BOGEY units are under gear/unit, STRUCTURE units under contact/unit.
    wheel = Uint8Array.from(units, unit => unit.prefix.startsWith("gear/") ? 1 : 0);
    return true;
  }

  /** Footprints of the contacts; returns the lowest active contact's depth below the centre. */
  function readGeometry(input: SupportInput): number {
    if (!discover()) return Number.NaN;
    try { reader!.readInto(values); }
    catch {
      // A model load disposes the native batch; find the contacts again.
      forget();
      return Number.NaN;
    }
    if (values[0] !== count) { forget(); return Number.NaN; }
    const gearDown = values[1] > GEAR_DOWN_POSITION;
    cg.xIn = values[2]; cg.yIn = values[3]; cg.zIn = values[4];
    bodyToLocalRotation(input.roll, input.pitch, input.heading, rotation);
    metresPerDegree(input.lat, scale);
    let lowest = Number.NEGATIVE_INFINITY, reach = 0;
    for (let i = 0; i < count; i++) {
      point.xIn = values[5 + i * 3]; point.yIn = values[6 + i * 3]; point.zIn = values[7 + i * 3];
      groundContactBodyPosition(point, cg, body);
      bodyOffsetLocal(body, rotation, local);
      reach = Math.max(reach, Math.hypot(body.left, body.up, body.forward));
      down[i] = local.down;
      footLat[i] = input.lat + local.north / scale.lat;
      footLon[i] = ((input.lon + local.east / scale.lon + 540) % 360) - 180;
      // Native BOGEY support exists only above the gear-down position for
      // retractable units; fixed units and structures always participate.
      const on = !(units[i].retractable && !gearDown) && Number.isFinite(local.down)
        && Number.isFinite(footLat[i]) && Number.isFinite(footLon[i]);
      if (!on && active[i]) {
        filters[i].reset();
        heldRaw[i] = Number.NaN; ground[i] = Number.NaN; provisional[i] = 0; urgent[i] = 0;
      }
      active[i] = on ? 1 : 0;
      if (on) lowest = Math.max(lowest, local.down);
    }
    reachMeters = Number.isFinite(reach) ? reach : Number.POSITIVE_INFINITY;
    return lowest;
  }

  /** Travel after which a contact is sampled again: farther the less it matters. */
  function spacing(i: number): number {
    return FOOTPRINT_RESAMPLE_METERS + Math.min(Math.max(margin[i], 0), FOOTPRINT_REACH_METERS);
  }

  /** Forget the contacts and find them again, as after a model load. */
  function reset(): void {
    forget();
    unavailable = false;
    revalidateIn = 0;
  }

  function sampleFootprints(input: SupportInput): void {
    const lazy = revalidateIn <= 0;
    revalidateIn -= 1;
    steps += 1;
    for (let i = 0; i < count; i++) {
      wants[i] = 0;
      if (!active[i]) continue;
      if (Number.isNaN(heldRaw[i])) { wants[i] = SAMPLE_FIRST; rank[i] = 0; continue; }
      moved[i] = Math.hypot((footLat[i] - heldLat[i]) * scale.lat,
        (((footLon[i] - heldLon[i] + 540) % 360) - 180) * scale.lon);
      const stale = heldRevision[i] !== input.revision;
      if (stale && urgent[i]) {
        // The contact nearest to deciding the support first.
        wants[i] = SAMPLE_URGENT; rank[i] = -margin[i];
      } else if (moved[i] >= spacing(i)) {
        // The most overdue first.
        wants[i] = SAMPLE_MOVED; rank[i] = moved[i] / spacing(i);
      } else if (stale && lazy) {
        // The longest unconfirmed first: a revision that changes every frame
        // leaves every contact stale, and the nearest would always win.
        wants[i] = SAMPLE_LAZY; rank[i] = steps - checked[i];
      }
    }
    for (let allowance = input.samples; allowance > 0; allowance--) {
      let next = -1;
      for (let i = 0; i < count; i++) {
        if (wants[i] && (next < 0 || wants[i] > wants[next] || (wants[i] === wants[next] && rank[i] > rank[next]))) next = i;
      }
      if (next < 0) break;
      const kind = wants[next];
      wants[next] = 0;
      if (kind === SAMPLE_URGENT || kind === SAMPLE_LAZY) {
        if (kind === SAMPLE_LAZY) {
          revalidateIn = FOOTPRINT_REVALIDATE_STEPS;
          for (let i = 0; i < count; i++) if (wants[i] === SAMPLE_LAZY) wants[i] = 0;
        }
        // Measure the refinement where the sample was taken: two versions
        // at one spot, never two heights along a path.
        const hit = surface.sample(heldLat[next], heldLon[next]);
        checked[next] = steps;
        // On a miss (Google drops what is out of view), keep the estimate and
        // confirm it later rather than every step.
        urgent[next] = 0;
        if (hit && Number.isFinite(hit.heightMeters)) {
          const correction = hit.heightMeters - heldRaw[next] - provisional[next];
          filters[next].shift(correction);
          ground[next] += correction;
          shift[next] += correction;
          heldRaw[next] = hit.heightMeters; heldRevision[next] = hit.revision;
          provisional[next] = 0;
        }
        // Then follow the contact to where it is now, if there is a sample to spare.
        if (moved[next] >= spacing(next)) { wants[next] = SAMPLE_MOVED; rank[next] = moved[next] / spacing(next); }
        continue;
      }
      const hit = surface.sample(footLat[next], footLon[next]);
      // Keep the last sample over a hole; the move is retried next step.
      if (!hit || !Number.isFinite(hit.heightMeters)) continue;
      const fresh = kind === SAMPLE_FIRST;
      ground[next] = filters[next].height(hit.heightMeters, fresh ? 0 : moved[next],
        fresh || input.snap || input.altitude - down[next] - hit.heightMeters > WHEELS_CLEAR_METERS);
      heldLat[next] = footLat[next]; heldLon[next] = footLon[next];
      heldRaw[next] = hit.heightMeters; heldRevision[next] = hit.revision; checked[next] = steps;
      provisional[next] = 0; urgent[next] = 0;
    }
  }

  return {
    /**
     * The support plane for this step, or NaN when the centre's ground stands
     * in; and how far the world's refinement moved the previous plane.
     */
    measure(input: SupportInput): typeof result {
      const knownDelta = input.revisionChanged && Number.isFinite(input.cgDelta);
      // Without contact samples the change under the centre is the change.
      result.refinement = knownDelta ? input.cgDelta : 0;
      result.elevation = Number.NaN;
      if (live) {
        for (let i = 0; i < count; i++) {
          shift[i] = knownDelta ? input.cgDelta : 0;
          if (!input.revisionChanged || Number.isNaN(heldRaw[i])) continue;
          // Refinements are spatially coherent: the sample moves with the
          // change measured under the centre until measured at its own spot.
          if (knownDelta) {
            filters[i].shift(input.cgDelta);
            ground[i] += input.cgDelta;
            provisional[i] += input.cgDelta;
          }
          if (input.urgent || !knownDelta) urgent[i] = 1;
        }
      }
      const far = input.altitude - input.cgGround > FOOTPRINT_RANGE_METERS + reachMeters + 1;
      const lowest = far ? Number.NaN : readGeometry(input);
      const inRange = Number.isFinite(lowest) && input.altitude - lowest - input.cgGround < FOOTPRINT_RANGE_METERS;
      if (inRange) {
        live = true;
        // A surface that throws abandons the step, and the samples it already
        // gave with it, so no contact is left half a step on.
        try { sampleFootprints(input); }
        catch (error) { reset(); throw error; }
      }
      if (hadSupport) {
        // The previous plane under the refined world, at the previous pose,
        // chosen by the same rule. Reach is judged from where the aircraft
        // stood, moved by the change measured without the contacts it decides
        // on: under the centre, or else the least any supporting contact's
        // ground moved. Otherwise a roof refined in over a wingtip would lift
        // the plane that then judges the roof within reach.
        let lift = Number.POSITIVE_INFINITY;
        if (knownDelta) lift = input.cgDelta;
        else for (let i = 0; i < count; i++) if (prevSupported[i]) lift = Math.min(lift, shift[i]);
        const reachLevel = prevLevel + (Number.isFinite(lift) ? lift : 0);
        let before = prevCgGround + prevLowest;
        let after = before + (knownDelta ? input.cgDelta : 0);
        for (let i = 0; i < count; i++) {
          if (prevSupported[i]) before = Math.max(before, prevGround[i] + prevDown[i]);
          if (!prevSampled[i]) continue;
          // A contact's refined ground counts if it is within reach now,
          // whether or not it was before.
          const refined = prevGround[i] + shift[i];
          if (!wheel[i] && refined > reachLevel - prevDown[i] + FOOTPRINT_REACH_METERS) continue;
          after = Math.max(after, refined + prevDown[i]);
        }
        result.refinement = after - before;
      }
      if (!inRange) {
        clearContacts();
        return result;
      }
      const reference = Number.isFinite(input.previousElevation)
        ? input.previousElevation + result.refinement : input.cgGround;
      // Where the lowest contact would be resting on that plane, or is now.
      const level = Math.max(input.altitude, reference + lowest);
      let top = input.cgGround + lowest;
      for (let i = 0; i < count; i++) {
        supported[i] = 0;
        if (!active[i] || Number.isNaN(ground[i])) continue;
        if (!wheel[i] && ground[i] > level - down[i] + FOOTPRINT_REACH_METERS) {
          margin[i] = FOOTPRINT_REACH_METERS;
          continue;
        }
        supported[i] = 1;
        top = Math.max(top, ground[i] + down[i]);
      }
      for (let i = 0; i < count; i++) {
        prevSupported[i] = supported[i];
        prevSampled[i] = active[i] && !Number.isNaN(ground[i]) ? 1 : 0;
        if (!prevSampled[i]) continue;
        prevGround[i] = ground[i];
        prevDown[i] = down[i];
        if (supported[i]) margin[i] = top - ground[i] - down[i];
      }
      prevCgGround = input.cgGround;
      prevLowest = lowest;
      prevLevel = level;
      hadSupport = true;
      result.elevation = top - lowest;
      return result;
    },
    reset,
    /** Whether it holds contact samples a switch to the kernel would lose. */
    holdsContacts: (): boolean => reader !== null || live,
  };
}

type SupportResult = ReturnType<ReturnType<typeof createReferenceSupport>["measure"]>;

// SUPPORT.requestStride and SUPPORT.GEOMETRY_* of the kernel.
const REQUEST_STRIDE = 6;
const GEOMETRY_READ = 0, GEOMETRY_FAILED = 1, GEOMETRY_UNAVAILABLE = 2;

/**
 * `createReferenceSupport` on the core's contact-support kernel. The kernel
 * keeps every contact's state; this side discovers the units, reads their
 * properties into the kernel and samples the surface where it asks.
 */
function createWasmSupport(sdk: JSBSimSdk, surface: SurfaceQuery, core: GroundContactCore) {
  let units: readonly GroundContactUnit[] = [];
  let reader: ReturnType<typeof createPropertyReadBatch> | null = null;
  let unavailable = false;
  let kernel: WasmContactSupport | null = null;
  let kernelUnits: readonly GroundContactUnit[] = [];
  // Set while the SDK or the surface is called, whose errors are the caller's.
  let inHost = false;
  const result = { elevation: Number.NaN, refinement: 0 };

  const sameUnits = (a: readonly GroundContactUnit[], b: readonly GroundContactUnit[]) => a.length === b.length
    && a.every((unit, i) => unit.retractable === b[i].retractable && unit.prefix.startsWith("gear/") === b[i].prefix.startsWith("gear/"));

  /** The reference's `discover`; a kernel for units it has not got replaces the forgotten one. */
  function discover(): boolean {
    if (reader) return true;
    if (unavailable) return false;
    inHost = true;
    try { units = groundContactUnits(sdk); }
    catch { unavailable = true; return false; }
    finally { inHost = false; }
    if (units.length === 0) { unavailable = true; return false; }
    const paths = ["gear/num-units", "gear/gear-pos-norm", "inertia/cg-x-in", "inertia/cg-y-in", "inertia/cg-z-in"];
    for (const unit of units) paths.push(`${unit.prefix}x-position`, `${unit.prefix}y-position`, `${unit.prefix}z-position`);
    inHost = true;
    reader = createPropertyReadBatch(sdk, paths);
    inHost = false;
    if (!kernel || !sameUnits(kernelUnits, units)) {
      // The reference forgets the contacts but not its step count or its
      // revalidation countdown, which the previous kernel hands on.
      const next = core.createContactSupport(units, kernel ?? undefined);
      kernel?.dispose();
      kernel = next;
      kernelUnits = units;
    }
    return true;
  }

  /** The reference's reset: the kernel forgets the contacts, and the units are found again. */
  function reset(): void {
    kernel?.reset();
    reader?.dispose();
    reader = null;
    unavailable = false;
  }

  function fill(input: SupportInput): void {
    const row = kernel!.input;
    row[0] = input.lat; row[1] = input.lon; row[2] = input.altitude;
    row[3] = input.roll; row[4] = input.pitch; row[5] = input.heading;
    row[6] = input.cgGround; row[7] = input.revision; row[8] = input.revisionChanged ? 1 : 0;
    row[9] = input.cgDelta; row[10] = input.urgent ? 1 : 0; row[11] = input.snap ? 1 : 0;
    row[12] = input.samples; row[13] = input.previousElevation;
  }

  return {
    /** Whether the last error thrown came from the SDK or the surface rather than the kernel. */
    get hostError() { return inHost; },
    measure(input: SupportInput): SupportResult {
      inHost = false;
      // Undiscovered, the reference reads the geometry whatever the height.
      if (!kernel && !discover()) {
        result.elevation = Number.NaN;
        result.refinement = input.revisionChanged && Number.isFinite(input.cgDelta) ? input.cgDelta : 0;
        return result;
      }
      fill(input);
      let geometry = GEOMETRY_READ;
      if (kernel!.wantsGeometry()) {
        const before = kernel;
        if (!discover()) geometry = GEOMETRY_UNAVAILABLE;
        else {
          if (kernel !== before) fill(input);
          try { reader!.readInto(kernel!.values); }
          catch {
            // A model load disposes the native batch; find the contacts again.
            reader?.dispose();
            reader = null;
            geometry = GEOMETRY_FAILED;
          }
        }
      }
      const planned = kernel!.plan(geometry);
      // A changed unit count forgot the contacts, as the reference's forget() does.
      if (geometry === GEOMETRY_READ && !kernel!.discovered) { reader?.dispose(); reader = null; }
      const requests = kernel!.requests;
      for (let k = 0; k < planned; k++) {
        const at = k * REQUEST_STRIDE;
        inHost = true;
        let hit: ReturnType<SurfaceQuery["sample"]>;
        try { hit = surface.sample(requests[at + 2], requests[at + 3]); }
        catch (error) {
          // Abandon the step rather than finish it half sampled, as the
          // reference and the kernel's own measure() do.
          inHost = false;
          reset();
          inHost = true;
          throw error;
        }
        inHost = false;
        requests[at + 4] = hit ? hit.heightMeters : Number.NaN;
        requests[at + 5] = hit ? hit.revision : Number.NaN;
      }
      const output = kernel!.finish();
      result.elevation = output[0];
      result.refinement = output[1];
      return result;
    },
    reset,
    dispose(): void {
      reader?.dispose();
      reader = null;
      kernel?.dispose();
      kernel = null;
    },
  };
}

/**
 * The reference, or the kernel when a core is supplied. They hand over only
 * where neither holds contact samples, at the start and after a reset, so
 * a core that loads in flight never drops the ground under a contact.
 */
function createContactSupport(sdk: JSBSimSdk, surface: SurfaceQuery, core: GroundContactCoreSource) {
  const reference = createReferenceSupport(sdk, surface);
  const slot = createKernelSlot(core, "wasm-contact-support", loaded => createWasmSupport(sdk, surface, loaded));
  let onWasm = false;
  return {
    measure(input: SupportInput): SupportResult {
      const kernel = slot.current;
      if (kernel && (onWasm || !reference.holdsContacts())) {
        onWasm = true;
        try { return kernel.measure(input); }
        catch (error) {
          if (kernel.hostError) throw error;
          slot.fail(error);
          onWasm = false;
        }
      }
      return reference.measure(input);
    },
    reset(): void {
      reference.reset();
      try { slot.current?.reset(); }
      catch (error) { slot.fail(error); }
      onWasm = false;
    },
    status(): GroundContactKernelStatus {
      const status = slot.status();
      // A core that arrived in flight waits for the next reset.
      if (status.kernel === "single" && !onWasm && reference.holdsContacts()) {
        return { ...status, kernel: "typescript", fallbackReason: "wasm-contact-support-pending: the reference holds contact samples until a reset" };
      }
      return status;
    },
    dispose(): void {
      reference.reset();
      slot.dispose();
    },
  };
}

export type TerrainBlockReason = "missing" | "coarse" | "relocation" | null;

/** A nonphysical terrain correction. Preparing one never changes the aircraft pose. */
export interface TerrainRelocationProposal {
  id: number;
  reason: "placement" | "surface regained above the aircraft" | "surface refinement";
  currentAltitudeMeters: number;
  altitudeMeters: number;
  groundHeightMeters: number;
  deltaMeters: number;
}

interface TerrainContactReference {
  lat: number;
  lon: number;
  height: number;
  ground: number;
  revision: number;
  clearance: number;
}

export function createTerrainContact(
  sdk: JSBSimSdk,
  surface: SurfaceQuery,
  clearanceProfile: AircraftClearanceStance,
  /** A WASM ground-contact core, or its load, for the contact support plane. */
  options: {
    core?: GroundContactCoreSource;
    bodyProbes?: readonly BodyCollisionProbe[];
    /**
     * Corrections no larger than this, in metres, are made without asking:
     * the aircraft is moved onto the ground and `update` returns "reset".
     * Larger ones become proposals. Read on every correction; absent asks
     * about all of them.
     */
    automaticMoveMeters?: () => number;
  } = {},
) {
  // `height` is the raw sample under the centre, used to detect the world
  // changing. `ground` is the support plane the contacts ride on and JSBSim is
  // given.
  let previous: TerrainContactReference | null = null;
  let placement = true;
  let hasReference = false;
  let rejectedSamples = 0;
  // True while the simulation is stepping without a measured surface, which is
  // allowed only well clear of the ground. The first sample after that is
  // trusted the way a fresh placement is, because the aircraft may have flown
  // over a hill it could not see.
  let blindSteps = 0;
  let blockReason: TerrainBlockReason = null;
  const wheelGround = createWheelGroundFilter();
  const contacts = createContactSupport(sdk, surface, options.core);
  const supportInput: SupportInput = {
    lat: 0, lon: 0, altitude: 0, roll: 0, pitch: 0, heading: 0, cgGround: 0, revision: 0,
    revisionChanged: false, cgDelta: 0, urgent: false, snap: false, samples: 0, previousElevation: Number.NaN,
  };
  let proposal: TerrainRelocationProposal | null = null;
  let proposedContact: TerrainContactReference | null = null;
  let proposalId = 0;
  let ignoreOverlap = false;
  // The floor the aircraft had when the world changed under it. It stays under
  // an aircraft that stands on it while the pilot is asked, so the question
  // neither stops physics nor drops it, and it is given back to an aircraft
  // whose pilot chose to stay, until the overlap ends. Null where no floor was
  // trusted, such as a placement: support is withheld there instead.
  let keptFloor: number | null = null;
  // The pending proposal holds physics: the aircraft was not standing on its floor.
  let holding = false;
  const exteriorRotation = new Float64Array(9);
  const exteriorScale = { lat: 0, lon: 0 };
  const exteriorCg = { xIn: 0, yIn: 0, zIn: 0 };
  const exteriorPoint = { xIn: 0, yIn: 0, zIn: 0 };
  const exteriorBody = { left: 0, up: 0, forward: 0 };
  const exteriorLocal = { north: 0, east: 0, down: 0 };

  const clearContactHistory = (): void => {
    previous = null; placement = true; hasReference = false; rejectedSamples = 0; blindSteps = 0; blockReason = null;
    wheelGround.reset();
    contacts.reset();
  };
  // A rejected/newly appearing floor must not launch JSBSim through its gear
  // springs. This is deliberately unknown physical support, not real terrain.
  const withholdSupport = (): void => {
    const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
    if (Number.isFinite(altitude)) sdk.setPropertyValue("position/terrain-elevation-asl-ft", (altitude - WITHHELD_SUPPORT_DEPTH_METERS) / 0.3048);
  };
  const floorMeters = (): number => sdk.getPropertyValue("position/terrain-elevation-asl-ft") * 0.3048;
  const setFloor = (meters: number): void => sdk.setPropertyValue("position/terrain-elevation-asl-ft", meters / 0.3048);
  /** Depth of the lowest active contact below the centre of gravity; the stance's clearance for a model that reports none. */
  const lowestContactMeters = (roll: number, pitch: number, stanceClearance: number): number => {
    try { if (groundContactUnits(sdk).length > 0) return groundContactClearanceMeters(sdk, roll, pitch); }
    catch { /* The model does not report its contacts completely. */ }
    return stanceClearance;
  };
  /**
   * The aircraft's lowest contact is on `floor` or pressed into it, with its
   * centre still above it: one under its floor has sunk, not settled.
   */
  const standsOn = (floor: number, altitude: number, roll: number, pitch: number, stanceClearance: number): boolean =>
    altitude > floor && altitude - lowestContactMeters(roll, pitch, stanceClearance) - floor <= STANDING_GAP_METERS;
  /** Every hull probe and active contact passes `clear` at the surface sampled under it. */
  const everyPoint = (lat: number, lon: number, clear: (pointMeters: number, surfaceMeters: number, contact: boolean) => boolean): boolean => {
    const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
    const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
    const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
    bodyToLocalRotation(roll, pitch, sdk.getPropertyValue("attitude/psi-deg") * Math.PI / 180, exteriorRotation);
    metresPerDegree(lat, exteriorScale);
    exteriorCg.xIn = sdk.getPropertyValue("inertia/cg-x-in");
    exteriorCg.yIn = sdk.getPropertyValue("inertia/cg-y-in");
    exteriorCg.zIn = sdk.getPropertyValue("inertia/cg-z-in");
    const passes = (point: { xIn: number; yIn: number; zIn: number }, contact: boolean): boolean => {
      groundContactBodyPosition(point, exteriorCg, exteriorBody);
      bodyOffsetLocal(exteriorBody, exteriorRotation, exteriorLocal);
      const sample = surface.sample(lat + exteriorLocal.north / exteriorScale.lat,
        ((lon + exteriorLocal.east / Math.max(0.001, exteriorScale.lon) + 540) % 360) - 180);
      return sample !== null && Number.isFinite(sample.heightMeters)
        && clear(altitude - exteriorLocal.down, sample.heightMeters, contact);
    };
    for (const probe of options.bodyProbes ?? []) if (!passes(probe, false)) return false;
    let units: readonly GroundContactUnit[];
    try { units = groundContactUnits(sdk); } catch { return false; }
    const gear = sdk.getPropertyValue("gear/gear-pos-norm");
    for (const unit of units) {
      if (unit.retractable && gear <= GEAR_DOWN_POSITION) continue;
      exteriorPoint.xIn = sdk.getPropertyValue(unit.prefix + "x-position");
      exteriorPoint.yIn = sdk.getPropertyValue(unit.prefix + "y-position");
      exteriorPoint.zIn = sdk.getPropertyValue(unit.prefix + "z-position");
      if (!passes(exteriorPoint, true)) return false;
    }
    return true;
  };
  /** The aircraft has climbed clear of the surface: its centre and every probe and contact. */
  const fullyExterior = (lat: number, lon: number, ground: number): boolean => {
    const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
    const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
    const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
    const clearance = aircraftClearanceMeters(clearanceProfile, roll, pitch);
    return altitude > ground + clearance + 0.2 && everyPoint(lat, lon, (point, surfaceMeters) => point > surfaceMeters + 0.2);
  };
  /**
   * The surface has come down to `floor` under the centre and every contact,
   * and no probe is inside it: the support measured there is no higher than
   * the floor the aircraft already stands on.
   */
  const overlapGone = (lat: number, lon: number, ground: number, floor: number): boolean =>
    ground <= floor + PLACEMENT_TOLERANCE_METERS
    && everyPoint(lat, lon, (point, surfaceMeters, contact) => contact ? surfaceMeters <= floor + PLACEMENT_TOLERANCE_METERS : point >= surfaceMeters);
  /** Moves the aircraft as `accepted` proposed and adopts the contact measured for it. */
  const applyRelocation = (accepted: TerrainRelocationProposal): void => {
    if (accepted.reason === "surface refinement") {
      // Only the position moves, by the change itself: velocities, rates,
      // the oleo state and the integrators' history carry on, so nothing
      // drops onto its springs. An aircraft that stood on its floor kept
      // flying while it was asked, so it moves from where it is now.
      sdk.setPropertyValue("position/terrain-elevation-asl-ft", accepted.groundHeightMeters / 0.3048);
      sdk.setPropertyValue("position/h-sl-ft", sdk.getPropertyValue("position/h-sl-ft") + accepted.deltaMeters / 0.3048);
    } else {
      const snapshot = captureSimulation(sdk);
      snapshot.initial["ic/terrain-elevation-ft"] = accepted.groundHeightMeters / 0.3048;
      snapshot.initial["ic/h-sl-ft"] = accepted.altitudeMeters / 0.3048;
      // Preserve lateral motion; remove only downward velocity into the plane.
      snapshot.initial["ic/vd-fps"] = Math.min(0, snapshot.initial["ic/vd-fps"]);
      restoreSimulation(sdk, snapshot);
    }
    proposal = null;
    ignoreOverlap = false;
    keptFloor = null;
    holding = false;
    previous = proposedContact;
    proposedContact = null;
    blockReason = null;
    // A refinement proposal can preserve suspension compression; accepting
    // it must not immediately create a second nominal-stance proposal.
    placement = false;
  };
  /** Fly on without a relocation, on the floor kept for it or, without one, with support withheld. */
  const stayHere = (floor: number | null): void => {
    proposal = null;
    proposedContact = null;
    holding = false;
    ignoreOverlap = true;
    clearContactHistory();
    keptFloor = floor;
    if (floor === null) withholdSupport();
    else setFloor(floor);
  };
  /** Judge the ground afresh, with no question, waiver or kept floor left over. */
  const reset = (): void => {
    clearContactHistory();
    proposal = null;
    proposedContact = null;
    ignoreOverlap = false;
    keptFloor = null;
    holding = false;
  };
  /** The state a hard impact stopped the aircraft in, with its fall stopped: that fall was the impact. */
  const impactSnapshot = () => {
    const snapshot = captureSimulation(sdk);
    snapshot.initial["ic/vd-fps"] = Math.min(0, snapshot.initial["ic/vd-fps"]!);
    return snapshot;
  };
  return {
    reset,
    getRelocationProposal: (): TerrainRelocationProposal | null => proposal,
    /** Only call after confirmation of this exact proposal. */
    acceptRelocation(id: number): boolean {
      if (!proposal || proposal.id !== id) return false;
      applyRelocation(proposal);
      return true;
    },
    /** Stay put. An aircraft keeps the floor it had, so one standing on the ground stays on it. */
    declineRelocation(id: number): void {
      if (!proposal || proposal.id !== id) return;
      stayHere(keptFloor);
    },
    /** Explicitly waive the current terrain overlap/missing-ground episode. */
    ignoreTerrainOverlap(): void {
      if (proposal) { stayHere(keptFloor); return; }
      const floor = floorMeters();
      const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
      const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
      const standing = standsOn(floor, sdk.getPropertyValue("position/h-sl-ft") * 0.3048, roll, pitch,
        aircraftClearanceMeters(clearanceProfile, roll, pitch));
      stayHere(standing ? floor : null);
    },
    /**
     * Fly on from where a hard impact stopped the aircraft inside the surface,
     * with its fall stopped. The floor is held under its lowest contact until
     * it is clear of the surface, so it starts on the floor rather than in it.
     * Returns the floor, in metres.
     */
    flyOnFromImpact(): number {
      const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
      const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
      const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
      const floor = altitude - lowestContactMeters(roll, pitch, aircraftClearanceMeters(clearanceProfile, roll, pitch));
      const snapshot = impactSnapshot();
      snapshot.initial["ic/terrain-elevation-ft"] = floor / 0.3048;
      restoreSimulation(sdk, snapshot);
      stayHere(floor);
      return floor;
    },
    /**
     * Lift the aircraft `meters` straight up from where a hard impact stopped
     * it, with its fall stopped. The ground measured under it is still the
     * ground under it, so this is no new placement: a banked aircraft that
     * its stance would put higher is not asked about the ground again.
     */
    liftFromImpact(meters: number): void {
      const snapshot = impactSnapshot();
      snapshot.initial["ic/h-sl-ft"] = snapshot.initial["ic/h-sl-ft"]! + meters / 0.3048;
      restoreSimulation(sdk, snapshot);
      proposal = null;
      proposedContact = null;
      holding = false;
      ignoreOverlap = false;
      keptFloor = null;
      blockReason = null;
    },
    isTerrainIgnored: (): boolean => ignoreOverlap,
    getBlockReason: (): TerrainBlockReason => blockReason,
    /** The kernel the support plane is measured on, and why it is not the WASM one if it was asked for. */
    status: contacts.status,
    /** Releases the contacts' property batch and the WASM kernel; the core stays the caller's. */
    dispose(): void {
      contacts.dispose();
    },
    /**
     * `allowGoogleTerrainSurface` is the caller's World-detail policy. Google
     * `geometricError` describes tile simplification, not a measured vertical
     * accuracy, so terrain contact deliberately does not infer a metre-level
     * safety decision from that metadata. `allowUninitialisedSurface` is used
     * only after spawn preparation has already placed the aircraft on known
     * ground; it prevents streaming gaps from becoming a second readiness gate.
     */
    update(
      blockOnMissingSurface = true,
      googleTiles = false,
      allowGoogleTerrainSurface = false,
      allowUninitialisedSurface = false,
    ): boolean | "reset" {
      const lat = sdk.getPropertyValue("position/lat-geod-deg"), lon = sdk.getPropertyValue("position/long-gc-deg");
      const hit = surface.sample(lat, lon);
      const surfaceKnown = hit !== null && Number.isFinite(hit.heightMeters);
      if (proposal && holding) {
        withholdSupport();
        blockReason = "relocation";
        return false;
      }
      if (proposal) {
        if (surfaceKnown && fullyExterior(lat, lon, hit.heightMeters)) {
          // It took off and climbed clear of the surface: nothing is left to move.
          proposal = null;
          proposedContact = null;
          keptFloor = null;
          clearContactHistory();
        } else {
          // It stood on its floor when the world changed and stays on it
          // while asked; physics carries on.
          if (keptFloor !== null) setFloor(keptFloor);
          blockReason = null;
          return true;
        }
      }
      if (ignoreOverlap) {
        if (surfaceKnown && (fullyExterior(lat, lon, hit.heightMeters)
          || (keptFloor !== null && overlapGone(lat, lon, hit.heightMeters, keptFloor)))) {
          ignoreOverlap = false;
          keptFloor = null;
          clearContactHistory();
        } else {
          if (keptFloor !== null) setFloor(keptFloor);
          else withholdSupport();
          blockReason = null;
          return true;
        }
      }
      if (!hit || !Number.isFinite(hit.heightMeters)) {
        const knownGround = previous?.ground ?? sdk.getPropertyValue("position/terrain-elevation-asl-ft") * 0.3048;
        const aboveGround = sdk.getPropertyValue("position/h-sl-ft") * 0.3048 - knownGround;
        if (Number.isFinite(aboveGround) && aboveGround > SURFACE_REQUIRED_AGL_METERS) {
          // Nothing the gear can reach; the surface is irrelevant this frame.
          if (blindSteps === BLIND_STEPS) {
            flightLog.info("terrain", "Flying without a measured surface", {
              aboveGroundMeters: Number(aboveGround.toFixed(0)),
              knownGroundMeters: Number(knownGround.toFixed(2)),
            });
          }
          blindSteps += 1;
          return true;
        }
        // Google only exposes currently visible photogrammetry, so a transient
        // miss AFTER a height has been established must not halt the
        // simulation. Raster keeps its strict behaviour: it has an independent
        // adopted coverage index and a miss means no surface is safe to
        // integrate against.
        //
        // Before any height has been established the two are the same, and
        // neither may proceed: JSBSim's terrain elevation is still unset, so
        // stepping drops the aircraft toward a ground plane that is not there,
        // and the gear model resolves the accumulated penetration explosively
        // the moment real terrain arrives.
        if (!hasReference) {
          if (allowUninitialisedSurface) {
            blindSteps += 1;
            blockReason = null;
            return true;
          }
          blockReason = "missing";
          return false;
        }
        // Google only publishes currently visible mesh. Once the selected
        // World detail meets the flight requirement, a culling/replacement
        // miss may continue on the previously measured floor.
        if (googleTiles && allowGoogleTerrainSurface) {
          blindSteps += 1;
          return true;
        }
        blindSteps += 1;
        if (blockOnMissingSurface) blockReason = "missing";
        return !blockOnMissingSurface;
      }
      if (googleTiles && !allowGoogleTerrainSurface) {
        // The pilot selected a renderer target coarser than the flight
        // requirement. This is a policy hold, not a claim about this tile's
        // vertical error.
        blockReason = "coarse";
        return false;
      }
      blockReason = null;
      if (!hasReference) {
        hasReference = true;
        flightLog.info("terrain", "First terrain height established", {
          heightMeters: Number(hit.heightMeters.toFixed(2)), latDeg: Number(lat.toFixed(5)), lonDeg: Number(lon.toFixed(5)),
        });
      }
      const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
      const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
      const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
      const clearance = aircraftClearanceMeters(clearanceProfile, roll, pitch);
      // Judged on the raw sample, before any filter adopts it.
      const penetration = hit.heightMeters + clearance - altitude;
      // Placement is exempt from both limits: spawning at a mountain airport
      // legitimately puts the aircraft far below a surface it has never seen.
      const stepMeters = previous ? hit.heightMeters - previous.height : 0;
      const implausible = placement ? null
        : penetration > MAX_BELIEVABLE_RISE_METERS
          ? `${penetration.toFixed(0)} m above the aircraft`
          : stepMeters > MAX_BELIEVABLE_STEP_METERS
            ? `${stepMeters.toFixed(0)} m from the previous sample`
            : null;
      if (implausible) {
        if (rejectedSamples === 0) {
          flightLog.error("terrain", `Ignoring a surface sample ${implausible}`, {
            aircraftMeters: Number(altitude.toFixed(2)), terrainMeters: Number(hit.heightMeters.toFixed(2)),
            previousTerrainMeters: previous ? Number(previous.height.toFixed(2)) : null,
            latDeg: Number(lat.toFixed(5)), lonDeg: Number(lon.toFixed(5)),
            meshId: hit.meshId, revision: hit.revision, quality: hit.quality,
          });
        }
        rejectedSamples += 1;
        // Keep the previous terrain elevation rather than adopting this one.
        // The aircraft is flying on a height it can no longer confirm.
        blindSteps += 1;
        return !blockOnMissingSurface;
      }
      if (rejectedSamples > 0) {
        flightLog.info("terrain", "Surface samples are believable again", { ignoredSamples: rejectedSamples });
        rejectedSamples = 0;
      }
      const revisionChanged = previous !== null && hit.revision !== previous.revision;
      // NaN: the world changed and the change could not be measured.
      let cgDelta = 0;
      if (previous && revisionChanged) {
        // Compare two versions at ONE coordinate, never two heights along a flight path.
        const anchor = surface.sample(previous.lat, previous.lon);
        cgDelta = anchor && Number.isFinite(anchor.heightMeters) ? anchor.heightMeters - previous.height : Number.NaN;
      }
      // A wheel rides its own footprint, not the point under the axle.
      const movedMeters = previous ? Math.hypot((lat - previous.lat) * 111320,
        (((lon - previous.lon + 540) % 360) - 180) * Math.cos(lat * Math.PI / 180) * 111320) : 0;
      // A measured refinement moves the filtered ground with the surface, so the
      // wheel keeps bridging the noise it was bridging instead of jolting.
      if (revisionChanged && Number.isFinite(cgDelta)) wheelGround.shift(cgDelta);
      const cgGround = wheelGround.height(hit.heightMeters, movedMeters,
        placement || blindSteps > 0 || (revisionChanged && !Number.isFinite(cgDelta))
        || altitude - hit.heightMeters > WHEELS_CLEAR_METERS);
      const fresh = placement || blindSteps > 0;
      supportInput.lat = lat; supportInput.lon = lon; supportInput.altitude = altitude;
      supportInput.roll = roll; supportInput.pitch = pitch;
      supportInput.heading = sdk.getPropertyValue("attitude/psi-deg") * Math.PI / 180;
      supportInput.cgGround = cgGround;
      supportInput.revision = hit.revision;
      supportInput.revisionChanged = revisionChanged;
      supportInput.cgDelta = cgDelta;
      supportInput.urgent = !googleTiles || fresh || !(Math.abs(cgDelta) <= 0.00001);
      supportInput.snap = fresh;
      supportInput.samples = googleTiles && !fresh ? GOOGLE_FOOTPRINT_SAMPLES_PER_STEP : Number.POSITIVE_INFINITY;
      supportInput.previousElevation = previous ? previous.ground : Number.NaN;
      const measured = contacts.measure(supportInput);
      const ground = Number.isFinite(measured.elevation) ? measured.elevation : cgGround;
      const refinement = measured.refinement;
      const support = ground + clearance;
      // A changed world moves a resting aircraft with it, and lifts one it
      // came up through. Moving by the change itself keeps the oleo
      // compression and the integrator history; any penetration that was not
      // the world's doing stays, so an aircraft that flew into a hill while
      // some other tile refined is not lifted out of it.
      const changedHere = Math.abs(refinement) > 0.00001;
      // The floor JSBSim took the last step on; this update has not replaced it yet.
      const floor = floorMeters();
      // Judged by the contacts, not the stance: a drone's legs say whether it
      // is on the ground or hovering a few centimetres over it.
      let depth = Number.NaN;
      const contactDepth = (): number => Number.isNaN(depth) ? (depth = lowestContactMeters(roll, pitch, clearance)) : depth;
      const standing = (): boolean => altitude > floor && altitude - contactDepth() - floor <= STANDING_GAP_METERS;
      const followGround = changedHere && previous !== null
        && Math.abs(sdk.getPropertyValue("velocities/v-down-fps")) < 3 && standing();
      // The new plane is above where the aircraft would rest on it, and above
      // its lowest contact: the ground came up through it.
      const liftedThroughPlane = refinement > 0 && previous !== null && previous.clearance >= -0.5
        && altitude < support && altitude < ground + contactDepth();
      // Deliberately NOT a cause: an aircraft that flew into a hill in full
      // view of the surface is deep below it too, and lifting it out would fly
      // it through terrain. Only a world that changed under a level aircraft,
      // or one the aircraft could not see, is repaired here.
      // Regaining the surface only repairs a real penetration of the ground
      // itself, not the centimetres of oleo travel that separate a settled
      // aircraft from its nominal stance.
      const regained = blindSteps >= BLIND_STEPS && altitude < ground;
      // An aircraft standing on this very ground rests there, whatever its
      // nominal stance says: its contacts settle it, not the stance.
      const misplaced = placement && altitude < support - PLACEMENT_TOLERANCE_METERS
        && !(ground <= floor + PLACEMENT_TOLERANCE_METERS && standing());
      const cause = misplaced ? "placement" : regained ? "surface regained above the aircraft" : null;
      const follow = !cause && (followGround || liftedThroughPlane);
      blindSteps = 0;
      if (cause || follow) {
        const target = cause ? support : altitude + refinement;
        proposal = {
          id: ++proposalId,
          reason: cause ?? "surface refinement",
          currentAltitudeMeters: altitude,
          altitudeMeters: target,
          groundHeightMeters: ground,
          deltaMeters: target - altitude,
        };
        proposedContact = { lat, lon, height: hit.heightMeters, ground, revision: hit.revision,
          clearance: target - ground - clearance };
        // A small correction is made at once: the aircraft floats onto the
        // ground instead of stopping to ask about centimetres.
        const automatic = options.automaticMoveMeters?.() ?? 0;
        if (Math.abs(target - altitude) <= automatic) {
          flightLog.info("terrain", "Moved onto the ground without asking", {
            reason: proposal.reason, deltaMeters: Number((target - altitude).toFixed(3)), limitMeters: automatic,
          });
          applyRelocation(proposal);
          return "reset";
        }
        // An aircraft standing on its floor keeps it and keeps flying while
        // asked: holding it would freeze it, and withholding its floor would
        // drop it. Any other is held; its floor is given back if the pilot
        // stays, so it can still land where the ground was.
        holding = !standing();
        keptFloor = holding ? previous?.ground ?? null : floor;
        if (holding) {
          withholdSupport();
          blockReason = "relocation";
          return false;
        }
        blockReason = null;
        return true;
      }
      sdk.setPropertyValue("position/terrain-elevation-asl-ft", ground / 0.3048);
      const height = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
      if (previous) {
        previous.lat = lat; previous.lon = lon; previous.height = hit.heightMeters; previous.ground = ground;
        previous.revision = hit.revision; previous.clearance = height - ground - clearance;
      } else {
        previous = { lat, lon, height: hit.heightMeters, ground, revision: hit.revision,
          clearance: height - ground - clearance };
      }
      placement = false;
      return true;
    },
  };
}

/** Feed the displayed raster surface into JSBSim's existing gear contacts.
 * A miss blocks integration; it must never reuse another airport's ground.
 * This is terrain support, not a general aircraft-versus-building solver. */
export function syncTerrainContact(sdk: JSBSimSdk, surface: SurfaceQuery): boolean {
  const hit = surface.sample(sdk.getPropertyValue("position/lat-geod-deg"), sdk.getPropertyValue("position/long-gc-deg"));
  if (!hit || !Number.isFinite(hit.heightMeters)) return false;
  sdk.setPropertyValue("position/terrain-elevation-asl-ft", hit.heightMeters / 0.3048);
  return true;
}
