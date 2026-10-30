import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import { DEG_TO_RAD, ecefToGeodetic } from "foss-earth/cameraMath";
import type { SurfaceQuery } from "foss-earth/runtime";
import { bodyToLocalRotation, MAX_BODY_COLLISION_ALTITUDE_METERS, type BodyCollisionProbe } from "./collisionGeometry";
import { createKernelSlot, type GroundContactCoreSource, type WasmHullCollision } from "./groundContactCore";
import { createPropertyReadBatch } from "./propertyReadBatch";
import { captureSimulation, restoreSimulation } from "./safeFlightState";

const CONTACT_CLEARANCE_METERS = 0.01;
const RESTITUTION = 0.25;
/** Coulomb friction of an airframe sliding on what it struck, such as metal on concrete or rock. */
const HULL_FRICTION = 0.5;
const METERS_PER_FOOT = 0.3048;
const FEET_PER_METER = 1 / METERS_PER_FOOT;
const FEET_PER_INCH = 1 / 12;
const WGS84_A = 6378137;
const WGS84_E2 = 6.69437999014e-3;

// One native read per physics step. Velocities are ground-relative NED and
// body rates; inertia is JSBSim's live tensor about the live CG.
const PATHS = [
  "position/lat-geod-deg", "position/long-gc-deg", "position/h-sl-ft",
  "attitude/phi-deg", "attitude/theta-deg", "attitude/psi-deg",
  "velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps",
  "velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec",
  "inertia/cg-x-in", "inertia/cg-y-in", "inertia/cg-z-in", "inertia/mass-slugs",
  "inertia/ixx-slugs_ft2", "inertia/iyy-slugs_ft2", "inertia/izz-slugs_ft2",
  "inertia/ixy-slugs_ft2", "inertia/ixz-slugs_ft2", "inertia/iyz-slugs_ft2",
] as const;
const LAT = 0, LON = 1, ALT = 2, ROLL = 3, PITCH = 4, HEADING = 5, NORTH = 6, EAST = 7, DOWN = 8;
const P = 9, Q = 10, R = 11, CG_X = 12, CG_Y = 13, CG_Z = 14, MASS = 15;
const IXX = 16, IYY = 17, IZZ = 18, IXY = 19, IXZ = 20, IYZ = 21;

export interface VisibleMeshCollisionOptions {
  /** The aircraft's hull probes, in structural inches. */
  bodyProbes: readonly BodyCollisionProbe[];
  /** Values above one deliberately add energy, for the optional arcade mode. */
  getRestitution?: () => number;
  /** Coulomb friction between the hull and what it struck. Default 0.5. */
  friction?: number;
  /**
   * A WASM ground-contact core, or its load. The sweep's arithmetic and the
   * impact then run on its hull kernel; the raycasts and JSBSim stay here.
   */
  core?: GroundContactCoreSource;
}

const SEGMENT_STRIDE = 7;
const HIT_STRIDE = 7;
const MIN_SWEEP_METERS = 0.0001;

/**
 * Sweeps each hull probe from its last position to its current one against
 * the visible surface. A hit is a rigid-body impact at that point: a normal
 * impulse with restitution and a Coulomb friction impulse change both the
 * CG velocity (J / m) and the body rates (I⁻¹ (r × J)), and the aircraft is
 * moved out along the surface normal. JSBSim is then restored at that state.
 */
export function createVisibleMeshCollision(sdk: JSBSimSdk, surface: SurfaceQuery, options: VisibleMeshCollisionOptions) {
  const probes = options.bodyProbes;
  const count = probes.length;
  const friction = Number.isFinite(options.friction) ? Math.max(0, options.friction!) : HULL_FRICTION;
  let reader: ReturnType<typeof createPropertyReadBatch> | null = null;
  const values = new Float64Array(PATHS.length);
  // Probe positions in ECEF metres, three per probe: before and after the step.
  let previous = new Float64Array(count * 3), current = new Float64Array(count * 3);
  let hasPrevious = false;
  // The pose: CG in ECEF, body (forward, right, down) to local NED, and body to ECEF.
  const cg = new Float64Array(3), bodyToLocal = new Float64Array(9), bodyToEcef = new Float64Array(9);
  // This step's hits, one at most per probe, in the order they occur along the step.
  const hitProbe = new Int32Array(count), hitFraction = new Float64Array(count);
  const hitPoint = new Float64Array(count * 3), hitNormal = new Float64Array(count * 3);
  // Rigid-body state while impacts are resolved: body-axis feet, slugs and radians.
  const velocity = new Float64Array(3), rate = new Float64Array(3), inverseInertia = new Float64Array(9);
  const lever = new Float64Array(3), normal = new Float64Array(3), tangent = new Float64Array(3);
  const arm = new Float64Array(3), turn = new Float64Array(3), shift = new Float64Array(3);
  // The surface query reads its arguments and keeps nothing; one pair serves every probe.
  const from = { x: 0, y: 0, z: 0 }, movement = { x: 0, y: 0, z: 0 };
  const slot = createKernelSlot(options.core, "wasm-hull", core => {
    const kernel = core.createHullCollision(count);
    const rows = kernel.probes;
    for (let i = 0; i < count; i++) {
      rows[i * 4] = probes[i].xIn; rows[i * 4 + 1] = probes[i].yIn; rows[i * 4 + 2] = probes[i].zIn; rows[i * 4 + 3] = probes[i].insetIn;
    }
    return kernel;
  });
  // Which kernel holds the previous points: null for this module's arrays.
  let previousIn: WasmHullCollision | null = null;
  let terrainOverlapIgnored = false;
  // Set while the surface or JSBSim is called from the WASM path, whose
  // errors are the caller's, as on the reference path, not the kernel's.
  let inHost = false;

  function read(): boolean {
    try {
      reader ??= createPropertyReadBatch(sdk, PATHS);
      reader.readInto(values);
    } catch {
      // A model load disposes the native batch; resolve the paths again next step.
      reader?.dispose();
      reader = null;
      return false;
    }
    return true;
  }

  /** The pose from the values just read, and every probe's position into `out`. */
  function place(out: Float64Array): boolean {
    for (let i = 0; i < PATHS.length; i++) if (!Number.isFinite(values[i])) return false;
    const lat = values[LAT] * DEG_TO_RAD, lon = values[LON] * DEG_TO_RAD, alt = values[ALT] * METERS_PER_FOOT;
    const sinLat = Math.sin(lat), cosLat = Math.cos(lat), sinLon = Math.sin(lon), cosLon = Math.cos(lon);
    const n = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
    cg[0] = (n + alt) * cosLat * cosLon;
    cg[1] = (n + alt) * cosLat * sinLon;
    cg[2] = (n * (1 - WGS84_E2) + alt) * sinLat;
    bodyToLocalRotation(values[ROLL] * DEG_TO_RAD, values[PITCH] * DEG_TO_RAD, values[HEADING] * DEG_TO_RAD, bodyToLocal);
    // Columns of the local frame in ECEF: north, east, down.
    const nx = -sinLat * cosLon, ny = -sinLat * sinLon, nz = cosLat;
    const ex = -sinLon, ey = cosLon;
    const dx = -cosLat * cosLon, dy = -cosLat * sinLon, dz = -sinLat;
    for (let column = 0; column < 3; column++) {
      const north = bodyToLocal[column], east = bodyToLocal[3 + column], down = bodyToLocal[6 + column];
      bodyToEcef[column] = nx * north + ex * east + dx * down;
      bodyToEcef[3 + column] = ny * north + ey * east + dy * down;
      bodyToEcef[6 + column] = nz * north + dz * down;
    }
    for (let i = 0; i < count; i++) {
      probeLever(i, lever);
      out[i * 3] = cg[0] + (bodyToEcef[0] * lever[0] + bodyToEcef[1] * lever[1] + bodyToEcef[2] * lever[2]) * METERS_PER_FOOT;
      out[i * 3 + 1] = cg[1] + (bodyToEcef[3] * lever[0] + bodyToEcef[4] * lever[1] + bodyToEcef[5] * lever[2]) * METERS_PER_FOOT;
      out[i * 3 + 2] = cg[2] + (bodyToEcef[6] * lever[0] + bodyToEcef[7] * lever[1] + bodyToEcef[8] * lever[2]) * METERS_PER_FOOT;
    }
    return true;
  }

  /** A probe's swept point from the live CG, in body feet: forward, right, down. */
  function probeLever(index: number, out: Float64Array): void {
    const probe = probes[index];
    out[0] = (values[CG_X] - probe.xIn) * FEET_PER_INCH;
    out[1] = (probe.yIn - values[CG_Y]) * FEET_PER_INCH;
    out[2] = (values[CG_Z] - probe.zIn - probe.insetIn) * FEET_PER_INCH;
  }

  function settle(): void {
    hasPrevious = read() && place(previous);
    previousIn = null;
  }

  /** `sweep`'s casts for the kernel: each probe's hit as the surface returned it, NaN for none. */
  function cast(kernel: WasmHullCollision): boolean {
    const segments = kernel.segments, rows = kernel.hits;
    let any = false;
    for (let i = 0; i < count; i++) {
      const at = i * SEGMENT_STRIDE, row = i * HIT_STRIDE;
      const distance = segments[at + 6];
      rows[row] = Number.NaN;
      if (!Number.isFinite(distance) || distance < MIN_SWEEP_METERS) continue;
      from.x = segments[at]; from.y = segments[at + 1]; from.z = segments[at + 2];
      movement.x = segments[at + 3]; movement.y = segments[at + 4]; movement.z = segments[at + 5];
      inHost = true;
      const hit = surface.raycast(from, movement, distance);
      inHost = false;
      if (!hit) continue;
      rows[row] = hit.distanceMeters;
      rows[row + 1] = hit.point.x; rows[row + 2] = hit.point.y; rows[row + 3] = hit.point.z;
      rows[row + 4] = hit.normal.x; rows[row + 5] = hit.normal.y; rows[row + 6] = hit.normal.z;
      any = true;
    }
    return any;
  }

  /**
   * `update` on the kernel: it places the probes, judges the hits and
   * resolves them; the casts and the restore stay here. Null when the kernel
   * failed before JSBSim was touched, for the reference to take the step.
   */
  function updateWasm(kernel: WasmHullCollision): boolean | null {
    let restored = false;
    try {
      if (!read()) { hasPrevious = false; return false; }
      kernel.values.set(values);
      if (!kernel.place()) { hasPrevious = false; return false; }
      inHost = true;
      const support = surface.sample(values[LAT], values[LON]);
      inHost = false;
      if (!hasPrevious || previousIn !== kernel || !support
        || values[ALT] * METERS_PER_FOOT - support.heightMeters > MAX_BODY_COLLISION_ALTITUDE_METERS) {
        kernel.swap();
        hasPrevious = true;
        previousIn = kernel;
        return false;
      }
      if (!cast(kernel)) { kernel.swap(); return false; }
      inHost = true;
      const restitution = options.getRestitution?.() ?? Number.NaN;
      inHost = false;
      if (kernel.resolve(restitution, Number.isFinite(options.friction) ? options.friction : Number.NaN) === 0) {
        kernel.swap();
        return false;
      }
      const output = kernel.output;
      inHost = true;
      const snapshot = captureSimulation(sdk);
      snapshot.initial["ic/lat-geod-deg"] = output[1];
      snapshot.initial["ic/long-gc-deg"] = output[2];
      snapshot.initial["ic/h-sl-ft"] = output[3];
      snapshot.initial["ic/vn-fps"] = output[4];
      snapshot.initial["ic/ve-fps"] = output[5];
      snapshot.initial["ic/vd-fps"] = output[6];
      snapshot.initial["ic/p-rad_sec"] = output[7];
      snapshot.initial["ic/q-rad_sec"] = output[8];
      snapshot.initial["ic/r-rad_sec"] = output[9];
      restoreSimulation(sdk, snapshot);
      inHost = false;
      restored = true;
      hasPrevious = read();
      if (hasPrevious) {
        kernel.values.set(values);
        hasPrevious = kernel.place();
        if (hasPrevious) kernel.swap();
      }
      previousIn = kernel;
      return true;
    } catch (error) {
      if (inHost) { inHost = false; throw error; }
      slot.fail(error);
      hasPrevious = false;
      return restored ? true : null;
    }
  }

  /** The earliest crossing of each probe's path this step, in step order. */
  function sweep(): number {
    let hits = 0;
    for (let i = 0; i < count; i++) {
      from.x = previous[i * 3]; from.y = previous[i * 3 + 1]; from.z = previous[i * 3 + 2];
      movement.x = current[i * 3] - from.x; movement.y = current[i * 3 + 1] - from.y; movement.z = current[i * 3 + 2] - from.z;
      // Not Math.hypot, which boxes its result: this runs for every probe every step.
      const distance = Math.sqrt(movement.x * movement.x + movement.y * movement.y + movement.z * movement.z);
      if (!Number.isFinite(distance) || distance < 0.0001) continue;
      const hit = surface.raycast(from, movement, distance);
      if (!hit || !Number.isFinite(hit.distanceMeters) || hit.distanceMeters < 0 || hit.distanceMeters > distance) continue;
      const length = Math.hypot(hit.normal.x, hit.normal.y, hit.normal.z);
      if (!Number.isFinite(length) || length < 1e-10
        || !Number.isFinite(hit.point.x) || !Number.isFinite(hit.point.y) || !Number.isFinite(hit.point.z)) continue;
      // Face the normal against the probe's motion, whichever way the mesh is wound.
      const sign = movement.x * hit.normal.x + movement.y * hit.normal.y + movement.z * hit.normal.z > 0 ? -1 : 1;
      const fraction = hit.distanceMeters / distance;
      let at = hits++;
      for (; at > 0 && hitFraction[at - 1] > fraction; at--) {
        hitFraction[at] = hitFraction[at - 1]; hitProbe[at] = hitProbe[at - 1];
        for (let axis = 0; axis < 3; axis++) {
          hitPoint[at * 3 + axis] = hitPoint[(at - 1) * 3 + axis];
          hitNormal[at * 3 + axis] = hitNormal[(at - 1) * 3 + axis];
        }
      }
      hitFraction[at] = fraction; hitProbe[at] = i;
      hitPoint[at * 3] = hit.point.x; hitPoint[at * 3 + 1] = hit.point.y; hitPoint[at * 3 + 2] = hit.point.z;
      hitNormal[at * 3] = sign * hit.normal.x / length;
      hitNormal[at * 3 + 1] = sign * hit.normal.y / length;
      hitNormal[at * 3 + 2] = sign * hit.normal.z / length;
    }
    return hits;
  }

  /** JSBSim's inertia tensor about the CG in body axes, inverted into `inverseInertia`; zero when singular. */
  function invertInertia(): void {
    // The property getters return -J12, +J13 and -J23 of the native tensor.
    const a = values[IXX], b = -values[IXY], c = values[IXZ];
    const d = values[IYY], e = -values[IYZ], f = values[IZZ];
    const c00 = d * f - e * e, c01 = c * e - b * f, c02 = b * e - c * d;
    const determinant = a * c00 + b * c01 + c * c02;
    if (!(Math.abs(determinant) > 0) || !Number.isFinite(determinant)) { inverseInertia.fill(0); return; }
    const scale = 1 / determinant;
    inverseInertia[0] = c00 * scale; inverseInertia[1] = c01 * scale; inverseInertia[2] = c02 * scale;
    inverseInertia[3] = c01 * scale; inverseInertia[4] = (a * f - c * c) * scale; inverseInertia[5] = (b * c - a * e) * scale;
    inverseInertia[6] = c02 * scale; inverseInertia[7] = (b * c - a * e) * scale; inverseInertia[8] = (a * d - b * b) * scale;
  }

  function cross(a: Float64Array, b: Float64Array, out: Float64Array): void {
    const x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
  }

  function rotateByInverseInertia(vector: Float64Array, out: Float64Array): void {
    const x = vector[0], y = vector[1], z = vector[2];
    out[0] = inverseInertia[0] * x + inverseInertia[1] * y + inverseInertia[2] * z;
    out[1] = inverseInertia[3] * x + inverseInertia[4] * y + inverseInertia[5] * z;
    out[2] = inverseInertia[6] * x + inverseInertia[7] * y + inverseInertia[8] * z;
  }

  /** Velocity of the point at `lever` along `direction`: (v + ω × r) · d. */
  function pointSpeed(direction: Float64Array): number {
    cross(rate, lever, turn);
    return (velocity[0] + turn[0]) * direction[0] + (velocity[1] + turn[1]) * direction[1] + (velocity[2] + turn[2]) * direction[2];
  }

  /** 1 / m_eff along `direction` at `lever`: 1/m + (r × d)ᵀ I⁻¹ (r × d). */
  function inverseEffectiveMass(direction: Float64Array, mass: number): number {
    cross(lever, direction, arm);
    rotateByInverseInertia(arm, turn);
    return 1 / mass + arm[0] * turn[0] + arm[1] * turn[1] + arm[2] * turn[2];
  }

  /** Apply the impulse `magnitude` along `direction` at `lever`: Δv = J/m, Δω = I⁻¹ (r × J). */
  function applyImpulse(direction: Float64Array, magnitude: number, mass: number): void {
    for (let axis = 0; axis < 3; axis++) velocity[axis] += direction[axis] * magnitude / mass;
    cross(lever, direction, arm);
    rotateByInverseInertia(arm, turn);
    for (let axis = 0; axis < 3; axis++) rate[axis] += turn[axis] * magnitude;
  }

  /** Each hit in step order: impulses on the rigid body, then the aircraft moved out along the normal. */
  function resolve(hits: number): void {
    const mass = values[MASS];
    const requestedRestitution = options.getRestitution?.() ?? RESTITUTION;
    const restitution = Number.isFinite(requestedRestitution) ? Math.min(2, Math.max(0, requestedRestitution)) : RESTITUTION;
    invertInertia();
    // Body velocity from the ground-relative NED velocity: v_body = Mᵀ v_local.
    for (let axis = 0; axis < 3; axis++) {
      velocity[axis] = bodyToLocal[axis] * values[NORTH] + bodyToLocal[3 + axis] * values[EAST] + bodyToLocal[6 + axis] * values[DOWN];
    }
    rate[0] = values[P]; rate[1] = values[Q]; rate[2] = values[R];
    shift.fill(0);
    for (let k = 0; k < hits; k++) {
      const i = hitProbe[k];
      probeLever(i, lever);
      // The ECEF normal in body axes.
      for (let axis = 0; axis < 3; axis++) {
        normal[axis] = bodyToEcef[axis] * hitNormal[k * 3] + bodyToEcef[3 + axis] * hitNormal[k * 3 + 1] + bodyToEcef[6 + axis] * hitNormal[k * 3 + 2];
      }
      const approach = pointSpeed(normal);
      if (approach < 0 && mass > 0) {
        const normalImpulse = -(1 + restitution) * approach / inverseEffectiveMass(normal, mass);
        applyImpulse(normal, normalImpulse, mass);
        // Coulomb friction opposes the point's sliding after the normal impulse, up to mu j_n.
        const along = pointSpeed(normal);
        cross(rate, lever, turn);
        for (let axis = 0; axis < 3; axis++) tangent[axis] = velocity[axis] + turn[axis] - along * normal[axis];
        const sliding = Math.hypot(tangent[0], tangent[1], tangent[2]);
        if (sliding > 1e-9) {
          for (let axis = 0; axis < 3; axis++) tangent[axis] /= -sliding;
          const frictionImpulse = Math.min(sliding / inverseEffectiveMass(tangent, mass), friction * normalImpulse);
          applyImpulse(tangent, frictionImpulse, mass);
        }
      }
      // Out of the surface: the swept point, and the hull point it sits inside, clear of it.
      const nx = hitNormal[k * 3], ny = hitNormal[k * 3 + 1], nz = hitNormal[k * 3 + 2];
      const depth = (hitPoint[k * 3] - current[i * 3] - shift[0]) * nx + (hitPoint[k * 3 + 1] - current[i * 3 + 1] - shift[1]) * ny
        + (hitPoint[k * 3 + 2] - current[i * 3 + 2] - shift[2]) * nz;
      // The hull point lies the inset along body down, the third column of body-to-ECEF.
      const insetMeters = probes[i].insetIn * 0.0254;
      const hullDepth = depth - insetMeters * (bodyToEcef[2] * nx + bodyToEcef[5] * ny + bodyToEcef[8] * nz);
      const lift = Math.max(depth, hullDepth) + CONTACT_CLEARANCE_METERS;
      if (lift > 0) { shift[0] += lift * nx; shift[1] += lift * ny; shift[2] += lift * nz; }
    }
  }

  return {
    reset(): void { hasPrevious = false; },
    /** A user-acknowledged world overlap; resets sweeps at both boundaries. */
    setTerrainOverlapIgnored(ignored: boolean): void {
      if (terrainOverlapIgnored === ignored) return;
      terrainOverlapIgnored = ignored;
      hasPrevious = false;
    },
    update(): boolean {
      // The terrain contact owner checks the full aircraft footprint before
      // ending this episode. Do not turn an inside-to-outside crossing into
      // an underside impact, or sweep across the ignored interval afterwards.
      if (terrainOverlapIgnored) { hasPrevious = false; return false; }
      const kernel = slot.current;
      if (kernel) {
        const updated = updateWasm(kernel);
        if (updated !== null) return updated;
      }
      if (!read() || !place(current)) { hasPrevious = false; return false; }
      const support = surface.sample(values[LAT], values[LON]);
      if (!hasPrevious || previousIn !== null || !support
        || values[ALT] * METERS_PER_FOOT - support.heightMeters > MAX_BODY_COLLISION_ALTITUDE_METERS) {
        const swap = previous; previous = current; current = swap;
        hasPrevious = true;
        previousIn = null;
        return false;
      }
      const hits = sweep();
      if (hits === 0) {
        const swap = previous; previous = current; current = swap;
        return false;
      }
      resolve(hits);
      const position = ecefToGeodetic(cg[0] + shift[0], cg[1] + shift[1], cg[2] + shift[2]);
      // Terrain contact may have just adopted a newer surface. Capture now, so
      // that floor, the attitude and the controls are the current ones.
      const snapshot = captureSimulation(sdk);
      snapshot.initial["ic/lat-geod-deg"] = position.latRad / DEG_TO_RAD;
      snapshot.initial["ic/long-gc-deg"] = position.lonRad / DEG_TO_RAD;
      snapshot.initial["ic/h-sl-ft"] = position.altMeters * FEET_PER_METER;
      // The velocity in ECEF, then in the local frame where the CG now is.
      const vx = bodyToEcef[0] * velocity[0] + bodyToEcef[1] * velocity[1] + bodyToEcef[2] * velocity[2];
      const vy = bodyToEcef[3] * velocity[0] + bodyToEcef[4] * velocity[1] + bodyToEcef[5] * velocity[2];
      const vz = bodyToEcef[6] * velocity[0] + bodyToEcef[7] * velocity[1] + bodyToEcef[8] * velocity[2];
      const sinLat = Math.sin(position.latRad), cosLat = Math.cos(position.latRad);
      const sinLon = Math.sin(position.lonRad), cosLon = Math.cos(position.lonRad);
      snapshot.initial["ic/vn-fps"] = -sinLat * cosLon * vx - sinLat * sinLon * vy + cosLat * vz;
      snapshot.initial["ic/ve-fps"] = -sinLon * vx + cosLon * vy;
      snapshot.initial["ic/vd-fps"] = -cosLat * cosLon * vx - cosLat * sinLon * vy - sinLat * vz;
      snapshot.initial["ic/p-rad_sec"] = rate[0];
      snapshot.initial["ic/q-rad_sec"] = rate[1];
      snapshot.initial["ic/r-rad_sec"] = rate[2];
      restoreSimulation(sdk, snapshot);
      settle();
      return true;
    },
    /** The kernel the sweeps resolve on, and why it is not the WASM one if it was asked for. */
    status: slot.status,
    /** Release the property batch and the WASM kernel; call before the SDK is destroyed. The core stays the caller's. */
    dispose(): void {
      reader?.dispose();
      reader = null;
      hasPrevious = false;
      slot.dispose();
    },
  };
}
