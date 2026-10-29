/** Row layouts, in f64 values; the Rust modules document every field. */
export const HULL: Readonly<{ valuesLength: 22; probeStride: 4; poseLength: 21; segmentStride: 7; hitStride: 7; headLength: 16; rowStride: 9; maxProbes: 64; minSweepMeters: 0.0001 }>;
export const SUPPORT: Readonly<{
  unitStride: 2; inputLength: 14; valuesHead: 5; requestStride: 6; outputLength: 6; globals: 30; contactStride: 24; maxContacts: 64;
  GEOMETRY_READ: 0; GEOMETRY_FAILED: 1; GEOMETRY_UNAVAILABLE: 2;
  state: Readonly<{ DISCOVERED: 0; LIVE: 1; HAD_SUPPORT: 2; REVALIDATE_IN: 6; STEPS: 7; REACH: 8; PHASE: 9; REQUESTS: 13 }>;
  contact: Readonly<{ FILTER: 0; DOWN: 2; FOOT_LAT: 3; FOOT_LON: 4; HELD_RAW: 7; GROUND: 11; MARGIN: 12; ACTIVE: 19; SUPPORTED: 20 }>;
}>;
export const WHEELS: Readonly<{ bodyLength: 16; wheelStride: 10; contactStride: 25; reportLength: 5; resultStride: 11; maxWheels: 16; maxContacts: 64 }>;
export const SPIN: Readonly<{ wheelStride: 20; stateOffset: 0; configOffset: 9; inputOffset: 14; maxWheels: 16; INSTANT: 0; INERTIA: 1 }>;
export const ERRORS: Readonly<{ INVALID: 1; ALLOCATION: 2; INPUT: 3 }>;

/** Views live in WASM memory and stay valid until dispose(); re-read the getter after creating another kernel. */
interface Kernel { dispose(): void }

/**
 * `createVisibleMeshCollision` without the SDK and the surface. Per step: read the 22 properties into
 * `values`; place(); on the first step, out of range or with no surface sample, swap(); else raycast each
 * segment at least `HULL.minSweepMeters` long into `hits` (NaN distance for none), resolve(); with hits,
 * restore JSBSim from `output`, read again, place() and swap(); with none, swap().
 */
export interface HullCollision extends Kernel {
  readonly probeCount: number;
  /** visibleMeshCollision.ts's PATHS, in order. */
  readonly values: Float64Array;
  /** Per probe [x, y, z, inset] structural inches. */
  readonly probes: Float64Array;
  /** [cg ECEF m 3, body-to-local 9, body-to-ECEF 9]. */
  readonly pose: Float64Array;
  /** ECEF metres, 3 per probe. */
  readonly previous: Float64Array;
  readonly current: Float64Array;
  /** Per probe [from xyz, movement xyz, distance m]. */
  readonly segments: Float64Array;
  /** Per probe [distance m, point ECEF xyz, normal ECEF xyz], as the raycast returned it. */
  readonly hits: Float64Array;
  /** [hits, latDeg, lonDeg, h-sl-ft, vn, ve, vd fps, p, q, r, shift ECEF xyz, body velocity fps xyz], then 9 per hit. */
  readonly output: Float64Array;
  /** False, with nothing written, when a value is not finite. */
  place(): boolean;
  swap(): void;
  /** The hit count. NaN restitution or friction for the TypeScript defaults (0.25, 0.5). */
  resolve(restitution?: number, friction?: number): number;
}

export interface SupportUnit { retractable: boolean; wheel?: boolean; prefix?: string }
export interface SurfaceSample { heightMeters: number; revision: number }

/**
 * terrainContact.ts's createContactSupport for one model's units. Per step: fill `input`; if wantsGeometry(),
 * read [gear/num-units, gear/gear-pos-norm, inertia/cg-x/y/z-in, then each unit's x/y/z-position] into
 * `values` (GEOMETRY_FAILED if the read threw, GEOMETRY_UNAVAILABLE if unit discovery failed); then
 * measure(geometry, sample), or plan(), sample each request row, finish().
 */
export interface ContactSupport extends Kernel {
  readonly contactCount: number;
  readonly units: Float64Array;
  /** [lat, lon, altitude, roll, pitch, heading, cgGround, revision, revisionChanged, cgDelta, urgent, snap, samples, previousElevation]. */
  readonly input: Float64Array;
  readonly values: Float64Array;
  readonly state: Float64Array;
  /** Per request [contact, kind, latDeg, lonDeg, heightMeters, revision]; the host writes the last two (NaN: no sample). */
  readonly requests: Float64Array;
  /** [elevation (NaN: the centre's ground stands in), refinement, lowest, inRange, requests, applied]. */
  readonly output: Float64Array;
  /** False once a failed read or a changed unit count has forgotten the contacts: rediscover the units. */
  readonly discovered: boolean;
  wantsGeometry(): boolean;
  /** The request count. */
  plan(geometry: 0 | 1 | 2): number;
  finish(): Float64Array;
  measure(geometry: 0 | 1 | 2, sample: (latDeg: number, lonDeg: number) => SurfaceSample | null | undefined): Float64Array;
  reset(): void;
}

/** `solveCoupledRigidWheels` on flat rows; body and wheel states update in place. */
export interface WheelSolver extends Kernel {
  readonly wheelCount: number;
  readonly maxContacts: number;
  readonly body: Float64Array;
  readonly wheels: Float64Array;
  readonly contacts: Float64Array;
  readonly output: Float64Array;
  /** `iterations` NaN means absent (8). */
  run(dtSeconds: number, contactCount: number, iterations?: number, warmStart?: boolean): void;
}

/** `stepWheelSpin` per wheel row [state 9, config 5, input 6]; onGround is 0 or 1. */
export interface WheelSpin extends Kernel {
  readonly wheelCount: number;
  readonly wheels: Float64Array;
  run(dtSeconds: number, mode: "instant" | "inertia" | 0 | 1): void;
}

export interface GroundContactCore {
  readonly backend: "single";
  readonly validationStatus: "unvalidated";
  readonly abiVersion: 2;
  heapAllocations(): number;
  /** The core's libm, which can differ from V8's Math by an ulp. */
  readonly wasmMath: Readonly<{ sin(x: number): number; cos(x: number): number; atan2(y: number, x: number): number }>;
  createHullCollision(probeCount: number): HullCollision;
  createContactSupport(units: readonly SupportUnit[], previous?: ContactSupport): ContactSupport;
  createWheelSolver(wheelCount: number, maxContacts?: number): WheelSolver;
  createWheelSpin(wheelCount: number): WheelSpin;
  dispose(): void;
}

export function loadGroundContact(options?: { wasmUrl?: URL | string; fetch?: typeof fetch }): Promise<GroundContactCore>;
export function instantiateGroundContact(bytes: BufferSource | WebAssembly.Module): Promise<GroundContactCore>;
export function createGroundContact(exports: WebAssembly.Exports): GroundContactCore;
