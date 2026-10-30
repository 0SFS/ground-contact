import { readFileSync } from "node:fs";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { DEG_TO_RAD, ecefToGeodetic, geodeticToEcef } from "foss-earth/cameraMath";
import type { SurfaceQuery } from "foss-earth/runtime";
import { bodyToLocalRotation, type BodyCollisionProbe } from "../src/collisionGeometry";
import type { GroundContactCore } from "../src/groundContactCore";
import { createVisibleMeshCollision as createCollision, type VisibleMeshCollisionOptions } from "../src/visibleMeshCollision";
import { instantiateGroundContact } from "../wasm/index.js";

const wasm = () => instantiateGroundContact(readFileSync(new URL("../wasm/single/ground_contact.wasm", import.meta.url)));
// A core per test: one holds a bounded number of live kernels, and a test's
// collisions are not all disposed.
let wasmCore: Awaited<ReturnType<typeof wasm>>;
beforeEach(async () => { wasmCore = await wasm(); });
afterEach(() => wasmCore.dispose());

/** The kernel the shared tests run on: the TypeScript reference, then the WASM one. */
const active: { core?: GroundContactCore } = {};
function createVisibleMeshCollision(sdk: never, surface: SurfaceQuery, options: VisibleMeshCollisionOptions) {
  return createCollision(sdk, surface, { core: active.core, ...options });
}

type Vector = { x: number; y: number; z: number };

// The CG is at (100, 0, 0) structural inches: x aft, y right, z up.
const probe = (name: string, xIn: number, yIn: number, zIn: number, insetIn = 0): BodyCollisionProbe => ({ name, xIn, yIn, zIn, insetIn });
const CENTER = probe("center", 100, 0, 0);
const NOSE = probe("nose", 0, 0, 0);
const TAIL = probe("tail", 220, 0, 0);
const RIGHT_WING = probe("right-wing", 100, 120, 0);
const LEFT_WING = probe("left-wing", 100, -120, 0);
const MASS = 100, IXX = 1000, IYY = 2000, IZZ = 2500;
// At latitude and longitude zero, ECEF x is up, y east and z north.
const UP = { x: 1, y: 0, z: 0 }, SOUTH = { x: 0, y: 0, z: -1 };

function sdkAt(altitude = 320) {
  const values: Record<string, number> = {
    "position/lat-geod-deg": 0, "position/long-gc-deg": 0, "position/h-sl-ft": altitude / 0.3048,
    "position/terrain-elevation-asl-ft": 300 / 0.3048,
    "attitude/phi-deg": 0, "attitude/theta-deg": 0, "attitude/psi-deg": 0,
    "velocities/vc-kts": 80, "velocities/v-down-fps": 3 / 0.3048,
    "velocities/v-north-fps": 10 / 0.3048, "velocities/v-east-fps": 4 / 0.3048,
    "velocities/p-rad_sec": 0, "velocities/q-rad_sec": 0, "velocities/r-rad_sec": 0,
    "inertia/cg-x-in": 100, "inertia/cg-y-in": 0, "inertia/cg-z-in": 0, "inertia/mass-slugs": MASS,
    "inertia/ixx-slugs_ft2": IXX, "inertia/iyy-slugs_ft2": IYY, "inertia/izz-slugs_ft2": IZZ,
    "inertia/ixy-slugs_ft2": 0, "inertia/ixz-slugs_ft2": 0, "inertia/iyz-slugs_ft2": 0,
    "fcs/throttle-cmd-norm": 0.5, "propulsion/engine/engine-rpm": 1000,
  };
  const initialProperties: Record<string, string> = {
    "ic/lat-geod-deg": "position/lat-geod-deg", "ic/long-gc-deg": "position/long-gc-deg",
    "ic/h-sl-ft": "position/h-sl-ft", "ic/terrain-elevation-ft": "position/terrain-elevation-asl-ft",
    "ic/phi-deg": "attitude/phi-deg", "ic/theta-deg": "attitude/theta-deg", "ic/psi-true-deg": "attitude/psi-deg",
    "ic/vn-fps": "velocities/v-north-fps", "ic/ve-fps": "velocities/v-east-fps", "ic/vd-fps": "velocities/v-down-fps",
    "ic/p-rad_sec": "velocities/p-rad_sec", "ic/q-rad_sec": "velocities/q-rad_sec", "ic/r-rad_sec": "velocities/r-rad_sec",
  };
  return {
    values,
    getPropertyValue: vi.fn((name: string) => values[name] ?? 0),
    setPropertyValue: vi.fn((name: string, value: number) => { values[name] = value; }),
    resetToInitialConditions: vi.fn(),
    runIc: vi.fn(() => {
      for (const [initial, property] of Object.entries(initialProperties)) if (initial in values) values[property] = values[initial];
      return true;
    }),
  };
}
type Sdk = ReturnType<typeof sdkAt>;

function position(sdk: Sdk): Vector {
  return geodeticToEcef(sdk.values["position/lat-geod-deg"] * DEG_TO_RAD,
    sdk.values["position/long-gc-deg"] * DEG_TO_RAD, sdk.values["position/h-sl-ft"] * 0.3048);
}

function move(sdk: Sdk, delta: Vector): void {
  const before = position(sdk);
  const next = ecefToGeodetic(before.x + delta.x, before.y + delta.y, before.z + delta.z);
  sdk.setPropertyValue("position/lat-geod-deg", next.latRad / DEG_TO_RAD);
  sdk.setPropertyValue("position/long-gc-deg", next.lonRad / DEG_TO_RAD);
  sdk.setPropertyValue("position/h-sl-ft", next.altMeters / 0.3048);
}

/** The local north, east and down axes in ECEF. */
function localAxes(sdk: Sdk) {
  const lat = sdk.values["position/lat-geod-deg"] * DEG_TO_RAD, lon = sdk.values["position/long-gc-deg"] * DEG_TO_RAD;
  return {
    north: { x: -Math.sin(lat) * Math.cos(lon), y: -Math.sin(lat) * Math.sin(lon), z: Math.cos(lat) },
    east: { x: -Math.sin(lon), y: Math.cos(lon), z: 0 },
    down: { x: -Math.cos(lat) * Math.cos(lon), y: -Math.cos(lat) * Math.sin(lon), z: -Math.sin(lat) },
  };
}

/** A body vector (forward, right, down) in ECEF. */
function bodyToEcef(sdk: Sdk, body: readonly number[]): Vector {
  const m = bodyToLocalRotation(sdk.values["attitude/phi-deg"] * DEG_TO_RAD,
    sdk.values["attitude/theta-deg"] * DEG_TO_RAD, sdk.values["attitude/psi-deg"] * DEG_TO_RAD);
  const north = m[0] * body[0] + m[1] * body[1] + m[2] * body[2];
  const east = m[3] * body[0] + m[4] * body[1] + m[5] * body[2];
  const down = m[6] * body[0] + m[7] * body[1] + m[8] * body[2];
  const axes = localAxes(sdk);
  return {
    x: axes.north.x * north + axes.east.x * east + axes.down.x * down,
    y: axes.north.y * north + axes.east.y * east + axes.down.y * down,
    z: axes.north.z * north + axes.east.z * east + axes.down.z * down,
  };
}

/** Where a probe's hull point (or, with `swept`, the point it sweeps) is in ECEF. */
function probeEcef(sdk: Sdk, at: BodyCollisionProbe, swept = false): Vector {
  const z = at.zIn + (swept ? at.insetIn : 0);
  const offset = bodyToEcef(sdk, [(sdk.values["inertia/cg-x-in"] - at.xIn) * 0.0254,
    (at.yIn - sdk.values["inertia/cg-y-in"]) * 0.0254, (sdk.values["inertia/cg-z-in"] - z) * 0.0254]);
  const cg = position(sdk);
  return { x: cg.x + offset.x, y: cg.y + offset.y, z: cg.z + offset.z };
}

/** The probes' paths as cast, copied: the collision reuses its argument objects. */
type Cast = { from: Vector; movement: Vector; distance: number };

function surfaceWithHits(fractions: Array<number | null>, normal: Vector = SOUTH) {
  let index = 0;
  const casts: Cast[] = [];
  const raycast = vi.fn((from: Vector, movement: Vector, distance: number) => {
    casts.push({ from: { ...from }, movement: { ...movement }, distance });
    const fraction = fractions[index++ % fractions.length];
    if (fraction === null) return null;
    return {
      point: { x: from.x + movement.x * fraction, y: from.y + movement.y * fraction, z: from.z + movement.z * fraction },
      normal, distanceMeters: distance * fraction,
    };
  });
  return { sample: () => ({ heightMeters: 300 }), raycast, casts } as unknown as SurfaceQuery & { raycast: typeof raycast; casts: Cast[] };
}

/** An infinite plane through `point`, struck from either side as Babylon picking is. */
function planeSurface(point: Vector, rawNormal: Vector) {
  const length = Math.hypot(rawNormal.x, rawNormal.y, rawNormal.z);
  const n = { x: rawNormal.x / length, y: rawNormal.y / length, z: rawNormal.z / length };
  const raycast = vi.fn((from: Vector, movement: Vector, distance: number) => {
    const speed = Math.hypot(movement.x, movement.y, movement.z);
    const along = (movement.x * n.x + movement.y * n.y + movement.z * n.z) / speed;
    if (Math.abs(along) < 1e-12) return null;
    const t = ((point.x - from.x) * n.x + (point.y - from.y) * n.y + (point.z - from.z) * n.z) / along;
    if (t < 0 || t > distance) return null;
    return {
      point: { x: from.x + movement.x / speed * t, y: from.y + movement.y / speed * t, z: from.z + movement.z / speed * t },
      normal: rawNormal, distanceMeters: t,
    };
  });
  const height = (at: Vector) => (at.x - point.x) * n.x + (at.y - point.y) * n.y + (at.z - point.z) * n.z;
  return { sample: () => ({ heightMeters: 300 }), raycast, height } as unknown as SurfaceQuery & { height: typeof height };
}

function worldVelocity(sdk: Sdk): Vector {
  const axes = localAxes(sdk);
  const north = sdk.values["velocities/v-north-fps"] * 0.3048;
  const east = sdk.values["velocities/v-east-fps"] * 0.3048;
  const down = sdk.values["velocities/v-down-fps"] * 0.3048;
  return {
    x: axes.north.x * north + axes.east.x * east + axes.down.x * down,
    y: axes.north.y * north + axes.east.y * east + axes.down.y * down,
    z: axes.north.z * north + axes.east.z * east + axes.down.z * down,
  };
}

const rates = (sdk: Sdk) => ["p", "q", "r"].map(axis => sdk.values[`velocities/${axis}-rad_sec`]);

/** Translational plus rotational kinetic energy, in foot-pounds. */
function kineticEnergy(sdk: Sdk): number {
  const speed = ["north", "east", "down"].reduce((sum, axis) => sum + sdk.values[`velocities/v-${axis}-fps`] ** 2, 0);
  const [p, q, r] = rates(sdk);
  return 0.5 * MASS * speed + 0.5 * (IXX * p * p + IYY * q * q + IZZ * r * r);
}

const dot = (a: Vector, b: Vector) => a.x * b.x + a.y * b.y + a.z * b.z;

/** One step: the first update records the probes, the aircraft moves, the second sweeps. */
function strike(sdk: Sdk, surface: SurfaceQuery, probes: readonly BodyCollisionProbe[], motion: Vector,
  options: { friction?: number; getRestitution?: () => number } = {}) {
  const collision = createVisibleMeshCollision(sdk as never, surface, { bodyProbes: probes, ...options });
  expect(collision.update()).toBe(false);
  move(sdk, motion);
  return collision.update();
}

describe.each(["typescript", "single"] as const)("visible mesh body collision on %s", kernel => {
  beforeEach(() => { active.core = kernel === "single" ? wasmCore : undefined; });
  afterEach(() => { active.core = undefined; });

  it("runs on the kernel asked for", () => {
    const collision = createVisibleMeshCollision(sdkAt() as never, surfaceWithHits([null]), { bodyProbes: [CENTER] });
    expect(collision.status()).toEqual({ requestedKernel: kernel, kernel, fallbackReason: null });
    collision.dispose();
    expect(collision.status().kernel).toBe("typescript");
  });

  it("sweeps structural-inch probes from the live CG, turned with the attitude", () => {
    const sdk = sdkAt();
    const surface = surfaceWithHits([null]);
    const inset = probe("belly", 100, 0, 0, 3);
    const collision = createVisibleMeshCollision(sdk as never, surface, { bodyProbes: [NOSE, RIGHT_WING, inset] });
    // A centimetre's climb moves every probe, so each is cast; its end is where it is now.
    const ends = () => {
      move(sdk, { x: 0.01, y: 0, z: 0 });
      collision.update();
      const cg = position(sdk);
      return surface.casts.slice(-3).map(({ from, movement }) =>
        ({ x: from.x + movement.x - cg.x, y: from.y + movement.y - cg.y, z: from.z + movement.z - cg.z }));
    };
    collision.update();
    sdk.values["inertia/cg-x-in"] = 110;
    let [nose, right, belly] = ends();
    // 110 inches ahead of the CG is north; the wingtip is ten feet east; the inset is three inches up.
    expect(nose.z).toBeCloseTo(110 * 0.0254, 9);
    expect(right.y).toBeCloseTo(3.048, 9);
    expect(belly.x).toBeCloseTo(3 * 0.0254, 9);
    // Rolled right through 90 degrees, the right wing points down and the top of the aircraft east.
    sdk.values["attitude/phi-deg"] = 90;
    [nose, right, belly] = ends();
    expect(nose.z).toBeCloseTo(110 * 0.0254, 9);
    expect(right.x).toBeCloseTo(-3.048, 9);
    expect(belly.y).toBeCloseTo(3 * 0.0254, 9);
    sdk.values["attitude/phi-deg"] = 0;
    sdk.values["attitude/theta-deg"] = 90;
    [nose] = ends();
    expect(nose.x).toBeCloseTo(110 * 0.0254, 9);
  });

  it.each([0, 0.1, undefined])("reflects a CG impact with restitution and slows its slide with Coulomb friction %s", friction => {
    const sdk = sdkAt();
    const before = position(sdk);
    const collision = createVisibleMeshCollision(sdk as never, surfaceWithHits([0.5]), { bodyProbes: [CENTER], friction });
    collision.update();
    move(sdk, { x: 0, y: 0, z: 1 });
    // The velocity as the collision reads it, in the frame where the aircraft now is.
    const incoming = worldVelocity(sdk);
    expect(collision.update()).toBe(true);
    const outgoing = worldVelocity(sdk);
    const approach = dot(incoming, SOUTH);
    expect(dot(outgoing, SOUTH)).toBeCloseTo(-0.25 * approach, 9);
    // Friction takes mu (1 + e) |v_n| of the 5 m/s slide: none, 1.25 m/s, or all of it at the default 0.5.
    const slide = { x: incoming.x, y: incoming.y }, slideSpeed = Math.hypot(slide.x, slide.y);
    const loss = Math.min(slideSpeed, (friction ?? 0.5) * 1.25 * -approach);
    expect(loss).toBeCloseTo([0, 1.25, 5][[0, 0.1, undefined].indexOf(friction)], 5);
    for (const axis of ["x", "y"] as const) expect(outgoing[axis]).toBeCloseTo(slide[axis] * (1 - loss / slideSpeed), 9);
    // No lever arm, no turn; the CG stays where it went, a centimetre clear of the hit.
    expect(rates(sdk)).toEqual([0, 0, 0]);
    const after = position(sdk);
    expect(after.x - before.x).toBeCloseTo(0, 6);
    expect(after.y - before.y).toBeCloseTo(0, 6);
    expect(after.z - before.z).toBeCloseTo(0.5 - 0.01, 6);
  });

  it("turns an off-centre impact into the rigid-body rates m_eff predicts", () => {
    const sdk = sdkAt();
    sdk.values["velocities/v-north-fps"] = 30;
    sdk.values["velocities/v-east-fps"] = 0;
    sdk.values["velocities/v-down-fps"] = 10;
    expect(strike(sdk, surfaceWithHits([0.5], UP), [RIGHT_WING], { x: -0.1, y: 0, z: 0 }, { friction: 0 })).toBe(true);
    // r = 10 ft right, n = up: (r × n)ᵀ I⁻¹ (r × n) = 100 / Ixx.
    const impulse = 1.25 * 10 / (1 / MASS + 100 / IXX);
    expect(sdk.values["velocities/v-down-fps"]).toBeCloseTo(10 - impulse / MASS, 9);
    expect(sdk.values["velocities/v-north-fps"]).toBeCloseTo(30, 9);
    const [p, q, r] = rates(sdk);
    expect(p).toBeCloseTo(-10 * impulse / IXX, 9);
    expect(q).toBeCloseTo(0, 12);
    expect(r).toBeCloseTo(0, 12);
    // The wingtip itself leaves at e times its approach: down at the tip is v_d + p r_y.
    expect(sdk.values["velocities/v-down-fps"] + p * 10).toBeCloseTo(-0.25 * 10, 9);
  });

  it("turns the aircraft through the whole inertia tensor, products of inertia included", () => {
    const sdk = sdkAt();
    // JSBSim reports -J12, +J13 and -J23. This tensor is 500 [[4, 1, 1], [1, 4, 1], [1, 1, 4]]
    // slug ft², whose inverse is [[5, -1, -1], [-1, 5, -1], [-1, -1, 5]] / 9000.
    Object.assign(sdk.values, {
      "inertia/ixx-slugs_ft2": 2000, "inertia/iyy-slugs_ft2": 2000, "inertia/izz-slugs_ft2": 2000,
      "inertia/ixy-slugs_ft2": -500, "inertia/ixz-slugs_ft2": 500, "inertia/iyz-slugs_ft2": -500,
    });
    expect(strike(sdk, surfaceWithHits([0.5], UP), [NOSE], { x: -1, y: 0, z: 0 }, { friction: 0 })).toBe(true);
    // r = 100 inches ahead, n = up: r × n is r about the pitch axis, so Δω is the
    // inverse's middle column times r J, and (r × n)ᵀ I⁻¹ (r × n) = 5 r² / 9000.
    const arm = 100 / 12, approach = 3 / 0.3048;
    const impulse = 1.25 * approach / (1 / MASS + 5 * arm * arm / 9000);
    expect(sdk.values["velocities/v-down-fps"]).toBeCloseTo(approach - impulse / MASS, 9);
    const [p, q, r] = rates(sdk);
    expect(p).toBeCloseTo(-arm * impulse / 9000, 9);
    expect(q).toBeCloseTo(5 * arm * impulse / 9000, 9);
    expect(r).toBeCloseTo(-arm * impulse / 9000, 9);
  });

  it.each([
    // A raised wing rolls away from the hit, and drag at the tip yaws the nose toward it.
    { at: RIGHT_WING, signs: [-1, 0, 1] },
    { at: LEFT_WING, signs: [1, 0, -1] },
    // An upward hit ahead of the CG pitches the nose up; behind it, down.
    { at: NOSE, signs: [0, 1, 0] },
    { at: TAIL, signs: [0, -1, 0] },
  ])("gives a $at.name hit the physical signs of p, q and r", ({ at, signs }) => {
    const sdk = sdkAt();
    // Straight ahead and down: friction at the nose or tail then has no lever for yaw.
    sdk.values["velocities/v-east-fps"] = 0;
    expect(strike(sdk, surfaceWithHits([0.5], UP), [at], { x: -0.1, y: 0, z: 0 })).toBe(true);
    rates(sdk).forEach((rate, axis) => {
      if (signs[axis] === 0) expect(Math.abs(rate)).toBeLessThan(1e-12);
      else expect(Math.sign(rate)).toBe(signs[axis]);
    });
  });

  it("slows the aircraft with friction where a frictionless hit keeps its slide", () => {
    const slideAfter = (friction: number) => {
      const sdk = sdkAt();
      expect(strike(sdk, surfaceWithHits([0.5], UP), [NOSE], { x: -0.1, y: 0, z: 0 }, { friction })).toBe(true);
      return Math.hypot(sdk.values["velocities/v-north-fps"], sdk.values["velocities/v-east-fps"]);
    };
    const incoming = Math.hypot(10, 4) / 0.3048;
    expect(slideAfter(0)).toBeCloseTo(incoming, 9);
    expect(slideAfter(0.5)).toBeLessThan(incoming - 1);
  });

  it.each([0, 0.25, 1])("never adds kinetic energy with restitution %s, whatever the hit", restitution => {
    let cases = 0;
    for (const at of [NOSE, TAIL, RIGHT_WING, LEFT_WING, probe("aft-tip", 160, -200, 40, 3)]) {
      for (const [roll, pitch, heading] of [[0, 0, 0], [35, -12, 65], [-60, 20, 200]]) {
        for (const rawNormal of [UP, { x: 4, y: 1, z: -3 }, { x: 0, y: -2, z: 5 }]) {
          const sdk = sdkAt();
          Object.assign(sdk.values, {
            "attitude/phi-deg": roll, "attitude/theta-deg": pitch, "attitude/psi-deg": heading,
            "velocities/p-rad_sec": 0.3, "velocities/q-rad_sec": -0.2, "velocities/r-rad_sec": 0.1,
          });
          const before = kineticEnergy(sdk);
          // A movement against the normal; a normal facing along it is flipped.
          if (!strike(sdk, surfaceWithHits([0.5], rawNormal), [at], { x: -0.3, y: 0.2, z: 0.4 },
            { getRestitution: () => restitution })) continue;
          cases++;
          expect(kineticEnergy(sdk)).toBeLessThanOrEqual(before + 1e-9);
        }
      }
    }
    expect(cases).toBe(45);
  });

  it.each([
    { name: "level ground", rawNormal: UP, attitude: [0, 0, 0] },
    { name: "a scaled slope", rawNormal: { x: 6, y: 1.5, z: -2 }, attitude: [0, 0, 30] },
    { name: "level ground, banked", rawNormal: UP, attitude: [25, 4, 120] },
  ])("lifts every inset hull point clear of $name without clipping", ({ rawNormal, attitude }) => {
    const sdk = sdkAt();
    Object.assign(sdk.values, { "attitude/phi-deg": attitude[0], "attitude/theta-deg": attitude[1], "attitude/psi-deg": attitude[2] });
    const probes = [probe("belly", 100, 0, -60, 3), probe("left-tip", 110, -200, -20, 3), probe("right-tip", 110, 200, -20, 3), NOSE];
    const length = Math.hypot(rawNormal.x, rawNormal.y, rawNormal.z);
    const n = { x: rawNormal.x / length, y: rawNormal.y / length, z: rawNormal.z / length };
    // The surface three metres below the CG along its normal: every probe starts above it.
    const cg = position(sdk);
    const surface = planeSurface({ x: cg.x - 3 * n.x, y: cg.y - 3 * n.y, z: cg.z - 3 * n.z }, rawNormal);
    const collision = createVisibleMeshCollision(sdk as never, surface, { bodyProbes: probes });
    const before = probes.map(at => surface.height(probeEcef(sdk, at)));
    for (const height of before) expect(height).toBeGreaterThan(0);
    expect(collision.update()).toBe(false);
    // One step into the surface: the lowest hull point ends 1.2 metres inside it.
    const drop = Math.min(...before) + 1.2;
    move(sdk, { x: -drop * n.x, y: -drop * n.y, z: -drop * n.z });
    expect(collision.update()).toBe(true);
    const heights = probes.map(at => surface.height(probeEcef(sdk, at)));
    for (const height of heights) expect(height).toBeGreaterThanOrEqual(0.01 - 1e-5);
    expect(Math.min(...heights)).toBeCloseTo(0.01, 5);
    // The next step starts there: nothing to sweep, nothing still inside.
    expect(collision.update()).toBe(false);
  });

  it("resolves hits in step order, so a point an earlier hit already sent away takes no second impulse", () => {
    // Listed nose first, the centre crosses first: it rebounds the whole aircraft
    // without turning it, and the nose, already rising, takes no impulse.
    const sdk = sdkAt();
    const surface = surfaceWithHits([0.6, 0.3], UP);
    expect(strike(sdk, surface, [NOSE, CENTER], { x: -1, y: 0, z: 0 }, { friction: 0 })).toBe(true);
    expect(surface.raycast).toHaveBeenCalledTimes(2);
    expect(sdk.values["velocities/v-down-fps"] * 0.3048).toBeCloseTo(-0.25 * 3, 9);
    for (const rate of rates(sdk)) expect(rate).toBeCloseTo(0, 12);
    // One lift out of the earlier, deeper hit; the later one is already clear.
    expect(position(sdk).x - geodeticToEcef(0, 0, 320).x).toBeCloseTo(-(0.3 - 0.01), 6);
    // The nose crossing first pitches the nose up and leaves the centre still
    // descending, so the centre takes a second impulse that turns it upward.
    const reversed = sdkAt();
    expect(strike(reversed, surfaceWithHits([0.3, 0.6], UP), [NOSE, CENTER], { x: -1, y: 0, z: 0 }, { friction: 0 })).toBe(true);
    expect(rates(reversed)[1]).toBeGreaterThan(0);
    expect(reversed.values["velocities/v-down-fps"]).toBeLessThan(0);
  });

  it("applies every probe that struck, nose and wing together", () => {
    const sdk = sdkAt();
    expect(strike(sdk, surfaceWithHits([0.4, 0.7], UP), [NOSE, RIGHT_WING], { x: -0.1, y: 0, z: 0 }, { friction: 0 })).toBe(true);
    const [p, q] = rates(sdk);
    expect(p).toBeLessThan(0);
    expect(q).toBeGreaterThan(0);
  });

  it("keeps the current terrain, controls and attitude when restoring an impact", () => {
    const sdk = sdkAt();
    const collision = createVisibleMeshCollision(sdk as never, surfaceWithHits([0.5]), { bodyProbes: [CENTER] });
    collision.update();
    sdk.setPropertyValue("position/terrain-elevation-asl-ft", 301 / 0.3048);
    sdk.setPropertyValue("fcs/throttle-cmd-norm", 0.75);
    sdk.setPropertyValue("fcs/elevator-cmd-norm", 0.3);
    sdk.setPropertyValue("attitude/theta-deg", 7);
    sdk.setPropertyValue("velocities/q-rad_sec", 0.1);
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(true);
    expect(sdk.values["ic/terrain-elevation-ft"] * 0.3048).toBeCloseTo(301, 6);
    expect(sdk.values["fcs/throttle-cmd-norm"]).toBe(0.75);
    expect(sdk.values["fcs/elevator-cmd-norm"]).toBe(0.3);
    expect(sdk.values["ic/theta-deg"]).toBe(7);
    expect(sdk.values["ic/q-rad_sec"]).toBe(0.1);
  });

  it.each([
    { x: 0, y: 0, z: 0 }, { x: NaN, y: 0, z: 1 }, { x: Infinity, y: 0, z: 1 },
  ])("ignores invalid collision normals %j", normal => {
    const sdk = sdkAt();
    expect(strike(sdk, surfaceWithHits([0.5], normal), [CENTER], { x: 0, y: 0, z: 1 })).toBe(false);
    expect(sdk.resetToInitialConditions).not.toHaveBeenCalled();
  });

  it.each([0.5, 1])("detects slow taxi collisions including an impact at segment fraction %s", fraction => {
    expect(strike(sdkAt(), surfaceWithHits([fraction]), [CENTER], { x: 0, y: 0, z: 0.005 })).toBe(true);
  });

  it("reset discards motion caused by a placement or terrain refinement", () => {
    const sdk = sdkAt();
    const surface = surfaceWithHits([0.5]);
    const collision = createVisibleMeshCollision(sdk as never, surface, { bodyProbes: [CENTER] });
    collision.update();
    move(sdk, { x: 10, y: 0, z: 0 });
    collision.reset();
    expect(collision.update()).toBe(false);
    expect(surface.raycast).not.toHaveBeenCalled();
  });

  it("lets an acknowledged overlap exit without an underside impact, then restores collisions", () => {
    const sdk = sdkAt();
    const surface = surfaceWithHits([0.5]);
    const collision = createVisibleMeshCollision(sdk as never, surface, { bodyProbes: [CENTER] });
    collision.update();
    collision.setTerrainOverlapIgnored(true);
    move(sdk, { x: 10, y: 0, z: 0 });
    expect(collision.update()).toBe(false);
    collision.setTerrainOverlapIgnored(false);
    expect(collision.update()).toBe(false);
    expect(surface.raycast).not.toHaveBeenCalled();
    expect(sdk.runIc).not.toHaveBeenCalled();
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(true);
    expect(surface.raycast).toHaveBeenCalled();
  });

  it("sweeps nothing far above the surface", () => {
    const sdk = sdkAt(300 + 160);
    const surface = surfaceWithHits([0.5]);
    expect(strike(sdk, surface, [CENTER], { x: -1, y: 0, z: 0 })).toBe(false);
    expect(surface.raycast).not.toHaveBeenCalled();
  });

  it.each([[1.35, 1.35], [99, 2], [NaN, 0.25]])("permits explicit arcade restitution %s with finite bounds", (requested, expected) => {
    const sdk = sdkAt();
    strike(sdk, surfaceWithHits([0.5]), [CENTER], { x: 0, y: 0, z: 1 }, { friction: 0, getRestitution: () => requested });
    expect(sdk.values["velocities/v-north-fps"] * 0.3048).toBeCloseTo(-10 * expected, 5);
  });

  /** A native batch double that copies from the mock's values. */
  function batched(sdk: Sdk) {
    const batches: Array<{ dispose: ReturnType<typeof vi.fn>; fail: boolean }> = [];
    const native = Object.assign(sdk, {
      createPropertyBatch: vi.fn((paths: readonly string[]) => {
        const batch = {
          missing: [], fail: false, dispose: vi.fn(),
          read: (target: Float64Array) => {
            if (batch.fail) throw new Error("released by a model load");
            paths.forEach((path, slot) => { target[slot] = sdk.values[path] ?? 0; });
            return target;
          },
        };
        batches.push(batch);
        return batch;
      }),
    });
    return { native, batches };
  }

  it("reads through one native batch, rebuilt after a model load and released on dispose", () => {
    const { native, batches } = batched(sdkAt());
    const collision = createVisibleMeshCollision(native as never, surfaceWithHits([null]), { bodyProbes: [CENTER] });
    collision.update();
    collision.update();
    expect(native.createPropertyBatch).toHaveBeenCalledTimes(1);
    expect(native.getPropertyValue).not.toHaveBeenCalled();
    batches[0].fail = true;
    expect(collision.update()).toBe(false);
    expect(batches[0].dispose).toHaveBeenCalledTimes(1);
    collision.update();
    expect(native.createPropertyBatch).toHaveBeenCalledTimes(2);
    collision.dispose();
    expect(batches[1].dispose).toHaveBeenCalledTimes(1);
  });
});

describe("WASM hull kernel", () => {
  /** Mulberry32: the same draws on every run. */
  function random(seed: number) {
    let state = seed >>> 0;
    return () => {
      state = (state + 0x6d2b79f5) >>> 0;
      let t = state;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /** The same aircraft, surface and options twice: one on the reference, one on the kernel. */
  function pair(draw: () => number) {
    const spread = (size: number) => (draw() * 2 - 1) * size;
    const pick = <T,>(values: readonly T[]) => values[Math.floor(draw() * values.length)];
    const lat = spread(70), lon = spread(180);
    const state: Record<string, number> = {
      "position/lat-geod-deg": lat, "position/long-gc-deg": lon, "position/h-sl-ft": (305 + draw() * 30) / 0.3048,
      "attitude/phi-deg": spread(60), "attitude/theta-deg": spread(30), "attitude/psi-deg": draw() * 360,
      "velocities/v-north-fps": spread(40), "velocities/v-east-fps": spread(40), "velocities/v-down-fps": 8 + spread(5),
      "velocities/p-rad_sec": spread(0.5), "velocities/q-rad_sec": spread(0.5), "velocities/r-rad_sec": spread(0.5),
      "inertia/cg-x-in": 90 + draw() * 20, "inertia/cg-y-in": spread(2), "inertia/cg-z-in": spread(5),
      "inertia/mass-slugs": pick([50 + draw() * 250, 50 + draw() * 250, 50 + draw() * 250, 0]),
      "inertia/ixx-slugs_ft2": 500 + draw() * 2500, "inertia/iyy-slugs_ft2": 500 + draw() * 2500, "inertia/izz-slugs_ft2": 1000 + draw() * 3000,
      "inertia/ixy-slugs_ft2": spread(200), "inertia/ixz-slugs_ft2": spread(200), "inertia/iyz-slugs_ft2": spread(200),
    };
    const probes = Array.from({ length: 1 + Math.floor(draw() * 6) }, (_, index) =>
      probe(`p${index}`, draw() * 220, spread(200), -60 + draw() * 100, pick([0, 0, 3 * draw()])));
    const up = { x: Math.cos(lat * DEG_TO_RAD) * Math.cos(lon * DEG_TO_RAD), y: Math.cos(lat * DEG_TO_RAD) * Math.sin(lon * DEG_TO_RAD), z: Math.sin(lat * DEG_TO_RAD) };
    const scale = pick([1, 3, 0.2]);
    const rawNormal = { x: (up.x + spread(0.3)) * scale, y: (up.y + spread(0.3)) * scale, z: (up.z + spread(0.3)) * scale };
    const length = Math.hypot(rawNormal.x, rawNormal.y, rawNormal.z);
    const n = { x: rawNormal.x / length, y: rawNormal.y / length, z: rawNormal.z / length };
    const plane = draw() < 0.7, depth = 3 + draw() * 3;
    // A fraction per probe, at least 0.05 apart: hits at the same instant within an
    // ulp may be taken in either order, which this compares nothing about.
    const fractions = Array.from({ length: 6 }, (_, k) => (draw() < 0.25 ? null : 0.05 + 0.15 * k + 0.1 * draw()));
    for (let k = fractions.length - 1; k > 0; k--) {
      const other = Math.floor(draw() * (k + 1));
      [fractions[k], fractions[other]] = [fractions[other], fractions[k]];
    }
    const hitNormal = pick([rawNormal, rawNormal, rawNormal, { x: -rawNormal.x, y: -rawNormal.y, z: -rawNormal.z }, { x: 0, y: 0, z: 0 }]);
    const friction = pick([undefined, 0, 0.3, Number.NaN, -1, 2]);
    const restitution = pick([undefined, 0, 0.6, Number.NaN, 3]);
    const side = (core: GroundContactCore | undefined) => {
      const sdk = sdkAt();
      Object.assign(sdk.values, state);
      const cg = position(sdk);
      const surface = plane ? planeSurface({ x: cg.x - depth * n.x, y: cg.y - depth * n.y, z: cg.z - depth * n.z }, rawNormal)
        : surfaceWithHits(fractions, hitNormal);
      const casts: Cast[] = [];
      const raycast = surface.raycast.bind(surface);
      surface.raycast = (from, movement, distance) => {
        casts.push({ from: { ...from }, movement: { ...movement }, distance });
        return raycast(from, movement, distance);
      };
      const collision = createCollision(sdk as never, surface, { bodyProbes: probes, friction,
        getRestitution: restitution === undefined ? undefined : () => restitution, core });
      return { sdk, collision, casts };
    };
    const motion = () => {
      const along = 0.3 + draw() * 2.5;
      return { x: -along * n.x + spread(0.5), y: -along * n.y + spread(0.5), z: -along * n.z + spread(0.5) };
    };
    return { reference: side(undefined), wasmSide: side(wasmCore), motion };
  }

  it("matches the reference to rounding over random hulls, attitudes, surfaces and steps", () => {
    const draw = random(23);
    // The kernel's sin, cos and atan2 are libm's, which can differ from V8's by
    // an ulp; the rest is the same arithmetic in the same order.
    let worstValue = 0, worstCast = 0, impacts = 0, misses = 0;
    for (let trial = 0; trial < 300; trial++) {
      const { reference, wasmSide, motion } = pair(draw);
      expect(wasmSide.collision.status().kernel).toBe("single");
      for (let step = 0; step < 6; step++) {
        const delta = step === 0 ? { x: 0, y: 0, z: 0 } : motion();
        move(reference.sdk, delta);
        move(wasmSide.sdk, delta);
        const expected = reference.collision.update(), actual = wasmSide.collision.update();
        expect(actual, `trial ${trial} step ${step}`).toBe(expected);
        if (expected) impacts++; else misses++;
        expect(wasmSide.casts.length, `trial ${trial} step ${step} casts`).toBe(reference.casts.length);
        reference.casts.forEach((cast, index) => {
          const other = wasmSide.casts[index];
          for (const [a, b] of [[other.from, cast.from], [other.movement, cast.movement]] as const) {
            worstCast = Math.max(worstCast, Math.abs(a.x - b.x), Math.abs(a.y - b.y), Math.abs(a.z - b.z));
          }
          worstCast = Math.max(worstCast, Math.abs(other.distance - cast.distance));
        });
        reference.casts.length = wasmSide.casts.length = 0;
        expect(Object.keys(wasmSide.sdk.values).sort()).toEqual(Object.keys(reference.sdk.values).sort());
        for (const [key, value] of Object.entries(reference.sdk.values)) {
          const other = wasmSide.sdk.values[key];
          const difference = Math.abs(other - value) / Math.max(1, Math.abs(value));
          if (!(difference <= 1e-9)) expect.fail(`trial ${trial} step ${step} ${key}: ${other} is not ${value}`);
          worstValue = Math.max(worstValue, difference);
        }
      }
      expect(wasmSide.collision.status().fallbackReason).toBeNull();
      wasmSide.collision.dispose();
    }
    // Both outcomes are exercised, and the paths cast agree to well under a micrometre.
    expect(impacts).toBeGreaterThan(300);
    expect(misses).toBeGreaterThan(300);
    expect(worstCast).toBeLessThan(1e-6);
    expect(worstValue).toBeLessThan(1e-9);
  });

  it("takes over from the reference when a loading core arrives, sweeping from its own points", async () => {
    let arrive!: (core: GroundContactCore) => void;
    const sdk = sdkAt();
    const surface = surfaceWithHits([0.5]);
    const collision = createCollision(sdk as never, surface, { bodyProbes: [CENTER], core: new Promise<GroundContactCore>(done => { arrive = done; }) });
    expect(collision.status()).toEqual({ requestedKernel: "single", kernel: "typescript", fallbackReason: null });
    expect(collision.update()).toBe(false);
    arrive(wasmCore);
    await Promise.resolve();
    expect(collision.status().kernel).toBe("single");
    // The reference holds the last points; the kernel records its own before it sweeps.
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(false);
    expect(surface.raycast).not.toHaveBeenCalled();
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(true);
    expect(surface.raycast).toHaveBeenCalledOnce();
  });

  it("gives a failed kernel up for good and continues on the reference", async () => {
    const own = await wasm();
    const sdk = sdkAt();
    const collision = createCollision(sdk as never, surfaceWithHits([0.5]), { bodyProbes: [CENTER], core: own });
    expect(collision.update()).toBe(false);
    own.dispose();
    move(sdk, { x: 0, y: 0, z: 1 });
    // The kernel held the last points, so the reference records its own this step.
    expect(collision.update()).toBe(false);
    expect(collision.status()).toEqual({ requestedKernel: "single", kernel: "typescript",
      fallbackReason: "wasm-hull-failed: WASM ground-contact core is disposed" });
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(true);
  });

  it("keeps the impact it restored when the kernel fails after it", async () => {
    const own = await wasm();
    const sdk = sdkAt();
    const collision = createCollision(sdk as never, surfaceWithHits([0.5]), { bodyProbes: [CENTER], friction: 0, core: own });
    expect(collision.update()).toBe(false);
    move(sdk, { x: 0, y: 0, z: 1 });
    const runIc = sdk.runIc.getMockImplementation()!;
    sdk.runIc.mockImplementationOnce(() => { own.dispose(); return runIc(); });
    const approach = dot(worldVelocity(sdk), SOUTH);
    expect(collision.update()).toBe(true);
    expect(dot(worldVelocity(sdk), SOUTH)).toBeCloseTo(-0.25 * approach, 9);
    expect(collision.status().kernel).toBe("typescript");
    expect(collision.status().fallbackReason).toBe("wasm-hull-failed: WASM ground-contact core is disposed");
  });

  it("passes the surface's and the callback's errors to the caller and keeps the kernel", () => {
    const sdk = sdkAt();
    const surface = surfaceWithHits([0.5]);
    let restitution = () => 0.25;
    const collision = createCollision(sdk as never, surface, { bodyProbes: [CENTER], getRestitution: () => restitution(), core: wasmCore });
    collision.update();
    move(sdk, { x: 0, y: 0, z: 1 });
    surface.raycast.mockImplementationOnce(() => { throw new Error("tile released"); });
    expect(() => collision.update()).toThrow("tile released");
    restitution = () => { throw new Error("settings closed"); };
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(() => collision.update()).toThrow("settings closed");
    expect(collision.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    restitution = () => 0.25;
    move(sdk, { x: 0, y: 0, z: 1 });
    expect(collision.update()).toBe(true);
  });

  it("allocates nothing in WASM per step and releases its kernel on dispose", () => {
    const build = vi.spyOn(wasmCore, "createHullCollision");
    const sdk = sdkAt();
    const collision = createCollision(sdk as never, surfaceWithHits([0.5, null, 0.3], UP),
      { bodyProbes: [NOSE, TAIL, RIGHT_WING, LEFT_WING], core: wasmCore });
    const kernel = build.mock.results[0].value as { dispose(): void };
    build.mockRestore();
    const release = vi.spyOn(kernel, "dispose");
    collision.update();
    const before = wasmCore.heapAllocations();
    let impacts = 0;
    for (let step = 0; step < 50; step++) {
      move(sdk, { x: -0.2, y: 0.05, z: 0.1 });
      if (collision.update()) impacts++;
    }
    expect(impacts).toBeGreaterThan(10);
    expect(wasmCore.heapAllocations()).toBe(before);
    collision.dispose();
    expect(release).toHaveBeenCalledOnce();
    expect(collision.status().kernel).toBe("typescript");
  });
});
