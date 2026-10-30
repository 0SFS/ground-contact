import { readFileSync } from "node:fs";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import type { SurfaceHit, SurfaceQuery } from "foss-earth/runtime";
import { C172_GROUND_CONTACTS, metresPerDegree } from "../src/collisionGeometry";
import type { GroundContactCore } from "../src/groundContactCore";
import type { AircraftClearanceStance } from "../src/safeFlightState";
import { createTerrainContact } from "../src/terrainContact";
import { instantiateGroundContact } from "../wasm/index.js";

const wasm = () => instantiateGroundContact(readFileSync(new URL("../wasm/single/ground_contact.wasm", import.meta.url)));
// A core per test: one holds a bounded number of live kernels.
let core: Awaited<ReturnType<typeof wasm>>;
beforeEach(async () => { core = await wasm(); });
afterEach(() => core.dispose());

const LAT0 = 34, LON0 = -118.3;
const SCALE = metresPerDegree(LAT0);
const METERS_PER_FOOT = 0.3048;
const STANCE: AircraftClearanceStance = { staticMeters: 1, staticPitchRad: 0.04, pitchArmMeters: 4, rollArmMeters: 5.5 };

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

type Unit = { prefix: string; retractable: boolean; xIn: number; yIn: number; zIn: number };

/** The C172's contacts, moved up to `jitter` inches, numbered as JSBSim numbers them. */
function layout(draw: () => number, { structures = true, retractable = false, jitter = 2 } = {}): Unit[] {
  const move = () => (draw() * 2 - 1) * jitter;
  return C172_GROUND_CONTACTS.filter(contact => structures || contact.kind === "wheel").map((contact, index) => ({
    prefix: `${contact.kind === "wheel" ? "gear" : "contact"}/unit${index ? `[${index}]` : ""}/`,
    retractable: retractable && contact.kind === "wheel",
    xIn: contact.xIn + move(), yIn: contact.yIn + move(), zIn: contact.zIn + move(),
  }));
}

const INITIAL: Record<string, string> = {
  "ic/lat-geod-deg": "position/lat-geod-deg", "ic/long-gc-deg": "position/long-gc-deg", "ic/h-sl-ft": "position/h-sl-ft",
  "ic/terrain-elevation-ft": "position/terrain-elevation-asl-ft", "ic/vd-fps": "velocities/v-down-fps",
};

/** A JSBSim double with the contact units' catalog and positions; `load` is a model load. */
function aircraft(units: Unit[]) {
  const values: Record<string, number> = { "inertia/cg-x-in": 41, "inertia/cg-y-in": 0, "inertia/cg-z-in": 36.5, "gear/gear-pos-norm": 1 };
  let catalog = "";
  const sdk = {
    values,
    getPropertyValue: vi.fn((name: string) => values[name] ?? 0),
    setPropertyValue: vi.fn((name: string, value: number) => { values[name] = value; }),
    resetToInitialConditions: vi.fn(),
    runIc: vi.fn(() => {
      for (const [initial, property] of Object.entries(INITIAL)) if (initial in values) values[property] = values[initial];
      return true;
    }),
    queryPropertyCatalog: vi.fn((prefix: string) => (prefix === "/unit" ? catalog : "")),
    load(next: Unit[]) {
      for (const key of Object.keys(values)) if (/^(gear|contact)\/unit/.test(key)) delete values[key];
      values["gear/num-units"] = next.length;
      catalog = next.map(unit => [`${unit.prefix}x-position (RW)`, ...(unit.retractable ? [`${unit.prefix}pos-norm (R)`] : [])].join("\n")).join("\n") + "\n";
      for (const unit of next) {
        values[`${unit.prefix}x-position`] = unit.xIn;
        values[`${unit.prefix}y-position`] = unit.yIn;
        values[`${unit.prefix}z-position`] = unit.zIn;
      }
    },
  };
  sdk.load(units);
  return sdk;
}
type Aircraft = ReturnType<typeof aircraft>;

/** Ground in metres at metres north and east of the start, in the world's current revision. */
type Ground = (north: number, east: number, revision: number) => number | null;

/**
 * `hidden`: the surface returns nothing at all, as Google does with the tiles
 * out of view. `failAt`: the sample, counted from 0 since `calls` was emptied,
 * that throws.
 */
interface World { revision: number; hidden?: boolean; failAt?: number }

function surfaceOf(ground: Ground, world: World) {
  const calls: Array<[number, number]> = [];
  const surface = {
    calls,
    raycast: () => null,
    sample: vi.fn((lat: number, lon: number) => {
      calls.push([lat, lon]);
      if (calls.length - 1 === world.failAt) throw new Error("tile released");
      if (world.hidden) return null;
      const height = ground((lat - LAT0) * SCALE.lat, (lon - LON0) * SCALE.lon, world.revision);
      return height === null ? null : ({ heightMeters: height, revision: world.revision, quality: 15 } as SurfaceHit);
    }),
  };
  return surface as typeof surface & SurfaceQuery;
}

/** A pose in the scripted flight: metres north and east, height above the start, angles in degrees. */
interface Pose { north: number; east: number; height: number; roll: number; pitch: number; heading: number; sink: number }

function place(sdk: Aircraft, pose: Pose): void {
  sdk.values["position/lat-geod-deg"] = LAT0 + pose.north / SCALE.lat;
  sdk.values["position/long-gc-deg"] = LON0 + pose.east / SCALE.lon;
  sdk.values["position/h-sl-ft"] = pose.height / METERS_PER_FOOT;
  sdk.values["attitude/phi-deg"] = pose.roll;
  sdk.values["attitude/theta-deg"] = pose.pitch;
  sdk.values["attitude/psi-deg"] = pose.heading;
  sdk.values["velocities/v-down-fps"] = pose.sink;
}

/** One aircraft flying the same script on the reference and on the kernel. */
function flight(core: GroundContactCore | undefined, units: Unit[], ground: Ground, world: World) {
  const sdk = aircraft(units);
  const surface = surfaceOf(ground, world);
  const rawContact = createTerrainContact(sdk as never, surface, STANCE, { core });
  // These paired tests exercise support arithmetic after an explicit accepted
  // correction. The consent/decline behavior has its own focused suite.
  const contact = { ...rawContact, update(...args: Parameters<typeof rawContact.update>): ReturnType<typeof rawContact.update> {
    const result = rawContact.update(...args);
    const proposal = rawContact.getRelocationProposal();
    if (!proposal) return result;
    expect(rawContact.acceptRelocation(proposal.id)).toBe(true);
    return "reset";
  } };
  return { sdk, surface, contact };
}

const near = (a: number, b: number, scale = 1) => Math.abs(a - b) / Math.max(scale, Math.abs(a), Math.abs(b));

describe("WASM contact support", () => {
  it("matches the reference to rounding over random gear, terrain, refinements, resets, model loads and surface errors", () => {
    const draw = random(31);
    // The kernel's sin and cos are libm's, which can differ from V8's by an ulp.
    let worstWrite = 0, worstSample = 0, steps = 0, sampled = 0, followed = 0, abandoned = 0;
    for (let trial = 0; trial < 60; trial++) {
      const spread = (size: number) => (draw() * 2 - 1) * size;
      const google = draw() < 0.5, retractable = draw() < 0.3;
      const slopeNorth = spread(0.15), slopeEast = spread(0.15), wallEast = 4 + draw() * 6, wall = draw() < 0.5 ? draw() * 1.5 : 0;
      const holeNorth = draw() < 0.3 ? 20 + draw() * 40 : Infinity;
      const refinement = Array.from({ length: 512 }, () => (draw() < 0.3 ? spread(0.2) : 0));
      const ground: Ground = (north, east, revision) => {
        if (north > holeNorth && north < holeNorth + 3) return null;
        return 300 + slopeNorth * north + slopeEast * east + 0.15 * Math.sin(north / 7) * Math.cos(east / 5)
          + (east > wallEast ? wall : 0) + refinement[Math.floor(revision / (google ? 8 : 1)) % refinement.length];
      };
      const world: World = { revision: 0, hidden: false };
      const units = layout(draw, { retractable });
      const reference = flight(undefined, units, ground, world);
      const wasmRun = flight(core, units, ground, world);
      expect(wasmRun.contact.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
      const pose: Pose = { north: 0, east: 0, height: 0, roll: spread(4), pitch: 2 + spread(4), heading: draw() * 360, sink: 0 };
      const speed = draw() * 15, hop = draw() < 0.4 ? 5 + draw() * 30 : 0;
      const resetAt = draw() < 0.5 ? Math.floor(draw() * 200) : -1, loadAt = draw() < 0.3 ? Math.floor(draw() * 200) : -1;
      const raiseAt = retractable ? 60 + Math.floor(draw() * 60) : -1;
      // A few steps with no surface at all: the step after snaps to the samples.
      const blindAt = draw() < 0.6 ? 20 + Math.floor(draw() * 150) : -1, blindFor = 1 + Math.floor(draw() * 20);
      // Now and then a sample throws, often among the contacts' samples.
      const failEvery = draw() < 0.6 ? 3 + Math.floor(draw() * 20) : 0;
      for (let step = 0; step < 200; step++) {
        if (google || step % 40 === 39) world.revision++;
        world.hidden = step >= blindAt && step < blindAt + blindFor;
        world.failAt = failEvery && step % failEvery === failEvery - 1 ? Math.floor(draw() * 12) : undefined;
        if (step === resetAt) { reference.contact.reset(); wasmRun.contact.reset(); }
        if (step === loadAt) {
          const next = layout(draw, { structures: draw() < 0.5, retractable });
          reference.sdk.load(next);
          wasmRun.sdk.load(next);
        }
        const gear = step >= raiseAt && step < raiseAt + 40 ? 0 : 1;
        reference.sdk.values["gear/gear-pos-norm"] = wasmRun.sdk.values["gear/gear-pos-norm"] = gear;
        const turn = (pose.heading + spread(1)) % 360;
        pose.heading = turn < 0 ? turn + 360 : turn;
        pose.north += speed / 120 * Math.cos(pose.heading * Math.PI / 180);
        pose.east += speed / 120 * Math.sin(pose.heading * Math.PI / 180);
        pose.roll = Math.max(-15, Math.min(15, pose.roll + spread(0.5)));
        pose.pitch = Math.max(-5, Math.min(15, pose.pitch + spread(0.5)));
        pose.sink = spread(2);
        const terrain = ground(pose.north, pose.east, world.revision) ?? 300;
        pose.height = terrain + 0.95 + spread(0.08) + hop * Math.max(0, Math.sin(step / 25));
        place(reference.sdk, pose);
        place(wasmRun.sdk, pose);
        reference.sdk.setPropertyValue.mockClear();
        wasmRun.sdk.setPropertyValue.mockClear();
        reference.surface.calls.length = wasmRun.surface.calls.length = 0;
        const update = (side: typeof reference) => {
          try { return side.contact.update(false, google, true, true); }
          catch (error) { return `threw: ${(error as Error).message}`; }
        };
        const expected = update(reference);
        const actual = update(wasmRun);
        const where = `trial ${trial} step ${step}`;
        expect(actual, where).toBe(expected);
        if (expected === "reset") followed++;
        // Past the centre's sample and its anchor: a contact's sample threw.
        if (expected === "threw: tile released" && world.failAt! >= 2) abandoned++;
        steps++;
        // The same samples at the same spots, in the same order.
        expect(wasmRun.surface.calls.length, `${where} samples`).toBe(reference.surface.calls.length);
        reference.surface.calls.forEach(([lat, lon], index) => {
          const [otherLat, otherLon] = wasmRun.surface.calls[index];
          worstSample = Math.max(worstSample, Math.abs(otherLat - lat) * SCALE.lat, Math.abs(otherLon - lon) * SCALE.lon);
        });
        sampled += reference.surface.calls.length;
        // The same writes to JSBSim, to rounding.
        const writes = reference.sdk.setPropertyValue.mock.calls, others = wasmRun.sdk.setPropertyValue.mock.calls;
        expect(others.map(([name]) => name), `${where} writes`).toEqual(writes.map(([name]) => name));
        writes.forEach(([name, value], index) => {
          const difference = near(others[index][1], value);
          if (!(difference <= 1e-12)) expect.fail(`${where} ${name}: ${others[index][1]} is not ${value}`);
          worstWrite = Math.max(worstWrite, difference);
        });
      }
      expect(wasmRun.contact.status(), `trial ${trial}`).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
      wasmRun.contact.dispose();
    }
    // Footprints, refinements and the follow are all exercised.
    expect(steps).toBe(12_000);
    expect(sampled).toBeGreaterThan(steps * 2);
    expect(followed).toBeGreaterThan(20);
    expect(abandoned).toBeGreaterThan(50);
    expect(worstSample).toBeLessThan(1e-6);
    expect(worstWrite).toBeLessThan(1e-12);
  });

  const flat: Ground = () => 300;
  const rest = (sdk: Aircraft, step = 0) => place(sdk, { north: step * 0.05, east: 0, height: 300.95, roll: 0.5, pitch: 2, heading: 0, sink: 0 });

  it("waits for a reset to take over from the reference when a core loads in flight", async () => {
    let arrive!: (core: GroundContactCore) => void;
    const world = { revision: 1 };
    const { sdk, surface, contact } = flight(new Promise<GroundContactCore>(done => { arrive = done; }) as never, layout(random(1)), flat, world);
    const build = vi.spyOn(core, "createContactSupport");
    rest(sdk);
    expect(contact.update(false, false, true, true)).not.toBe(false);
    arrive(core);
    await Promise.resolve();
    // The reference holds every contact's sample; the kernel would start without them.
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "typescript",
      fallbackReason: "wasm-contact-support-pending: the reference holds contact samples until a reset" });
    rest(sdk, 1);
    contact.update(false, false, true, true);
    expect(build).not.toHaveBeenCalled();
    contact.reset();
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    const before = surface.calls.length;
    rest(sdk, 2);
    expect(contact.update(false, false, true, true)).not.toBe(false);
    expect(build).toHaveBeenCalledOnce();
    // The centre, then every contact's first footprint.
    expect(surface.calls.length - before).toBe(1 + C172_GROUND_CONTACTS.length);
    expect(sdk.values["position/terrain-elevation-asl-ft"] * METERS_PER_FOOT).toBeCloseTo(300, 9);
  });

  it("gives a failed kernel up for good and carries on with the reference", async () => {
    const own = await wasm();
    const world = { revision: 1 };
    const { sdk, contact } = flight(own, layout(random(2)), flat, world);
    rest(sdk);
    contact.update(false, false, true, true);
    own.dispose();
    rest(sdk, 1);
    expect(contact.update(false, false, true, true)).not.toBe(false);
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "typescript",
      fallbackReason: "wasm-contact-support-failed: WASM ground-contact core is disposed" });
    expect(sdk.values["position/terrain-elevation-asl-ft"] * METERS_PER_FOOT).toBeCloseTo(300, 9);
    rest(sdk, 2);
    expect(contact.update(false, false, true, true)).not.toBe(false);
  });

  it("passes the surface's errors to the caller and abandons the step as a reset does, keeping the kernel", () => {
    const world: World = { revision: 1 };
    const build = vi.spyOn(core, "createContactSupport");
    const units = layout(random(3));
    const { sdk, surface, contact } = flight(core, units, flat, world);
    rest(sdk);
    contact.update(false, false, true, true);
    const kernel = build.mock.results[0].value as ReturnType<typeof core.createContactSupport>;
    const finish = vi.spyOn(kernel, "finish"), reset = vi.spyOn(kernel, "reset");
    // A refinement: the centre, its anchor, then the contacts' samples; the second of those fails.
    world.revision++;
    world.failAt = 3;
    surface.calls.length = 0;
    rest(sdk, 1);
    expect(() => contact.update(false, false, true, true)).toThrow("tile released");
    expect(surface.calls).toHaveLength(4);
    // The contact sampled before the error is not kept: no half-sampled step is finished.
    expect(finish).not.toHaveBeenCalled();
    expect(reset).toHaveBeenCalledOnce();
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    // The same kernel finds the contacts again: the centre, its anchor (the
    // step that threw did not keep the refinement), then every first footprint.
    world.failAt = undefined;
    surface.calls.length = 0;
    rest(sdk, 2);
    expect(contact.update(false, false, true, true)).not.toBe(false);
    expect(build).toHaveBeenCalledOnce();
    expect(finish).toHaveBeenCalledOnce();
    expect(surface.calls).toHaveLength(2 + units.length);
    expect(sdk.values["position/terrain-elevation-asl-ft"] * METERS_PER_FOOT).toBeCloseTo(300, 9);
    expect(contact.status().fallbackReason).toBeNull();
  });

  it("abandons the step on the reference too, which then hands over to a core that loaded in flight", async () => {
    let arrive!: (core: GroundContactCore) => void;
    const world: World = { revision: 1 };
    const { sdk, surface, contact } = flight(new Promise<GroundContactCore>(done => { arrive = done; }) as never, layout(random(4)), flat, world);
    const build = vi.spyOn(core, "createContactSupport");
    rest(sdk);
    contact.update(false, false, true, true);
    arrive(core);
    await Promise.resolve();
    expect(contact.status().fallbackReason).toMatch(/^wasm-contact-support-pending/);
    world.revision++;
    world.failAt = 3;
    surface.calls.length = 0;
    rest(sdk, 1);
    expect(() => contact.update(false, false, true, true)).toThrow("tile released");
    // The reference let its samples go, so nothing is lost by the switch.
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    world.failAt = undefined;
    rest(sdk, 2);
    expect(contact.update(false, false, true, true)).not.toBe(false);
    expect(build).toHaveBeenCalledOnce();
    expect(sdk.values["position/terrain-elevation-asl-ft"] * METERS_PER_FOOT).toBeCloseTo(300, 9);
  });

  it("hands its step counters on across model loads and releases each kernel", () => {
    const world = { revision: 1 };
    const draw = random(4);
    const { sdk, contact } = flight(core, layout(draw), flat, world);
    let releases = 0;
    const original = core.createContactSupport.bind(core);
    const tracked = vi.spyOn(core, "createContactSupport").mockImplementation((units, previous) => {
      const kernel = original(units, previous);
      const dispose = kernel.dispose.bind(kernel);
      kernel.dispose = () => { releases++; dispose(); };
      return kernel;
    });
    // A core holds a bounded number of kernels: a leak would end on the reference.
    for (let load = 0; load < 40; load++) {
      sdk.load(layout(draw, { structures: load % 2 === 0 }));
      // As on the reference, a step finds the count changed and the next finds the units.
      for (let step = 0; step < 2; step++) {
        rest(sdk, load * 2 + step);
        expect(contact.update(false, false, true, true)).not.toBe(false);
      }
    }
    expect(contact.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    expect(tracked).toHaveBeenCalledTimes(40);
    // Every load after the first hands the previous kernel on.
    for (const [, previous] of tracked.mock.calls.slice(1)) expect(previous).toBeDefined();
    expect(releases).toBe(39);
    contact.dispose();
    expect(releases).toBe(40);
    expect(contact.status().kernel).toBe("typescript");
  });

  it("allocates nothing in WASM per step once the contacts are found", () => {
    const world = { revision: 1 };
    const { sdk, contact } = flight(core, layout(random(5)), (north, east) => 300 + 0.1 * north + 0.05 * Math.sin(east), world);
    rest(sdk);
    contact.update(false, true, true, true);
    const before = core.heapAllocations();
    for (let step = 1; step < 200; step++) {
      world.revision++;
      place(sdk, { north: step * 0.05, east: 0, height: 300.95 + 0.1 * step * 0.05, roll: 0.5, pitch: 2, heading: 0, sink: 0 });
      contact.update(false, true, true, true);
    }
    expect(core.heapAllocations()).toBe(before);
    expect(contact.status().kernel).toBe("single");
  });
});
