// Times single/ground_contact.wasm on one full 120 Hz step for the C172: the
// hull's place and resolve for its five probes (clear, and two wingtip
// strikes), the support for its nine contact units with two surface samples,
// the coupled wheels and wheel spin for its three wheels. The surface queries
// are the host's and are not timed. Prints the median and fastest time per
// step (with and without the host's writes into the views) and per kernel,
// the host and its power state. A laptop's numbers move with power source,
// thermal state and load; compare runs only on the same machine in the same
// state. Correctness is scripts/check.mjs's job, not this script's.
import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { HULL, instantiateGroundContact, SPIN, SUPPORT } from "../index.js";

const root = fileURLToPath(new URL("../", import.meta.url));
const core = await instantiateGroundContact(readFileSync(path.join(root, "single/ground_contact.wasm")));
const quiet = (command, args) => { try { return execFileSync(command, args, { encoding: "utf8" }).trim(); } catch { return "unknown"; } };

// collisionGeometry.ts's C172_BODY_COLLISION_PROBES and C172_GROUND_CONTACTS, structural inches (aft, right, up).
const PROBES = [[-58.7, 0, 34.6, 0], [42, 0, 11.9, 3], [255.9, 0, 35.5, 3], [53.7, -216.5, 68.5, 3], [53.7, 216.5, 68.5, 3]];
const UNITS = [[-6.8, 0, -19.5, 1], [58.2, -43, -15.5, 1], [58.2, 43, -15.5, 1], [-37.7, 0, 26.6, 0], [175.2, 0, 20.6, 0],
  [53.7, -216.5, 68.5, 0], [53.7, 216.5, 68.5, 0], [42, 0, 11.9, 0], [255.9, 0, 35.5, 0]];
const LAT = 44.88, LON = -93.22, GROUND = 260;

const hull = core.createHullCollision(PROBES.length);
const support = core.createContactSupport(UNITS.map(([, , , wheel]) => ({ retractable: false, wheel: wheel === 1 })));
const wheels = core.createWheelSolver(3), spin = core.createWheelSpin(3);

// The hull 1.5 m over the ground, sinking 5 fps and turning, from 0.3 ft higher a step ago.
const hullValues = Float64Array.from([LAT, LON, (GROUND + 1.5) / 0.3048, 2, 3, 45, 80, 10, 5, 0.05, -0.02, 0.01, 41, 0, 36.5, 75, 948, 1346, 1967, 0, 0, 0]);
hull.probes.set(PROBES.flat());
hull.values.set(hullValues);
hull.values[2] += 0.3;
hull.place(); hull.swap();
hull.values.set(hullValues);
hull.place();
// The wingtips strike midway along their sweeps; nothing else is struck.
const strike = new Float64Array(PROBES.length * HULL.hitStride).fill(NaN), clear = new Float64Array(PROBES.length * HULL.hitStride).fill(NaN);
for (const i of [3, 4]) {
  const s = hull.segments.subarray(i * HULL.segmentStride, (i + 1) * HULL.segmentStride);
  strike.set([s[6] / 2, s[0] + s[3] / 2, s[1] + s[4] / 2, s[2] + s[5] / 2, -s[3] / s[6], -s[4] / s[6], -s[5] / s[6]], i * HULL.hitStride);
}

// The support level over flat ground; moving 11 cm a step, so two contacts are sampled again each step.
const supportInput = Float64Array.from([LAT, LON, GROUND + 1.42, 0.02, 0.05, 1.2, GROUND, 1, 0, 0, 0, 0, 2, NaN]);
const supportValues = Float64Array.from([UNITS.length, 1, 41, 0, 36.5, ...UNITS.flatMap(([x, y, z]) => [x, y, z])]);
const answer = { heightMeters: GROUND, revision: 1 }, sample = () => answer;
let moved = 0;
function measure() {
  moved ^= 1;
  support.input[0] = LAT + moved * 1e-6;
  return support.measure(SUPPORT.GEOMETRY_READ, sample);
}

const body = Float64Array.from([1100, 1 / 1285, 0, 0, 0, 1 / 1825, 0, 0, 0, 1 / 2667, 20, 0, -0.5, 0, 0.01, 0]);
const wheelRows = Float64Array.from([1, 2, 3].flatMap((id, i) => [id, 0, -1, 0, 0, i ? 0.124 : 0.052, 0.5, i ? 450 : 0, 0.2, i ? 0.3 : 0]));
const wheelContacts = Float64Array.from([1, 2, 3].flatMap((id, i) => [id, [1.4, -0.6, -0.6][i], [0, -1.1, 1.1][i], 1.4, 0, 0, -1, 1, 0, 0, 0, 1, 0,
  20, 0, 0, 0, 0, 0, [1500, 3200, 3200][i], 0.05, 0, i ? 0.22 : 0.18, 1, 1]));
const spinRows = Float64Array.from([0.18, 0.22, 0.22].flatMap((radius, i) => [0, 0, 0, 0, 0, 0, 0, 0, 0, radius, i ? 0.152 : 0.127, i ? 0.124 : 0.052, 0.5, i ? 450 : 0,
  1, 20, [1500, 3200, 3200][i], 0.05, 0, i ? 0.3 : 0]));
wheels.contacts.set(wheelContacts);
// The host's per-step writes: the property reads, the casts' hits and the wheel inputs.
function writeInputs(hits) {
  hull.values.set(hullValues);
  hull.hits.set(hits);
  support.input.set(supportInput);
  support.values.set(supportValues);
  wheels.body.set(body);
  wheels.wheels.set(wheelRows);
  spin.wheels.set(spinRows);
}
writeInputs(strike);
for (let i = 0; i < 100; i++) measure();
// What one step solves, so a change in the work shows next to a change in the time.
const workload = {
  // Output rows are in hit order: the two wingtips are rows 0 and 1. The first
  // one's lift moves the CG out along the shared normal and clears the second.
  hullStrikeHits: (hull.place(), hull.resolve()), hullLiftMeters: [0, 1].map(k => hull.output[HULL.headLength + k * HULL.rowStride + 8]),
  supportRequests: measure()[4], supportElevation: support.output[0],
  wheelIterations: (wheels.run(1 / 120, 3), wheels.output[0]), wheelRejected: wheels.output[4],
};
hull.hits.set(clear);
workload.hullClearHits = hull.resolve();
if (workload.hullStrikeHits !== 2 || !(workload.hullLiftMeters[0] > 0.01) || workload.hullClearHits !== 0
  || workload.supportRequests !== 2 || workload.wheelRejected !== 0) {
  throw new Error(`Benchmark workload is not the intended step: ${JSON.stringify(workload)}`);
}

const kernels = {
  hullPlace: () => hull.place(),
  hullResolveClear: () => hull.resolve(),
  hullResolveStrike: () => hull.resolve(),
  support: measure,
  wheels: () => wheels.run(1 / 120, 3),
  spin: () => spin.run(1 / 120, SPIN.INERTIA),
  hostWrites: () => writeInputs(strike),
};
const physics = () => { measure(); wheels.run(1 / 120, 3); spin.run(1 / 120, SPIN.INERTIA); };
const steps = {
  clear: () => { hull.place(); hull.resolve(); physics(); },
  strike: () => { hull.place(); hull.resolve(); physics(); },
  // As the host steps: its writes into the views first. Each step then starts
  // from the same wheel state, so this is not strike plus hostWrites.
  strikeWithWrites: () => { writeInputs(strike); hull.place(); hull.resolve(); physics(); },
};
const hitsFor = name => /Strike|strike/.test(name) ? strike : clear;

const BATCH = 10_000, BATCHES = 15;
function time(run, hits) {
  writeInputs(hits);
  for (let i = 0; i < 20_000; i++) run();
  const samples = [];
  for (let batch = 0; batch < BATCHES; batch++) {
    // The wheel and spin states evolve; restoring them keeps every batch the same work.
    writeInputs(hits);
    const start = process.hrtime.bigint();
    for (let i = 0; i < BATCH; i++) run();
    samples.push(Number(process.hrtime.bigint() - start) / BATCH);
  }
  samples.sort((a, b) => a - b);
  return { medianNs: Math.round(samples[BATCHES >> 1]), fastestNs: Math.round(samples[0]) };
}

const allocations = core.heapAllocations();
const perKernel = Object.fromEntries(Object.entries(kernels).map(([name, run]) => [name, time(run, hitsFor(name))]));
const perStep = Object.fromEntries(Object.entries(steps).map(([name, run]) => [name, time(run, hitsFor(name))]));
const budgetNs = 5_000;
console.log(JSON.stringify({
  host: { node: process.versions.node, cpu: quiet("sysctl", ["-n", "machdep.cpu.brand_string"]), power: quiet("pmset", ["-g", "batt"]).split("\n").slice(0, 2).join(" | ") },
  probes: PROBES.length, contactUnits: UNITS.length, wheels: 3, workload,
  perKernel, perStep,
  budget: { targetNs: budgetNs, worstMedianNs: Math.max(...Object.values(perStep).map(s => s.medianNs)), met: Object.values(perStep).every(s => s.medianNs < budgetNs) },
  heapAllocationsWhileTiming: core.heapAllocations() - allocations,
}, null, 2));
core.dispose();
