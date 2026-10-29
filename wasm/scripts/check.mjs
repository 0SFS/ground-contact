// Checks single/ground_contact.wasm against its build receipt and the live
// TypeScript in ../src (read only): coupled rigid wheels and wheel spin bit
// for bit; visibleMeshCollision.ts's createVisibleMeshCollision and
// terrainContact.ts's createContactSupport against the WASM kernels driven
// by the host flow README.md gives, and against the TypeScript's own WASM
// path (the same functions given this core, which must never fall back), over
// multi-step sequences on mock JSBSim models and surfaces; zero heap
// allocations while stepping; and the error codes. Correctness only: timings
// come from scripts/bench.mjs.
//
// The hull and the support run twice. With the TypeScript's Math.sin, cos
// and atan2 replaced by the core's (in memory, for modules imported with
// `?wasm-trig`), every value must match bit for bit: the rest of the
// arithmetic is the TypeScript's, in its order. Unmodified, V8's trig can
// differ from the core's libm by an ulp, so values must agree within a bound
// and the report gives the largest difference and how many matched exactly.
import { createHash } from "node:crypto";
import { existsSync, readFileSync, readdirSync } from "node:fs";
import { registerHooks } from "node:module";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import { createGroundContact, ERRORS, HULL, SPIN, SUPPORT, WHEELS } from "../index.js";

const root = fileURLToPath(new URL("../", import.meta.url));
const source = path.join(root, "..", "src");
// Sibling packages of ground-contact in the 0SFS checkout, and FOSS Earth beside it.
const packages = path.join(root, "..", "..");
const cameraMathPath = [process.env.FOSS_EARTH_ENGINE && path.join(process.env.FOSS_EARTH_ENGINE, "src/camera/cameraMath.ts"),
  path.join(packages, "../foss-earth/engine/src/camera/cameraMath.ts")].filter(Boolean).find(file => existsSync(file));
if (!cameraMathPath) throw new Error("visibleMeshCollision.ts needs foss-earth's cameraMath.ts: set FOSS_EARTH_ENGINE to foss-earth/engine");

// Receipt: every input and the binary are what build.mjs recorded.
const sha = file => createHash("sha256").update(readFileSync(path.join(root, file))).digest("hex");
const metadata = JSON.parse(readFileSync(path.join(root, "build-metadata.json"), "utf8"));
const crateFiles = directory => readdirSync(path.join(root, directory), { withFileTypes: true })
  .flatMap(entry => entry.isDirectory() ? crateFiles(path.posix.join(directory, entry.name)) : [path.posix.join(directory, entry.name)]);
const stale = [...Object.entries(metadata.inputs).filter(([file, hash]) => !existsSync(path.join(root, file)) || sha(file) !== hash).map(([file]) => file),
  ...crateFiles("crates").filter(file => !(file in metadata.inputs))];
const wasmFile = "single/ground_contact.wasm";
if (stale.length || sha(wasmFile) !== metadata.outputs[wasmFile]?.sha256) {
  throw new Error(`Build receipt is stale (${stale.join(", ") || wasmFile}); run scripts/build.mjs`);
}
const { instance } = await WebAssembly.instantiate(readFileSync(path.join(root, wasmFile)), {});
const raw = instance.exports;
const core = createGroundContact(raw);

// The TypeScript imports its siblings without an extension and two aliases
// the app's bundler resolves. Under `?wasm-trig` its trig is the core's;
// createContactSupport, module-private, is exported for the check.
const WASM_TRIG = "wasm-trig";
globalThis.__groundContactWasmMath = core.wasmMath;
const substituted = {};
function alias(specifier) {
  if (specifier === "foss-earth/cameraMath") return cameraMathPath;
  const match = /^@0sfs\/([^/]+)\/([^/]+)$/.exec(specifier);
  const file = match && path.join(packages, match[1], "src", `${match[2]}.ts`);
  return file && existsSync(file) ? file : null;
}
registerHooks({
  resolve(specifier, context, next) {
    const aliased = alias(specifier);
    let result;
    if (aliased) result = { url: pathToFileURL(aliased).href, shortCircuit: true };
    else {
      try { result = next(specifier, context); } catch (error) {
        if (error?.code !== "ERR_MODULE_NOT_FOUND" || !/^\.\.?\//.test(specifier) || specifier.endsWith(".ts")) throw error;
        result = next(`${specifier}.ts`, context);
      }
    }
    const wasm = new URL(context.parentURL ?? "file:///").searchParams.has(WASM_TRIG);
    if (wasm && result.url.startsWith("file:") && result.url.endsWith(".ts")) result = { ...result, url: `${result.url}?${WASM_TRIG}` };
    return result;
  },
  load(url, context, next) {
    const result = next(url, context);
    const location = new URL(url);
    if (!location.pathname.endsWith(".ts")) return result;
    let text = typeof result.source === "string" ? result.source : new TextDecoder().decode(result.source);
    const original = text;
    if (location.searchParams.has(WASM_TRIG)) {
      const file = path.basename(location.pathname);
      text = text.replace(/\bMath\.(sin|cos|atan2)\(/g, (_, name) => {
        substituted[file] = (substituted[file] ?? 0) + 1;
        return `globalThis.__groundContactWasmMath.${name}(`;
      });
    }
    if (location.pathname.endsWith("/terrainContact.ts")) text = text.replace(/^function createContactSupport\(/m, "export function createContactSupport(");
    return text === original ? result : { ...result, source: text };
  },
});
const typescript = (file, wasm = false) => import(`${pathToFileURL(path.join(source, file)).href}${wasm ? `?${WASM_TRIG}` : ""}`);
const coupled = await typescript("coupledRigidWheels.ts");
const spinTs = await typescript("wheelSpin.ts");
const geometry = await typescript("collisionGeometry.ts");
const clearance = await typescript("groundContactClearance.ts");
const flight = await typescript("safeFlightState.ts");
const cameraMath = await import(pathToFileURL(cameraMathPath).href);
const modes = {
  wasm: { trig: "core", collision: await typescript("visibleMeshCollision.ts", true), terrain: await typescript("terrainContact.ts", true) },
  v8: { trig: "V8", collision: await typescript("visibleMeshCollision.ts"), terrain: await typescript("terrainContact.ts") },
};
for (const [name, mode] of Object.entries(modes)) {
  if (typeof mode.terrain.createContactSupport !== "function") throw new Error(`terrainContact.ts (${name}) no longer declares createContactSupport`);
}
for (const file of ["visibleMeshCollision.ts", "collisionGeometry.ts", "cameraMath.ts"]) {
  if (!substituted[file]) throw new Error(`${file} imported with ?${WASM_TRIG} kept V8's trig`);
}

const failures = [];
let failureCount = 0;
function expect(condition, message) {
  if (!condition) { failureCount++; if (failures.length < 25) failures.push(message); }
  return condition;
}
const compared = { wheels: 0, spin: 0 };
function same(kind, label, expected, actual) {
  compared[kind]++;
  expect(Object.is(expected, actual), `${kind} ${label}: TypeScript ${expected} vs WASM ${actual}`);
}

// Deterministic inputs.
let seed = 0x0c172f35;
function rand() {
  seed = (seed + 0x6d2b79f5) >>> 0;
  let t = seed;
  t = Math.imul(t ^ (t >>> 15), t | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
const uniform = (low, high) => low + (high - low) * rand();
const chance = p => rand() < p;
const pick = values => values[Math.floor(rand() * values.length)];
const SPECIAL = [NaN, Infinity, -Infinity, -0, 0, 1e300, -1e300, 5e-324];
const maybe = (value, p = 0.03) => chance(p) ? pick(SPECIAL) : value;

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => scale(a, 1 / norm(a));
const at3 = (values, offset) => [values[offset], values[offset + 1], values[offset + 2]];
const xyz = v => [v.x, v.y, v.z];
const vec = ([x, y, z]) => ({ x, y, z });
const randomVector = size => [uniform(-size, size), uniform(-size, size), uniform(-size, size)];
function randomUnit() {
  for (;;) { const v = randomVector(1), length = norm(v); if (length > 0.1 && length < 1) return scale(v, 1 / length); }
}
const matVec = (m, v) => [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]];
const matMul = (a, b) => Array.from({ length: 9 }, (_, i) => a[i - i % 3] * b[i % 3] + a[i - i % 3 + 1] * b[3 + i % 3] + a[i - i % 3 + 2] * b[6 + i % 3]);
const transpose = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
function invert(m) {
  const det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  return [(m[4] * m[8] - m[5] * m[7]) / det, (m[2] * m[7] - m[1] * m[8]) / det, (m[1] * m[5] - m[2] * m[4]) / det,
    (m[5] * m[6] - m[3] * m[8]) / det, (m[0] * m[8] - m[2] * m[6]) / det, (m[2] * m[3] - m[0] * m[5]) / det,
    (m[3] * m[7] - m[4] * m[6]) / det, (m[1] * m[6] - m[0] * m[7]) / det, (m[0] * m[4] - m[1] * m[3]) / det];
}
function randomRotation() {
  let q;
  do q = [uniform(-1, 1), uniform(-1, 1), uniform(-1, 1), uniform(-1, 1)]; while (Math.hypot(...q) < 0.1);
  const length = Math.hypot(...q), [w, x, y, z] = q.map(value => value / length);
  return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
    2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)];
}

// 1. solveCoupledRigidWheels, bit for bit, over carried multi-step sequences.
const NAMES = ["NOSE", "LEFT_MAIN", "RIGHT_MAIN", "TAIL", "AUX"];
const wheelCoverage = { steps: 0, grounded: 0, rejected: 0, coldRetries: 0, warmStarts: 0, airborne: 0 };
function wheelContact(wheel, epoch) {
  const normal = randomUnit(), forward = unit(cross(normal, randomUnit())), lateral = cross(normal, forward);
  const contact = {
    wheel, axleOffsetM: vec(randomVector(3)), normalWorld: vec(normal), forwardWorld: vec(forward), lateralWorld: vec(lateral),
    wheelCenterVelocityMps: vec(randomVector(40)), groundVelocityMps: vec(chance(0.8) ? [0, 0, 0] : randomVector(5)),
    normalLoadN: chance(0.1) ? pick([0, -5, 6e5]) : uniform(0, 15000), compressionM: uniform(0, 0.2), steeringRad: uniform(-0.5, 0.5),
    radiusM: chance(0.05) ? pick([0.01, 3]) : pick([0.18, 0.22]), groundRevision: chance(0.03) ? pick([-1, 1.5, NaN]) : Math.floor(uniform(0, 100)),
    contactEpoch: chance(0.03) ? pick([-1, 2.5, NaN, 2 ** 53]) : epoch,
  };
  if (chance(0.04)) {
    const key = pick(["axleOffsetM", "normalWorld", "forwardWorld", "lateralWorld", "wheelCenterVelocityMps", "groundVelocityMps"]);
    contact[key] = chance(0.5) ? { ...contact[key], y: pick([NaN, Infinity]) } : vec(scale(xyz(contact[key]), 1.01 + (key.endsWith("Mps") ? 20 : 0)));
  }
  return contact;
}
const contactRow = (contact, names) => [names.indexOf(contact.wheel) >= 0 ? names.indexOf(contact.wheel) + 1 : 99, ...xyz(contact.axleOffsetM),
  ...xyz(contact.normalWorld), ...xyz(contact.forwardWorld), ...xyz(contact.lateralWorld), ...xyz(contact.wheelCenterVelocityMps),
  ...xyz(contact.groundVelocityMps), contact.normalLoadN, contact.compressionM, contact.steeringRad, contact.radiusM, contact.groundRevision, contact.contactEpoch];
for (let sequence = 0; sequence < 300; sequence++) {
  const n = pick([1, 2, 3, 3, 3, 5]), names = NAMES.slice(0, n);
  const solver = core.createWheelSolver(n, 12);
  const rotation = randomRotation(), principal = [1 / uniform(800, 2000), 1 / uniform(1000, 3000), 1 / uniform(1500, 4000)];
  const inverse = matMul(matMul(rotation, [principal[0], 0, 0, 0, principal[1], 0, 0, 0, principal[2]]), transpose(rotation));
  const body = { massKg: chance(0.03) ? pick([0, -1, NaN]) : uniform(600, 3000), inverseInertiaWorld: inverse,
    velocity: vec(randomVector(30)), angularVelocity: vec(randomVector(0.5)) };
  const states = names.map(() => coupled.createCoupledWheelState());
  const params = names.map(() => ({ inertiaKgM2: maybe(pick([0.052, 0.124, uniform(0.01, 2), 1e-4])), frictionCoefficient: maybe(uniform(0, 1.2)),
    maxBrakeTorqueNm: maybe(pick([0, 450, uniform(0, 2000)])), bearingDragTorqueNm: maybe(pick([0, 0.5, uniform(0, 5)])) }));
  const epochs = names.map(() => Math.floor(uniform(0, 5)));
  solver.body.set([body.massKg, ...inverse, ...xyz(body.velocity), ...xyz(body.angularVelocity)]);
  names.forEach((_, i) => solver.wheels.set([i + 1, 0, -1, 0, 0], i * WHEELS.wheelStride));
  for (let step = 0; step < 40; step++) {
    const wheels = solver.wheels;
    if (chance(0.1)) {
      body.velocity = vec(randomVector(30)); body.angularVelocity = vec(randomVector(0.5));
      solver.body.set([...xyz(body.velocity), ...xyz(body.angularVelocity)], 10);
    }
    if (chance(0.08)) {
      // Large stale warm impulses, so the energy guard's cold retry runs.
      states.forEach((state, i) => {
        state.warmTangentNs = uniform(-800, 800); state.warmBrakeNms = uniform(-20, 20);
        wheels[i * WHEELS.wheelStride + 3] = state.warmTangentNs; wheels[i * WHEELS.wheelStride + 4] = state.warmBrakeNms;
      });
    }
    const brakes = names.map(() => maybe(chance(0.5) ? 0 : uniform(-0.2, 1.2)));
    const contacts = [];
    for (let i = 0; i < n; i++) {
      if (chance(0.25)) continue;
      if (chance(0.1)) epochs[i]++;
      contacts.push(wheelContact(names[i], epochs[i]));
      if (chance(0.03)) contacts.push(wheelContact(names[i], epochs[i]));
    }
    if (chance(0.05)) contacts.push(wheelContact("GHOST", 0));
    const dt = chance(0.9) ? 1 / 120 : pick([0, -1, NaN, Infinity, 1, 1 / 30, 1 / 60]);
    const iterations = chance(0.7) ? undefined : pick([NaN, 0, 1, 4, 8, 16, 40, 2.7, -3, Infinity]);
    const warmStart = chance(0.8) ? undefined : pick([true, false]);
    const results = names.map(() => coupled.createWheelForceResult());
    const report = coupled.solveCoupledRigidWheels(body, names, states, params, contacts, brakes, dt, results, { iterations, warmStart });
    params.forEach((p, i) => wheels.set([p.inertiaKgM2, p.frictionCoefficient, p.maxBrakeTorqueNm, p.bearingDragTorqueNm, brakes[i]], i * WHEELS.wheelStride + 5));
    contacts.forEach((contact, k) => solver.contacts.set(contactRow(contact, names), k * WHEELS.contactStride));
    solver.run(dt, contacts.length, iterations ?? NaN, warmStart !== false);
    const tag = `sequence ${sequence} step ${step}`, out = solver.output, b = solver.body;
    [...xyz(body.velocity), ...xyz(body.angularVelocity)].forEach((value, k) => same("wheels", `${tag} body ${k}`, value, b[10 + k]));
    states.forEach((state, i) => [state.spinRadSec, state.contactEpoch, state.warmTangentNs, state.warmBrakeNms]
      .forEach((value, k) => same("wheels", `${tag} wheel ${i} state ${k}`, value, wheels[i * WHEELS.wheelStride + 1 + k])));
    [report.iterations, report.kineticEnergyBeforeJ, report.kineticEnergyAfterJ, report.coldRetry ? 1 : 0, report.rejectedContacts]
      .forEach((value, k) => same("wheels", `${tag} report ${k}`, value, out[k]));
    results.forEach((r, i) => [...xyz(r.bodyImpulseNs), ...xyz(r.bodyAngularImpulseNms), r.wheelAngularImpulseNms, r.longitudinalSlipMps,
      r.lateralSlipMps, r.normalImpulseNs, r.dissipatedJ].forEach((value, k) => same("wheels", `${tag} result ${i}.${k}`, value, out[WHEELS.reportLength + i * WHEELS.resultStride + k])));
    wheelCoverage.steps++;
    wheelCoverage.rejected += report.rejectedContacts;
    wheelCoverage.coldRetries += report.coldRetry ? 1 : 0;
    wheelCoverage.grounded += results.filter(r => r.normalImpulseNs > 0).length;
    wheelCoverage.airborne += n - results.filter(r => r.normalImpulseNs > 0).length;
    wheelCoverage.warmStarts += warmStart !== false && states.some(state => state.warmTangentNs !== 0) ? 1 : 0;
  }
  solver.dispose();
}

// 2. stepWheelSpin, bit for bit, both modes.
const spinCoverage = { steps: 0, inertia: 0, onGround: 0, slipFree: 0 };
for (let sequence = 0; sequence < 200; sequence++) {
  const n = pick([1, 3, 4]), spin = core.createWheelSpin(n);
  const configs = Array.from({ length: n }, (_, i) => chance(0.7) ? { ...spinTs.WHEEL_SPIN_CONFIGS[i % 3] }
    : { name: "NOSE", radiusMeters: maybe(uniform(0.005, 3)), widthMeters: 0.1, inertiaKgMetersSquared: maybe(uniform(0, 200)),
      frictionCoefficient: maybe(uniform(0, 3)), maxBrakeTorqueNewtonMeters: maybe(uniform(0, 20000)) });
  const states = configs.map(() => spinTs.createWheelSpinState());
  const rolls = configs.map(() => uniform(-40, 40));
  spin.wheels.fill(0);
  configs.forEach((c, i) => spin.wheels.set([c.radiusMeters, c.widthMeters, c.inertiaKgMetersSquared, c.frictionCoefficient, c.maxBrakeTorqueNewtonMeters], i * SPIN.wheelStride + SPIN.configOffset));
  for (let step = 0; step < 100; step++) {
    const rows = spin.wheels;
    const dt = chance(0.9) ? 1 / 120 : pick([0, -1, NaN, Infinity, 0.5, 1 / 30]);
    const mode = chance(0.6) ? "inertia" : "instant";
    if (chance(0.02)) states.forEach((state, i) => {
      state.omegaRadSec = maybe(uniform(-500, 500), 0.3); state.angleRad = maybe(uniform(-10, 10), 0.3);
      rows[i * SPIN.wheelStride + 1] = state.omegaRadSec; rows[i * SPIN.wheelStride] = state.angleRad;
    });
    configs.forEach((config, i) => {
      if (chance(0.15)) rolls[i] = chance(0.1) ? 0 : uniform(-80, 80);
      const input = { onGround: chance(0.75), rollMetersSec: maybe(rolls[i]), normalLoadNewtons: maybe(uniform(0, 12000)),
        compressionMeters: maybe(uniform(-0.1, 1.2)), steeringRad: maybe(uniform(-2, 2)), brake: maybe(chance(0.6) ? 0 : uniform(-0.2, 1.2)) };
      spinTs.stepWheelSpin(states[i], config, input, dt, mode);
      rows.set([input.onGround ? 1 : 0, input.rollMetersSec, input.normalLoadNewtons, input.compressionMeters, input.steeringRad, input.brake], i * SPIN.wheelStride + SPIN.inputOffset);
      spinCoverage.onGround += input.onGround ? 1 : 0;
    });
    spin.run(dt, mode);
    states.forEach((s, i) => {
      [s.angleRad, s.omegaRadSec, s.slipMetersSec, s.slipPowerWatts, s.onGround ? 1 : 0, s.compressionMeters, s.steeringRad, s.normalLoadNewtons, s.treadTravelMeters]
        .forEach((value, k) => same("spin", `sequence ${sequence} step ${step} wheel ${i} field ${k}`, value, rows[i * SPIN.wheelStride + k]));
      spinCoverage.slipFree += s.onGround && mode === "inertia" && s.slipMetersSec === 0 ? 1 : 0;
    });
    spinCoverage.steps++;
    spinCoverage.inertia += mode === "inertia" ? 1 : 0;
  }
  spin.dispose();
}

// Shared by the hull and the support: per-call surface behaviour from the
// call's index, so both sides see the same answers to the same calls.
function hashUnit(key, index) {
  let t = Math.imul(key ^ 0x9e3779b9, 0x85ebca6b) ^ Math.imul(index + 0x7f4a7c15, 0xc2b2ae35);
  t ^= t >>> 15; t = Math.imul(t, 0x2c1b3c6d); t ^= t >>> 12; t = Math.imul(t, 0x297a2d39); t ^= t >>> 15;
  return (t >>> 0) / 4294967296;
}
const createTally = () => ({ values: 0, exact: 0, maxAbsolute: 0, worst: null });
/** Bit for bit in wasm-trig mode; within `bound` with V8's trig. */
function agree(tally, mode, label, expected, actual, bound) {
  tally.values++;
  if (Object.is(expected, actual)) { tally.exact++; return true; }
  const apart = Math.abs(expected - actual);
  if (mode === "wasm" || !(apart <= bound)) return expect(false, `${label}: TypeScript ${expected} vs WASM ${actual}${mode === "wasm" ? "" : ` (${apart} apart)`}`);
  if (apart > tally.maxAbsolute) { tally.maxAbsolute = apart; tally.worst = label; }
  return true;
}
const attempt = run => { try { return run(); } catch (error) { return `threw ${error.message}`; } };

const INITIAL = [["ic/lat-geod-deg", "position/lat-geod-deg"], ["ic/long-gc-deg", "position/long-gc-deg"], ["ic/h-sl-ft", "position/h-sl-ft"],
  ["ic/terrain-elevation-ft", "position/terrain-elevation-asl-ft"], ["ic/phi-deg", "attitude/phi-deg"], ["ic/theta-deg", "attitude/theta-deg"],
  ["ic/psi-true-deg", "attitude/psi-deg"], ["ic/vn-fps", "velocities/v-north-fps"], ["ic/ve-fps", "velocities/v-east-fps"],
  ["ic/vd-fps", "velocities/v-down-fps"], ["ic/p-rad_sec", "velocities/p-rad_sec"], ["ic/q-rad_sec", "velocities/q-rad_sec"], ["ic/r-rad_sec", "velocities/r-rad_sec"]];
/** A JSBSim double: scalar properties, RunIC copying ic/* into the state, and the contact catalog when given. */
function createMockSdk(values, catalog = null) {
  const sdk = {
    values, catalog, failing: null,
    getPropertyValue(name) { if (name === sdk.failing) throw new Error(`${name} read failed`); return values[name] ?? 0; },
    setPropertyValue(name, value) { values[name] = value; },
    resetToInitialConditions() {},
    runIc() { for (const [initial, property] of INITIAL) if (initial in values) values[property] = values[initial]; return true; },
  };
  if (catalog !== null) sdk.queryPropertyCatalog = prefix => prefix === "/unit" ? sdk.catalog : "";
  return sdk;
}

// 3. createVisibleMeshCollision against the hull kernel in the host flow.
const PATHS = ["position/lat-geod-deg", "position/long-gc-deg", "position/h-sl-ft", "attitude/phi-deg", "attitude/theta-deg", "attitude/psi-deg",
  "velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps", "velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec",
  "inertia/cg-x-in", "inertia/cg-y-in", "inertia/cg-z-in", "inertia/mass-slugs", "inertia/ixx-slugs_ft2", "inertia/iyy-slugs_ft2",
  "inertia/izz-slugs_ft2", "inertia/ixy-slugs_ft2", "inertia/ixz-slugs_ft2", "inertia/iyz-slugs_ft2"];
{
  const declared = /const PATHS = \[([\s\S]*?)\] as const;/.exec(readFileSync(path.join(source, "visibleMeshCollision.ts"), "utf8"));
  const listed = declared ? [...declared[1].matchAll(/"([^"]+)"/g)].map(match => match[1]) : [];
  expect(listed.join() === PATHS.join(), "visibleMeshCollision.ts reads other properties than the hull kernel's values");
}

/** createVisibleMeshCollision's update() on the WASM kernel: the flow README.md gives the host. */
function createWasmHullCollision(sdk, surface, options, stats) {
  const probes = options.bodyProbes, count = probes.length;
  const kernel = core.createHullCollision(count);
  probes.forEach((probe, i) => kernel.probes.set([probe.xIn, probe.yIn, probe.zIn, probe.insetIn], i * HULL.probeStride));
  const from = { x: 0, y: 0, z: 0 }, movement = { x: 0, y: 0, z: 0 };
  let hasPrevious = false;
  function read() {
    const values = kernel.values;
    try { for (let i = 0; i < PATHS.length; i++) values[i] = sdk.getPropertyValue(PATHS[i]); } catch { return false; }
    return true;
  }
  function settle() {
    hasPrevious = read() && kernel.place();
    if (hasPrevious) kernel.swap();
  }
  return {
    reset() { hasPrevious = false; },
    update() {
      if (!read() || !kernel.place()) { hasPrevious = false; return false; }
      const values = kernel.values;
      const support = surface.sample(values[0], values[1]);
      if (!hasPrevious || !support || values[2] * 0.3048 - support.heightMeters > geometry.MAX_BODY_COLLISION_ALTITUDE_METERS) {
        kernel.swap();
        hasPrevious = true;
        return false;
      }
      stats.swept++;
      const segments = kernel.segments, hits = kernel.hits;
      let any = false;
      for (let i = 0; i < count; i++) {
        const s = i * HULL.segmentStride, h = i * HULL.hitStride, distance = segments[s + 6];
        hits[h] = NaN;
        if (!Number.isFinite(distance) || distance < HULL.minSweepMeters) continue;
        from.x = segments[s]; from.y = segments[s + 1]; from.z = segments[s + 2];
        movement.x = segments[s + 3]; movement.y = segments[s + 4]; movement.z = segments[s + 5];
        const hit = surface.raycast(from, movement, distance);
        if (!hit) continue;
        hits[h] = hit.distanceMeters;
        hits[h + 1] = hit.point.x; hits[h + 2] = hit.point.y; hits[h + 3] = hit.point.z;
        hits[h + 4] = hit.normal.x; hits[h + 5] = hit.normal.y; hits[h + 6] = hit.normal.z;
        any = true;
      }
      if (!any) { kernel.swap(); return false; }
      const resolved = kernel.resolve(options.getRestitution?.() ?? NaN, options.friction ?? NaN);
      stats.hits += resolved;
      if (resolved === 0) { kernel.swap(); return false; }
      const output = kernel.output;
      const snapshot = flight.captureSimulation(sdk);
      snapshot.initial["ic/lat-geod-deg"] = output[1];
      snapshot.initial["ic/long-gc-deg"] = output[2];
      snapshot.initial["ic/h-sl-ft"] = output[3];
      snapshot.initial["ic/vn-fps"] = output[4];
      snapshot.initial["ic/ve-fps"] = output[5];
      snapshot.initial["ic/vd-fps"] = output[6];
      snapshot.initial["ic/p-rad_sec"] = output[7];
      snapshot.initial["ic/q-rad_sec"] = output[8];
      snapshot.initial["ic/r-rad_sec"] = output[9];
      flight.restoreSimulation(sdk, snapshot);
      settle();
      return true;
    },
    dispose() { kernel.dispose(); },
  };
}

/** Planes in ECEF, two-sided; each raycast also has a 10% chance of a malformed or arbitrary answer. */
function createHullSurface(key, planes, groundHeight, stats) {
  let calls = 0;
  const junk = (u, from, movement, distance) => {
    const k = 0.5, point = { x: from.x + movement.x * k, y: from.y + movement.y * k, z: from.z + movement.z * k };
    const n = planes[0].normal, normal = { x: n[0], y: n[1], z: n[2] };
    switch (Math.floor(u * 11)) {
      case 0: return null;
      case 1: return { distanceMeters: NaN, point, normal };
      case 2: return { distanceMeters: -0.01, point, normal };
      case 3: return { distanceMeters: distance * 1.5 + 0.01, point, normal };
      case 4: return { distanceMeters: distance * k, point, normal: { x: 0, y: 0, z: 0 } };
      case 5: return { distanceMeters: distance * k, point, normal: { x: NaN, y: 0, z: 1 } };
      case 6: return { distanceMeters: distance * k, point: { x: NaN, y: point.y, z: point.z }, normal };
      case 7: return { distanceMeters: distance * k, point, normal: { x: 1e-11, y: 0, z: 0 } };
      // Valid but arbitrary: the step's ends, and ties that must keep probe order.
      case 8: return { distanceMeters: distance, point, normal: { x: -normal.x, y: -normal.y, z: -normal.z } };
      case 9: return { distanceMeters: 0, point, normal };
      default: return { distanceMeters: distance * k, point, normal };
    }
  };
  return {
    sample() {
      return hashUnit(key, calls++) < 0.04 ? null : { heightMeters: groundHeight, revision: 1 };
    },
    raycast(from, movement, distance) {
      const u = hashUnit(key, calls++);
      if (u < 0.1) { stats.junk++; return junk(u / 0.1, from, movement, distance); }
      let best = Infinity, struck = null;
      for (const plane of planes) {
        const [nx, ny, nz] = plane.normal, [px, py, pz] = plane.point;
        const along = (movement.x * nx + movement.y * ny + movement.z * nz) / distance;
        if (!(Math.abs(along) > 1e-12)) continue;
        const t = ((px - from.x) * nx + (py - from.y) * ny + (pz - from.z) * nz) / along;
        if (t >= 0 && t <= distance && t < best) { best = t; struck = plane; }
      }
      if (!struck) return null;
      const k = best / distance, s = struck.scale;
      return { distanceMeters: best, point: { x: from.x + movement.x * k, y: from.y + movement.y * k, z: from.z + movement.z * k },
        normal: { x: struck.normal[0] * s, y: struck.normal[1] * s, z: struck.normal[2] * s } };
    },
  };
}

const PROBE_SETS = {
  C172: [geometry.C172_BODY_COLLISION_PROBES, [41, 0, 36.5]],
  SF50: [geometry.SF50_BODY_COLLISION_PROBES, [180, 0, -45]],
  F35B: [geometry.F35B_BODY_COLLISION_PROBES, [400, 0, -5]],
};
function hullScenario(sequence) {
  const set = pick(["C172", "C172", "SF50", "F35B", "random"]);
  let probes, cg;
  if (set === "random") {
    const n = pick([1, 2, 3, 8, 8, 64]);
    probes = Array.from({ length: n }, (_, i) => ({ name: `p${i}`, xIn: uniform(-100, 400), yIn: uniform(-250, 250), zIn: uniform(-60, 80),
      insetIn: pick([0, 0, 3, uniform(0, 10)]) }));
    cg = [uniform(50, 250), uniform(-3, 3), uniform(-20, 40)];
  } else [probes, cg] = PROBE_SETS[set];
  const ground = uniform(-30, 2500);
  const high = chance(0.05);
  const principal = [uniform(300, 60000), uniform(300, 60000), uniform(300, 60000)];
  const product = () => chance(0.5) ? uniform(-0.1, 0.1) * Math.sqrt(principal[0] * principal[2]) : 0;
  const singular = chance(0.03);
  const state = [chance(0.03) ? pick([89.99, -89.99]) : uniform(-70, 70), chance(0.03) ? pick([180, -180, 179.9999999]) : uniform(-180, 180),
    (ground + (high ? 200 : uniform(1, 4))) / 0.3048,
    chance(0.1) ? uniform(-180, 180) : uniform(-40, 40), chance(0.05) ? pick([89.5, -89.5]) : uniform(-25, 25), uniform(0, 360),
    uniform(-120, 120), uniform(-120, 120), uniform(-5, 40),
    ...(chance(0.3) ? [0, 0, 0] : [uniform(-0.6, 0.6), uniform(-0.6, 0.6), uniform(-0.6, 0.6)]),
    ...cg, chance(0.03) ? pick([0, -10, 1e-6]) : uniform(30, 1500),
    ...(singular ? [0, 0, 0, 0, 0, 0] : [...principal, product(), product(), product()])];
  const values = Object.fromEntries(PATHS.map((name, i) => [name, state[i]]));
  values["position/terrain-elevation-asl-ft"] = ground / 0.3048;
  values["fcs/throttle-cmd-norm"] = 0.5;
  // Planes just under the lowest probe, and sometimes a wall ahead.
  const scout = core.createHullCollision(probes.length);
  probes.forEach((probe, i) => scout.probes.set([probe.xIn, probe.yIn, probe.zIn, probe.insetIn], i * HULL.probeStride));
  scout.values.set(state);
  scout.place();
  const points = Array.from({ length: probes.length }, (_, i) => at3(scout.current, i * 3)), pose = scout.pose;
  const column = c => [pose[12 + c], pose[15 + c], pose[18 + c]];
  scout.dispose();
  const latRad = state[0] * Math.PI / 180, lonRad = state[1] * Math.PI / 180;
  const up = [Math.cos(latRad) * Math.cos(lonRad), Math.cos(latRad) * Math.sin(lonRad), Math.sin(latRad)], tilt = uniform(0, 35) * Math.PI / 180, side = unit(cross(up, randomUnit()));
  const facing = (normal, gap) => {
    const lowest = points.reduce((best, point) => dot(point, normal) < dot(best, normal) ? point : best);
    return { normal, point: sub(lowest, scale(normal, gap)), scale: pick([1, 1, -1, 2.5, -0.3, 1e4]) };
  };
  const planes = [facing(add(scale(up, Math.cos(tilt)), scale(side, Math.sin(tilt))), uniform(-0.05, 0.6))];
  if (chance(0.3)) planes.push(facing(unit(add(scale(column(0), -1), scale(randomUnit(), 0.3))), uniform(0, 2)));
  // getRestitution is a pure getter: the TypeScript calls it only for a step with hits.
  const requested = pick([undefined, 0.25, uniform(0, 1), 3, -1, NaN]);
  const restitution = requested === undefined ? undefined : () => requested;
  const friction = pick([undefined, 0.5, uniform(0, 1.5), -1, NaN, Infinity]);
  return { set, probes, values, planes, groundHeight: high ? ground : ground - uniform(-1, 5), options: { bodyProbes: probes, getRestitution: restitution, friction } };
}

/** One physics step of the mock flight: ballistic, the attitude turning at the body rates. */
function advance(sdk, dt) {
  const v = sdk.values, D = Math.PI / 180;
  const lat = v["position/lat-geod-deg"] * D, lon = v["position/long-gc-deg"] * D;
  const p = cameraMath.geodeticToEcef(lat, lon, v["position/h-sl-ft"] * 0.3048);
  const north = v["velocities/v-north-fps"] * 0.3048, east = v["velocities/v-east-fps"] * 0.3048, down = v["velocities/v-down-fps"] * 0.3048;
  const sinLat = Math.sin(lat), cosLat = Math.cos(lat), sinLon = Math.sin(lon), cosLon = Math.cos(lon);
  const vx = -sinLat * cosLon * north - sinLon * east - cosLat * cosLon * down;
  const vy = -sinLat * sinLon * north + cosLon * east - cosLat * sinLon * down;
  const vz = cosLat * north - sinLat * down;
  const next = cameraMath.ecefToGeodetic(p.x + vx * dt, p.y + vy * dt, p.z + vz * dt);
  v["position/lat-geod-deg"] = next.latRad / D; v["position/long-gc-deg"] = next.lonRad / D; v["position/h-sl-ft"] = next.altMeters / 0.3048;
  v["attitude/phi-deg"] += v["velocities/p-rad_sec"] * dt / D;
  v["attitude/theta-deg"] += v["velocities/q-rad_sec"] * dt / D;
  v["attitude/psi-deg"] += v["velocities/r-rad_sec"] * dt / D;
  v["velocities/v-down-fps"] += 32.174 * dt;
}

const hullBound = name => /lat-geod-deg|long-gc-deg/.test(name) ? 1e-11 : /h-sl-ft/.test(name) ? 1e-5 : 0;
const hullStats = {};
for (const [modeName, mode] of Object.entries(modes)) {
  seed = 0x5f50c172;
  const tally = createTally(), wiredTally = createTally(), wiredFallbacks = [];
  const stats = { sequences: 0, steps: 0, swept: 0, restored: 0, hits: 0, junk: 0, notFinite: 0, resets: 0, threw: 0, sets: {} };
  for (let sequence = 0; sequence < 300; sequence++) {
    const scenario = hullScenario(sequence);
    stats.sets[scenario.set] = (stats.sets[scenario.set] ?? 0) + 1;
    const sdkTs = createMockSdk({ ...scenario.values }), sdkWasm = createMockSdk({ ...scenario.values }), sdkWired = createMockSdk({ ...scenario.values });
    const key = 0x4e11 + sequence;
    const ts = mode.collision.createVisibleMeshCollision(sdkTs, createHullSurface(key, scenario.planes, scenario.groundHeight, { junk: 0 }), scenario.options);
    const wasm = createWasmHullCollision(sdkWasm, createHullSurface(key, scenario.planes, scenario.groundHeight, stats), scenario.options, stats);
    const wired = mode.collision.createVisibleMeshCollision(sdkWired, createHullSurface(key, scenario.planes, scenario.groundHeight, { junk: 0 }), { ...scenario.options, core });
    for (let step = 0; step < 10; step++) {
      const label = `hull ${modeName} sequence ${sequence} (${scenario.set}) step ${step}`;
      if (chance(0.03)) { ts.reset(); wasm.reset(); wired.reset(); stats.resets++; }
      let broken = null;
      if (chance(0.03)) {
        const name = pick(PATHS);
        broken = [name, sdkTs.values[name]];
        sdkTs.values[broken[0]] = sdkWasm.values[broken[0]] = sdkWired.values[broken[0]] = pick([NaN, Infinity, -Infinity]);
        stats.notFinite++;
      }
      const expected = attempt(() => ts.update()), actual = attempt(() => wasm.update()), throughCore = attempt(() => wired.update());
      expect(expected === actual, `${label}: TypeScript update ${expected} vs WASM ${actual}`);
      expect(expected === throughCore, `${label}: TypeScript update ${expected} vs its WASM path ${throughCore}`);
      stats.restored += actual === true ? 1 : 0;
      stats.threw += typeof expected === "string" ? 1 : 0;
      if (broken) sdkTs.values[broken[0]] = sdkWasm.values[broken[0]] = sdkWired.values[broken[0]] = broken[1];
      for (const name of new Set([...Object.keys(sdkTs.values), ...Object.keys(sdkWasm.values), ...Object.keys(sdkWired.values)])) {
        const value = sdkTs.values[name], bound = hullBound(name) || 1e-9 * Math.max(1, Math.abs(value));
        agree(tally, modeName, `${label} ${name}`, value, sdkWasm.values[name], bound);
        agree(wiredTally, modeName, `${label} ${name} (its WASM path)`, value, sdkWired.values[name], bound);
      }
      // With V8's trig, each step starts from the TypeScript's state, so the bound is per step.
      if (modeName === "v8") { Object.assign(sdkWasm.values, sdkTs.values); Object.assign(sdkWired.values, sdkTs.values); }
      advance(sdkTs, 1 / 60);
      advance(sdkWasm, 1 / 60);
      advance(sdkWired, 1 / 60);
      stats.steps++;
    }
    const status = wired.status();
    if (status.kernel !== "single") wiredFallbacks.push(status.fallbackReason);
    ts.dispose();
    wasm.dispose();
    wired.dispose();
    stats.sequences++;
  }
  expect(!wiredFallbacks.length, `hull ${modeName}: the TypeScript's WASM path fell back ${wiredFallbacks.length} times: ${wiredFallbacks[0]}`);
  hullStats[modeName] = { trig: mode.trig, values: tally.values, exact: tally.exact, maxDifference: tally.maxAbsolute, worst: tally.worst,
    wasmPath: { values: wiredTally.values, exact: wiredTally.exact, maxDifference: wiredTally.maxAbsolute, worst: wiredTally.worst, fallbacks: wiredFallbacks.length }, ...stats };
}

// 4. createContactSupport against the support kernel in the host flow.
/** createContactSupport's measure() on the WASM kernel: the flow README.md gives the host. */
function createWasmSupport(sdk, surface, stats) {
  let kernel = null, units = [], unavailable = false;
  const result = { elevation: NaN, refinement: 0 };
  const sample = (latDeg, lonDeg) => surface.sample(latDeg, lonDeg);
  function discover() {
    if (kernel?.discovered) return true;
    if (unavailable) return false;
    let found;
    try { found = clearance.groundContactUnits(sdk); } catch { unavailable = true; return false; }
    if (found.length === 0) { unavailable = true; return false; }
    const flags = found.flatMap(unit => [unit.retractable ? 1 : 0, unit.prefix.startsWith("gear/") ? 1 : 0]);
    if (!kernel || flags.length !== kernel.units.length || flags.some((flag, i) => flag !== kernel.units[i])) {
      // Other units: the step counters carry over, as the TypeScript's do.
      const next = core.createContactSupport(found, kernel ?? undefined);
      kernel?.dispose();
      kernel = next;
      stats.kernels++;
    }
    units = found;
    return true;
  }
  function fill(input) {
    kernel.input.set([input.lat, input.lon, input.altitude, input.roll, input.pitch, input.heading, input.cgGround, input.revision,
      input.revisionChanged ? 1 : 0, input.cgDelta, input.urgent ? 1 : 0, input.snap ? 1 : 0, input.samples, input.previousElevation]);
  }
  function read() {
    const values = kernel.values;
    values[0] = sdk.getPropertyValue("gear/num-units");
    values[1] = sdk.getPropertyValue("gear/gear-pos-norm");
    values[2] = sdk.getPropertyValue("inertia/cg-x-in");
    values[3] = sdk.getPropertyValue("inertia/cg-y-in");
    values[4] = sdk.getPropertyValue("inertia/cg-z-in");
    units.forEach((unit, i) => {
      values[5 + i * 3] = sdk.getPropertyValue(`${unit.prefix}x-position`);
      values[6 + i * 3] = sdk.getPropertyValue(`${unit.prefix}y-position`);
      values[7 + i * 3] = sdk.getPropertyValue(`${unit.prefix}z-position`);
    });
  }
  return {
    measure(input) {
      if (kernel) fill(input);
      let status = SUPPORT.GEOMETRY_READ;
      // Until the units are discovered their reach is unknown, so measure reads, discovering them first.
      if (!kernel || kernel.wantsGeometry()) {
        if (!discover()) {
          if (!kernel) {
            result.elevation = NaN;
            result.refinement = input.revisionChanged && Number.isFinite(input.cgDelta) ? input.cgDelta : 0;
            stats.withoutKernel++;
            return result;
          }
          status = SUPPORT.GEOMETRY_UNAVAILABLE;
        } else {
          fill(input);
          try { read(); } catch { status = SUPPORT.GEOMETRY_FAILED; }
        }
      } else stats.far++;
      const output = kernel.measure(status, sample);
      for (let k = 0; k < output[4]; k++) stats.kinds[KINDS[kernel.requests[k * SUPPORT.requestStride + 1]]]++;
      stats.inRange += output[3];
      stats.requests += output[4];
      stats.applied += output[5];
      stats[["read", "failed", "unavailable"][status]]++;
      result.elevation = output[0];
      result.refinement = output[1];
      return result;
    },
    reset() { kernel?.reset(); unavailable = false; },
    dispose() { kernel?.dispose(); kernel = null; },
  };
}

const KINDS = { 4: "first", 3: "urgent", 2: "moved", 1: "lazy" };
function supportModel() {
  const kind = pick(["C172", "C172", "retractable", "random", "random", "none", "wide"]);
  let units, cg;
  if (kind === "C172") {
    units = geometry.C172_GROUND_CONTACTS.map(c => ({ kind: c.kind === "wheel" ? "gear" : "contact", retractable: false, xIn: c.xIn, yIn: c.yIn, zIn: c.zIn }));
    cg = [uniform(39, 44), uniform(-0.5, 0.5), uniform(34, 38)];
  } else if (kind === "retractable") {
    units = [["gear", true, 70, 0, -95], ["gear", true, 200, -60, -98], ["gear", true, 200, 60, -98], ["contact", false, 175, 0, -60],
      ["contact", false, 358, 0, -38], ["contact", false, 174, -226, -36], ["contact", false, 174, 226, -36]]
      .map(([kind, retractable, xIn, yIn, zIn]) => ({ kind, retractable, xIn, yIn, zIn }));
    cg = [uniform(185, 195), 0, uniform(-45, -35)];
  } else if (kind === "none") {
    units = [];
    cg = [41, 0, 36.5];
  } else {
    const n = kind === "wide" ? 64 : Math.floor(uniform(1, 13));
    units = Array.from({ length: n }, () => {
      const gear = chance(0.5);
      return { kind: gear ? "gear" : "contact", retractable: gear && chance(0.3), xIn: uniform(-50, 300), yIn: uniform(-250, 250), zIn: uniform(-120, 80) };
    });
    cg = [uniform(50, 200), uniform(-2, 2), uniform(-30, 40)];
  }
  return { kind, units, cg };
}
function catalogFor(units) {
  return units.map((unit, i) => {
    unit.prefix = `${unit.kind}/unit${i ? `[${i}]` : ""}/`;
    return ["x-position (RW)", "y-position (RW)", "z-position (RW)", ...(unit.retractable ? ["pos-norm (R)"] : [])].map(line => unit.prefix + line).join("\n");
  }).join("\n");
}
function modelValues(model) {
  const values = { "gear/num-units": model.units.length, "gear/gear-pos-norm": 1, "inertia/cg-x-in": model.cg[0], "inertia/cg-y-in": model.cg[1], "inertia/cg-z-in": model.cg[2] };
  for (const unit of model.units) Object.assign(values, { [`${unit.prefix}x-position`]: unit.xIn, [`${unit.prefix}y-position`]: unit.yIn, [`${unit.prefix}z-position`]: unit.zIn });
  return values;
}

function createWorld(lat0, lon0) {
  return { lat0, lon0, cosLat0: Math.cos(lat0 * Math.PI / 180), base: uniform(-20, 1500), gn: uniform(-0.08, 0.08), ge: uniform(-0.08, 0.08),
    offset: 0, tilt: 0, revision: 1,
    roof: chance(0.6) ? { north: uniform(-10, 10), east: uniform(-10, 10), half: uniform(1.5, 5), height: uniform(0.5, 4) } : null };
}
function heightAt(world, lat, lon) {
  const north = (lat - world.lat0) * 111320, east = (((lon - world.lon0 + 540) % 360) - 180) * 111320 * world.cosLat0;
  const roof = world.roof && Math.abs(north - world.roof.north) < world.roof.half && Math.abs(east - world.roof.east) < world.roof.half ? world.roof.height : 0;
  return world.base + world.gn * north + world.ge * east + world.offset + world.tilt * north + roof;
}
/** The world's height at the asked spot, with holes and NaN heights by call index. */
function createSupportSurface(world, key, calls) {
  let index = 0;
  return {
    sample(lat, lon) {
      calls.push(lat, lon);
      const u = hashUnit(key, index++);
      if (u < 0.05) return null;
      return { heightMeters: u < 0.07 ? NaN : heightAt(world, lat, lon), revision: world.revision };
    },
    raycast() { return null; },
  };
}

const supportStats = {};
for (const [modeName, mode] of Object.entries(modes)) {
  seed = 0x5a9907f1;
  const tally = createTally(), samples = createTally(), wiredTally = createTally(), wiredFallbacks = [];
  const stats = { sequences: 0, steps: 0, models: {}, kernels: 0, read: 0, failed: 0, unavailable: 0, far: 0, withoutKernel: 0, inRange: 0, requests: 0,
    applied: 0, kinds: { first: 0, urgent: 0, moved: 0, lazy: 0 }, resets: 0, readFailures: 0, countChanges: 0, unitsAdded: 0, gearMoves: 0, revisions: 0, threw: 0 };
  for (let sequence = 0; sequence < 160; sequence++) {
    const model = supportModel();
    stats.models[model.kind] = (stats.models[model.kind] ?? 0) + 1;
    const catalog = catalogFor(model.units);
    const sdkTs = createMockSdk(modelValues(model), catalog), sdkWasm = createMockSdk(modelValues(model), catalog), sdkWired = createMockSdk(modelValues(model), catalog);
    let lat = chance(0.05) ? pick([89.9, -89.9]) : uniform(-70, 70), lon = chance(0.05) ? 179.99995 : uniform(-180, 180);
    const world = createWorld(lat, lon);
    const callsTs = [], callsWasm = [], callsWired = [], key = 0x5e0 + sequence;
    const ts = mode.terrain.createContactSupport(sdkTs, createSupportSurface(world, key, callsTs));
    const wasm = createWasmSupport(sdkWasm, createSupportSurface(world, key, callsWasm), stats);
    const wired = mode.terrain.createContactSupport(sdkWired, createSupportSurface(world, key, callsWired), core);
    let heading = uniform(0, 2 * Math.PI), speed = pick([0, uniform(0, 5), uniform(5, 40)]), roll = uniform(-0.05, 0.05), pitch = uniform(-0.05, 0.12);
    let previousElevation = NaN, restoreCount = null;
    const lowestBelow = () => model.units.reduce((lowest, unit) => Math.max(lowest, (model.cg[2] - unit.zIn) * 0.0254), 0);
    for (let step = 0; step < 60; step++) {
      const label = `support ${modeName} sequence ${sequence} (${model.kind}) step ${step}`;
      // The model.
      const both = (name, value) => { sdkTs.values[name] = value; sdkWasm.values[name] = value; sdkWired.values[name] = value; };
      if (restoreCount !== null) { both("gear/num-units", restoreCount); restoreCount = null; }
      if (chance(0.02)) { ts.reset(); wasm.reset(); wired.reset(); stats.resets++; }
      sdkTs.failing = sdkWasm.failing = sdkWired.failing = chance(0.02) ? "inertia/cg-x-in" : null;
      stats.readFailures += sdkTs.failing ? 1 : 0;
      if (model.units.length && chance(0.012)) {
        restoreCount = model.units.length;
        both("gear/num-units", chance(0.5) ? model.units.length + 1 : 2.5);
        stats.countChanges++;
      }
      if (model.units.length && model.units.length < 64 && restoreCount === null && chance(0.004)) {
        const gear = chance(0.5);
        model.units.push({ kind: gear ? "gear" : "contact", retractable: gear && chance(0.5), xIn: uniform(-50, 300), yIn: uniform(-250, 250), zIn: uniform(-120, 80) });
        sdkTs.catalog = sdkWasm.catalog = sdkWired.catalog = catalogFor(model.units);
        for (const [name, value] of Object.entries(modelValues(model))) if (name !== "gear/gear-pos-norm") both(name, value);
        stats.unitsAdded++;
      }
      if (chance(0.04)) { both("gear/gear-pos-norm", pick([1, 0, 0.5, 0.995])); stats.gearMoves++; }
      // The flight and the world.
      if (chance(0.01)) { lat += uniform(-0.001, 0.001); lon += uniform(-0.001, 0.001); }
      const travelled = speed / 60;
      lat += travelled * Math.cos(heading) / 111320;
      lon += travelled * Math.sin(heading) / (111320 * Math.max(Math.cos(lat * Math.PI / 180), 1e-3));
      lon = ((lon + 540) % 360) - 180;
      heading += uniform(-0.02, 0.02); roll = Math.max(-0.3, Math.min(0.3, roll + uniform(-0.02, 0.02))); pitch = Math.max(-0.2, Math.min(0.25, pitch + uniform(-0.01, 0.01)));
      let revisionChanged = false, cgDelta = 0;
      if (chance(0.15)) {
        const before = heightAt(world, lat, lon);
        world.revision++; world.offset += uniform(-0.3, 0.3); world.tilt += uniform(-0.003, 0.003);
        revisionChanged = true;
        cgDelta = chance(0.25) ? NaN : heightAt(world, lat, lon) - before;
        stats.revisions++;
      }
      const cgGround = heightAt(world, lat, lon), height = chance(0.85) ? uniform(-0.15, 0.6) : pick([uniform(3, 9), uniform(10, 14), uniform(20, 200)]);
      const input = { lat, lon, altitude: cgGround + lowestBelow() + height, roll, pitch, heading, cgGround, revision: world.revision, revisionChanged,
        cgDelta: revisionChanged ? cgDelta : chance(0.5) ? 0 : NaN, urgent: chance(0.5), snap: chance(0.05),
        samples: pick([2, 2, 2, 2, Infinity, Infinity, 1, 0, 3, 1.5, NaN]), previousElevation: chance(0.1) ? NaN : previousElevation };
      callsTs.length = callsWasm.length = callsWired.length = 0;
      const expected = attempt(() => ({ ...ts.measure(input) })), actual = attempt(() => ({ ...wasm.measure(input) }));
      const throughCore = attempt(() => ({ ...wired.measure(input) }));
      for (const [name, result, into] of [["WASM", actual, tally], ["its WASM path", throughCore, wiredTally]]) {
        if (typeof expected === "string" || typeof result === "string") {
          expect(expected === result, `${label}: TypeScript ${JSON.stringify(expected)} vs ${name} ${JSON.stringify(result)}`);
        } else {
          agree(into, modeName, `${label} elevation (${name})`, expected.elevation, result.elevation, 1e-9);
          agree(into, modeName, `${label} refinement (${name})`, expected.refinement, result.refinement, 1e-9);
        }
      }
      if (typeof expected === "string") stats.threw++;
      else previousElevation = Number.isFinite(expected.elevation) ? expected.elevation : cgGround;
      for (const [name, calls] of [["WASM", callsWasm], ["its WASM path", callsWired]]) {
        if (expect(callsTs.length === calls.length, `${label}: TypeScript sampled ${callsTs.length / 2} spots, ${name} ${calls.length / 2}`)) {
          callsTs.forEach((value, i) => agree(samples, modeName, `${label} sample ${i >> 1} ${i & 1 ? "lon" : "lat"} (${name})`, value, calls[i], 1e-12));
        }
      }
      stats.steps++;
    }
    const status = wired.status();
    if (status.kernel !== "single") wiredFallbacks.push(status.fallbackReason);
    ts.dispose();
    wasm.dispose();
    wired.dispose();
    stats.sequences++;
  }
  expect(!wiredFallbacks.length, `support ${modeName}: the TypeScript's WASM path fell back ${wiredFallbacks.length} times: ${wiredFallbacks[0]}`);
  supportStats[modeName] = { trig: mode.trig, values: tally.values, exact: tally.exact, maxDifference: tally.maxAbsolute, worst: tally.worst,
    wasmPath: { values: wiredTally.values, exact: wiredTally.exact, maxDifference: wiredTally.maxAbsolute, worst: wiredTally.worst, fallbacks: wiredFallbacks.length },
    sampleCoordinates: { values: samples.values, exact: samples.exact, maxDifference: samples.maxAbsolute }, ...stats };
}

// 5. Zero heap allocations over many full steps, and stable views.
const C172_UNITS = geometry.C172_GROUND_CONTACTS.map(c => ({ retractable: false, wheel: c.kind === "wheel" }));
const C172_PROBES = geometry.C172_BODY_COLLISION_PROBES.flatMap(p => [p.xIn, p.yIn, p.zIn, p.insetIn]);
const HULL_VALUES = [44.88, -93.22, 1000, 2, 3, 45, 80, 10, 5, 0.01, 0.02, 0.03, 41, 0, 36.5, 75, 948, 1346, 1967, 0, 0, 0];
const step = { hull: core.createHullCollision(5), support: core.createContactSupport(C172_UNITS), wheels: core.createWheelSolver(3), spin: core.createWheelSpin(3) };
{
  const hull = step.hull;
  hull.probes.set(C172_PROBES);
  hull.values.set(HULL_VALUES);
  hull.place(); hull.swap();
  hull.values[2] -= 1;
  hull.place();
  // One hit midway along each sweep, against the motion.
  for (let i = 0; i < 5; i++) {
    const s = hull.segments.subarray(i * HULL.segmentStride, (i + 1) * HULL.segmentStride);
    hull.hits.set([s[6] * 0.5, s[0] + s[3] * 0.5, s[1] + s[4] * 0.5, s[2] + s[5] * 0.5, -s[3], -s[4], -s[5]], i * HULL.hitStride);
  }
  step.support.input.set([44.88, -93.22, 301.4, 0.02, 0.03, 1, 300, 1, 0, 0, 0, 0, 2, NaN]);
  step.support.values.set([9, 1, 41, 0, 36.5, ...geometry.C172_GROUND_CONTACTS.flatMap(c => [c.xIn, c.yIn, c.zIn])]);
  step.wheels.body.set([1100, 1 / 1285, 0, 0, 0, 1 / 1825, 0, 0, 0, 1 / 2667, 20, 0, -0.5, 0, 0.01, 0]);
  [1, 2, 3].forEach((id, i) => step.wheels.wheels.set([id, 0, -1, 0, 0, 0.124, 0.5, 450, 0.2, 0.3], i * WHEELS.wheelStride));
  [1, 2, 3].forEach((id, i) => step.wheels.contacts.set([id, 1, i - 1, 1.4, 0, 0, -1, 1, 0, 0, 0, 1, 0, 20, 0, 0, 0, 0, 0, 3500, 0.05, 0, 0.22, 1, 1], i * WHEELS.contactStride));
  spinTs.WHEEL_SPIN_CONFIGS.forEach((c, i) => step.spin.wheels.set([0, 0, 0, 0, 0, 0, 0, 0, 0, c.radiusMeters, c.widthMeters, c.inertiaKgMetersSquared, c.frictionCoefficient, c.maxBrakeTorqueNewtonMeters, 1, 20, 3000, 0.05, 0, 0.2], i * SPIN.wheelStride));
}
const answer = { heightMeters: 300, revision: 1 }, ground = () => answer;
const views = [step.hull.output, step.hull.segments, step.support.state, step.support.requests, step.wheels.body, step.spin.wheels];
expect(step.hull.resolve() === 5 && step.support.measure(SUPPORT.GEOMETRY_READ, ground)[4] > 0, "the allocation steps resolve no hits or sample nothing");
const allocationsBefore = core.heapAllocations();
for (let i = 0; i < 20_000; i++) {
  step.hull.place();
  step.hull.resolve();
  step.support.measure(SUPPORT.GEOMETRY_READ, ground);
  step.wheels.run(1 / 120, 3);
  step.spin.run(1 / 120, SPIN.INERTIA);
  // Rejected calls must not allocate either.
  raw.gc_hull_place(1, 22, 2, 20, 3, 15, 4, 21, 5, 15, 6, 34);
}
const allocationsAfter = core.heapAllocations();
expect(allocationsBefore > 0, "the allocation counter did not count the buffers");
expect(allocationsAfter === allocationsBefore, `${allocationsAfter - allocationsBefore} heap allocations while stepping`);
expect([step.hull.output, step.hull.segments, step.support.state, step.support.requests, step.wheels.body, step.spin.wheels].every((view, i) => view === views[i]),
  "a view was recreated while stepping");

// 6. Error codes from the raw ABI, and the wrapper's guards.
{
  const alloc = values => { const pointer = raw.gc_alloc(values.length); new Float64Array(raw.memory.buffer, pointer, values.length).set(values); return [pointer, values.length]; };
  const write = ([pointer, length], index, value) => { new Float64Array(raw.memory.buffer, pointer, length)[index] = value; };
  const status = (expected, actual, what) => expect(actual === expected && raw.gc_last_error() === expected, `${what}: status ${actual}, last error ${raw.gc_last_error()}, expected ${expected}`);
  const zeros = length => new Array(length).fill(0);
  // The hull.
  const values = alloc(HULL_VALUES), probes = alloc(C172_PROBES), previous = alloc(zeros(15)), pose = alloc(zeros(21)), current = alloc(zeros(15));
  const segments = alloc(zeros(35)), hits = alloc(new Array(35).fill(NaN)), hullOut = alloc(zeros(HULL.headLength + 5 * HULL.rowStride));
  status(0, raw.gc_hull_place(...values, ...probes, ...previous, ...pose, ...current, ...segments), "valid hull place");
  status(ERRORS.INVALID, raw.gc_hull_place(...values, ...probes, ...previous, ...pose, ...current, segments[0], 34), "short segments");
  status(ERRORS.INVALID, raw.gc_hull_place(...values, ...probes, ...previous, ...pose, ...previous, ...segments), "current aliasing previous");
  status(ERRORS.INVALID, raw.gc_hull_place(values[0] + 8, values[1], ...probes, ...previous, ...pose, ...current, ...segments), "foreign pointer");
  status(ERRORS.INVALID, raw.gc_hull_place(...values, probes[0], 19, ...previous, ...pose, ...current, ...segments), "probes not in rows of 4");
  status(ERRORS.INVALID, raw.gc_hull_place(...values, probes[0], 65 * 4, ...previous, ...pose, ...current, ...segments), "65 probes");
  write(values, 15, NaN);
  status(ERRORS.INPUT, raw.gc_hull_place(...values, ...probes, ...previous, ...pose, ...current, ...segments), "NaN mass");
  write(values, 15, 75);
  const resolve = output => raw.gc_hull_resolve(...values, ...probes, ...pose, ...previous, ...current, ...hits, NaN, NaN, ...output);
  status(0, resolve(hullOut), "valid hull resolve");
  status(ERRORS.INVALID, resolve([hullOut[0], hullOut[1] - 1]), "short hull output");
  status(ERRORS.INVALID, resolve(hits), "the hits passed as the hull output");
  // The support.
  const units = alloc([0, 1, 0, 1, 0, 0]), state = alloc(zeros(SUPPORT.globals + 3 * (SUPPORT.contactStride + 4)));
  const input = alloc([44.88, -93.22, 301.4, 0, 0, 1, 300, 1, 0, 0, 0, 0, 2, NaN]), supportValues = alloc([3, 1, 41, 0, 36.5, -6.8, 0, -19.5, 58.2, -43, -15.5, 42, 0, 11.9]);
  const requests = alloc(zeros(2 * 3 * SUPPORT.requestStride)), supportOut = alloc(zeros(SUPPORT.outputLength));
  const plan = (geometryStatus = 0) => raw.gc_support_plan(...units, ...state, ...input, ...supportValues, geometryStatus, ...requests);
  const finish = () => raw.gc_support_finish(...units, ...state, ...requests, ...supportOut);
  status(0, raw.gc_support_reset(...units, ...state), "valid support reset");
  status(ERRORS.INVALID, raw.gc_support_reset(units[0], units[1], state[0], state[1] - 1), "short support state");
  status(ERRORS.INVALID, finish(), "finish before plan");
  status(ERRORS.INVALID, plan(3), "geometry status 3");
  write(input, 10, 0.5);
  status(ERRORS.INPUT, plan(), "urgent 0.5");
  write(input, 10, 0);
  write(units, 1, 2);
  status(ERRORS.INPUT, plan(), "unit flag 2");
  write(units, 1, 1);
  status(0, plan(), "valid plan");
  status(ERRORS.INVALID, plan(), "plan twice");
  status(0, finish(), "valid finish");
  status(ERRORS.INVALID, finish(), "finish twice");
  // Wheels and spin.
  const body = alloc([1100, 1 / 1285, 0, 0, 0, 1 / 1825, 0, 0, 0, 1 / 2667, 20, 0, 0, 0, 0, 0]), shortContacts = alloc(zeros(8));
  const wheelRow = alloc([1, 0, -1, 0, 0, 0.124, 0.5, 450, 0, 0]), wheelOut = alloc(zeros(WHEELS.reportLength + WHEELS.resultStride));
  status(0, raw.gc_solve_wheels(1 / 120, NaN, 1, ...body, ...wheelRow, ...shortContacts, 0, ...wheelOut), "valid wheels");
  status(ERRORS.INVALID, raw.gc_solve_wheels(1 / 120, NaN, 2, ...body, ...wheelRow, ...shortContacts, 0, ...wheelOut), "warm start 2");
  status(ERRORS.INVALID, raw.gc_solve_wheels(1 / 120, NaN, 1, ...body, ...wheelRow, ...shortContacts, 1, ...wheelOut), "contact rows past the buffer");
  status(ERRORS.INVALID, raw.gc_solve_wheels(1 / 120, NaN, 1, ...body, ...wheelRow, ...shortContacts, 0, ...body), "body passed as the output");
  const spinRow = alloc([0, 0, 0, 0, 0, 0, 0, 0, 0, 0.22, 0.152, 0.124, 0.5, 450, 1, 20, 3000, 0, 0, 0]);
  status(0, raw.gc_step_wheel_spin(1 / 120, 1, ...spinRow), "valid spin");
  status(ERRORS.INVALID, raw.gc_step_wheel_spin(1 / 120, 2, ...spinRow), "spin mode 2");
  write(spinRow, 14, 0.5);
  status(ERRORS.INPUT, raw.gc_step_wheel_spin(1 / 120, 1, ...spinRow), "onGround 0.5");
  status(ERRORS.ALLOCATION, raw.gc_alloc(0) === 0 ? raw.gc_last_error() : -1, "zero-length allocation");
  status(ERRORS.ALLOCATION, raw.gc_alloc(3_000_000) === 0 ? raw.gc_last_error() : -1, "allocation past 16 MiB");
  for (const buffer of [values, probes, previous, pose, current, segments, hits, hullOut, units, state, input, supportValues, requests, supportOut,
    body, shortContacts, wheelRow, wheelOut, spinRow]) status(0, raw.gc_free(...buffer), "free");
  status(ERRORS.INVALID, raw.gc_free(...values), "double free");
  // The wrapper.
  const throws = (run, what, code) => {
    try { run(); expect(false, `${what} did not throw`); } catch (error) { expect(code === undefined || error.status === code, `${what} threw ${error.message}`); }
  };
  throws(() => core.createHullCollision(0), "no hull probes");
  throws(() => core.createHullCollision(65), "65 hull probes");
  throws(() => core.createContactSupport([]), "no support units");
  throws(() => core.createContactSupport(new Array(65).fill({ retractable: false })), "65 support units");
  throws(() => step.wheels.run(1 / 120, 4), "a wheel contact count past capacity");
  const support = core.createContactSupport(C172_UNITS);
  support.input.set([44.88, -93.22, 301.4, 0.02, 0.03, 1, 300, 1, 0, 0, 0, 0, 2, NaN]);
  support.values.set([9, 1, 41, 0, 36.5, ...geometry.C172_GROUND_CONTACTS.flatMap(c => [c.xIn, c.yIn, c.zIn])]);
  expect(support.wantsGeometry(), "a grounded aircraft must read its contacts");
  expect(support.plan(SUPPORT.GEOMETRY_READ) > 0 && support.discovered, "the first plan neither discovered the contacts nor sampled them");
  throws(() => support.plan(SUPPORT.GEOMETRY_READ), "plan twice", ERRORS.INVALID);
  support.finish();
  throws(() => support.finish(), "finish twice", ERRORS.INVALID);
  support.reset();
  throws(() => support.measure(SUPPORT.GEOMETRY_READ, () => { throw new Error("surface lost"); }), "a sampler that throws");
  expect(!support.discovered, "a throwing sampler left the contacts discovered");
  try { support.measure(SUPPORT.GEOMETRY_READ, ground); } catch (error) { expect(false, `support after a throwing sampler threw ${error.message}`); }
  const replacement = core.createContactSupport([...C172_UNITS, { retractable: true, wheel: true }], support);
  expect(replacement.state[SUPPORT.state.STEPS] === support.state[SUPPORT.state.STEPS] && replacement.state[SUPPORT.state.STEPS] > 0
    && !replacement.discovered, "a replacement kernel did not carry the step counters or start undiscovered");
  replacement.dispose();
  support.dispose();
  throws(() => support.plan(SUPPORT.GEOMETRY_READ), "a disposed kernel");
  const hull = core.createHullCollision(5);
  hull.probes.set(C172_PROBES);
  hull.values.set(HULL_VALUES);
  hull.values[0] = NaN;
  expect(hull.place() === false, "hull place with a NaN latitude did not return false");
  hull.values[0] = 44.88;
  expect(hull.place() === true, "hull place with valid values did not return true");
  hull.dispose();
}
// How often V8's trig and the core's libm differ, which is why the V8 runs need a bound.
seed = 0x7121;
const trig = { inputs: 100_000, sin: 0, cos: 0, atan2: 0 };
for (let i = 0; i < trig.inputs; i++) {
  const x = uniform(-7, 7), y = uniform(-7, 7);
  trig.sin += Object.is(Math.sin(x), core.wasmMath.sin(x)) ? 0 : 1;
  trig.cos += Object.is(Math.cos(x), core.wasmMath.cos(x)) ? 0 : 1;
  trig.atan2 += Object.is(Math.atan2(y, x), core.wasmMath.atan2(y, x)) ? 0 : 1;
}

for (const kernel of Object.values(step)) kernel.dispose();
core.dispose();

const summary = {
  wasm: { file: wasmFile, sha256: metadata.outputs[wasmFile].sha256, bytes: metadata.outputs[wasmFile].bytes, node: process.versions.node },
  bitForBit: {
    coupledRigidWheels: { values: compared.wheels, ...wheelCoverage },
    wheelSpin: { values: compared.spin, ...spinCoverage },
  },
  trigDiffersFromV8: trig,
  wasmTrigSubstitutions: substituted,
  visibleMeshCollision: hullStats,
  contactSupport: supportStats,
  allocations: { counted: allocationsBefore, whileStepping: allocationsAfter - allocationsBefore, steps: 20_000 },
  failures: failureCount,
};
console.log(JSON.stringify(summary, null, 2));
if (failureCount) {
  for (const failure of failures) console.error(`FAIL ${failure}`);
  process.exitCode = 1;
}
