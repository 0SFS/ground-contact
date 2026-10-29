import { readFileSync } from "node:fs";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  createCoupledWheelSolver, createCoupledWheelState, createWheelForceResult, solveCoupledRigidWheels,
  type CoupledSolveOptions, type CoupledWheelParams, type CoupledWheelState, type RigidBodyVelocityState,
} from "../src/coupledRigidWheels";
import { createWheelSpinExperiment } from "../src/createWheelSpinExperiment";
import type { GroundContactCore } from "../src/groundContactCore";
import type { Vec3, WheelContact, WheelForceResult } from "../src/wheelContact";
import {
  createWheelSpinState, createWheelSpinStepper, stepWheelSpin, WHEEL_SPIN_CONFIGS,
  type WheelSpinConfig, type WheelSpinInput, type WheelSpinMode, type WheelSpinState,
} from "../src/wheelSpin";
import { instantiateGroundContact } from "../wasm/index.js";

const freshCore = () => instantiateGroundContact(readFileSync(new URL("../wasm/single/ground_contact.wasm", import.meta.url)));
// A core per test: one holds a bounded number of live kernels.
let core: Awaited<ReturnType<typeof freshCore>>;
beforeEach(async () => { core = await freshCore(); });
afterEach(() => core.dispose());

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

/** Every number in `actual` is the same value as in `expected`, -0 and NaN included. */
function same(actual: unknown, expected: unknown, path = "value"): void {
  if (typeof expected === "number") {
    if (!Object.is(actual, expected)) expect.fail(`${path}: ${String(actual)} is not ${String(expected)}`);
    return;
  }
  if (expected && typeof expected === "object") {
    expect(Object.keys(actual as object).sort(), path).toEqual(Object.keys(expected).sort());
    for (const key of Object.keys(expected)) same((actual as Record<string, unknown>)[key], (expected as Record<string, unknown>)[key], `${path}.${key}`);
    return;
  }
  expect(actual, path).toBe(expected);
}

const NAMES = ["NOSE", "LEFT_MAIN", "RIGHT_MAIN"] as const;

/** A unit frame turned by `yaw` and tilted by `tilt`, normal first. */
function frame(yaw: number, tilt: number) {
  const normal = { x: Math.sin(tilt), y: 0, z: Math.cos(tilt) };
  const forward0 = { x: Math.cos(tilt), y: 0, z: -Math.sin(tilt) };
  const lateral0 = { x: 0, y: 1, z: 0 };
  const turn = (v: Vec3, n: Vec3): Vec3 => {
    // Rodrigues about the normal.
    const c = Math.cos(yaw), s = Math.sin(yaw), d = v.x * n.x + v.y * n.y + v.z * n.z;
    const cx = n.y * v.z - n.z * v.y, cy = n.z * v.x - n.x * v.z, cz = n.x * v.y - n.y * v.x;
    return { x: v.x * c + cx * s + n.x * d * (1 - c), y: v.y * c + cy * s + n.y * d * (1 - c), z: v.z * c + cz * s + n.z * d * (1 - c) };
  };
  return { normal, forward: turn(forward0, normal), lateral: turn(lateral0, normal) };
}

function wheelProblem(draw: () => number) {
  const pick = <T,>(values: readonly T[]) => values[Math.floor(draw() * values.length)];
  const odd = (normal: number) => pick([normal, normal, normal, normal, normal, normal, NaN, Infinity, -0]);
  const spread = (size: number) => (draw() * 2 - 1) * size;
  const inertia = [900 + draw() * 2_000, 1_200 + draw() * 2_000, 1_800 + draw() * 2_000];
  const body: RigidBodyVelocityState = {
    massKg: pick([1_000, 1_000, 1_000, 800 + draw() * 2_000, 0, -1]),
    inverseInertiaWorld: [1 / inertia[0], spread(1e-5), 0, spread(1e-5), 1 / inertia[1], 0, 0, 0, 1 / inertia[2]],
    velocity: { x: spread(40), y: spread(3), z: spread(1) }, angularVelocity: { x: spread(0.3), y: spread(0.3), z: spread(0.3) },
  };
  const wheels: CoupledWheelState[] = NAMES.map(() => ({ ...createCoupledWheelState(), spinRadSec: spread(200),
    contactEpoch: pick([-1, 1, 2]), warmTangentNs: spread(30), warmBrakeNms: spread(5) }));
  const params: CoupledWheelParams[] = NAMES.map((_, index) => ({ inertiaKgM2: odd(index ? 0.124 : 0.052),
    frictionCoefficient: odd(0.4 + draw() * 0.6), maxBrakeTorqueNm: index ? odd(draw() * 1_500) : 0,
    bearingDragTorqueNm: pick([0, 0, draw() * 3]) }));
  const contacts: WheelContact[] = [];
  for (let index = 0; index < 3; index++) {
    if (draw() < 0.2) continue;
    const { normal, forward, lateral } = frame(spread(0.3), spread(0.1));
    contacts.push({
      wheel: pick([NAMES[index], NAMES[index], NAMES[index], NAMES[(index + 1) % 3]]),
      axleOffsetM: { x: spread(2), y: spread(1.2), z: -0.9 + spread(0.1) },
      normalWorld: normal, forwardWorld: draw() < 0.05 ? { x: 2, y: 0, z: 0 } : forward, lateralWorld: lateral,
      wheelCenterVelocityMps: { x: spread(40), y: spread(2), z: spread(1) }, groundVelocityMps: { x: spread(1), y: 0, z: 0 },
      normalLoadN: pick([odd(draw() * 6_000), 0, -5]), compressionM: draw() * 0.1, steeringRad: spread(0.2),
      radiusM: pick([0.18, 0.22, 0.01]), groundRevision: pick([0, 3, 1.5]), contactEpoch: pick([1, 1, 2, -1]),
    });
  }
  const brakes = NAMES.map(() => pick([0, draw(), 1, NaN, 2]));
  const dt = pick([1 / 120, 1 / 120, 1 / 60, 0, NaN, 1, -1]);
  const options: CoupledSolveOptions = pick([{}, { iterations: 3 }, { iterations: 40 }, { warmStart: false }, { iterations: NaN }]);
  return { body, wheels, params, contacts, brakes, dt, options };
}

const clone = <T,>(value: T): T => structuredClone(value);

describe("WASM coupled rigid wheels", () => {
  it("matches the reference value for value, step after step", () => {
    const draw = random(17);
    const solver = createCoupledWheelSolver(3, { core });
    expect(solver.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    for (let trial = 0; trial < 400; trial++) {
      const problem = wheelProblem(draw);
      const expected = clone(problem), actual = clone(problem);
      const expectedResults = NAMES.map(() => createWheelForceResult());
      const actualResults: WheelForceResult[] = NAMES.map(() => createWheelForceResult());
      // Three steps on the same contacts carry the warm start and the contact epochs.
      for (let step = 0; step < 3; step++) {
        const reference = solveCoupledRigidWheels(expected.body, NAMES, expected.wheels, expected.params, expected.contacts,
          expected.brakes, expected.dt, expectedResults, expected.options);
        const wasm = solver.solve(actual.body, NAMES, actual.wheels, actual.params, actual.contacts,
          actual.brakes, actual.dt, actualResults, actual.options);
        same(wasm, reference, `trial ${trial} step ${step} report`);
        same(actual.body, expected.body, `trial ${trial} step ${step} body`);
        same(actual.wheels, expected.wheels, `trial ${trial} step ${step} wheels`);
        same(actualResults, expectedResults, `trial ${trial} step ${step} results`);
      }
    }
    expect(solver.status().fallbackReason).toBeNull();
    solver.dispose();
  });

  it("writes the results into the caller's vectors and clears any past the wheels", () => {
    const solver = createCoupledWheelSolver(3, { core });
    const problem = wheelProblem(random(3));
    while (problem.contacts.length === 0) Object.assign(problem, wheelProblem(random(4)));
    const results = [...NAMES, "spare"].map(() => ({ ...createWheelForceResult(), dissipatedJ: 7 }));
    const vectors = results.map(result => result.bodyImpulseNs);
    solver.solve(problem.body, NAMES, problem.wheels, problem.params, problem.contacts, problem.brakes, 1 / 120, results);
    expect(results.slice(0, 3).map(result => result.bodyImpulseNs)).toEqual(vectors.slice(0, 3));
    results.slice(0, 3).forEach((result, index) => expect(result.bodyImpulseNs).toBe(vectors[index]));
    same(results[3], createWheelForceResult());
    solver.dispose();
  });

  it("runs the reference for a call the kernel cannot take, and says so", () => {
    const solver = createCoupledWheelSolver(2, { core, maxContacts: 2 });
    const problem = wheelProblem(random(9));
    const expected = clone(problem);
    const results = NAMES.map(() => createWheelForceResult()), expectedResults = NAMES.map(() => createWheelForceResult());
    same(solver.solve(problem.body, NAMES, problem.wheels, problem.params, problem.contacts, problem.brakes, 1 / 120, results),
      solveCoupledRigidWheels(expected.body, NAMES, expected.wheels, expected.params, expected.contacts, expected.brakes, 1 / 120, expectedResults));
    same(problem.wheels, expected.wheels);
    expect(solver.status()).toEqual({ requestedKernel: "single", kernel: "single",
      fallbackReason: expect.stringMatching(/^wasm-wheel-solver-refused: 3 wheels/) });
    solver.dispose();
  });
});

function spinCase(draw: () => number) {
  const pick = <T,>(values: readonly T[]) => values[Math.floor(draw() * values.length)];
  const odd = (normal: number) => pick([normal, normal, normal, normal, normal, NaN, Infinity, -Infinity, -0]);
  const states: WheelSpinState[] = WHEEL_SPIN_CONFIGS.map(() => ({ ...createWheelSpinState(),
    angleRad: odd(draw() * 7 - 0.5), omegaRadSec: odd((draw() * 2 - 1) * 300) }));
  const configs: WheelSpinConfig[] = WHEEL_SPIN_CONFIGS.map(config => ({ ...config,
    radiusMeters: odd(config.radiusMeters), inertiaKgMetersSquared: odd(config.inertiaKgMetersSquared),
    frictionCoefficient: odd(config.frictionCoefficient), maxBrakeTorqueNewtonMeters: odd(config.maxBrakeTorqueNewtonMeters) }));
  const inputs: WheelSpinInput[] = WHEEL_SPIN_CONFIGS.map(() => ({ onGround: draw() < 0.7,
    rollMetersSec: odd((draw() * 2 - 1) * 60), normalLoadNewtons: odd(draw() * 8_000), compressionMeters: odd(draw() * 0.2),
    steeringRad: odd((draw() * 2 - 1) * 2), brake: odd(pick([0, draw(), 1, 1.5])) }));
  const dt = pick([1 / 120, 1 / 120, 1 / 60, 0.5, 0, -1, NaN]);
  const mode = pick<WheelSpinMode>(["instant", "inertia", "inertia"]);
  return { states, configs, inputs, dt, mode };
}

describe("WASM wheel spin", () => {
  it("matches stepWheelSpin value for value, step after step", () => {
    const draw = random(29);
    const stepper = createWheelSpinStepper(3, { core });
    for (let trial = 0; trial < 500; trial++) {
      const problem = spinCase(draw);
      const expected = clone(problem.states), actual = clone(problem.states);
      for (let step = 0; step < 4; step++) {
        expected.forEach((state, index) => stepWheelSpin(state, problem.configs[index], problem.inputs[index], problem.dt, problem.mode));
        stepper.step(actual, problem.configs, problem.inputs, problem.dt, problem.mode);
        same(actual, expected, `trial ${trial} step ${step}`);
      }
    }
    expect(stepper.status()).toEqual({ requestedKernel: "single", kernel: "single", fallbackReason: null });
    stepper.dispose();
  });

  it("keeps the experiment's states identical to the reference", () => {
    const draw = random(41);
    const values: Record<string, number> = {};
    const sdk = { getPropertyValue: (name: string) => values[name] ?? 0 };
    const reference = createWheelSpinExperiment(sdk as never);
    const wasm = createWheelSpinExperiment(sdk as never, { core });
    expect(reference.status().kernel).toBe("typescript");
    expect(wasm.status().kernel).toBe("single");
    for (let step = 0; step < 600; step++) {
      values["fcs/left-brake-cmd-norm"] = draw() < 0.3 ? draw() : 0;
      values["fcs/right-brake-cmd-norm"] = draw() < 0.3 ? draw() : 0;
      values["fcs/steer-pos-deg[0]"] = (draw() * 2 - 1) * 10;
      for (let unit = 0; unit < 3; unit++) {
        values[`gear/unit[${unit}]/WOW`] = draw() < 0.8 ? 1 : 0;
        values[`gear/unit[${unit}]/compression-ft`] = draw() * 0.3;
        values[`gear/unit[${unit}]/compression-velocity-fps`] = (draw() * 2 - 1) * 2;
        values[`gear/unit[${unit}]/wheel-speed-fps`] = draw() * 120;
      }
      const mode = step % 200 < 100 ? "inertia" : "instant";
      reference.step(1 / 120, mode);
      wasm.step(1 / 120, mode);
      same(wasm.getStates(), reference.getStates(), `step ${step}`);
    }
    wasm.dispose();
    expect(wasm.status().kernel).toBe("typescript");
  });
});

describe("WASM kernel lifecycle", () => {
  it("stays on the reference without a core, and reports why when the core cannot be had", async () => {
    expect(createWheelSpinStepper(3).status()).toEqual({ requestedKernel: "typescript", kernel: "typescript", fallbackReason: null });
    const rejected = createWheelSpinStepper(3, { core: Promise.reject(new Error("no WASM")) });
    await Promise.resolve();
    expect(rejected.status()).toEqual({ requestedKernel: "single", kernel: "typescript",
      fallbackReason: "ground-contact-core-unavailable: no WASM" });
    const tooMany = createWheelSpinStepper(17, { core });
    expect(tooMany.status().fallbackReason).toMatch(/^wasm-wheel-spin-unavailable: Spin wheels must be an integer from 1 to 16/);
  });

  it("moves to the kernel when a loading core arrives, and releases it when disposed first", async () => {
    let resolve!: (value: GroundContactCore) => void;
    const loading = createWheelSpinStepper(3, { core: new Promise<GroundContactCore>(done => { resolve = done; }) });
    expect(loading.status().kernel).toBe("typescript");
    const createWheelSpin = vi.spyOn(core, "createWheelSpin");
    resolve(core);
    await Promise.resolve();
    expect(loading.status().kernel).toBe("single");
    const kernel = createWheelSpin.mock.results[0].value as { dispose(): void };
    const release = vi.spyOn(kernel, "dispose");
    loading.dispose();
    expect(release).toHaveBeenCalledOnce();
    expect(loading.status()).toEqual({ requestedKernel: "single", kernel: "typescript", fallbackReason: null });

    let late!: (value: GroundContactCore) => void;
    const disposedFirst = createWheelSpinStepper(3, { core: new Promise<GroundContactCore>(done => { late = done; }) });
    disposedFirst.dispose();
    createWheelSpin.mockClear();
    late(core);
    await Promise.resolve();
    expect(createWheelSpin).not.toHaveBeenCalled();
    createWheelSpin.mockRestore();
  });

  it("gives the kernel up for good when it fails, and finishes that step on the reference", async () => {
    const own = await freshCore();
    const stepper = createWheelSpinStepper(3, { core: own });
    const solver = createCoupledWheelSolver(3, { core: own });
    own.dispose();
    const problem = spinCase(random(5));
    problem.dt = 1 / 120;
    const expected = clone(problem.states);
    expected.forEach((state, index) => stepWheelSpin(state, problem.configs[index], problem.inputs[index], problem.dt, problem.mode));
    stepper.step(problem.states, problem.configs, problem.inputs, problem.dt, problem.mode);
    same(problem.states, expected);
    expect(stepper.status()).toEqual({ requestedKernel: "single", kernel: "typescript",
      fallbackReason: "wasm-wheel-spin-failed: WASM ground-contact core is disposed" });

    const wheels = wheelProblem(random(11));
    const reference = clone(wheels);
    const results = NAMES.map(() => createWheelForceResult()), expectedResults = NAMES.map(() => createWheelForceResult());
    same(solver.solve(wheels.body, NAMES, wheels.wheels, wheels.params, wheels.contacts, wheels.brakes, wheels.dt, results, wheels.options),
      solveCoupledRigidWheels(reference.body, NAMES, reference.wheels, reference.params, reference.contacts, reference.brakes,
        reference.dt, expectedResults, reference.options));
    same(results, expectedResults);
    expect(solver.status().kernel).toBe("typescript");
    expect(solver.status().fallbackReason).toBe("wasm-wheel-solver-failed: WASM ground-contact core is disposed");
  });

  it("allocates nothing in WASM per step", () => {
    const stepper = createWheelSpinStepper(3, { core });
    const solver = createCoupledWheelSolver(3, { core });
    const problem = spinCase(random(13));
    const wheels = wheelProblem(random(14));
    const results = NAMES.map(() => createWheelForceResult());
    const before = core.heapAllocations();
    for (let step = 0; step < 100; step++) {
      stepper.step(problem.states, problem.configs, problem.inputs, 1 / 120, "inertia");
      solver.solve(wheels.body, NAMES, wheels.wheels, wheels.params, wheels.contacts, wheels.brakes, 1 / 120, results);
    }
    expect(core.heapAllocations()).toBe(before);
    stepper.dispose();
    solver.dispose();
  });
});
