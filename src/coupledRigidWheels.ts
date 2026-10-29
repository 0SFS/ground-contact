import { createKernelSlot, type GroundContactCoreSource, type GroundContactKernelStatus } from "./groundContactCore";
import { wheelContactProblem, type Vec3, type WheelContact, type WheelForceResult } from "./wheelContact";

/**
 * Reference coupled rigid-wheel impulse solver (roadmap step 4).
 *
 * NOT wired into the game. It needs a NativeWheelContactBridge that owns brake
 * and rolling friction exclusively; the installed JSBSim WASM provides none, so
 * Settings keeps "Coupled rigid wheel" unavailable. This module exists so the
 * contact accounting and its conservation tests are settled before that bridge.
 *
 * Accounting (per wheel, spin axis a = n × t, absolute wheel spin Ω about a):
 *   tread slip  u = t·(v + w × r_axle − v_ground) − R Ω
 *   tangent impulse λ (ground on tire, along t):
 *     airframe  Δp = λ t,   ΔL = (r_axle × t) λ     ← force acts at the axle
 *     wheel     Iw ΔΩ = −R λ
 *   brake/bearing impulse β (internal, wheel ↔ airframe):
 *     wheel     Iw ΔΩ = β,  airframe ΔL = −β a      ← equal and opposite
 * Total angular momentum then changes by exactly r_contact × λ t. Applying the
 * force at the contact point *and* adding R × F to the wheel double-counts.
 */
export interface RigidBodyVelocityState {
  massKg: number;
  /** Row-major symmetric inverse inertia in world axes. */
  inverseInertiaWorld: readonly number[];
  velocity: Vec3;
  angularVelocity: Vec3;
}

export interface CoupledWheelParams {
  inertiaKgM2: number;
  frictionCoefficient: number;
  maxBrakeTorqueNm: number;
  bearingDragTorqueNm: number;
}

export interface CoupledWheelState {
  /** Absolute spin about the axle axis; positive rolls forward. */
  spinRadSec: number;
  contactEpoch: number;
  warmTangentNs: number;
  warmBrakeNms: number;
}

export interface CoupledSolveOptions {
  iterations?: number;
  warmStart?: boolean;
}

export interface CoupledSolveReport {
  iterations: number;
  kineticEnergyBeforeJ: number;
  kineticEnergyAfterJ: number;
  /** A warm start that would have added energy was discarded and solved cold. */
  coldRetry: boolean;
  rejectedContacts: number;
}

export const MAX_COUPLED_DT = 1 / 30;
export const MAX_COUPLED_ITERATIONS = 16;

export function createCoupledWheelState(): CoupledWheelState {
  return { spinRadSec: 0, contactEpoch: -1, warmTangentNs: 0, warmBrakeNms: 0 };
}

export function createWheelForceResult(): WheelForceResult {
  return { bodyImpulseNs: { x: 0, y: 0, z: 0 }, bodyAngularImpulseNms: { x: 0, y: 0, z: 0 },
    wheelAngularImpulseNms: 0, longitudinalSlipMps: 0, lateralSlipMps: 0, normalImpulseNs: 0, dissipatedJ: 0 };
}

type V = { x: number; y: number; z: number };
const cross = (a: V, b: V, out: V): V => {
  const x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x;
  out.x = x; out.y = y; out.z = z; return out;
};
const dot = (a: V, b: V) => a.x * b.x + a.y * b.y + a.z * b.z;
const mulInv = (m: readonly number[], v: V, out: V): V => {
  const x = m[0] * v.x + m[1] * v.y + m[2] * v.z;
  const y = m[3] * v.x + m[4] * v.y + m[5] * v.z;
  const z = m[6] * v.x + m[7] * v.y + m[8] * v.z;
  out.x = x; out.y = y; out.z = z; return out;
};
const finite = (value: number, fallback = 0) => Number.isFinite(value) ? value : fallback;

interface Row {
  wheel: number;
  grounded: boolean;
  spinAxis: V;
  tangent: V;
  armCrossT: V;       // r_axle × t
  invIArmCrossT: V;   // I⁻¹ (r_axle × t)
  invIAxis: V;        // I⁻¹ a
  tangentMass: number;
  brakeMass: number;
  tangentLimit: number;
  brakeLimit: number;
  tangentAcc: number;
  brakeAcc: number;
  radius: number;
  groundAlongT: number;
  contact: WheelContact | null;
}

export function kineticEnergy(body: RigidBodyVelocityState, wheels: readonly CoupledWheelState[], params: readonly CoupledWheelParams[]): number {
  const w = body.angularVelocity;
  const m = body.inverseInertiaWorld;
  // Solve ½ wᵀ I w using the inverse via a 3×3 inversion (reference code; cost is irrelevant).
  const det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const inertia = [
    (m[4] * m[8] - m[5] * m[7]) / det, (m[2] * m[7] - m[1] * m[8]) / det, (m[1] * m[5] - m[2] * m[4]) / det,
    (m[5] * m[6] - m[3] * m[8]) / det, (m[0] * m[8] - m[2] * m[6]) / det, (m[2] * m[3] - m[0] * m[5]) / det,
    (m[3] * m[7] - m[4] * m[6]) / det, (m[1] * m[6] - m[0] * m[7]) / det, (m[0] * m[4] - m[1] * m[3]) / det,
  ];
  const Iw = mulInv(inertia, w, { x: 0, y: 0, z: 0 });
  let energy = 0.5 * body.massKg * dot(body.velocity, body.velocity) + 0.5 * dot(w, Iw);
  wheels.forEach((wheel, index) => { energy += 0.5 * params[index].inertiaKgM2 * wheel.spinRadSec ** 2; });
  return energy;
}

/**
 * One fixed-step tangential/brake solve. Updates body and wheel velocities in
 * place and writes one result per wheel. `contacts` holds only grounded wheels,
 * matched by index in `wheelNames`. Never creates a normal impulse.
 */
export function solveCoupledRigidWheels(
  body: RigidBodyVelocityState,
  wheelNames: readonly WheelContact["wheel"][],
  wheels: CoupledWheelState[],
  params: readonly CoupledWheelParams[],
  contacts: readonly WheelContact[],
  brakes: readonly number[],
  dtSeconds: number,
  results: WheelForceResult[],
  options: CoupledSolveOptions = {},
): CoupledSolveReport {
  const dt = Math.min(MAX_COUPLED_DT, Math.max(0, finite(dtSeconds)));
  const iterations = Math.min(MAX_COUPLED_ITERATIONS, Math.max(1, Math.floor(finite(options.iterations ?? 8, 8))));
  const before = kineticEnergy(body, wheels, params);
  const report: CoupledSolveReport = { iterations, kineticEnergyBeforeJ: before, kineticEnergyAfterJ: before,
    coldRetry: false, rejectedContacts: 0 };
  for (const result of results) Object.assign(result, createWheelForceResult());
  if (dt === 0 || !(body.massKg > 0)) return report;

  const rows: Row[] = [];
  for (let index = 0; index < wheels.length; index++) {
    const contact = contacts.find(candidate => candidate.wheel === wheelNames[index]) ?? null;
    const problem = contact ? wheelContactProblem(contact) : null;
    if (problem) report.rejectedContacts += 1;
    const grounded = contact !== null && problem === null && contact.normalLoadN > 0;
    const wheel = wheels[index];
    const config = params[index];
    // Stale warm-start impulses must never cross a contact epoch or an airborne step.
    if (!grounded || contact!.contactEpoch !== wheel.contactEpoch) {
      wheel.warmTangentNs = 0;
      wheel.warmBrakeNms = 0;
      wheel.contactEpoch = grounded ? contact!.contactEpoch : -1;
    }
    const normal = grounded ? contact!.normalWorld : { x: 0, y: 0, z: 1 };
    const tangent = grounded ? contact!.forwardWorld : { x: 1, y: 0, z: 0 };
    const spinAxis = cross(normal, tangent, { x: 0, y: 0, z: 0 });
    const arm = grounded ? contact!.axleOffsetM : { x: 0, y: 0, z: 0 };
    const armCrossT = cross(arm, tangent, { x: 0, y: 0, z: 0 });
    const invIArmCrossT = mulInv(body.inverseInertiaWorld, armCrossT, { x: 0, y: 0, z: 0 });
    const invIAxis = mulInv(body.inverseInertiaWorld, spinAxis, { x: 0, y: 0, z: 0 });
    const inertia = Math.max(1e-3, finite(config.inertiaKgM2, 1));
    const radius = grounded ? contact!.radiusM : 0;
    const brake = Math.min(1, Math.max(0, finite(brakes[index])));
    rows.push({
      wheel: index, grounded, spinAxis, tangent, armCrossT, invIArmCrossT, invIAxis,
      tangentMass: grounded ? 1 / (1 / body.massKg + dot(armCrossT, invIArmCrossT) + radius * radius / inertia) : 0,
      brakeMass: 1 / (1 / inertia + dot(spinAxis, invIAxis)),
      tangentLimit: grounded ? Math.max(0, finite(config.frictionCoefficient)) * contact!.normalLoadN * dt : 0,
      brakeLimit: (brake * Math.max(0, finite(config.maxBrakeTorqueNm)) + Math.max(0, finite(config.bearingDragTorqueNm))) * dt,
      tangentAcc: 0, brakeAcc: 0, radius,
      groundAlongT: grounded ? dot(contact!.groundVelocityMps, tangent) : 0,
      contact: grounded ? contact : null,
    });
  }

  const v = body.velocity;
  const w = body.angularVelocity;
  const saved = { v: { ...v }, w: { ...w }, spins: wheels.map(wheel => wheel.spinRadSec) };
  const dissipated = new Array<number>(wheels.length).fill(0);

  const applyTangent = (row: Row, impulse: number) => {
    const inertia = Math.max(1e-3, params[row.wheel].inertiaKgM2);
    v.x += row.tangent.x * impulse / body.massKg;
    v.y += row.tangent.y * impulse / body.massKg;
    v.z += row.tangent.z * impulse / body.massKg;
    w.x += row.invIArmCrossT.x * impulse;
    w.y += row.invIArmCrossT.y * impulse;
    w.z += row.invIArmCrossT.z * impulse;
    wheels[row.wheel].spinRadSec -= row.radius * impulse / inertia;
  };
  const applyBrake = (row: Row, impulse: number) => {
    const inertia = Math.max(1e-3, params[row.wheel].inertiaKgM2);
    wheels[row.wheel].spinRadSec += impulse / inertia;
    w.x -= row.invIAxis.x * impulse;
    w.y -= row.invIAxis.y * impulse;
    w.z -= row.invIAxis.z * impulse;
  };
  const tangentSlip = (row: Row) =>
    dot(row.tangent, v) + dot(row.armCrossT, w) - row.groundAlongT - row.radius * wheels[row.wheel].spinRadSec;
  const relativeSpin = (row: Row) => wheels[row.wheel].spinRadSec - dot(row.spinAxis, w);

  const solve = (warm: boolean) => {
    for (const row of rows) {
      row.tangentAcc = row.brakeAcc = 0;
      if (!warm) continue;
      const wheel = wheels[row.wheel];
      const tangentWarm = Math.max(-row.tangentLimit, Math.min(row.tangentLimit, wheel.warmTangentNs));
      const brakeWarm = Math.max(-row.brakeLimit, Math.min(row.brakeLimit, wheel.warmBrakeNms));
      // Exact for any impulse along a constraint: ΔKE = (u1² − u0²) · m_eff / 2.
      if (row.grounded && tangentWarm !== 0) {
        const u0 = tangentSlip(row);
        applyTangent(row, tangentWarm);
        row.tangentAcc = tangentWarm;
        const u1 = tangentSlip(row);
        dissipated[row.wheel] += (u0 * u0 - u1 * u1) * row.tangentMass / 2;
      }
      if (brakeWarm !== 0) {
        const u0 = relativeSpin(row);
        applyBrake(row, brakeWarm);
        row.brakeAcc = brakeWarm;
        const u1 = relativeSpin(row);
        dissipated[row.wheel] += (u0 * u0 - u1 * u1) * row.brakeMass / 2;
      }
    }
    for (let iteration = 0; iteration < iterations; iteration++) {
      for (const row of rows) {
        // Clamped accumulated impulses: each update moves its constraint velocity toward, never past, zero.
        if (row.brakeLimit > 0) {
          const target = row.brakeAcc - relativeSpin(row) * row.brakeMass;
          const clamped = Math.max(-row.brakeLimit, Math.min(row.brakeLimit, target));
          const delta = clamped - row.brakeAcc;
          if (delta !== 0) {
            const u0 = relativeSpin(row);
            applyBrake(row, delta);
            row.brakeAcc = clamped;
            const u1 = relativeSpin(row);
            dissipated[row.wheel] += (u0 * u0 - u1 * u1) / (2 / row.brakeMass);
          }
        }
        if (row.grounded && row.tangentLimit > 0) {
          const u0 = tangentSlip(row);
          const target = row.tangentAcc - u0 * row.tangentMass;
          const clamped = Math.max(-row.tangentLimit, Math.min(row.tangentLimit, target));
          const delta = clamped - row.tangentAcc;
          if (delta !== 0) {
            applyTangent(row, delta);
            row.tangentAcc = clamped;
            const u1 = tangentSlip(row);
            dissipated[row.wheel] += (u0 * u0 - u1 * u1) / (2 / row.tangentMass);
          }
        }
      }
    }
  };

  const warm = options.warmStart !== false;
  solve(warm);
  let after = kineticEnergy(body, wheels, params);
  if (warm && after > before * (1 + 1e-12) + 1e-9) {
    // A warm start from a changed contact can inject energy; a cold solve cannot.
    v.x = saved.v.x; v.y = saved.v.y; v.z = saved.v.z;
    w.x = saved.w.x; w.y = saved.w.y; w.z = saved.w.z;
    wheels.forEach((wheel, index) => { wheel.spinRadSec = saved.spins[index]; });
    dissipated.fill(0);
    solve(false);
    after = kineticEnergy(body, wheels, params);
    report.coldRetry = true;
  }
  report.kineticEnergyAfterJ = after;

  for (const row of rows) {
    const wheel = wheels[row.wheel];
    wheel.warmTangentNs = row.grounded ? row.tangentAcc : 0;
    wheel.warmBrakeNms = row.brakeAcc;
    const result = results[row.wheel];
    if (!result) continue;
    const t = row.tangent;
    result.bodyImpulseNs.x = t.x * row.tangentAcc;
    result.bodyImpulseNs.y = t.y * row.tangentAcc;
    result.bodyImpulseNs.z = t.z * row.tangentAcc;
    result.bodyAngularImpulseNms.x = row.armCrossT.x * row.tangentAcc - row.spinAxis.x * row.brakeAcc;
    result.bodyAngularImpulseNms.y = row.armCrossT.y * row.tangentAcc - row.spinAxis.y * row.brakeAcc;
    result.bodyAngularImpulseNms.z = row.armCrossT.z * row.tangentAcc - row.spinAxis.z * row.brakeAcc;
    result.wheelAngularImpulseNms = -row.radius * row.tangentAcc + row.brakeAcc;
    result.longitudinalSlipMps = row.grounded ? tangentSlip(row) : 0;
    result.lateralSlipMps = row.contact
      ? dot(row.contact.lateralWorld, v) + dot(cross(row.contact.axleOffsetM, row.contact.lateralWorld, { x: 0, y: 0, z: 0 }), w)
        - dot(row.contact.groundVelocityMps, row.contact.lateralWorld)
      : 0;
    result.normalImpulseNs = row.contact ? row.contact.normalLoadN * dt : 0;
    // Net work removed by this wheel's impulses; the step total is never negative.
    result.dissipatedJ = dissipated[row.wheel];
  }
  return report;
}

export interface CoupledWheelSolver {
  /** `solveCoupledRigidWheels` with the same arguments, on the WASM kernel when one is ready. */
  solve(
    body: RigidBodyVelocityState,
    wheelNames: readonly WheelContact["wheel"][],
    wheels: CoupledWheelState[],
    params: readonly CoupledWheelParams[],
    contacts: readonly WheelContact[],
    brakes: readonly number[],
    dtSeconds: number,
    results: WheelForceResult[],
    options?: CoupledSolveOptions,
  ): CoupledSolveReport;
  status(): GroundContactKernelStatus;
  dispose(): void;
}

const BODY = { velocity: 10, angularVelocity: 13 } as const;
const WHEEL_STRIDE = 10;
const CONTACT_STRIDE = 25;
const REPORT_LENGTH = 5;
const RESULT_STRIDE = 11;

const writeVec = (rows: Float64Array, at: number, vector: Vec3) => {
  rows[at] = vector.x; rows[at + 1] = vector.y; rows[at + 2] = vector.z;
};
const readVec = (rows: Float64Array, at: number, out: Vec3 | undefined): Vec3 => {
  const vector = out ?? { x: 0, y: 0, z: 0 };
  vector.x = rows[at]; vector.y = rows[at + 1]; vector.z = rows[at + 2];
  return vector;
};

/**
 * `solveCoupledRigidWheels` for `wheelCount` wheels and up to `maxContacts`
 * contacts (default `wheelCount`), delegated to the core's
 * `createWheelSolver` when one is supplied. Identical to the reference,
 * value for value: wheels and contacts are matched by the first index of
 * their name in `wheelNames`, which keeps the reference's `===`. Results are
 * written into the caller's vectors rather than replacing them.
 */
export function createCoupledWheelSolver(wheelCount: number,
  options: { core?: GroundContactCoreSource; maxContacts?: number } = {}): CoupledWheelSolver {
  const slot = createKernelSlot(options.core, "wasm-wheel-solver",
    core => core.createWheelSolver(wheelCount, options.maxContacts ?? wheelCount));
  return {
    solve(body, wheelNames, wheels, params, contacts, brakes, dtSeconds, results, solveOptions = {}) {
      const kernel = slot.current;
      const reference = () => solveCoupledRigidWheels(body, wheelNames, wheels, params, contacts, brakes, dtSeconds, results, solveOptions);
      if (!kernel) return reference();
      const count = kernel.wheelCount;
      if (wheels.length !== count || params.length < count || wheelNames.length < count || contacts.length > kernel.maxContacts) {
        slot.refuse(`${wheels.length} wheels and ${contacts.length} contacts for a ${count}-wheel, ${kernel.maxContacts}-contact kernel`);
        return reference();
      }
      try {
        const state = kernel.body;
        state[0] = body.massKg;
        for (let index = 0; index < 9; index++) state[1 + index] = body.inverseInertiaWorld[index];
        writeVec(state, BODY.velocity, body.velocity);
        writeVec(state, BODY.angularVelocity, body.angularVelocity);
        const rows = kernel.wheels;
        for (let index = 0, at = 0; index < count; index++, at += WHEEL_STRIDE) {
          const wheel = wheels[index], config = params[index];
          rows[at] = 1 + wheelNames.indexOf(wheelNames[index]);
          rows[at + 1] = wheel.spinRadSec;
          rows[at + 2] = wheel.contactEpoch;
          rows[at + 3] = wheel.warmTangentNs;
          rows[at + 4] = wheel.warmBrakeNms;
          rows[at + 5] = config.inertiaKgM2;
          rows[at + 6] = config.frictionCoefficient;
          rows[at + 7] = config.maxBrakeTorqueNm;
          rows[at + 8] = config.bearingDragTorqueNm;
          rows[at + 9] = brakes[index];
        }
        const packed = kernel.contacts;
        for (let index = 0, at = 0; index < contacts.length; index++, at += CONTACT_STRIDE) {
          const contact = contacts[index];
          packed[at] = 1 + wheelNames.indexOf(contact.wheel);
          writeVec(packed, at + 1, contact.axleOffsetM);
          writeVec(packed, at + 4, contact.normalWorld);
          writeVec(packed, at + 7, contact.forwardWorld);
          writeVec(packed, at + 10, contact.lateralWorld);
          writeVec(packed, at + 13, contact.wheelCenterVelocityMps);
          writeVec(packed, at + 16, contact.groundVelocityMps);
          packed[at + 19] = contact.normalLoadN;
          packed[at + 20] = contact.compressionM;
          packed[at + 21] = contact.steeringRad;
          packed[at + 22] = contact.radiusM;
          packed[at + 23] = contact.groundRevision;
          packed[at + 24] = contact.contactEpoch;
        }
        kernel.run(dtSeconds, contacts.length,
          typeof solveOptions.iterations === "number" ? solveOptions.iterations : NaN, solveOptions.warmStart !== false);
      } catch (error) {
        slot.fail(error);
        return reference();
      }
      const state = kernel.body;
      readVec(state, BODY.velocity, body.velocity);
      readVec(state, BODY.angularVelocity, body.angularVelocity);
      const rows = kernel.wheels;
      for (let index = 0, at = 0; index < count; index++, at += WHEEL_STRIDE) {
        const wheel = wheels[index];
        wheel.spinRadSec = rows[at + 1];
        wheel.contactEpoch = rows[at + 2];
        wheel.warmTangentNs = rows[at + 3];
        wheel.warmBrakeNms = rows[at + 4];
      }
      const output = kernel.output;
      for (let index = 0; index < results.length; index++) {
        const result = results[index];
        if (!result) continue;
        if (index >= count) { Object.assign(result, createWheelForceResult()); continue; }
        const at = REPORT_LENGTH + index * RESULT_STRIDE;
        result.bodyImpulseNs = readVec(output, at, result.bodyImpulseNs);
        result.bodyAngularImpulseNms = readVec(output, at + 3, result.bodyAngularImpulseNms);
        result.wheelAngularImpulseNms = output[at + 6];
        result.longitudinalSlipMps = output[at + 7];
        result.lateralSlipMps = output[at + 8];
        result.normalImpulseNs = output[at + 9];
        result.dissipatedJ = output[at + 10];
      }
      return { iterations: output[0], kineticEnergyBeforeJ: output[1], kineticEnergyAfterJ: output[2],
        coldRetry: output[3] === 1, rejectedContacts: output[4] };
    },
    status: slot.status,
    dispose: slot.dispose,
  };
}
