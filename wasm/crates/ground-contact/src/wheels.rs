//! Coupled rigid-wheel impulse solver: `wheelContactProblem`, `kineticEnergy`
//! and `solveCoupledRigidWheels` from `src/wheelContact.ts` and
//! `src/coupledRigidWheels.ts`, operation for operation, on flat f64 rows.
//!
//! Layouts (f64):
//! - body `[massKg, inverseInertiaWorld (row-major 9), velocity xyz, angularVelocity xyz]`
//! - wheel state `[spinRadSec, contactEpoch, warmTangentNs, warmBrakeNms]` per wheel
//! - params `[inertiaKgM2, frictionCoefficient, maxBrakeTorqueNm, bearingDragTorqueNm]` per wheel
//! - contact `[wheel, axleOffsetM xyz, normalWorld xyz, forwardWorld xyz,
//!   lateralWorld xyz, wheelCenterVelocityMps xyz, groundVelocityMps xyz,
//!   normalLoadN, compressionM, steeringRad, radiusM, groundRevision, contactEpoch]`;
//!   `wheel` is matched against the wheel IDs with `===`, first contact wins
//! - result `[bodyImpulseNs xyz, bodyAngularImpulseNms xyz, wheelAngularImpulseNms,
//!   longitudinalSlipMps, lateralSlipMps, normalImpulseNs, dissipatedJ]` per wheel
//! - report `[iterations, kineticEnergyBeforeJ, kineticEnergyAfterJ, coldRetry, rejectedContacts]`

use crate::jsmath::{finite_or, hypot, is_safe_integer, max, min};

pub const BODY_LEN: usize = 16;
pub const STATE_STRIDE: usize = 4;
pub const PARAMS_STRIDE: usize = 4;
pub const CONTACT_STRIDE: usize = 25;
pub const RESULT_STRIDE: usize = 11;
pub const REPORT_LEN: usize = 5;
pub const MAX_WHEELS: usize = 16;
pub const MAX_CONTACTS: usize = 64;
pub const MAX_COUPLED_DT: f64 = 1.0 / 30.0;
pub const MAX_COUPLED_ITERATIONS: f64 = 16.0;

type V = [f64; 3];

#[inline]
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
fn mul_inv(m: &[f64], v: V) -> V {
    [
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    ]
}
#[inline]
fn vec_at(values: &[f64], at: usize) -> V {
    [values[at], values[at + 1], values[at + 2]]
}

/// Contact field offsets.
const WHEEL: usize = 0;
const AXLE: usize = 1;
const NORMAL: usize = 4;
const FORWARD: usize = 7;
const LATERAL: usize = 10;
const CENTER_VELOCITY: usize = 13;
const GROUND_VELOCITY: usize = 16;
const NORMAL_LOAD: usize = 19;
const COMPRESSION: usize = 20;
const STEERING: usize = 21;
const RADIUS: usize = 22;
const GROUND_REVISION: usize = 23;
const CONTACT_EPOCH: usize = 24;

/// `wheelContactProblem(contact) !== null`: the same checks in the same order.
pub fn contact_problem(contact: &[f64]) -> bool {
    if contact[AXLE..NORMAL_LOAD].iter().any(|v| !v.is_finite())
        || [NORMAL_LOAD, COMPRESSION, STEERING, RADIUS].iter().any(|&key| !contact[key].is_finite())
    {
        return true;
    }
    for key in [CONTACT_EPOCH, GROUND_REVISION] {
        if !is_safe_integer(contact[key]) || contact[key] < 0.0 {
            return true;
        }
    }
    let length = |at: usize| hypot(&contact[at..at + 3]);
    let load = contact[NORMAL_LOAD];
    let radius = contact[RADIUS];
    if load < 0.0 || load > 500_000.0 || radius < 0.05 || radius > 2.0 || length(AXLE) > 50.0 {
        return true;
    }
    if length(CENTER_VELOCITY) > 400.0 || length(GROUND_VELOCITY) > 400.0 {
        return true;
    }
    if [NORMAL, FORWARD, LATERAL].iter().any(|&at| (length(at) - 1.0).abs() > 1e-3) {
        return true;
    }
    let (n, f, l) = (vec_at(contact, NORMAL), vec_at(contact, FORWARD), vec_at(contact, LATERAL));
    dot(n, f).abs() > 1e-3 || dot(n, l).abs() > 1e-3 || dot(f, l).abs() > 1e-3
}

/// `kineticEnergy(body, wheels, params)`, inverting the inverse inertia as the
/// reference does rather than taking an inertia tensor.
pub fn kinetic_energy(body: &[f64], states: &[f64], params: &[f64]) -> f64 {
    let m = &body[1..10];
    let w = vec_at(body, 13);
    let v = vec_at(body, 10);
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);
    let inertia = [
        (m[4] * m[8] - m[5] * m[7]) / det,
        (m[2] * m[7] - m[1] * m[8]) / det,
        (m[1] * m[5] - m[2] * m[4]) / det,
        (m[5] * m[6] - m[3] * m[8]) / det,
        (m[0] * m[8] - m[2] * m[6]) / det,
        (m[2] * m[3] - m[0] * m[5]) / det,
        (m[3] * m[7] - m[4] * m[6]) / det,
        (m[1] * m[6] - m[0] * m[7]) / det,
        (m[0] * m[4] - m[1] * m[3]) / det,
    ];
    let iw = mul_inv(&inertia, w);
    let mut energy = 0.5 * body[0] * dot(v, v) + 0.5 * dot(w, iw);
    for (state, param) in states.chunks_exact(STATE_STRIDE).zip(params.chunks_exact(PARAMS_STRIDE)) {
        // `spin ** 2` is an exact square in V8.
        energy += 0.5 * param[0] * (state[0] * state[0]);
    }
    energy
}

#[derive(Clone, Copy)]
struct Row {
    grounded: bool,
    spin_axis: V,
    tangent: V,
    arm_cross_t: V,
    inv_i_arm_cross_t: V,
    inv_i_axis: V,
    tangent_mass: f64,
    brake_mass: f64,
    tangent_limit: f64,
    brake_limit: f64,
    tangent_acc: f64,
    brake_acc: f64,
    radius: f64,
    ground_along_t: f64,
    /// `Math.max(1e-3, params[i].inertiaKgM2)` used by the impulse updates.
    apply_inertia: f64,
    /// Start of the accepted contact row when grounded.
    contact: Option<usize>,
}

const EMPTY_ROW: Row = Row {
    grounded: false,
    spin_axis: [0.0; 3],
    tangent: [0.0; 3],
    arm_cross_t: [0.0; 3],
    inv_i_arm_cross_t: [0.0; 3],
    inv_i_axis: [0.0; 3],
    tangent_mass: 0.0,
    brake_mass: 0.0,
    tangent_limit: 0.0,
    brake_limit: 0.0,
    tangent_acc: 0.0,
    brake_acc: 0.0,
    radius: 0.0,
    ground_along_t: 0.0,
    apply_inertia: 0.0,
    contact: None,
};

/// Solver scratch: body velocities and wheel spins, updated in place.
struct Motion<'a> {
    mass: f64,
    v: V,
    w: V,
    states: &'a mut [f64],
}
impl Motion<'_> {
    #[inline]
    fn spin(&self, wheel: usize) -> f64 {
        self.states[wheel * STATE_STRIDE]
    }
    #[inline]
    fn apply_tangent(&mut self, wheel: usize, row: &Row, impulse: f64) {
        for axis in 0..3 {
            self.v[axis] += row.tangent[axis] * impulse / self.mass;
        }
        for axis in 0..3 {
            self.w[axis] += row.inv_i_arm_cross_t[axis] * impulse;
        }
        self.states[wheel * STATE_STRIDE] -= row.radius * impulse / row.apply_inertia;
    }
    #[inline]
    fn apply_brake(&mut self, wheel: usize, row: &Row, impulse: f64) {
        self.states[wheel * STATE_STRIDE] += impulse / row.apply_inertia;
        for axis in 0..3 {
            self.w[axis] -= row.inv_i_axis[axis] * impulse;
        }
    }
    #[inline]
    fn tangent_slip(&self, wheel: usize, row: &Row) -> f64 {
        dot(row.tangent, self.v) + dot(row.arm_cross_t, self.w)
            - row.ground_along_t
            - row.radius * self.spin(wheel)
    }
    #[inline]
    fn relative_spin(&self, wheel: usize, row: &Row) -> f64 {
        self.spin(wheel) - dot(row.spin_axis, self.w)
    }
}

/// Inputs that the solve reads but never changes.
pub struct WheelProblem<'a> {
    pub wheel_ids: &'a [f64],
    pub params: &'a [f64],
    pub contacts: &'a [f64],
    pub brakes: &'a [f64],
    pub dt_seconds: f64,
    /// `options.iterations`; NaN stands for an absent option.
    pub iterations: f64,
    pub warm_start: bool,
}

/// `solveCoupledRigidWheels`. Updates `body` velocities and `states` in place
/// and writes one result per wheel and the report. Callers guarantee the row
/// counts (at most [`MAX_WHEELS`] wheels).
pub fn solve(
    problem: &WheelProblem,
    body: &mut [f64],
    states: &mut [f64],
    results: &mut [f64],
    report: &mut [f64],
) {
    let count = problem.wheel_ids.len();
    debug_assert!(count <= MAX_WHEELS);
    let params = problem.params;
    let dt = min(MAX_COUPLED_DT, max(0.0, finite_or(problem.dt_seconds, 0.0)));
    let iterations = min(
        MAX_COUPLED_ITERATIONS,
        max(1.0, finite_or(problem.iterations, 8.0).floor()),
    );
    let before = kinetic_energy(body, states, params);
    report.copy_from_slice(&[iterations, before, before, 0.0, 0.0]);
    results.fill(0.0);
    let mass = body[0];
    if dt == 0.0 || !(mass > 0.0) {
        return;
    }

    let inverse_inertia: [f64; 9] = body[1..10].try_into().unwrap();
    let mut rows = [EMPTY_ROW; MAX_WHEELS];
    let mut rejected = 0.0;
    for (index, row) in rows.iter_mut().enumerate().take(count) {
        let contact = problem
            .contacts
            .chunks_exact(CONTACT_STRIDE)
            .position(|c| c[WHEEL] == problem.wheel_ids[index])
            .map(|at| at * CONTACT_STRIDE);
        let bad = contact.is_some_and(|at| contact_problem(&problem.contacts[at..at + CONTACT_STRIDE]));
        if bad {
            rejected += 1.0;
        }
        let accepted = contact.filter(|_| !bad);
        let c = accepted.map(|at| &problem.contacts[at..at + CONTACT_STRIDE]);
        let grounded = c.is_some_and(|c| c[NORMAL_LOAD] > 0.0);
        let state = &mut states[index * STATE_STRIDE..(index + 1) * STATE_STRIDE];
        let config = &params[index * PARAMS_STRIDE..(index + 1) * PARAMS_STRIDE];
        // Stale warm-start impulses must never cross a contact epoch or an airborne step.
        if !grounded || c.unwrap()[CONTACT_EPOCH] != state[1] {
            state[2] = 0.0;
            state[3] = 0.0;
            state[1] = if grounded { c.unwrap()[CONTACT_EPOCH] } else { -1.0 };
        }
        let c = if grounded { c } else { None };
        let normal = c.map_or([0.0, 0.0, 1.0], |c| vec_at(c, NORMAL));
        let tangent = c.map_or([1.0, 0.0, 0.0], |c| vec_at(c, FORWARD));
        let spin_axis = cross(normal, tangent);
        let arm = c.map_or([0.0; 3], |c| vec_at(c, AXLE));
        let arm_cross_t = cross(arm, tangent);
        let inv_i_arm_cross_t = mul_inv(&inverse_inertia, arm_cross_t);
        let inv_i_axis = mul_inv(&inverse_inertia, spin_axis);
        let inertia = max(1e-3, finite_or(config[0], 1.0));
        let radius = c.map_or(0.0, |c| c[RADIUS]);
        let brake = min(1.0, max(0.0, finite_or(problem.brakes[index], 0.0)));
        *row = Row {
            grounded,
            spin_axis,
            tangent,
            arm_cross_t,
            inv_i_arm_cross_t,
            inv_i_axis,
            tangent_mass: if grounded {
                1.0 / (1.0 / mass + dot(arm_cross_t, inv_i_arm_cross_t) + radius * radius / inertia)
            } else {
                0.0
            },
            brake_mass: 1.0 / (1.0 / inertia + dot(spin_axis, inv_i_axis)),
            tangent_limit: c.map_or(0.0, |c| {
                max(0.0, finite_or(config[1], 0.0)) * c[NORMAL_LOAD] * dt
            }),
            brake_limit: (brake * max(0.0, finite_or(config[2], 0.0))
                + max(0.0, finite_or(config[3], 0.0)))
                * dt,
            tangent_acc: 0.0,
            brake_acc: 0.0,
            radius,
            ground_along_t: c.map_or(0.0, |c| dot(vec_at(c, GROUND_VELOCITY), tangent)),
            apply_inertia: max(1e-3, config[0]),
            contact: if grounded { accepted } else { None },
        };
    }
    report[4] = rejected;
    let rows = &mut rows[..count];

    let saved_v = vec_at(body, 10);
    let saved_w = vec_at(body, 13);
    let mut saved_spins = [0.0; MAX_WHEELS];
    for (index, spin) in saved_spins.iter_mut().enumerate().take(count) {
        *spin = states[index * STATE_STRIDE];
    }
    let mut dissipated = [0.0_f64; MAX_WHEELS];
    let mut motion = Motion { mass, v: saved_v, w: saved_w, states };

    let solve_rows = |motion: &mut Motion, rows: &mut [Row], dissipated: &mut [f64], warm: bool| {
        for (wheel, row) in rows.iter_mut().enumerate() {
            row.tangent_acc = 0.0;
            row.brake_acc = 0.0;
            if !warm {
                continue;
            }
            let state = wheel * STATE_STRIDE;
            let tangent_warm = max(-row.tangent_limit, min(row.tangent_limit, motion.states[state + 2]));
            let brake_warm = max(-row.brake_limit, min(row.brake_limit, motion.states[state + 3]));
            // Exact for any impulse along a constraint: ΔKE = (u1² − u0²) · m_eff / 2.
            if row.grounded && tangent_warm != 0.0 {
                let u0 = motion.tangent_slip(wheel, row);
                motion.apply_tangent(wheel, row, tangent_warm);
                row.tangent_acc = tangent_warm;
                let u1 = motion.tangent_slip(wheel, row);
                dissipated[wheel] += (u0 * u0 - u1 * u1) * row.tangent_mass / 2.0;
            }
            if brake_warm != 0.0 {
                let u0 = motion.relative_spin(wheel, row);
                motion.apply_brake(wheel, row, brake_warm);
                row.brake_acc = brake_warm;
                let u1 = motion.relative_spin(wheel, row);
                dissipated[wheel] += (u0 * u0 - u1 * u1) * row.brake_mass / 2.0;
            }
        }
        for _ in 0..iterations as usize {
            for (wheel, row) in rows.iter_mut().enumerate() {
                // Clamped accumulated impulses: each update moves its constraint velocity toward, never past, zero.
                if row.brake_limit > 0.0 {
                    let target = row.brake_acc - motion.relative_spin(wheel, row) * row.brake_mass;
                    let clamped = max(-row.brake_limit, min(row.brake_limit, target));
                    let delta = clamped - row.brake_acc;
                    if delta != 0.0 {
                        let u0 = motion.relative_spin(wheel, row);
                        motion.apply_brake(wheel, row, delta);
                        row.brake_acc = clamped;
                        let u1 = motion.relative_spin(wheel, row);
                        dissipated[wheel] += (u0 * u0 - u1 * u1) / (2.0 / row.brake_mass);
                    }
                }
                if row.grounded && row.tangent_limit > 0.0 {
                    let u0 = motion.tangent_slip(wheel, row);
                    let target = row.tangent_acc - u0 * row.tangent_mass;
                    let clamped = max(-row.tangent_limit, min(row.tangent_limit, target));
                    let delta = clamped - row.tangent_acc;
                    if delta != 0.0 {
                        motion.apply_tangent(wheel, row, delta);
                        row.tangent_acc = clamped;
                        let u1 = motion.tangent_slip(wheel, row);
                        dissipated[wheel] += (u0 * u0 - u1 * u1) / (2.0 / row.tangent_mass);
                    }
                }
            }
        }
    };

    let energy = |motion: &Motion, body: &mut [f64]| {
        body[10..13].copy_from_slice(&motion.v);
        body[13..16].copy_from_slice(&motion.w);
        kinetic_energy(body, motion.states, params)
    };
    let warm = problem.warm_start;
    solve_rows(&mut motion, rows, &mut dissipated, warm);
    let mut after = energy(&motion, body);
    if warm && after > before * (1.0 + 1e-12) + 1e-9 {
        // A warm start from a changed contact can inject energy; a cold solve cannot.
        motion.v = saved_v;
        motion.w = saved_w;
        for (index, spin) in saved_spins.iter().enumerate().take(count) {
            motion.states[index * STATE_STRIDE] = *spin;
        }
        dissipated.fill(0.0);
        solve_rows(&mut motion, rows, &mut dissipated, false);
        after = energy(&motion, body);
        report[3] = 1.0;
    }
    report[2] = after;

    let (v, w) = (motion.v, motion.w);
    for (wheel, row) in rows.iter().enumerate() {
        let state = wheel * STATE_STRIDE;
        motion.states[state + 2] = if row.grounded { row.tangent_acc } else { 0.0 };
        motion.states[state + 3] = row.brake_acc;
        let t = row.tangent;
        let result = &mut results[wheel * RESULT_STRIDE..(wheel + 1) * RESULT_STRIDE];
        for axis in 0..3 {
            result[axis] = t[axis] * row.tangent_acc;
            result[3 + axis] =
                row.arm_cross_t[axis] * row.tangent_acc - row.spin_axis[axis] * row.brake_acc;
        }
        result[6] = -row.radius * row.tangent_acc + row.brake_acc;
        result[7] = if row.grounded { motion.tangent_slip(wheel, row) } else { 0.0 };
        if let Some(at) = row.contact {
            let c = &problem.contacts[at..at + CONTACT_STRIDE];
            let lateral = vec_at(c, LATERAL);
            result[8] = dot(lateral, v) + dot(cross(vec_at(c, AXLE), lateral), w)
                - dot(vec_at(c, GROUND_VELOCITY), lateral);
            result[9] = c[NORMAL_LOAD] * dt;
        }
        result[10] = dissipated[wheel];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ID_NOSE: f64 = 0.0;
    const ID_LEFT: f64 = 1.0;
    const ID_RIGHT: f64 = 2.0;

    fn body() -> [f64; BODY_LEN] {
        let mut body = [0.0; BODY_LEN];
        body[0] = 1100.0;
        body[1..10].copy_from_slice(&[1.0 / 1300.0, 0.0, 0.0, 0.0, 1.0 / 1800.0, 0.0, 0.0, 0.0, 1.0 / 2600.0]);
        body[10..13].copy_from_slice(&[22.0, 0.3, 0.0]);
        body[13..16].copy_from_slice(&[0.0, 0.02, 0.01]);
        body
    }

    fn contact(id: f64, axle: V, load: f64, epoch: f64) -> [f64; CONTACT_STRIDE] {
        let mut c = [0.0; CONTACT_STRIDE];
        c[WHEEL] = id;
        c[AXLE..AXLE + 3].copy_from_slice(&axle);
        c[NORMAL..NORMAL + 3].copy_from_slice(&[0.0, 0.0, 1.0]);
        c[FORWARD..FORWARD + 3].copy_from_slice(&[1.0, 0.0, 0.0]);
        c[LATERAL..LATERAL + 3].copy_from_slice(&[0.0, 1.0, 0.0]);
        c[CENTER_VELOCITY..CENTER_VELOCITY + 3].copy_from_slice(&[22.0, 0.3, 0.0]);
        c[NORMAL_LOAD] = load;
        c[COMPRESSION] = 0.02;
        c[RADIUS] = 0.22;
        c[GROUND_REVISION] = 3.0;
        c[CONTACT_EPOCH] = epoch;
        c
    }

    fn solve_once(
        body: &mut [f64; BODY_LEN],
        states: &mut [f64],
        contacts: &[f64],
        brakes: &[f64],
        warm_start: bool,
    ) -> ([f64; 3 * RESULT_STRIDE], [f64; REPORT_LEN]) {
        let params = [
            0.052, 0.5, 0.0, 0.2, //
            0.124, 0.5, 450.0, 0.2, //
            0.124, 0.5, 450.0, 0.2,
        ];
        let mut results = [0.0; 3 * RESULT_STRIDE];
        let mut report = [0.0; REPORT_LEN];
        let problem = WheelProblem {
            wheel_ids: &[ID_NOSE, ID_LEFT, ID_RIGHT],
            params: &params,
            contacts,
            brakes,
            dt_seconds: 1.0 / 120.0,
            iterations: f64::NAN,
            warm_start,
        };
        solve(&problem, body, states, &mut results, &mut report);
        (results, report)
    }

    #[test]
    fn braking_dissipates_and_conserves_momentum() {
        let mut body = body();
        let mut states = [0.0, -1.0, 0.0, 0.0, 100.0, -1.0, 0.0, 0.0, 100.0, -1.0, 0.0, 0.0];
        let contacts: Vec<f64> = [
            contact(ID_NOSE, [1.5, 0.0, -1.2], 1500.0, 1.0),
            contact(ID_LEFT, [-0.3, -1.2, -1.2], 4500.0, 1.0),
            contact(ID_RIGHT, [-0.3, 1.2, -1.2], 4500.0, 1.0),
        ]
        .concat();
        let before = body;
        let (results, report) = solve_once(&mut body, &mut states, &contacts, &[0.0, 1.0, 1.0], true);
        assert_eq!(report[0], 8.0);
        assert_eq!(report[4], 0.0);
        assert!(report[2] <= report[1]);
        let dissipated: f64 = results.chunks_exact(RESULT_STRIDE).map(|r| r[10]).sum();
        assert!(dissipated >= 0.0);
        assert!((report[1] - report[2] - dissipated).abs() < 1e-6 * report[1]);
        // Linear momentum change equals the summed tangent impulses.
        for axis in 0..3 {
            let impulse: f64 = results.chunks_exact(RESULT_STRIDE).map(|r| r[axis]).sum();
            let dp = body[0] * (body[10 + axis] - before[10 + axis]);
            assert!((dp - impulse).abs() < 1e-9 * (1.0 + impulse.abs()), "axis {axis}");
        }
        assert!(states[0] > 0.0 && states[1] == 1.0);
    }

    #[test]
    fn rejected_and_airborne_wheels_reset_warm_state() {
        let mut body = body();
        let mut states = [5.0, 7.0, 3.0, 2.0, 0.0, 4.0, 1.0, 1.0, 0.0, -1.0, 0.0, 0.0];
        let mut bad = contact(ID_LEFT, [-0.3, -1.2, -1.2], 4500.0, 4.0);
        bad[RADIUS] = 3.0;
        let (results, report) = solve_once(&mut body, &mut states, &bad, &[0.0; 3], true);
        assert_eq!(report[4], 1.0);
        // Bearing drag still couples an airborne wheel, so only the tangent warm start must clear.
        assert_eq!(&states[1..3], &[-1.0, 0.0]);
        assert_eq!(&states[5..7], &[-1.0, 0.0]);
        assert_eq!(results[RESULT_STRIDE + 9], 0.0);
    }

    #[test]
    fn zero_dt_only_clears_results() {
        let mut body = body();
        let mut states = [1.0, 2.0, 3.0, 4.0];
        let mut results = [9.0; RESULT_STRIDE];
        let mut report = [0.0; REPORT_LEN];
        let problem = WheelProblem {
            wheel_ids: &[ID_NOSE],
            params: &[0.05, 0.5, 0.0, 0.0],
            contacts: &contact(ID_NOSE, [1.0, 0.0, -1.0], 1000.0, 2.0),
            brakes: &[0.0],
            dt_seconds: f64::NAN,
            iterations: 40.0,
            warm_start: true,
        };
        solve(&problem, &mut body, &mut states, &mut results, &mut report);
        assert_eq!(states, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(results, [0.0; RESULT_STRIDE]);
        assert_eq!(report[0], 16.0);
    }
}
