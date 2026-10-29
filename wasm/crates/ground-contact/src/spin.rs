//! One-way wheel spin: `stepWheelSpin` from `src/wheelSpin.ts`, operation for
//! operation, on flat f64 rows.
//!
//! Layouts (f64):
//! - state `[angleRad, omegaRadSec, slipMetersSec, slipPowerWatts, onGround,
//!   compressionMeters, steeringRad, normalLoadNewtons, treadTravelMeters]`
//! - config `[radiusMeters, widthMeters, inertiaKgMetersSquared,
//!   frictionCoefficient, maxBrakeTorqueNewtonMeters]`
//! - input `[onGround, rollMetersSec, normalLoadNewtons, compressionMeters,
//!   steeringRad, brake]`
//!
//! Booleans are 0 or 1.

use crate::jsmath::{exp, max, min, sign};
use core::f64::consts::PI;

pub const STATE_STRIDE: usize = 9;
pub const CONFIG_STRIDE: usize = 5;
pub const INPUT_STRIDE: usize = 6;

const TWO_PI: f64 = 2.0 * PI;
const AIRBORNE_COAST_SECONDS: f64 = 20.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    Instant,
    Inertia,
}

#[inline]
fn bounded(value: f64, low: f64, high: f64) -> f64 {
    if value.is_finite() { min(high, max(low, value)) } else { 0.0 }
}

/// `stepWheelSpin(state, config, input, dt, mode)`. Allocation-free; a
/// non-finite or non-positive `dt` leaves the state untouched.
pub fn step(state: &mut [f64], config: &[f64], input: &[f64], dt: f64, mode: Mode) {
    if !dt.is_finite() || dt <= 0.0 {
        return;
    }
    let on_ground = input[0] != 0.0;
    let seconds = min(dt, 0.25);
    let radius = max(0.01, bounded(config[0], 0.01, 2.0));
    let inertia = max(0.001, bounded(config[2], 0.001, 100.0));
    let speed = bounded(input[1], -200.0, 200.0);
    let target = speed / radius;
    let brake_torque = bounded(input[5], 0.0, 1.0) * bounded(config[4], 0.0, 10_000.0);
    let mut omega = bounded(state[1], -20_000.0, 20_000.0);
    let mut angle = bounded(state[0], -TWO_PI, TWO_PI);
    let start_angle = angle;
    let mut heat = 0.0;

    state[4] = if on_ground { 1.0 } else { 0.0 };
    state[5] = if on_ground { bounded(input[3], 0.0, 1.0) } else { 0.0 };
    state[7] = if on_ground { bounded(input[2], 0.0, 200_000.0) } else { 0.0 };
    state[6] = bounded(input[4], -PI / 2.0, PI / 2.0);

    if !on_ground {
        // Keep angular state through bounces; brakes and drag dissipate it locally.
        let brake_acceleration = brake_torque / inertia;
        let braking_time = if brake_acceleration > 0.0 {
            min(seconds, omega.abs() / brake_acceleration)
        } else {
            seconds
        };
        let braked_omega = sign(omega) * max(0.0, omega.abs() - brake_acceleration * seconds);
        angle += (omega + braked_omega) * braking_time / 2.0;
        omega = braked_omega * exp(-seconds / AIRBORNE_COAST_SECONDS);
    } else if mode == Mode::Instant {
        omega = target;
        angle += omega * seconds;
    } else {
        let friction_force = bounded(config[3], 0.0, 2.0) * bounded(input[2], 0.0, 200_000.0);
        let tire_torque_limit = friction_force * radius;
        let mut remaining = seconds;
        let mut segment = 0;
        while segment < 4 && remaining > 0.0 {
            let tire_torque;
            let brake;
            if omega == target {
                brake = -sign(omega) * brake_torque;
                tire_torque = -sign(brake) * min(brake.abs(), tire_torque_limit);
            } else {
                tire_torque = sign(target - omega) * tire_torque_limit;
                brake = if omega == 0.0 {
                    -sign(tire_torque) * min(tire_torque.abs(), brake_torque)
                } else {
                    -sign(omega) * brake_torque
                };
            }
            let acceleration = (tire_torque + brake) / inertia;
            let mut duration = remaining;
            let mut boundary = None;
            if acceleration != 0.0 {
                let zero_time = -omega / acceleration;
                if zero_time > 0.0 && zero_time <= duration {
                    duration = zero_time;
                    boundary = Some(0.0);
                }
                let rolling_time = (target - omega) / acceleration;
                if rolling_time > 0.0 && rolling_time <= duration {
                    duration = rolling_time;
                    boundary = Some(target);
                }
            }
            let rotation = omega * duration + acceleration * duration * duration / 2.0;
            // Tire force × relative contact travel.
            heat += max(0.0, tire_torque / radius * (speed * duration - radius * rotation));
            angle += rotation;
            omega = boundary.unwrap_or(omega + acceleration * duration);
            remaining -= duration;
            segment += 1;
        }
    }

    state[8] = (angle - start_angle) * radius;
    state[0] = ((angle % TWO_PI) + TWO_PI) % TWO_PI;
    state[1] = omega;
    state[2] = if on_ground && mode == Mode::Inertia { speed - radius * omega } else { 0.0 };
    state[3] = heat / seconds;
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAIN: [f64; CONFIG_STRIDE] = [0.22, 0.152, 0.124, 0.5, 450.0];

    #[test]
    fn spin_up_reaches_rolling_without_overshoot() {
        let mut state = [0.0; STATE_STRIDE];
        let input = [1.0, 20.0, 4000.0, 0.03, 0.0, 0.0];
        for _ in 0..120 {
            step(&mut state, &MAIN, &input, 1.0 / 120.0, Mode::Inertia);
            assert!(state[1] <= 20.0 / 0.22);
            assert!(state[0] >= 0.0 && state[0] < TWO_PI);
        }
        assert_eq!(state[1], 20.0 / 0.22);
        assert_eq!(state[2], 20.0 - 0.22 * (20.0 / 0.22));
        assert_eq!(state[3], 0.0);
    }

    #[test]
    fn airborne_brake_stops_and_coasts() {
        // 450 N·m on 0.124 kg·m² removes about 30 rad/s in one 120 Hz step.
        let mut state = [1.0, 20.0, 0.0, 0.0, 1.0, 0.1, 0.0, 100.0, 0.0];
        let input = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        step(&mut state, &MAIN, &input, 1.0 / 120.0, Mode::Inertia);
        assert_eq!(state[1], 0.0);
        assert_eq!(state[4], 0.0);
        assert_eq!(state[7], 0.0);
        assert!(state[8] > 0.0);
    }

    #[test]
    fn invalid_dt_leaves_state() {
        let mut state = [1.0; STATE_STRIDE];
        for dt in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            step(&mut state, &MAIN, &[1.0; INPUT_STRIDE], dt, Mode::Instant);
            assert_eq!(state, [1.0; STATE_STRIDE]);
        }
    }
}
