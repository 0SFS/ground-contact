//! `createVisibleMeshCollision` from `src/visibleMeshCollision.ts`, the
//! arithmetic in the same order, without the SDK and the surface: the host
//! reads the properties, casts the sweeps and restores JSBSim.
//!
//! [`place`] is the TypeScript `place`: the pose from the 22 property values
//! and each probe's swept point in ECEF, plus this step's sweep from the
//! previous points. The host casts each sweep at least 0.1 mm long against
//! the visible surface and writes what the cast returned; [`resolve`] then
//! takes the hits as `sweep` does (each validated, its normal faced against
//! the motion, ordered by step fraction) and resolves them as `resolve` does,
//! in JSBSim's body axes, feet, slugs and radians: per hit a normal impulse
//! with restitution, a Coulomb friction impulse up to `μ j_n`, then the CG
//! moved out along the normal until the swept point, and the hull point its
//! inset below it, clear the surface by 1 cm. The output is what `update`
//! restores: the new geodetic position, NED velocity and body rates.
//!
//! Layouts (f64):
//! - values: `visibleMeshCollision.ts`'s `PATHS` in order: lat-geod-deg,
//!   long-gc-deg, h-sl-ft, phi, theta, psi (deg), v-north, v-east, v-down
//!   (fps), p, q, r (rad/s), cg x, y, z (in), mass (slugs), ixx, iyy, izz,
//!   ixy, ixz, iyz (slug·ft², JSBSim's property signs)
//! - probe `[x, y, z, inset]` in structural inches (aft, right, up)
//! - pose `[cg ECEF m 3, body-to-local 9, body-to-ECEF 9]`, row-major
//! - points: ECEF metres, 3 per probe, the previous and the current step
//! - segment `[from xyz, movement xyz, distance m]` per probe
//! - hit `[distance m, point ECEF xyz, normal ECEF xyz]` per probe, the
//!   surface's raycast result as returned; a NaN distance is no hit
//! - output `[hits, latDeg, lonDeg, h-sl-ft, vn, ve, vd fps, p, q, r rad/s,
//!   shift ECEF m xyz, body velocity fps xyz]`, then per hit in step order
//!   `[probe, fraction, normal ECEF xyz, approach fps, normal impulse,
//!   friction impulse (slug·ft/s), lift m]`

use crate::geo::{DEG_TO_RAD, FOOT_METERS, FOOTPRINT_E2, INCH_METERS, V3, WGS84_A, all_finite, body_to_local_rotation, cross, ecef_to_geodetic};
use crate::jsmath::{hypot, max, min};

pub const VALUES_LEN: usize = 22;
pub const PROBE_STRIDE: usize = 4;
pub const POSE_LEN: usize = 21;
pub const SEGMENT_STRIDE: usize = 7;
pub const HIT_STRIDE: usize = 7;
pub const HEAD_LEN: usize = 16;
pub const ROW_STRIDE: usize = 9;
pub const MAX_PROBES: usize = 64;

const CONTACT_CLEARANCE_METERS: f64 = 0.01;
const RESTITUTION: f64 = 0.25;
const HULL_FRICTION: f64 = 0.5;
const FEET_PER_METER: f64 = 1.0 / FOOT_METERS;
const FEET_PER_INCH: f64 = 1.0 / 12.0;
/// Sweeps shorter than this are not cast.
pub const MIN_SWEEP_METERS: f64 = 0.0001;

const LAT: usize = 0;
const LON: usize = 1;
const ALT: usize = 2;
const ROLL: usize = 3;
const PITCH: usize = 4;
const HEADING: usize = 5;
const NORTH: usize = 6;
const EAST: usize = 7;
const DOWN: usize = 8;
const P: usize = 9;
const CG_X: usize = 12;
const CG_Y: usize = 13;
const CG_Z: usize = 14;
const MASS: usize = 15;
const IXX: usize = 16;
const IYY: usize = 17;
const IZZ: usize = 18;
const IXY: usize = 19;
const IXZ: usize = 20;
const IYZ: usize = 21;

const CG: usize = 0;
const BODY_TO_LOCAL: usize = 3;
const BODY_TO_ECEF: usize = 12;

/// A probe's swept point from the live CG, in body feet: forward, right, down.
#[inline]
fn probe_lever(values: &[f64], probe: &[f64]) -> V3 {
    [
        (values[CG_X] - probe[0]) * FEET_PER_INCH,
        (probe[1] - values[CG_Y]) * FEET_PER_INCH,
        (values[CG_Z] - probe[2] - probe[3]) * FEET_PER_INCH,
    ]
}

/// `place(current)`, then each probe's sweep from `previous`. False, with
/// nothing written, when a value is not finite.
pub fn place(values: &[f64], probes: &[f64], previous: &[f64], pose: &mut [f64], current: &mut [f64], segments: &mut [f64]) -> bool {
    if !all_finite(values) {
        return false;
    }
    let lat = values[LAT] * DEG_TO_RAD;
    let lon = values[LON] * DEG_TO_RAD;
    let alt = values[ALT] * FOOT_METERS;
    let (sin_lat, cos_lat, sin_lon, cos_lon) = (lat.sin(), lat.cos(), lon.sin(), lon.cos());
    let n = WGS84_A / (1.0 - FOOTPRINT_E2 * sin_lat * sin_lat).sqrt();
    pose[CG] = (n + alt) * cos_lat * cos_lon;
    pose[CG + 1] = (n + alt) * cos_lat * sin_lon;
    pose[CG + 2] = (n * (1.0 - FOOTPRINT_E2) + alt) * sin_lat;
    let body_to_local = body_to_local_rotation(values[ROLL] * DEG_TO_RAD, values[PITCH] * DEG_TO_RAD, values[HEADING] * DEG_TO_RAD);
    pose[BODY_TO_LOCAL..BODY_TO_LOCAL + 9].copy_from_slice(&body_to_local);
    // Columns of the local frame in ECEF: north, east, down.
    let (nx, ny, nz) = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    let (ex, ey) = (-sin_lon, cos_lon);
    let (dx, dy, dz) = (-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat);
    for column in 0..3 {
        let (north, east, down) = (body_to_local[column], body_to_local[3 + column], body_to_local[6 + column]);
        pose[BODY_TO_ECEF + column] = nx * north + ex * east + dx * down;
        pose[BODY_TO_ECEF + 3 + column] = ny * north + ey * east + dy * down;
        pose[BODY_TO_ECEF + 6 + column] = nz * north + dz * down;
    }
    let m = &pose[BODY_TO_ECEF..BODY_TO_ECEF + 9];
    for (i, probe) in probes.chunks_exact(PROBE_STRIDE).enumerate() {
        let lever = probe_lever(values, probe);
        for axis in 0..3 {
            let row = axis * 3;
            current[i * 3 + axis] = pose[CG + axis] + (m[row] * lever[0] + m[row + 1] * lever[1] + m[row + 2] * lever[2]) * FOOT_METERS;
        }
        let segment = &mut segments[i * SEGMENT_STRIDE..(i + 1) * SEGMENT_STRIDE];
        for axis in 0..3 {
            segment[axis] = previous[i * 3 + axis];
            segment[3 + axis] = current[i * 3 + axis] - segment[axis];
        }
        segment[6] = (segment[3] * segment[3] + segment[4] * segment[4] + segment[5] * segment[5]).sqrt();
    }
    true
}

/// JSBSim's inertia tensor about the CG in body axes, inverted; zero when singular.
fn invert_inertia(values: &[f64]) -> [f64; 9] {
    // The property getters return -J12, +J13 and -J23 of the native tensor.
    let (a, b, c) = (values[IXX], -values[IXY], values[IXZ]);
    let (d, e, f) = (values[IYY], -values[IYZ], values[IZZ]);
    let (c00, c01, c02) = (d * f - e * e, c * e - b * f, b * e - c * d);
    let determinant = a * c00 + b * c01 + c * c02;
    if !(determinant.abs() > 0.0) || !determinant.is_finite() {
        return [0.0; 9];
    }
    let scale = 1.0 / determinant;
    [
        c00 * scale,
        c01 * scale,
        c02 * scale,
        c01 * scale,
        (a * f - c * c) * scale,
        (b * c - a * e) * scale,
        c02 * scale,
        (b * c - a * e) * scale,
        (a * d - b * b) * scale,
    ]
}

/// The rigid body while impacts are resolved: body-axis feet, slugs and radians.
struct Body {
    velocity: V3,
    rate: V3,
    inverse_inertia: [f64; 9],
    mass: f64,
}

impl Body {
    fn rotate_by_inverse_inertia(&self, v: V3) -> V3 {
        let m = &self.inverse_inertia;
        [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
        ]
    }
    /// Velocity of the point at `lever` along `direction`: (v + ω × r) · d.
    fn point_speed(&self, lever: V3, direction: V3) -> f64 {
        let turn = cross(self.rate, lever);
        let v = self.velocity;
        (v[0] + turn[0]) * direction[0] + (v[1] + turn[1]) * direction[1] + (v[2] + turn[2]) * direction[2]
    }
    /// 1 / m_eff along `direction` at `lever`: 1/m + (r × d)ᵀ I⁻¹ (r × d).
    fn inverse_effective_mass(&self, lever: V3, direction: V3) -> f64 {
        let arm = cross(lever, direction);
        let turn = self.rotate_by_inverse_inertia(arm);
        1.0 / self.mass + arm[0] * turn[0] + arm[1] * turn[1] + arm[2] * turn[2]
    }
    /// The impulse `magnitude` along `direction` at `lever`: Δv = J/m, Δω = I⁻¹ (r × J).
    fn apply_impulse(&mut self, lever: V3, direction: V3, magnitude: f64) {
        for (v, d) in self.velocity.iter_mut().zip(direction) {
            *v += d * magnitude / self.mass;
        }
        let turn = self.rotate_by_inverse_inertia(cross(lever, direction));
        for (w, t) in self.rate.iter_mut().zip(turn) {
            *w += t * magnitude;
        }
    }
}

/// `sweep` over the host's hits, then `resolve` and the restored state as
/// `update` computes it. `restitution` and `friction` are as requested; a
/// non-finite one takes the default (0.25, 0.5). Returns the hit count; with
/// none, only `output[0]` is written.
#[allow(clippy::too_many_arguments)]
pub fn resolve(
    values: &[f64],
    probes: &[f64],
    pose: &[f64],
    previous: &[f64],
    current: &[f64],
    hits: &[f64],
    restitution: f64,
    friction: f64,
    output: &mut [f64],
) -> usize {
    // This step's hits, one at most per probe, in the order they occur along the step.
    let mut hit_probe = [0usize; MAX_PROBES];
    let mut hit_fraction = [0.0; MAX_PROBES];
    let mut hit_point = [[0.0; 3]; MAX_PROBES];
    let mut hit_normal = [[0.0; 3]; MAX_PROBES];
    let mut count = 0;
    for (i, hit) in hits.chunks_exact(HIT_STRIDE).take(probes.len() / PROBE_STRIDE).enumerate() {
        let from = &previous[i * 3..i * 3 + 3];
        let movement = [current[i * 3] - from[0], current[i * 3 + 1] - from[1], current[i * 3 + 2] - from[2]];
        let distance = (movement[0] * movement[0] + movement[1] * movement[1] + movement[2] * movement[2]).sqrt();
        if !distance.is_finite() || distance < MIN_SWEEP_METERS {
            continue;
        }
        let at_distance = hit[0];
        if !at_distance.is_finite() || at_distance < 0.0 || at_distance > distance {
            continue;
        }
        let normal = [hit[4], hit[5], hit[6]];
        let length = hypot(&normal);
        if !length.is_finite() || length < 1e-10 || !all_finite(&hit[1..4]) {
            continue;
        }
        // Face the normal against the probe's motion, whichever way the mesh is wound.
        let sign = if movement[0] * normal[0] + movement[1] * normal[1] + movement[2] * normal[2] > 0.0 { -1.0 } else { 1.0 };
        let fraction = at_distance / distance;
        let mut at = count;
        count += 1;
        while at > 0 && hit_fraction[at - 1] > fraction {
            hit_fraction[at] = hit_fraction[at - 1];
            hit_probe[at] = hit_probe[at - 1];
            hit_point[at] = hit_point[at - 1];
            hit_normal[at] = hit_normal[at - 1];
            at -= 1;
        }
        hit_fraction[at] = fraction;
        hit_probe[at] = i;
        hit_point[at] = [hit[1], hit[2], hit[3]];
        hit_normal[at] = [sign * normal[0] / length, sign * normal[1] / length, sign * normal[2] / length];
    }
    output[0] = count as f64;
    if count == 0 {
        return 0;
    }

    let restitution = if restitution.is_finite() { min(2.0, max(0.0, restitution)) } else { RESTITUTION };
    let friction = if friction.is_finite() { max(0.0, friction) } else { HULL_FRICTION };
    let body_to_local = &pose[BODY_TO_LOCAL..BODY_TO_LOCAL + 9];
    let body_to_ecef = &pose[BODY_TO_ECEF..BODY_TO_ECEF + 9];
    // Body velocity from the ground-relative NED velocity: v_body = Mᵀ v_local.
    let mut velocity = [0.0; 3];
    for (axis, v) in velocity.iter_mut().enumerate() {
        *v = body_to_local[axis] * values[NORTH] + body_to_local[3 + axis] * values[EAST] + body_to_local[6 + axis] * values[DOWN];
    }
    let mut body = Body {
        velocity,
        rate: [values[P], values[P + 1], values[P + 2]],
        inverse_inertia: invert_inertia(values),
        mass: values[MASS],
    };
    let mut shift = [0.0; 3];
    for k in 0..count {
        let i = hit_probe[k];
        let probe = &probes[i * PROBE_STRIDE..(i + 1) * PROBE_STRIDE];
        let lever = probe_lever(values, probe);
        let [nx, ny, nz] = hit_normal[k];
        // The ECEF normal in body axes.
        let mut normal = [0.0; 3];
        for (axis, n) in normal.iter_mut().enumerate() {
            *n = body_to_ecef[axis] * nx + body_to_ecef[3 + axis] * ny + body_to_ecef[6 + axis] * nz;
        }
        let approach = body.point_speed(lever, normal);
        let (mut normal_impulse, mut friction_impulse) = (0.0, 0.0);
        if approach < 0.0 && body.mass > 0.0 {
            normal_impulse = -(1.0 + restitution) * approach / body.inverse_effective_mass(lever, normal);
            body.apply_impulse(lever, normal, normal_impulse);
            // Coulomb friction opposes the point's sliding after the normal impulse, up to mu j_n.
            let along = body.point_speed(lever, normal);
            let turn = cross(body.rate, lever);
            let mut tangent = [0.0; 3];
            for (axis, t) in tangent.iter_mut().enumerate() {
                *t = body.velocity[axis] + turn[axis] - along * normal[axis];
            }
            let sliding = hypot(&tangent);
            if sliding > 1e-9 {
                for t in &mut tangent {
                    *t /= -sliding;
                }
                friction_impulse = min(sliding / body.inverse_effective_mass(lever, tangent), friction * normal_impulse);
                body.apply_impulse(lever, tangent, friction_impulse);
            }
        }
        // Out of the surface: the swept point, and the hull point it sits inside, clear of it.
        let point = hit_point[k];
        let depth = (point[0] - current[i * 3] - shift[0]) * nx
            + (point[1] - current[i * 3 + 1] - shift[1]) * ny
            + (point[2] - current[i * 3 + 2] - shift[2]) * nz;
        // The hull point lies the inset along body down, the third column of body-to-ECEF.
        let inset_meters = probe[3] * INCH_METERS;
        let hull_depth = depth - inset_meters * (body_to_ecef[2] * nx + body_to_ecef[5] * ny + body_to_ecef[8] * nz);
        let lift = max(depth, hull_depth) + CONTACT_CLEARANCE_METERS;
        if lift > 0.0 {
            shift[0] += lift * nx;
            shift[1] += lift * ny;
            shift[2] += lift * nz;
        }
        output[HEAD_LEN + k * ROW_STRIDE..HEAD_LEN + (k + 1) * ROW_STRIDE].copy_from_slice(&[
            i as f64,
            hit_fraction[k],
            nx,
            ny,
            nz,
            approach,
            normal_impulse,
            friction_impulse,
            lift,
        ]);
    }

    let [lat, lon, alt] = ecef_to_geodetic([pose[CG] + shift[0], pose[CG + 1] + shift[1], pose[CG + 2] + shift[2]]);
    // The velocity in ECEF, then in the local frame where the CG now is.
    let v = body.velocity;
    let m = body_to_ecef;
    let vx = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    let vy = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    let vz = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
    let (sin_lat, cos_lat) = (lat.sin(), lat.cos());
    let (sin_lon, cos_lon) = (lon.sin(), lon.cos());
    output[1..HEAD_LEN].copy_from_slice(&[
        lat / DEG_TO_RAD,
        lon / DEG_TO_RAD,
        alt * FEET_PER_METER,
        -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz,
        -sin_lon * vx + cos_lon * vy,
        -cos_lat * cos_lon * vx - cos_lat * sin_lon * vy - sin_lat * vz,
        body.rate[0],
        body.rate[1],
        body.rate[2],
        shift[0],
        shift[1],
        shift[2],
        v[0],
        v[1],
        v[2],
    ]);
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    const C172_PROBES: [f64; 20] = [
        -58.7, 0.0, 34.6, 0.0, 42.0, 0.0, 11.9, 3.0, 255.9, 0.0, 35.5, 3.0, 53.7, -216.5, 68.5, 3.0, 53.7, 216.5, 68.5, 3.0,
    ];

    fn values() -> [f64; VALUES_LEN] {
        [
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 30.0, 0.0, 10.0, 0.0, 0.0, 0.0, 41.0, 0.0, 36.5, 70.0, 948.0, 1346.0, 1967.0,
            0.0, 0.0, 0.0,
        ]
    }

    fn placed(values: &[f64; VALUES_LEN]) -> ([f64; POSE_LEN], [f64; 15], [f64; 35]) {
        let (mut pose, mut current, mut segments) = ([0.0; POSE_LEN], [0.0; 15], [0.0; 35]);
        assert!(place(values, &C172_PROBES, &[0.0; 15], &mut pose, &mut current, &mut segments));
        (pose, current, segments)
    }

    #[test]
    fn place_rejects_non_finite_values() {
        let mut v = values();
        v[IYZ] = f64::NAN;
        let (mut pose, mut current, mut segments) = ([0.0; POSE_LEN], [0.0; 15], [0.0; 35]);
        assert!(!place(&v, &C172_PROBES, &[0.0; 15], &mut pose, &mut current, &mut segments));
        assert_eq!(pose, [0.0; POSE_LEN]);
    }

    #[test]
    fn probes_sit_on_the_body_axes() {
        // Level, heading north, at the equator and the prime meridian: ECEF x
        // is up, y east and z north; body forward is north and down is -x.
        let (pose, current, _) = placed(&values());
        let cg = [pose[0], pose[1], pose[2]];
        // The nose is 99.7 in ahead of the CG and 1.9 in below it.
        assert!((current[2] - cg[2] - 99.7 * INCH_METERS).abs() < 1e-9);
        assert!((current[0] - cg[0] + 1.9 * INCH_METERS).abs() < 1e-9);
        // The belly's swept point is its inset above the hull point.
        assert!((current[3] - cg[0] - (11.9 + 3.0 - 36.5) * INCH_METERS).abs() < 1e-9);
        // The right wingtip is east.
        assert!((current[13] - cg[1] - 216.5 * INCH_METERS).abs() < 1e-9);
    }

    #[test]
    fn a_belly_strike_stops_the_sink_and_lifts_the_hull_point_clear() {
        let mut v = values();
        // The CG straight above the belly probe.
        v[CG_X] = 42.0;
        let (pose, current, _) = placed(&v);
        let mut previous = current;
        // The belly came down 5 cm onto ground 2 cm above its swept point.
        previous[3] += 0.05;
        let mut hits = [f64::NAN; 5 * HIT_STRIDE];
        let ground = current[3] + 0.02;
        hits[HIT_STRIDE..2 * HIT_STRIDE].copy_from_slice(&[0.03, ground, current[4], current[5], 1.0, 0.0, 0.0]);
        let mut output = [0.0; HEAD_LEN + 5 * ROW_STRIDE];
        assert_eq!(resolve(&v, &C172_PROBES, &pose, &previous, &current, &hits, 0.0, 0.0, &mut output), 1);
        assert_eq!(output[HEAD_LEN], 1.0);
        // Inelastic, frictionless, under the CG: the CG stops sinking.
        assert!(output[15].abs() < 1e-12, "{}", output[15]);
        assert_eq!(&output[7..10], &[0.0; 3]);
        // The hull point, 3 in below the swept point, ends 1 cm above the ground.
        let lift = output[HEAD_LEN + 8];
        assert!((lift - (0.02 + 3.0 * INCH_METERS + 0.01)).abs() < 1e-9, "{lift}");
        assert!((output[10] - lift).abs() < 1e-12);
    }

    #[test]
    fn hits_resolve_in_step_order_and_bad_hits_are_dropped() {
        let v = values();
        let (pose, current, _) = placed(&v);
        let mut previous = current;
        for p in previous.iter_mut().step_by(3) {
            *p += 0.1;
        }
        let mut hits = [f64::NAN; 5 * HIT_STRIDE];
        let hit = |probe: usize, distance: f64, nx: f64| [distance, current[probe * 3], current[probe * 3 + 1], current[probe * 3 + 2], nx, 0.0, 0.0];
        hits[..7].copy_from_slice(&hit(0, 0.08, -2.0));
        hits[7..14].copy_from_slice(&hit(1, 0.02, 2.0));
        hits[14..21].copy_from_slice(&hit(2, 0.11, 2.0)); // past the sweep
        hits[21..28].copy_from_slice(&[0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-11]); // no normal
        hits[28..35].copy_from_slice(&hit(4, 0.05, 2.0));
        let mut output = [0.0; HEAD_LEN + 5 * ROW_STRIDE];
        assert_eq!(resolve(&v, &C172_PROBES, &pose, &previous, &current, &hits, f64::NAN, f64::NAN, &mut output), 3);
        let order: Vec<f64> = (0..3).map(|k| output[HEAD_LEN + k * ROW_STRIDE]).collect();
        assert_eq!(order, [1.0, 4.0, 0.0]);
        // Normals are unit; one along the motion is turned to face it.
        assert_eq!(output[HEAD_LEN + 2], 1.0);
        assert_eq!(output[HEAD_LEN + 2 * ROW_STRIDE + 2], 1.0);
    }

    #[test]
    fn jsbsim_product_signs() {
        let mut v = values();
        v[IXZ] = 50.0;
        v[IXY] = 7.0;
        let inverse = invert_inertia(&v);
        // I = [[ixx, -ixy, ixz], [-ixy, iyy, -iyz], [ixz, -iyz, izz]] times its inverse is the identity.
        let i = [v[IXX], -v[IXY], v[IXZ], -v[IXY], v[IYY], -v[IYZ], v[IXZ], -v[IYZ], v[IZZ]];
        for r in 0..3 {
            for c in 0..3 {
                let e: f64 = (0..3).map(|k| i[r * 3 + k] * inverse[k * 3 + c]).sum();
                assert!((e - if r == c { 1.0 } else { 0.0 }).abs() < 1e-12);
            }
        }
        v[IXX] = 0.0;
        v[IXZ] = 0.0;
        v[IXY] = 0.0;
        assert_eq!(invert_inertia(&v), [0.0; 9]);
    }
}
