//! `createContactSupport` from `src/terrainContact.ts`: the ground under each
//! contact unit and the one horizontal plane JSBSim is given for all of them,
//! the arithmetic in the same order, with `createWheelGroundFilter` from
//! `src/wheelGroundFilter.ts` per contact.
//!
//! `measure(input)` is split where it samples the surface, which only the
//! host can. Which spots it samples, and in what order, never depends on what
//! a sample returns, so:
//!
//! 1. [`plan`] runs `measure` up to the samples: the refinement carried onto
//!    each held sample, the footprints from the property values (when the
//!    aircraft is within reach of the ground), and the requests
//!    `sampleFootprints` would make, in its order.
//! 2. The host samples the surface at each request.
//! 3. [`finish`] applies the samples in that order and completes `measure`:
//!    the previous plane under the refined world, then this step's plane.
//!
//! The host owns what touches the SDK: it discovers the units
//! (`groundContactUnits`), reads the properties, and tells [`plan`] whether
//! it read them ([`GEOMETRY_READ`]), the read threw ([`GEOMETRY_FAILED`],
//! which forgets the contacts) or there are no units to read
//! ([`GEOMETRY_UNAVAILABLE`]). It need not read them when
//! `altitude - cgGround > 10 + reach + 1` (the state's [`REACH`]); the
//! kernel then ignores them, as `measure` skips the read.
//!
//! Layouts (f64):
//! - unit `[retractable, wheel]`, 0 or 1 each: a retractable BOGEY; a BOGEY
//!   (`gear/unit`) rather than a STRUCTURE (`contact/unit`)
//! - input: `SupportInput` `[lat, lon deg, altitude m, roll, pitch, heading
//!   rad, cgGround m, revision, revisionChanged, cgDelta m, urgent, snap,
//!   samples, previousElevation m]`, flags 0 or 1, samples possibly infinite
//! - values: `[gear/num-units, gear/gear-pos-norm, cg x, y, z in]`, then
//!   `[x, y, z in]` per unit, the properties `readGeometry` reads
//! - state: [`GLOBALS`] values, then [`CONTACT_STRIDE`] per contact, then the
//!   plan, `[contact, kind]` per request; carried between steps
//! - request `[contact, kind, latDeg, lonDeg, heightMeters, revision]`: the
//!   kernel writes the first four, the host the sample's height (NaN for no
//!   sample) and revision; at most two per contact
//! - output `[elevation m (NaN: the centre's ground stands in), refinement m,
//!   lowest contact depth m, in range, requests, samples applied]`

use crate::geo::{body_offset_local, body_to_local_rotation, contact_body_position, metres_per_degree, wrap_lon};
use crate::jsmath::{exp, hypot, max, min};

pub const UNIT_STRIDE: usize = 2;
pub const INPUT_LEN: usize = 14;
pub const VALUES_HEAD: usize = 5;
pub const REQUEST_STRIDE: usize = 6;
pub const OUTPUT_LEN: usize = 6;
pub const MAX_CONTACTS: usize = 64;
pub const GROUND_FILTER_LENGTH_METERS: f64 = 0.29;

pub const GEOMETRY_READ: u32 = 0;
pub const GEOMETRY_FAILED: u32 = 1;
pub const GEOMETRY_UNAVAILABLE: u32 = 2;

const GEAR_DOWN_POSITION: f64 = 0.99;
const WHEELS_CLEAR_METERS: f64 = 5.0;
const FOOTPRINT_RANGE_METERS: f64 = 10.0;
const FOOTPRINT_REACH_METERS: f64 = 1.0;
const FOOTPRINT_RESAMPLE_METERS: f64 = 0.05;
const FOOTPRINT_REVALIDATE_STEPS: f64 = 30.0;

pub const SAMPLE_FIRST: f64 = 4.0;
pub const SAMPLE_URGENT: f64 = 3.0;
pub const SAMPLE_MOVED: f64 = 2.0;
pub const SAMPLE_LAZY: f64 = 1.0;

// Input.
const LAT: usize = 0;
const LON: usize = 1;
const ALTITUDE: usize = 2;
const ROLL: usize = 3;
const PITCH: usize = 4;
const HEADING: usize = 5;
const CG_GROUND: usize = 6;
const REVISION: usize = 7;
const REVISION_CHANGED: usize = 8;
const CG_DELTA: usize = 9;
const URGENT: usize = 10;
const SNAP: usize = 11;
const SAMPLES: usize = 12;
const PREVIOUS_ELEVATION: usize = 13;
pub const INPUT_FLAGS: [usize; 3] = [REVISION_CHANGED, URGENT, SNAP];

// State: the closure's variables, then this step's between plan and finish.
/// 1 while the contacts are discovered: `reader` exists and `count` is the unit count.
pub const DISCOVERED: usize = 0;
pub const LIVE: usize = 1;
pub const HAD_SUPPORT: usize = 2;
pub const PREV_CG_GROUND: usize = 3;
pub const PREV_LOWEST: usize = 4;
pub const PREV_LEVEL: usize = 5;
pub const REVALIDATE_IN: usize = 6;
pub const STEPS: usize = 7;
/// `reachMeters`: the farthest contact from the CG.
pub const REACH: usize = 8;
/// 1 between [`plan`] and [`finish`].
pub const PHASE: usize = 9;
pub const LOWEST: usize = 10;
pub const IN_RANGE: usize = 11;
pub const REFINEMENT: usize = 12;
pub const REQUESTS: usize = 13;
pub const SCALE_LAT: usize = 14;
pub const SCALE_LON: usize = 15;
/// The input [`plan`] was given, which [`finish`] uses.
pub const INPUT: usize = 16;
pub const GLOBALS: usize = INPUT + INPUT_LEN;

// Per contact: its filter, then `createContactSupport`'s arrays.
pub const FILTER: usize = 0;
pub const DOWN: usize = 2;
pub const FOOT_LAT: usize = 3;
pub const FOOT_LON: usize = 4;
pub const HELD_LAT: usize = 5;
pub const HELD_LON: usize = 6;
pub const HELD_RAW: usize = 7;
pub const HELD_REVISION: usize = 8;
pub const CHECKED: usize = 9;
pub const PROVISIONAL: usize = 10;
pub const GROUND: usize = 11;
pub const MARGIN: usize = 12;
pub const MOVED: usize = 13;
pub const SHIFT: usize = 14;
pub const WANTS: usize = 15;
pub const RANK: usize = 16;
pub const PREV_GROUND: usize = 17;
pub const PREV_DOWN: usize = 18;
pub const ACTIVE: usize = 19;
pub const SUPPORTED: usize = 20;
pub const PREV_SUPPORTED: usize = 21;
pub const PREV_SAMPLED: usize = 22;
pub const URGENT_FLAG: usize = 23;
pub const CONTACT_STRIDE: usize = 24;

pub const fn state_len(contacts: usize) -> usize {
    GLOBALS + contacts * (CONTACT_STRIDE + 4)
}

/// `createWheelGroundFilter(length).height(raw, moved, snap)` on `[has, filtered]`.
#[inline]
pub fn filter_height(filter: &mut [f64], raw: f64, moved: f64, snap: bool, length: f64) -> f64 {
    let max_bridged = length * 2.0;
    if filter[0] == 0.0 || snap || !moved.is_finite() || (raw - filter[1]).abs() > max_bridged {
        filter[0] = 1.0;
        filter[1] = raw;
        return raw;
    }
    filter[1] += (raw - filter[1]) * (1.0 - exp(-max(moved, 0.0) / length));
    filter[1]
}

/// `shift(delta)`.
#[inline]
pub fn filter_shift(filter: &mut [f64], delta: f64) {
    if filter[0] != 0.0 && delta.is_finite() {
        filter[1] += delta;
    }
}

/// The state split into the globals, the contacts and the plan.
fn split(state: &mut [f64], count: usize) -> (&mut [f64], &mut [f64], &mut [f64]) {
    let (globals, rest) = state.split_at_mut(GLOBALS);
    let (contacts, plan) = rest.split_at_mut(count * CONTACT_STRIDE);
    (globals, contacts, plan)
}

fn rows(contacts: &mut [f64]) -> core::slice::ChunksExactMut<'_, f64> {
    contacts.chunks_exact_mut(CONTACT_STRIDE)
}

/// The arrays `discover` allocates.
fn fresh(contacts: &mut [f64]) {
    for row in rows(contacts) {
        row.fill(0.0);
        row[HELD_RAW] = f64::NAN;
        row[GROUND] = f64::NAN;
    }
}

fn clear_contacts(globals: &mut [f64], contacts: &mut [f64]) {
    if globals[LIVE] == 0.0 {
        return;
    }
    globals[LIVE] = 0.0;
    globals[HAD_SUPPORT] = 0.0;
    for row in rows(contacts) {
        row[FILTER] = 0.0;
        row[HELD_RAW] = f64::NAN;
        row[GROUND] = f64::NAN;
        for field in [PROVISIONAL, MARGIN, URGENT_FLAG, SUPPORTED, PREV_SUPPORTED, PREV_SAMPLED] {
            row[field] = 0.0;
        }
    }
}

fn forget(globals: &mut [f64], contacts: &mut [f64]) {
    clear_contacts(globals, contacts);
    globals[DISCOVERED] = 0.0;
    globals[REACH] = f64::INFINITY;
}

/// `reset()`; also the state of a new kernel.
pub fn reset(state: &mut [f64], count: usize) {
    let (globals, contacts, _) = split(state, count);
    forget(globals, contacts);
    globals[REVALIDATE_IN] = 0.0;
    globals[PHASE] = 0.0;
}

/// `readGeometry`: the footprints; returns the lowest active contact's depth below the centre.
fn read_geometry(units: &[f64], globals: &mut [f64], contacts: &mut [f64], input: &[f64], values: &[f64], geometry: u32) -> f64 {
    if geometry == GEOMETRY_UNAVAILABLE {
        return f64::NAN;
    }
    if globals[DISCOVERED] == 0.0 {
        fresh(contacts);
        globals[DISCOVERED] = 1.0;
    }
    let count = units.len() / UNIT_STRIDE;
    // A model load disposes the native batch, or changes the units; find the contacts again.
    if geometry == GEOMETRY_FAILED || values[0] != count as f64 {
        forget(globals, contacts);
        return f64::NAN;
    }
    let gear_down = values[1] > GEAR_DOWN_POSITION;
    let cg = [values[2], values[3], values[4]];
    let rotation = body_to_local_rotation(input[ROLL], input[PITCH], input[HEADING]);
    let [scale_lat, scale_lon] = metres_per_degree(input[LAT]);
    globals[SCALE_LAT] = scale_lat;
    globals[SCALE_LON] = scale_lon;
    let (mut lowest, mut reach) = (f64::NEG_INFINITY, 0.0);
    for (i, row) in rows(contacts).enumerate() {
        let at = VALUES_HEAD + i * 3;
        let body = contact_body_position([values[at], values[at + 1], values[at + 2]], cg);
        let [north, east, down] = body_offset_local(body, &rotation);
        reach = max(reach, hypot(&body));
        row[DOWN] = down;
        row[FOOT_LAT] = input[LAT] + north / scale_lat;
        row[FOOT_LON] = wrap_lon(input[LON] + east / scale_lon);
        // Native BOGEY support exists only above the gear-down position for
        // retractable units; fixed units and structures always participate.
        let retractable = units[i * UNIT_STRIDE] != 0.0;
        let on = !(retractable && !gear_down) && down.is_finite() && row[FOOT_LAT].is_finite() && row[FOOT_LON].is_finite();
        if !on && row[ACTIVE] != 0.0 {
            row[FILTER] = 0.0;
            row[HELD_RAW] = f64::NAN;
            row[GROUND] = f64::NAN;
            row[PROVISIONAL] = 0.0;
            row[URGENT_FLAG] = 0.0;
        }
        row[ACTIVE] = if on { 1.0 } else { 0.0 };
        if on {
            lowest = max(lowest, down);
        }
    }
    globals[REACH] = if reach.is_finite() { reach } else { f64::INFINITY };
    lowest
}

/// Travel after which a contact is sampled again: farther the less it matters.
#[inline]
fn spacing(row: &[f64]) -> f64 {
    FOOTPRINT_RESAMPLE_METERS + min(max(row[MARGIN], 0.0), FOOTPRINT_REACH_METERS)
}

/// `sampleFootprints` up to the samples: the requests in order, into the plan
/// and the request rows. Returns their count.
fn plan_samples(globals: &mut [f64], contacts: &mut [f64], plan: &mut [f64], input: &[f64], requests: &mut [f64]) -> usize {
    let lazy = globals[REVALIDATE_IN] <= 0.0;
    globals[REVALIDATE_IN] -= 1.0;
    globals[STEPS] += 1.0;
    let steps = globals[STEPS];
    let (scale_lat, scale_lon) = (globals[SCALE_LAT], globals[SCALE_LON]);
    for row in rows(contacts) {
        row[WANTS] = 0.0;
        if row[ACTIVE] == 0.0 {
            continue;
        }
        if row[HELD_RAW].is_nan() {
            row[WANTS] = SAMPLE_FIRST;
            row[RANK] = 0.0;
            continue;
        }
        row[MOVED] = hypot(&[(row[FOOT_LAT] - row[HELD_LAT]) * scale_lat, wrap_lon(row[FOOT_LON] - row[HELD_LON]) * scale_lon]);
        let stale = row[HELD_REVISION] != input[REVISION];
        if stale && row[URGENT_FLAG] != 0.0 {
            // The contact nearest to deciding the support first.
            row[WANTS] = SAMPLE_URGENT;
            row[RANK] = -row[MARGIN];
        } else if row[MOVED] >= spacing(row) {
            // The most overdue first.
            row[WANTS] = SAMPLE_MOVED;
            row[RANK] = row[MOVED] / spacing(row);
        } else if stale && lazy {
            // The longest unconfirmed first.
            row[WANTS] = SAMPLE_LAZY;
            row[RANK] = steps - row[CHECKED];
        }
    }
    let count = contacts.len() / CONTACT_STRIDE;
    let mut planned = 0;
    let mut allowance = input[SAMPLES];
    while allowance > 0.0 {
        let mut next: Option<usize> = None;
        for i in 0..count {
            let (wants, rank) = (contacts[i * CONTACT_STRIDE + WANTS], contacts[i * CONTACT_STRIDE + RANK]);
            let better = match next {
                None => true,
                Some(n) => {
                    let (best, best_rank) = (contacts[n * CONTACT_STRIDE + WANTS], contacts[n * CONTACT_STRIDE + RANK]);
                    wants > best || (wants == best && rank > best_rank)
                }
            };
            if wants != 0.0 && better {
                next = Some(i);
            }
        }
        let Some(next) = next else { break };
        let kind = contacts[next * CONTACT_STRIDE + WANTS];
        contacts[next * CONTACT_STRIDE + WANTS] = 0.0;
        let (lat, lon) = if kind == SAMPLE_URGENT || kind == SAMPLE_LAZY {
            if kind == SAMPLE_LAZY {
                globals[REVALIDATE_IN] = FOOTPRINT_REVALIDATE_STEPS;
                for row in rows(contacts) {
                    if row[WANTS] == SAMPLE_LAZY {
                        row[WANTS] = 0.0;
                    }
                }
            }
            let row = &mut contacts[next * CONTACT_STRIDE..(next + 1) * CONTACT_STRIDE];
            // Then follow the contact to where it is now, if there is a sample to spare.
            if row[MOVED] >= spacing(row) {
                row[WANTS] = SAMPLE_MOVED;
                row[RANK] = row[MOVED] / spacing(row);
            }
            // The refinement is measured where the sample was taken.
            (row[HELD_LAT], row[HELD_LON])
        } else {
            (contacts[next * CONTACT_STRIDE + FOOT_LAT], contacts[next * CONTACT_STRIDE + FOOT_LON])
        };
        plan[planned * 2] = next as f64;
        plan[planned * 2 + 1] = kind;
        requests[planned * REQUEST_STRIDE..(planned + 1) * REQUEST_STRIDE].copy_from_slice(&[next as f64, kind, lat, lon, f64::NAN, f64::NAN]);
        planned += 1;
        allowance -= 1.0;
    }
    planned
}

/// `measure(input)` up to its surface samples. `requests` holds two rows per contact.
pub fn plan(units: &[f64], state: &mut [f64], input: &[f64], values: &[f64], geometry: u32, requests: &mut [f64]) {
    let count = units.len() / UNIT_STRIDE;
    let (globals, contacts, plan) = split(state, count);
    globals[INPUT..INPUT + INPUT_LEN].copy_from_slice(input);
    let revision_changed = input[REVISION_CHANGED] != 0.0;
    let cg_delta = input[CG_DELTA];
    let known_delta = revision_changed && cg_delta.is_finite();
    // Without contact samples the change under the centre is the change.
    globals[REFINEMENT] = if known_delta { cg_delta } else { 0.0 };
    if globals[LIVE] != 0.0 {
        for row in rows(contacts) {
            row[SHIFT] = if known_delta { cg_delta } else { 0.0 };
            if !revision_changed || row[HELD_RAW].is_nan() {
                continue;
            }
            // Refinements are spatially coherent: the sample moves with the
            // change measured under the centre until measured at its own spot.
            if known_delta {
                filter_shift(&mut row[FILTER..FILTER + 2], cg_delta);
                row[GROUND] += cg_delta;
                row[PROVISIONAL] += cg_delta;
            }
            if input[URGENT] != 0.0 || !known_delta {
                row[URGENT_FLAG] = 1.0;
            }
        }
    }
    let far = input[ALTITUDE] - input[CG_GROUND] > FOOTPRINT_RANGE_METERS + globals[REACH] + 1.0;
    let lowest = if far { f64::NAN } else { read_geometry(units, globals, contacts, input, values, geometry) };
    let in_range = lowest.is_finite() && input[ALTITUDE] - lowest - input[CG_GROUND] < FOOTPRINT_RANGE_METERS;
    let mut planned = 0;
    if in_range {
        globals[LIVE] = 1.0;
        planned = plan_samples(globals, contacts, plan, input, requests);
    }
    globals[LOWEST] = lowest;
    globals[IN_RANGE] = if in_range { 1.0 } else { 0.0 };
    globals[REQUESTS] = planned as f64;
    globals[PHASE] = 1.0;
}

/// Whether the state's plan names only contacts and kinds [`plan`] writes.
pub fn valid_plan(state: &[f64], count: usize) -> bool {
    let planned = state[REQUESTS];
    if !(0.0..=(2 * count) as f64).contains(&planned) || planned.fract() != 0.0 {
        return false;
    }
    state[GLOBALS + count * CONTACT_STRIDE..][..planned as usize * 2]
        .chunks_exact(2)
        .all(|entry| (0.0..count as f64).contains(&entry[0]) && entry[0].fract() == 0.0 && (1.0..=4.0).contains(&entry[1]) && entry[1].fract() == 0.0)
}

/// The samples in plan order, then the rest of `measure(input)`.
pub fn finish(units: &[f64], state: &mut [f64], requests: &[f64], output: &mut [f64]) {
    let count = units.len() / UNIT_STRIDE;
    let (globals, contacts, plan) = split(state, count);
    globals[PHASE] = 0.0;
    let mut input = [0.0; INPUT_LEN];
    input.copy_from_slice(&globals[INPUT..INPUT + INPUT_LEN]);
    let steps = globals[STEPS];
    let planned = globals[REQUESTS] as usize;
    let mut applied = 0;
    for k in 0..planned {
        let (next, kind) = (plan[k * 2] as usize, plan[k * 2 + 1]);
        let (height, revision) = (requests[k * REQUEST_STRIDE + 4], requests[k * REQUEST_STRIDE + 5]);
        let row = &mut contacts[next * CONTACT_STRIDE..(next + 1) * CONTACT_STRIDE];
        if kind == SAMPLE_URGENT || kind == SAMPLE_LAZY {
            row[CHECKED] = steps;
            // On a miss (Google drops what is out of view), keep the estimate
            // and confirm it later rather than every step.
            row[URGENT_FLAG] = 0.0;
            if height.is_finite() {
                let correction = height - row[HELD_RAW] - row[PROVISIONAL];
                filter_shift(&mut row[FILTER..FILTER + 2], correction);
                row[GROUND] += correction;
                row[SHIFT] += correction;
                row[HELD_RAW] = height;
                row[HELD_REVISION] = revision;
                row[PROVISIONAL] = 0.0;
                applied += 1;
            }
            continue;
        }
        // Keep the last sample over a hole; the move is retried next step.
        if !height.is_finite() {
            continue;
        }
        let first = kind == SAMPLE_FIRST;
        let snap = first || input[SNAP] != 0.0 || input[ALTITUDE] - row[DOWN] - height > WHEELS_CLEAR_METERS;
        let moved = if first { 0.0 } else { row[MOVED] };
        row[GROUND] = filter_height(&mut row[FILTER..FILTER + 2], height, moved, snap, GROUND_FILTER_LENGTH_METERS);
        row[HELD_LAT] = row[FOOT_LAT];
        row[HELD_LON] = row[FOOT_LON];
        row[HELD_RAW] = height;
        row[HELD_REVISION] = revision;
        row[CHECKED] = steps;
        row[PROVISIONAL] = 0.0;
        row[URGENT_FLAG] = 0.0;
        applied += 1;
    }

    let cg_delta = input[CG_DELTA];
    let known_delta = input[REVISION_CHANGED] != 0.0 && cg_delta.is_finite();
    let mut refinement = globals[REFINEMENT];
    if globals[HAD_SUPPORT] != 0.0 {
        // The previous plane under the refined world, at the previous pose,
        // chosen by the same rule, its reach judged from where the aircraft
        // stood moved by the change measured without the contacts it decides on.
        let mut lift = f64::INFINITY;
        if known_delta {
            lift = cg_delta;
        } else {
            for row in rows(contacts) {
                if row[PREV_SUPPORTED] != 0.0 {
                    lift = min(lift, row[SHIFT]);
                }
            }
        }
        let reach_level = globals[PREV_LEVEL] + if lift.is_finite() { lift } else { 0.0 };
        let mut before = globals[PREV_CG_GROUND] + globals[PREV_LOWEST];
        let mut after = before + if known_delta { cg_delta } else { 0.0 };
        for (i, row) in rows(contacts).enumerate() {
            if row[PREV_SUPPORTED] != 0.0 {
                before = max(before, row[PREV_GROUND] + row[PREV_DOWN]);
            }
            if row[PREV_SAMPLED] == 0.0 {
                continue;
            }
            // A contact's refined ground counts if it is within reach now.
            let refined = row[PREV_GROUND] + row[SHIFT];
            if units[i * UNIT_STRIDE + 1] == 0.0 && refined > reach_level - row[PREV_DOWN] + FOOTPRINT_REACH_METERS {
                continue;
            }
            after = max(after, refined + row[PREV_DOWN]);
        }
        refinement = after - before;
    }
    let lowest = globals[LOWEST];
    output.copy_from_slice(&[f64::NAN, refinement, lowest, globals[IN_RANGE], planned as f64, applied as f64]);
    if globals[IN_RANGE] == 0.0 {
        clear_contacts(globals, contacts);
        return;
    }
    let reference = if input[PREVIOUS_ELEVATION].is_finite() { input[PREVIOUS_ELEVATION] + refinement } else { input[CG_GROUND] };
    // Where the lowest contact would be resting on that plane, or is now.
    let level = max(input[ALTITUDE], reference + lowest);
    let mut top = input[CG_GROUND] + lowest;
    for (i, row) in rows(contacts).enumerate() {
        row[SUPPORTED] = 0.0;
        if row[ACTIVE] == 0.0 || row[GROUND].is_nan() {
            continue;
        }
        if units[i * UNIT_STRIDE + 1] == 0.0 && row[GROUND] > level - row[DOWN] + FOOTPRINT_REACH_METERS {
            row[MARGIN] = FOOTPRINT_REACH_METERS;
            continue;
        }
        row[SUPPORTED] = 1.0;
        top = max(top, row[GROUND] + row[DOWN]);
    }
    for row in rows(contacts) {
        row[PREV_SUPPORTED] = row[SUPPORTED];
        row[PREV_SAMPLED] = if row[ACTIVE] != 0.0 && !row[GROUND].is_nan() { 1.0 } else { 0.0 };
        if row[PREV_SAMPLED] == 0.0 {
            continue;
        }
        row[PREV_GROUND] = row[GROUND];
        row[PREV_DOWN] = row[DOWN];
        if row[SUPPORTED] != 0.0 {
            row[MARGIN] = top - row[GROUND] - row[DOWN];
        }
    }
    globals[PREV_CG_GROUND] = input[CG_GROUND];
    globals[PREV_LOWEST] = lowest;
    globals[PREV_LEVEL] = level;
    globals[HAD_SUPPORT] = 1.0;
    output[0] = top - lowest;
}

#[cfg(test)]
mod tests {
    use super::*;

    // Three C172 wheels and its belly STRUCTURE, structural inches.
    const UNITS: [f64; 8] = [0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0];
    const VALUES: [f64; 17] = [4.0, 1.0, 41.0, 0.0, 36.5, -6.8, 0.0, -19.5, 58.2, -43.0, -15.5, 58.2, 43.0, -15.5, 42.0, 0.0, 11.9];
    const N: usize = 4;

    struct Kernel {
        state: Vec<f64>,
        requests: [f64; 2 * N * REQUEST_STRIDE],
        output: [f64; OUTPUT_LEN],
    }

    impl Kernel {
        fn new() -> Self {
            let mut state = vec![0.0; state_len(N)];
            reset(&mut state, N);
            Self { state, requests: [0.0; 2 * N * REQUEST_STRIDE], output: [0.0; OUTPUT_LEN] }
        }
        fn input(altitude: f64, cg_ground: f64) -> [f64; INPUT_LEN] {
            [44.88, -93.22, altitude, 0.0, 0.0, 0.0, cg_ground, 1.0, 0.0, 0.0, 1.0, 0.0, f64::INFINITY, f64::NAN]
        }
        /// One step on `ground(lat, lon)`.
        fn measure(&mut self, input: &[f64; INPUT_LEN], ground: impl Fn(f64, f64) -> f64) -> [f64; OUTPUT_LEN] {
            plan(&UNITS, &mut self.state, input, &VALUES, GEOMETRY_READ, &mut self.requests);
            for row in self.requests.chunks_exact_mut(REQUEST_STRIDE).take(self.state[REQUESTS] as usize) {
                row[4] = ground(row[2], row[3]);
                row[5] = input[REVISION];
            }
            assert!(valid_plan(&self.state, N));
            finish(&UNITS, &mut self.state, &self.requests, &mut self.output);
            self.output
        }
    }

    #[test]
    fn flat_ground_is_the_plane() {
        let mut kernel = Kernel::new();
        let out = kernel.measure(&Kernel::input(101.5, 100.0), |_, _| 100.0);
        assert_eq!(out[4], 4.0, "every contact sampled first");
        assert!((out[0] - 100.0).abs() < 1e-12, "{}", out[0]);
        // Nothing moved: no samples the next step.
        let out = kernel.measure(&Kernel::input(101.5, 100.0), |_, _| 100.0);
        assert_eq!(out[4], 0.0);
    }

    #[test]
    fn a_rise_under_the_nose_wheel_lifts_the_plane() {
        let mut kernel = Kernel::new();
        // Ground rising to the north: the nose wheel is 1.65 m ahead of the mains.
        let slope = |lat: f64, _: f64| 100.0 + (lat - 44.88) * 111_000.0 * 0.1;
        let out = kernel.measure(&Kernel::input(101.5, 100.0), slope);
        // The nose wheel, deepest below the CG, rests on its own ground; the plane is that ground.
        let [scale_lat, _] = metres_per_degree(44.88);
        let nose_ground = 100.0 + (41.0 + 6.8) * 0.0254 / scale_lat * 111_000.0 * 0.1;
        assert!((out[0] - nose_ground).abs() < 1e-8, "{out:?} {nose_ground}");
    }

    #[test]
    fn a_roof_over_the_belly_is_not_support() {
        let mut kernel = Kernel::new();
        let belly = |lat: f64, lon: f64| {
            let near = (lat - 44.88).abs() < 1e-6 && (lon + 93.22).abs() < 1e-6;
            if near { 103.0 } else { 100.0 }
        };
        let out = kernel.measure(&Kernel::input(101.5, 100.0), belly);
        assert!((out[0] - 100.0).abs() < 1e-12, "{out:?}");
        let belly_row = GLOBALS + 3 * CONTACT_STRIDE;
        assert_eq!(kernel.state[belly_row + SUPPORTED], 0.0);
        assert_eq!(kernel.state[belly_row + MARGIN], FOOTPRINT_REACH_METERS);
    }

    #[test]
    fn far_above_the_ground_nothing_is_read_and_the_contacts_clear() {
        let mut kernel = Kernel::new();
        kernel.measure(&Kernel::input(101.5, 100.0), |_, _| 100.0);
        assert_eq!(kernel.state[LIVE], 1.0);
        plan(&UNITS, &mut kernel.state, &Kernel::input(200.0, 100.0), &[0.0; 17], GEOMETRY_FAILED, &mut kernel.requests);
        assert_eq!(kernel.state[DISCOVERED], 1.0, "the far check skips the read");
        finish(&UNITS, &mut kernel.state, &kernel.requests, &mut kernel.output);
        assert!(kernel.output[0].is_nan());
        assert_eq!(kernel.state[LIVE], 0.0);
        assert!(kernel.state[GLOBALS + GROUND].is_nan());
    }

    #[test]
    fn a_failed_read_forgets_the_contacts() {
        let mut kernel = Kernel::new();
        kernel.measure(&Kernel::input(101.5, 100.0), |_, _| 100.0);
        plan(&UNITS, &mut kernel.state, &Kernel::input(101.5, 100.0), &VALUES, GEOMETRY_FAILED, &mut kernel.requests);
        assert_eq!(kernel.state[DISCOVERED], 0.0);
        assert_eq!(kernel.state[REACH], f64::INFINITY);
        finish(&UNITS, &mut kernel.state, &kernel.requests, &mut kernel.output);
        assert!(kernel.output[0].is_nan());
    }
}
