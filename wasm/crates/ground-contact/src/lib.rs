//! Ground-contact kernels for `wasm32-unknown-unknown` with `simd128`:
//! hull collision (`src/visibleMeshCollision.ts`), contact support
//! (`createContactSupport` in `src/terrainContact.ts`), coupled rigid wheels
//! and wheel spin. Stepping never allocates.
//!
//! Raw C ABI, version 2. The host allocates f64 buffers once with `gc_alloc`
//! and passes `(pointer, length)` pairs; every pair must name a whole live
//! allocation, and a buffer a kernel writes must not be passed twice. Each
//! kernel returns 0 or an error code and leaves its outputs unspecified on
//! error. Row layouts are documented in each module.

// `!(x > 0.0)` is deliberate where NaN must fail, as in the TypeScript.
#![allow(clippy::neg_cmp_op_on_partial_ord)]
// Rows are runtime-length slices of host buffers.
#![allow(clippy::chunks_exact_to_as_chunks)]

pub mod geo;
pub mod hull;
pub mod jsmath;
pub mod spin;
pub mod support;
pub mod wheels;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::RefCell;
use std::sync::atomic::{AtomicU32, Ordering};

/// A pointer or length does not name a live allocation, a written buffer is
/// aliased, or a count or length is out of bounds.
pub const INVALID: u32 = 1;
pub const ALLOCATION: u32 = 2;
/// A value is non-finite, out of range or an unknown flag.
pub const INPUT: u32 = 3;

const MAX_BYTES: usize = 16 * 1024 * 1024;
const MAX_BUFFERS: usize = 64;

/// Counts every heap allocation, so a host can prove a step made none.
struct Counting;
static ALLOCATIONS: AtomicU32 = AtomicU32::new(0);
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}
#[global_allocator]
static GLOBAL: Counting = Counting;

struct Registry {
    buffers: Vec<Vec<f64>>,
    bytes: usize,
    error: u32,
}

type View = (*mut f64, usize);

impl Registry {
    const fn new() -> Self {
        Self { buffers: Vec::new(), bytes: 0, error: 0 }
    }
    fn alloc(&mut self, length: u32) -> Result<usize, u32> {
        let len = length as usize;
        let bytes = len.checked_mul(8).ok_or(ALLOCATION)?;
        if len == 0 || self.buffers.len() >= MAX_BUFFERS || bytes > MAX_BYTES - self.bytes {
            return Err(ALLOCATION);
        }
        // The registry's own storage is reserved once, never while stepping.
        if self.buffers.capacity() == 0 {
            self.buffers.try_reserve_exact(MAX_BUFFERS).map_err(|_| ALLOCATION)?;
        }
        let mut buffer = Vec::new();
        buffer.try_reserve_exact(len).map_err(|_| ALLOCATION)?;
        buffer.resize(len, 0.0);
        let pointer = buffer.as_ptr() as usize;
        self.buffers.push(buffer);
        self.bytes += bytes;
        Ok(pointer)
    }
    fn free(&mut self, pointer: usize, length: u32) -> Result<(), u32> {
        let index = self
            .buffers
            .iter()
            .position(|b| b.as_ptr() as usize == pointer && b.len() == length as usize)
            .ok_or(INVALID)?;
        let buffer = self.buffers.swap_remove(index);
        self.bytes -= buffer.len() * 8;
        Ok(())
    }
    /// Resolves `(pointer, length, written)` arguments to whole allocations.
    fn views<const N: usize>(&mut self, args: [(usize, u32, bool); N]) -> Result<[View; N], u32> {
        let mut views = [(core::ptr::null_mut(), 0); N];
        for (view, &(pointer, length, _)) in views.iter_mut().zip(&args) {
            let buffer = self
                .buffers
                .iter_mut()
                .find(|b| b.as_ptr() as usize == pointer && b.len() == length as usize)
                .ok_or(INVALID)?;
            *view = (buffer.as_mut_ptr(), buffer.len());
        }
        for (i, a) in args.iter().enumerate() {
            if a.2 && args.iter().enumerate().any(|(j, b)| i != j && a.0 == b.0) {
                return Err(INVALID);
            }
        }
        Ok(views)
    }
}

/// # Safety
/// The view comes from [`Registry::views`] in the same registry borrow, and
/// no written view aliases it.
unsafe fn read<'a>(view: View) -> &'a [f64] {
    unsafe { core::slice::from_raw_parts(view.0, view.1) }
}
/// # Safety
/// As [`read`], and the view aliases no other view.
unsafe fn write<'a>(view: View) -> &'a mut [f64] {
    unsafe { core::slice::from_raw_parts_mut(view.0, view.1) }
}

thread_local! { static REGISTRY: RefCell<Registry> = const { RefCell::new(Registry::new()) }; }

fn with(run: impl FnOnce(&mut Registry) -> Result<(), u32>) -> u32 {
    REGISTRY.with(|r| match r.try_borrow_mut() {
        Ok(mut r) => {
            let status = run(&mut r).err().unwrap_or(0);
            r.error = status;
            status
        }
        Err(_) => INVALID,
    })
}

fn check(condition: bool, code: u32) -> Result<(), u32> {
    if condition { Ok(()) } else { Err(code) }
}

/// Rows of `stride` in `length`, between 1 and `max`.
fn rows(length: u32, stride: usize, max: usize) -> Result<usize, u32> {
    let length = length as usize;
    let count = length / stride;
    check(length.is_multiple_of(stride) && (1..=max).contains(&count), INVALID)?;
    Ok(count)
}

#[unsafe(no_mangle)]
pub extern "C" fn gc_abi_version() -> u32 {
    2
}

#[unsafe(no_mangle)]
pub extern "C" fn gc_last_error() -> u32 {
    REGISTRY.with(|r| r.try_borrow().map_or(INVALID, |r| r.error))
}

/// Heap allocations since instantiation.
#[unsafe(no_mangle)]
pub extern "C" fn gc_heap_allocations() -> u32 {
    ALLOCATIONS.load(Ordering::Relaxed)
}

/// A zeroed f64 buffer of `length`; 0 on failure (see `gc_last_error`).
#[unsafe(no_mangle)]
pub extern "C" fn gc_alloc(length: u32) -> usize {
    let mut pointer = 0;
    with(|r| {
        pointer = r.alloc(length)?;
        Ok(())
    });
    pointer
}

#[unsafe(no_mangle)]
pub extern "C" fn gc_free(pointer: usize, length: u32) -> u32 {
    with(|r| r.free(pointer, length))
}

/// The core's `sin`, `cos` and `atan2` (libm's), which can differ from V8's
/// by an ulp; a host or check that needs this core's exact results uses them.
#[unsafe(no_mangle)]
pub extern "C" fn gc_sin(x: f64) -> f64 {
    x.sin()
}

#[unsafe(no_mangle)]
pub extern "C" fn gc_cos(x: f64) -> f64 {
    x.cos()
}

#[unsafe(no_mangle)]
pub extern "C" fn gc_atan2(y: f64, x: f64) -> f64 {
    y.atan2(x)
}

/// `place` (see [`hull`]): the pose, the probes' points into `current` and
/// their sweeps from `previous`. Probes: 1 to 64. [`INPUT`], with nothing
/// written, when a property value is not finite.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn gc_hull_place(
    values: usize,
    values_len: u32,
    probes: usize,
    probes_len: u32,
    previous: usize,
    previous_len: u32,
    pose: usize,
    pose_len: u32,
    current: usize,
    current_len: u32,
    segments: usize,
    segments_len: u32,
) -> u32 {
    with(|r| {
        let count = rows(probes_len, hull::PROBE_STRIDE, hull::MAX_PROBES)?;
        check(
            values_len as usize == hull::VALUES_LEN
                && previous_len as usize == count * 3
                && pose_len as usize == hull::POSE_LEN
                && current_len as usize == count * 3
                && segments_len as usize == count * hull::SEGMENT_STRIDE,
            INVALID,
        )?;
        let [v, p, b, o, c, s] = r.views([
            (values, values_len, false),
            (probes, probes_len, false),
            (previous, previous_len, false),
            (pose, pose_len, true),
            (current, current_len, true),
            (segments, segments_len, true),
        ])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (values, probes, previous, pose, current, segments) = unsafe { (read(v), read(p), read(b), write(o), write(c), write(s)) };
        check(hull::place(values, probes, previous, pose, current, segments), INPUT)
    })
}

/// `sweep` over the host's raycast hits and `resolve` (see [`hull`]); the hit
/// count is `output[0]`. Restitution and friction as requested, NaN for the
/// defaults.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn gc_hull_resolve(
    values: usize,
    values_len: u32,
    probes: usize,
    probes_len: u32,
    pose: usize,
    pose_len: u32,
    previous: usize,
    previous_len: u32,
    current: usize,
    current_len: u32,
    hits: usize,
    hits_len: u32,
    restitution: f64,
    friction: f64,
    output: usize,
    output_len: u32,
) -> u32 {
    with(|r| {
        let count = rows(probes_len, hull::PROBE_STRIDE, hull::MAX_PROBES)?;
        check(
            values_len as usize == hull::VALUES_LEN
                && pose_len as usize == hull::POSE_LEN
                && previous_len as usize == count * 3
                && current_len as usize == count * 3
                && hits_len as usize == count * hull::HIT_STRIDE
                && output_len as usize == hull::HEAD_LEN + count * hull::ROW_STRIDE,
            INVALID,
        )?;
        let [v, p, o, b, c, h, out] = r.views([
            (values, values_len, false),
            (probes, probes_len, false),
            (pose, pose_len, false),
            (previous, previous_len, false),
            (current, current_len, false),
            (hits, hits_len, false),
            (output, output_len, true),
        ])?;
        // SAFETY: views from this borrow; the output aliases no input.
        let (values, probes, pose, previous, current, hits, output) =
            unsafe { (read(v), read(p), read(o), read(b), read(c), read(h), write(out)) };
        hull::resolve(values, probes, pose, previous, current, hits, restitution, friction, output);
        Ok(())
    })
}

/// Contact units, 1 to 64, each `[retractable, wheel]` of 0 or 1.
fn support_units(units: &[f64]) -> Result<(), u32> {
    check(units.iter().all(|&v| v == 0.0 || v == 1.0), INPUT)
}

/// `reset()` (see [`support`]); also readies a new state.
#[unsafe(no_mangle)]
pub extern "C" fn gc_support_reset(units: usize, units_len: u32, state: usize, state_len: u32) -> u32 {
    with(|r| {
        let count = rows(units_len, support::UNIT_STRIDE, support::MAX_CONTACTS)?;
        check(state_len as usize == support::state_len(count), INVALID)?;
        let [_, s] = r.views([(units, units_len, false), (state, state_len, true)])?;
        // SAFETY: the only written view in this borrow.
        support::reset(unsafe { write(s) }, count);
        Ok(())
    })
}

/// `measure(input)` up to its surface samples (see [`support`]): writes
/// `state[REQUESTS]` request rows. `geometry`: 0 the values were read, 1 the
/// read threw, 2 no units. Not again before [`gc_support_finish`] or a reset.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn gc_support_plan(
    units: usize,
    units_len: u32,
    state: usize,
    state_len: u32,
    input: usize,
    input_len: u32,
    values: usize,
    values_len: u32,
    geometry: u32,
    requests: usize,
    requests_len: u32,
) -> u32 {
    with(|r| {
        let count = rows(units_len, support::UNIT_STRIDE, support::MAX_CONTACTS)?;
        check(
            state_len as usize == support::state_len(count)
                && input_len as usize == support::INPUT_LEN
                && values_len as usize == support::VALUES_HEAD + count * 3
                && requests_len as usize == 2 * count * support::REQUEST_STRIDE
                && geometry <= support::GEOMETRY_UNAVAILABLE,
            INVALID,
        )?;
        let [u, s, i, v, q] = r.views([
            (units, units_len, false),
            (state, state_len, true),
            (input, input_len, false),
            (values, values_len, false),
            (requests, requests_len, true),
        ])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (units, state, input, values, requests) = unsafe { (read(u), write(s), read(i), read(v), write(q)) };
        check(state[support::PHASE] == 0.0, INVALID)?;
        support_units(units)?;
        check(support::INPUT_FLAGS.iter().all(|&f| input[f] == 0.0 || input[f] == 1.0), INPUT)?;
        support::plan(units, state, input, values, geometry, requests);
        Ok(())
    })
}

/// The samples the host wrote into the request rows, then the rest of
/// `measure` (see [`support`]): output `[elevation, refinement, lowest, in
/// range, requests, applied]`.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn gc_support_finish(
    units: usize,
    units_len: u32,
    state: usize,
    state_len: u32,
    requests: usize,
    requests_len: u32,
    output: usize,
    output_len: u32,
) -> u32 {
    with(|r| {
        let count = rows(units_len, support::UNIT_STRIDE, support::MAX_CONTACTS)?;
        check(
            state_len as usize == support::state_len(count)
                && requests_len as usize == 2 * count * support::REQUEST_STRIDE
                && output_len as usize == support::OUTPUT_LEN,
            INVALID,
        )?;
        let [u, s, q, o] = r.views([
            (units, units_len, false),
            (state, state_len, true),
            (requests, requests_len, false),
            (output, output_len, true),
        ])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (units, state, requests, output) = unsafe { (read(u), write(s), read(q), write(o)) };
        check(state[support::PHASE] == 1.0 && support::valid_plan(state, count), INVALID)?;
        support_units(units)?;
        support::finish(units, state, requests, output);
        Ok(())
    })
}

/// Per wheel: `[id, spinRadSec, contactEpoch, warmTangentNs, warmBrakeNms,
/// inertiaKgM2, frictionCoefficient, maxBrakeTorqueNm, bearingDragTorqueNm, brake]`.
pub const WHEEL_STRIDE: usize = 10;

/// `solveCoupledRigidWheels` (see [`wheels`]). Body and wheel states update in
/// place; wheels: 1 to 16; `contact_count` rows of `contacts` (0 to 64); output
/// `[report 5, results 11 per wheel]`. `iterations` NaN stands for absent.
#[unsafe(no_mangle)]
pub extern "C" fn gc_solve_wheels(
    dt_seconds: f64,
    iterations: f64,
    warm_start: u32,
    body: usize,
    body_len: u32,
    wheel_rows: usize,
    wheels_len: u32,
    contacts: usize,
    contacts_len: u32,
    contact_count: u32,
    output: usize,
    output_len: u32,
) -> u32 {
    with(|r| {
        let count = rows(wheels_len, WHEEL_STRIDE, wheels::MAX_WHEELS)?;
        let contact_count = contact_count as usize;
        check(
            body_len as usize == wheels::BODY_LEN
                && warm_start <= 1
                && contact_count <= wheels::MAX_CONTACTS
                && contact_count * wheels::CONTACT_STRIDE <= contacts_len as usize
                && output_len as usize == wheels::REPORT_LEN + count * wheels::RESULT_STRIDE,
            INVALID,
        )?;
        let [b, w, c, o] = r.views([
            (body, body_len, true),
            (wheel_rows, wheels_len, true),
            (contacts, contacts_len, false),
            (output, output_len, true),
        ])?;
        // SAFETY: views from this borrow; written views alias nothing.
        let (body, wheel_rows, contacts, output) = unsafe { (write(b), write(w), read(c), write(o)) };
        let mut ids = [0.0; wheels::MAX_WHEELS];
        let mut states = [0.0; wheels::MAX_WHEELS * wheels::STATE_STRIDE];
        let mut params = [0.0; wheels::MAX_WHEELS * wheels::PARAMS_STRIDE];
        let mut brakes = [0.0; wheels::MAX_WHEELS];
        for (index, row) in wheel_rows.chunks_exact(WHEEL_STRIDE).enumerate() {
            ids[index] = row[0];
            states[index * 4..index * 4 + 4].copy_from_slice(&row[1..5]);
            params[index * 4..index * 4 + 4].copy_from_slice(&row[5..9]);
            brakes[index] = row[9];
        }
        let problem = wheels::WheelProblem {
            wheel_ids: &ids[..count],
            params: &params[..count * 4],
            contacts: &contacts[..contact_count * wheels::CONTACT_STRIDE],
            brakes: &brakes[..count],
            dt_seconds,
            iterations,
            warm_start: warm_start == 1,
        };
        let (report, results) = output.split_at_mut(wheels::REPORT_LEN);
        wheels::solve(&problem, body, &mut states[..count * 4], results, report);
        for (index, row) in wheel_rows.chunks_exact_mut(WHEEL_STRIDE).enumerate() {
            row[1..5].copy_from_slice(&states[index * 4..index * 4 + 4]);
        }
        Ok(())
    })
}

/// Per wheel: `[state 9, config 5, input 6]` (see [`spin`]); states update in place.
pub const SPIN_STRIDE: usize = spin::STATE_STRIDE + spin::CONFIG_STRIDE + spin::INPUT_STRIDE;

/// `stepWheelSpin` for 1 to 16 wheels; mode 0 is instant, 1 inertia.
#[unsafe(no_mangle)]
pub extern "C" fn gc_step_wheel_spin(dt_seconds: f64, mode: u32, wheel_rows: usize, wheels_len: u32) -> u32 {
    with(|r| {
        rows(wheels_len, SPIN_STRIDE, wheels::MAX_WHEELS)?;
        let mode = match mode {
            0 => spin::Mode::Instant,
            1 => spin::Mode::Inertia,
            _ => return Err(INVALID),
        };
        let [w] = r.views([(wheel_rows, wheels_len, true)])?;
        // SAFETY: the only view in this borrow.
        let wheel_rows = unsafe { write(w) };
        let config = spin::STATE_STRIDE;
        let input = config + spin::CONFIG_STRIDE;
        check(
            wheel_rows.chunks_exact(SPIN_STRIDE).all(|row| row[input] == 0.0 || row[input] == 1.0),
            INPUT,
        )?;
        for row in wheel_rows.chunks_exact_mut(SPIN_STRIDE) {
            let (state, rest) = row.split_at_mut(config);
            let (config, input) = rest.split_at(spin::CONFIG_STRIDE);
            spin::step(state, config, input, dt_seconds, mode);
        }
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alloc(values: &[f64]) -> (usize, u32) {
        let pointer = gc_alloc(values.len() as u32);
        assert_ne!(pointer, 0);
        REGISTRY.with(|r| {
            let mut r = r.borrow_mut();
            let buffer = r.buffers.iter_mut().find(|b| b.as_ptr() as usize == pointer).unwrap();
            buffer.copy_from_slice(values);
        });
        (pointer, values.len() as u32)
    }

    #[test]
    fn rejects_foreign_aliased_and_short_buffers() {
        let mut values = [0.0; hull::VALUES_LEN];
        values[2] = 1000.0;
        let (values, values_len) = alloc(&values);
        let (probes, probes_len) = alloc(&[0.0, 0.0, 0.0, 3.0]);
        let (previous, previous_len) = alloc(&[0.0; 3]);
        let (pose, pose_len) = alloc(&[0.0; hull::POSE_LEN]);
        let (current, current_len) = alloc(&[0.0; 3]);
        let (segments, segments_len) = alloc(&[0.0; hull::SEGMENT_STRIDE]);
        let place = |values: usize, current: (usize, u32), segments_len: u32| {
            gc_hull_place(values, values_len, probes, probes_len, previous, previous_len, pose, pose_len, current.0, current.1, segments, segments_len)
        };
        assert_eq!(place(values, (current, current_len), segments_len), 0);
        assert_eq!(place(values, (current, current_len), segments_len - 1), INVALID);
        assert_eq!(place(values, (pose, pose_len), segments_len), INVALID);
        assert_eq!(place(values, (previous, previous_len), segments_len), INVALID);
        assert_eq!(place(values + 8, (current, current_len), segments_len), INVALID);
        assert_eq!(gc_last_error(), INVALID);
        let mut bad = [0.0; hull::VALUES_LEN];
        bad[20] = f64::NAN;
        let (bad, bad_len) = alloc(&bad);
        assert_eq!(place(bad, (current, current_len), segments_len), INPUT);
        for (pointer, length) in [
            (values, values_len),
            (probes, probes_len),
            (previous, previous_len),
            (pose, pose_len),
            (current, current_len),
            (segments, segments_len),
            (bad, bad_len),
        ] {
            assert_eq!(gc_free(pointer, length), 0);
        }
        assert_eq!(gc_free(values, values_len), INVALID);
        assert_eq!(gc_alloc(0), 0);
        assert_eq!(gc_last_error(), ALLOCATION);
    }

    #[test]
    fn support_phases_run_in_order() {
        let (units, units_len) = alloc(&[0.0, 1.0]);
        let (state, state_len) = alloc(&vec![0.0; support::state_len(1)]);
        let (input, input_len) = alloc(&[44.88, -93.22, 101.0, 0.0, 0.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0, 0.0, 2.0, f64::NAN]);
        let (values, values_len) = alloc(&[1.0, 1.0, 41.0, 0.0, 36.5, 41.0, 0.0, -19.5]);
        let (requests, requests_len) = alloc(&[0.0; 2 * support::REQUEST_STRIDE]);
        let (output, output_len) = alloc(&[0.0; support::OUTPUT_LEN]);
        let plan = |geometry| gc_support_plan(units, units_len, state, state_len, input, input_len, values, values_len, geometry, requests, requests_len);
        let finish = || gc_support_finish(units, units_len, state, state_len, requests, requests_len, output, output_len);
        assert_eq!(gc_support_reset(units, units_len, state, state_len), 0);
        assert_eq!(finish(), INVALID);
        assert_eq!(plan(3), INVALID);
        assert_eq!(plan(0), 0);
        assert_eq!(plan(0), INVALID);
        assert_eq!(finish(), 0);
        assert_eq!(gc_support_reset(units, units_len, state, state_len - 1), INVALID);
        let (flags, flags_len) = alloc(&[0.0, 2.0]);
        assert_eq!(gc_support_reset(flags, flags_len, state, state_len), 0);
        assert_eq!(gc_support_plan(flags, flags_len, state, state_len, input, input_len, values, values_len, 0, requests, requests_len), INPUT);
        for (pointer, length) in [
            (units, units_len),
            (state, state_len),
            (input, input_len),
            (values, values_len),
            (requests, requests_len),
            (output, output_len),
            (flags, flags_len),
        ] {
            assert_eq!(gc_free(pointer, length), 0);
        }
    }

    #[test]
    fn spin_rejects_unknown_mode_and_flags() {
        let mut row = [0.0; SPIN_STRIDE];
        row[9..14].copy_from_slice(&[0.22, 0.152, 0.124, 0.5, 450.0]);
        row[14] = 1.0;
        row[15] = 10.0;
        let (pointer, length) = alloc(&row);
        assert_eq!(gc_step_wheel_spin(1.0 / 120.0, 2, pointer, length), INVALID);
        assert_eq!(gc_step_wheel_spin(1.0 / 120.0, 1, pointer, length), 0);
        row[14] = 0.5;
        let (other, other_len) = alloc(&row);
        assert_eq!(gc_step_wheel_spin(1.0 / 120.0, 1, other, other_len), INPUT);
        assert_eq!(gc_free(pointer, length), 0);
        assert_eq!(gc_free(other, other_len), 0);
    }
}
