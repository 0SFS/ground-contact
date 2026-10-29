const MAX_MEMORY = 32 * 1024 * 1024;
const LITTLE_ENDIAN = new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;
const ABI_VERSION = 2;
const functions = ["gc_abi_version", "gc_last_error", "gc_heap_allocations", "gc_alloc", "gc_free", "gc_sin", "gc_cos", "gc_atan2",
  "gc_hull_place", "gc_hull_resolve", "gc_support_reset", "gc_support_plan", "gc_support_finish", "gc_solve_wheels", "gc_step_wheel_spin"];

/** Row layouts, in f64 values; the Rust modules document every field. */
export const HULL = Object.freeze({ valuesLength: 22, probeStride: 4, poseLength: 21, segmentStride: 7, hitStride: 7, headLength: 16, rowStride: 9, maxProbes: 64, minSweepMeters: 0.0001 });
export const SUPPORT = Object.freeze({
  unitStride: 2, inputLength: 14, valuesHead: 5, requestStride: 6, outputLength: 6, globals: 30, contactStride: 24, maxContacts: 64,
  GEOMETRY_READ: 0, GEOMETRY_FAILED: 1, GEOMETRY_UNAVAILABLE: 2,
  /** State offsets: globals, then per contact at globals + i * contactStride. */
  state: Object.freeze({ DISCOVERED: 0, LIVE: 1, HAD_SUPPORT: 2, REVALIDATE_IN: 6, STEPS: 7, REACH: 8, PHASE: 9, REQUESTS: 13 }),
  /** FILTER is the ground filter's [has, height]. */
  contact: Object.freeze({ FILTER: 0, DOWN: 2, FOOT_LAT: 3, FOOT_LON: 4, HELD_RAW: 7, GROUND: 11, MARGIN: 12, ACTIVE: 19, SUPPORTED: 20 }),
});
export const WHEELS = Object.freeze({ bodyLength: 16, wheelStride: 10, contactStride: 25, reportLength: 5, resultStride: 11, maxWheels: 16, maxContacts: 64 });
export const SPIN = Object.freeze({ wheelStride: 20, stateOffset: 0, configOffset: 9, inputOffset: 14, maxWheels: 16, INSTANT: 0, INERTIA: 1 });
export const ERRORS = Object.freeze({ INVALID: 1, ALLOCATION: 2, INPUT: 3 });

/** No binary fetch or WASM work happens on import. No hidden JS fallback. */
export async function loadGroundContact({ wasmUrl = new URL("./single/ground_contact.wasm", import.meta.url), fetch: fetchBytes = globalThis.fetch } = {}) {
  const response = await fetchBytes(wasmUrl);
  if (!response.ok) throw new Error(`Ground-contact WASM request failed (${response.status})`);
  return instantiateGroundContact(await response.arrayBuffer());
}

export async function instantiateGroundContact(bytes) {
  const result = await WebAssembly.instantiate(bytes, {});
  return createGroundContact((result.instance ?? result).exports);
}

function count(value, max, what) {
  if (!Number.isSafeInteger(value) || value < 1 || value > max) throw new Error(`${what} must be an integer from 1 to ${max}`);
  return value;
}

/**
 * Raw ABI adapter. Each kernel owns its buffers in WASM memory from creation
 * to dispose(); its views are cached Float64Arrays that are replaced only if
 * the memory grows, which stepping never does. Fill the input views, call
 * run(), read the output views: no step allocates in JS or in Rust.
 */
export function createGroundContact(exports) {
  if (!LITTLE_ENDIAN) throw new Error("WASM ground-contact views need a little-endian host");
  for (const name of functions) if (typeof exports?.[name] !== "function") throw new Error(`Invalid ground-contact ABI: ${name}`);
  if (!(exports.memory instanceof WebAssembly.Memory) || exports.gc_abi_version() !== ABI_VERSION) throw new Error("Unsupported ground-contact ABI");
  let closed = false;
  const kernels = new Set();
  function buffer() {
    const value = exports.memory.buffer;
    if (!(value instanceof ArrayBuffer) || value.byteLength === 0 || value.byteLength > MAX_MEMORY) throw new Error("Invalid WASM ground-contact memory");
    return value;
  }
  function active() { if (closed) throw new Error("WASM ground-contact core is disposed"); }
  function failure(operation, status = exports.gc_last_error()) {
    const error = new Error(`WASM ground-contact ${operation} failed (${status})`);
    error.status = status;
    return error;
  }
  function region(length, owned) {
    const pointer = exports.gc_alloc(length);
    if (!pointer) throw failure("allocation");
    const entry = { pointer, length, memory: null, view: null };
    owned.push(entry);
    const bytes = buffer().byteLength;
    if (!Number.isSafeInteger(pointer) || pointer % 8 || pointer > bytes || length * 8 > bytes - pointer) throw new Error("Invalid WASM ground-contact allocation");
    for (const other of owned) if (other !== entry && pointer < other.pointer + other.length * 8 && other.pointer < pointer + length * 8) throw new Error("Overlapping WASM ground-contact allocations");
    return entry;
  }
  function view(entry) {
    const memory = exports.memory.buffer;
    if (entry.memory !== memory) { buffer(); entry.view = new Float64Array(memory, entry.pointer, entry.length); entry.memory = memory; }
    return entry.view;
  }
  function release(owned) {
    const errors = [];
    for (const { pointer, length } of owned.reverse()) {
      try { const status = exports.gc_free(pointer, length); if (status !== 0) throw new Error(`Ground-contact buffer release failed (${status})`); }
      catch (error) { errors.push(error); }
    }
    if (errors.length) throw new AggregateError(errors, "WASM ground-contact cleanup failed");
  }
  /** Allocates `lengths` and builds the kernel on them; frees them if that fails. */
  function kernel(lengths, build) {
    active();
    const owned = [];
    let regions;
    try { regions = lengths.map(length => region(length, owned)); }
    catch (error) { try { release(owned); } catch (cleanup) { throw new AggregateError([error, cleanup], error.message); } throw error; }
    let disposed = false;
    const live = () => { active(); if (disposed) throw new Error("WASM ground-contact kernel is disposed"); };
    const result = build(regions, live);
    result.dispose = () => {
      if (disposed) return;
      disposed = true; kernels.delete(result);
      release(owned);
    };
    kernels.add(result);
    return result;
  }
  function check(status, operation) { if (status !== 0) throw failure(operation, status); }
  buffer();
  return {
    backend: "single",
    validationStatus: "unvalidated",
    abiVersion: ABI_VERSION,
    /** Heap allocations made in WASM since instantiation. */
    heapAllocations: () => exports.gc_heap_allocations(),
    /** The core's libm sin, cos and atan2, which can differ from V8's by an ulp. */
    wasmMath: Object.freeze({ sin: exports.gc_sin, cos: exports.gc_cos, atan2: exports.gc_atan2 }),
    /**
     * `createVisibleMeshCollision` without the SDK and the surface. Per step:
     * read the properties into `values`; place(); on the first step, or with
     * nothing to sweep against, swap(); else cast each segment at least
     * `minSweepMeters` long, write each probe's hit (a NaN distance for none),
     * resolve(), restore JSBSim from the output, and place() and swap() again
     * from the restored properties.
     */
    createHullCollision(probeCount) {
      const n = count(probeCount, HULL.maxProbes, "Hull probes");
      return kernel([HULL.valuesLength, n * HULL.probeStride, HULL.poseLength, n * 3, n * 3, n * HULL.segmentStride, n * HULL.hitStride, HULL.headLength + n * HULL.rowStride],
        ([values, probes, pose, first, second, segments, hits, output], live) => {
          let previous = first, current = second;
          return {
            probeCount: n,
            /** visibleMeshCollision.ts's PATHS, in order. */
            get values() { return view(values); },
            /** Per probe [x, y, z, inset] structural inches. */
            get probes() { return view(probes); },
            /** [cg ECEF m 3, body-to-local 9, body-to-ECEF 9]. */
            get pose() { return view(pose); },
            get previous() { return view(previous); },
            get current() { return view(current); },
            /** Per probe [from xyz, movement xyz, distance m], from previous to current. */
            get segments() { return view(segments); },
            /** Per probe [distance m, point ECEF xyz, normal ECEF xyz] as cast; NaN distance for no hit. */
            get hits() { return view(hits); },
            /** [hits, latDeg, lonDeg, h-sl-ft, vn, ve, vd fps, p, q, r, shift xyz, body velocity xyz], then 9 per hit. */
            get output() { return view(output); },
            /** The pose and the probes' current points; false when a value is not finite. */
            place() {
              live();
              const status = exports.gc_hull_place(values.pointer, values.length, probes.pointer, probes.length, previous.pointer, previous.length,
                pose.pointer, pose.length, current.pointer, current.length, segments.pointer, segments.length);
              if (status === ERRORS.INPUT) return false;
              check(status, "hull place");
              return true;
            },
            /** The current points become the previous ones. */
            swap() { live(); const swap = previous; previous = current; current = swap; },
            /** The hit count; restitution and friction as requested, NaN for the defaults. */
            resolve(restitution = NaN, friction = NaN) {
              live();
              check(exports.gc_hull_resolve(values.pointer, values.length, probes.pointer, probes.length, pose.pointer, pose.length,
                previous.pointer, previous.length, current.pointer, current.length, hits.pointer, hits.length, restitution, friction,
                output.pointer, output.length), "hull resolve");
              return view(output)[0];
            },
          };
        });
    },
    /**
     * `createContactSupport` for one model's contact units, each
     * `{ retractable, wheel }` or a GroundContactUnit (`wheel` from its
     * `gear/` prefix). Per step: fill `input` (SupportInput); if
     * wantsGeometry(), read the properties into `values`; plan(geometry);
     * sample the surface at each request row; finish(). measure() does the
     * last three with a sampling callback.
     *
     * When rediscovery after a forget finds other units, pass the old kernel
     * as `previous`: its step counters carry over, as the TypeScript's do.
     */
    createContactSupport(units, previous) {
      const n = count(units?.length, SUPPORT.maxContacts, "Support contacts");
      const flags = units.flatMap(unit => [unit.retractable ? 1 : 0, (unit.wheel ?? unit.prefix?.startsWith("gear/")) ? 1 : 0]);
      const stateLength = SUPPORT.globals + n * (SUPPORT.contactStride + 4);
      const carried = previous ? [previous.state[SUPPORT.state.STEPS], previous.state[SUPPORT.state.REVALIDATE_IN]] : null;
      return kernel([n * SUPPORT.unitStride, stateLength, SUPPORT.inputLength, SUPPORT.valuesHead + n * 3, 2 * n * SUPPORT.requestStride, SUPPORT.outputLength],
        ([unitRows, state, input, values, requests, output], live) => {
          const reset = () => check(exports.gc_support_reset(unitRows.pointer, unitRows.length, state.pointer, state.length), "support reset");
          view(unitRows).set(flags);
          reset();
          if (carried) { view(state)[SUPPORT.state.STEPS] = carried[0]; view(state)[SUPPORT.state.REVALIDATE_IN] = carried[1]; }
          const plan = geometry => {
            live();
            check(exports.gc_support_plan(unitRows.pointer, unitRows.length, state.pointer, state.length, input.pointer, input.length,
              values.pointer, values.length, geometry, requests.pointer, requests.length), "support plan");
            return view(state)[SUPPORT.state.REQUESTS];
          };
          const finish = () => {
            live();
            check(exports.gc_support_finish(unitRows.pointer, unitRows.length, state.pointer, state.length, requests.pointer, requests.length,
              output.pointer, output.length), "support finish");
            return view(output);
          };
          return {
            contactCount: n,
            /** Per unit [retractable, wheel]. */
            get units() { return view(unitRows); },
            /** [lat, lon, altitude, roll, pitch, heading, cgGround, revision, revisionChanged, cgDelta, urgent, snap, samples, previousElevation]. */
            get input() { return view(input); },
            /** [gear/num-units, gear/gear-pos-norm, cg xyz in], then [x, y, z in] per unit. */
            get values() { return view(values); },
            /** Carried between steps; offsets in SUPPORT.state and SUPPORT.contact. */
            get state() { return view(state); },
            /** Per request [contact, kind, latDeg, lonDeg, heightMeters, revision]; write the last two. */
            get requests() { return view(requests); },
            /** [elevation (NaN: the centre's ground), refinement, lowest, inRange, requests, applied]. */
            get output() { return view(output); },
            /** Whether measure reads the properties this step: not when altitude - cgGround > 10 + reach + 1. */
            wantsGeometry() { live(); const i = view(input); return !(i[2] - i[6] > 10 + view(state)[SUPPORT.state.REACH] + 1); },
            /** False once a failed read or a changed unit count has forgotten the contacts. */
            get discovered() { return view(state)[SUPPORT.state.DISCOVERED] === 1; },
            /** The request count. */
            plan,
            finish,
            /**
             * plan(geometry), `sample(latDeg, lonDeg)` (a SurfaceQuery's
             * `{ heightMeters, revision }` or null) at each request, then
             * finish(). If `sample` throws, the step is abandoned and the
             * kernel reset.
             */
            measure(geometry, sample) {
              const planned = plan(geometry);
              const rows = view(requests);
              try {
                for (let k = 0; k < planned; k++) {
                  const at = k * SUPPORT.requestStride;
                  const hit = sample(rows[at + 2], rows[at + 3]);
                  rows[at + 4] = hit ? hit.heightMeters : NaN;
                  rows[at + 5] = hit ? hit.revision : NaN;
                }
              } catch (error) { reset(); throw error; }
              return finish();
            },
            reset() { live(); reset(); },
          };
        });
    },
    createWheelSolver(wheelCount, maxContacts = wheelCount) {
      const n = count(wheelCount, WHEELS.maxWheels, "Wheels");
      const contactsMax = count(maxContacts, WHEELS.maxContacts, "Wheel contacts");
      return kernel([WHEELS.bodyLength, n * WHEELS.wheelStride, contactsMax * WHEELS.contactStride, WHEELS.reportLength + n * WHEELS.resultStride],
        ([body, wheels, contacts, output], live) => ({
          wheelCount: n,
          maxContacts: contactsMax,
          /** [massKg, inverseInertiaWorld 9, velocity 3, angularVelocity 3]; updated in place. */
          get body() { return view(body); },
          /** Per wheel [id, spin, epoch, warmTangent, warmBrake, inertia, mu, maxBrake, drag, brake]; state updated in place. */
          get wheels() { return view(wheels); },
          get contacts() { return view(contacts); },
          /** [iterations, KE before, KE after, cold retry, rejected], then 11 per wheel. */
          get output() { return view(output); },
          run(dtSeconds, contactCount, iterations = NaN, warmStart = true) {
            live();
            if (!Number.isSafeInteger(contactCount) || contactCount < 0 || contactCount > contactsMax) throw new Error(`Wheel contact count must be an integer from 0 to ${contactsMax}`);
            check(exports.gc_solve_wheels(dtSeconds, iterations, warmStart ? 1 : 0, body.pointer, body.length,
              wheels.pointer, wheels.length, contacts.pointer, contacts.length, contactCount, output.pointer, output.length), "wheel solve");
          },
        }));
    },
    createWheelSpin(wheelCount) {
      const n = count(wheelCount, SPIN.maxWheels, "Spin wheels");
      return kernel([n * SPIN.wheelStride], ([wheels], live) => ({
        wheelCount: n,
        /** Per wheel [state 9, config 5, input 6]; state updated in place. */
        get wheels() { return view(wheels); },
        run(dtSeconds, mode) {
          live();
          const code = mode === "instant" ? SPIN.INSTANT : mode === "inertia" ? SPIN.INERTIA : mode;
          check(exports.gc_step_wheel_spin(dtSeconds, code, wheels.pointer, wheels.length), "wheel spin");
        },
      }));
    },
    dispose() {
      if (closed) return;
      const errors = [];
      for (const live of [...kernels]) { try { live.dispose(); } catch (error) { errors.push(error); } }
      closed = true;
      if (errors.length) throw new AggregateError(errors, "WASM ground-contact cleanup failed");
    },
  };
}
