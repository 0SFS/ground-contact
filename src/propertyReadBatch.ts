import type { JSBSimSdk, PropertyBatch } from "@felipegalind0/jsbsim";

/** A stable set of live properties; native nodes are resolved once per model. */
export function createPropertyReadBatch(
  sdk: JSBSimSdk,
  paths: readonly string[],
  canReadScalar?: (index: number, values: Float64Array) => boolean,
) {
  let batch: PropertyBatch | null = null;
  // Older SDKs and lightweight backends can still use the scalar interface.
  if (typeof sdk.createPropertyBatch === "function") {
    try { batch = sdk.createPropertyBatch(paths, { create: false }); }
    catch { /* An older WASM artifact may not expose the native binding. */ }
  }
  const values = new Float64Array(paths.length);
  // Scalar reads return the backend's missing-property value (usually zero).
  // Keep that behavior for absent optional controls without hiding live NaNs.
  const missing = new Set(batch?.missing ?? []);
  const scalarSlots = paths.flatMap((path, index) => !batch || missing.has(path) ? [index] : []);
  const readInto = (target: Float64Array): Float64Array => {
    if (target.length !== paths.length) throw new RangeError(`Expected ${paths.length} slots, got ${target.length}.`);
    if (batch) batch.read(target);
    // An indexed loop: a for-of iterator would be an allocation per read.
    for (let slot = 0; slot < scalarSlots.length; slot++) {
      const index = scalarSlots[slot];
      target[index] = !canReadScalar || canReadScalar(index, target) ? sdk.getPropertyValue(paths[index]) : Number.NaN;
    }
    return target;
  };
  return {
    /** The values in slot order, in an array this batch reuses on every read. */
    read: (): Float64Array => readInto(values),
    /** The values in slot order, written into the caller's array (one slot per path). */
    readInto,
    dispose(): void { batch?.dispose(); batch = null; },
  };
}
