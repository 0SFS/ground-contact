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
  return {
    read(): Float64Array {
      if (batch) batch.read(values);
      for (const index of scalarSlots) {
        values[index] = !canReadScalar || canReadScalar(index, values) ? sdk.getPropertyValue(paths[index]) : Number.NaN;
      }
      return values;
    },
    dispose(): void { batch?.dispose(); batch = null; },
  };
}
