/**
 * Optional WASM kernels from `ground-contact/wasm`. The TypeScript in this
 * package is the reference and the default; a module given a core delegates
 * its hot path to a kernel built on it, and goes back to the reference for
 * good if that kernel refuses or fails.
 *
 * These interfaces are the parts of the core the modules call, so a module
 * accepts the loader's own object without depending on its declarations.
 */
export interface GroundContactKernel {
  dispose(): void;
}

/** `solveCoupledRigidWheels` on flat rows; layouts in `WHEELS`. */
export interface WasmWheelSolver extends GroundContactKernel {
  readonly wheelCount: number;
  readonly maxContacts: number;
  readonly body: Float64Array;
  readonly wheels: Float64Array;
  readonly contacts: Float64Array;
  readonly output: Float64Array;
  run(dtSeconds: number, contactCount: number, iterations?: number, warmStart?: boolean): void;
}

/** `stepWheelSpin` per wheel row [state 9, config 5, input 6]. */
export interface WasmWheelSpin extends GroundContactKernel {
  readonly wheelCount: number;
  readonly wheels: Float64Array;
  run(dtSeconds: number, mode: "instant" | "inertia" | 0 | 1): void;
}

/** `createVisibleMeshCollision` without the SDK and the surface; layouts in `HULL`. */
export interface WasmHullCollision extends GroundContactKernel {
  readonly probeCount: number;
  readonly values: Float64Array;
  readonly probes: Float64Array;
  readonly pose: Float64Array;
  readonly previous: Float64Array;
  readonly current: Float64Array;
  readonly segments: Float64Array;
  readonly hits: Float64Array;
  readonly output: Float64Array;
  place(): boolean;
  swap(): void;
  resolve(restitution?: number, friction?: number): number;
}

/** `createContactSupport` for one model's contact units; layouts in `SUPPORT`. */
export interface WasmContactSupport extends GroundContactKernel {
  readonly contactCount: number;
  readonly input: Float64Array;
  readonly values: Float64Array;
  readonly state: Float64Array;
  readonly requests: Float64Array;
  readonly output: Float64Array;
  readonly discovered: boolean;
  wantsGeometry(): boolean;
  plan(geometry: number): number;
  finish(): Float64Array;
  reset(): void;
}

export interface GroundContactCore {
  createWheelSolver(wheelCount: number, maxContacts?: number): WasmWheelSolver;
  createWheelSpin(wheelCount: number): WasmWheelSpin;
  createHullCollision(probeCount: number): WasmHullCollision;
  /** Units `{ retractable, prefix }`; `previous`, a kernel for other units, hands on its step counters. */
  createContactSupport(units: readonly { retractable: boolean; prefix: string }[], previous?: WasmContactSupport): WasmContactSupport;
}

/** A core now, one still loading, or none for the TypeScript reference. */
export type GroundContactCoreSource = GroundContactCore | PromiseLike<GroundContactCore> | null | undefined;

export type GroundContactKernelName = "single" | "typescript";

export interface GroundContactKernelStatus {
  /** "single" when a core was supplied. */
  requestedKernel: GroundContactKernelName;
  /** What the next call runs on. */
  kernel: GroundContactKernelName;
  /** The latest fallback: why there is no kernel, or why a call it refused ran on the reference. */
  fallbackReason: string | null;
}

export interface GroundContactKernelSlot<K extends GroundContactKernel> {
  /** The kernel, once built and while it has not failed. */
  readonly current: K | null;
  status(): GroundContactKernelStatus;
  /** Records a call the kernel cannot take, which runs on the reference; the kernel stays. */
  refuse(reason: string): void;
  /** Gives the kernel up for good after it threw. */
  fail(error: unknown): void;
  dispose(): void;
}

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

/**
 * Builds `name` on the core from `source` when it arrives. Never throws: a
 * missing, rejected or refusing core leaves `current` null with the reason.
 * The core stays the caller's; dispose() releases only the kernel.
 */
export function createKernelSlot<K extends GroundContactKernel>(
  source: GroundContactCoreSource, name: string, build: (core: GroundContactCore) => K,
): GroundContactKernelSlot<K> {
  let current: K | null = null;
  let reason: string | null = null;
  let disposed = false;
  const adopt = (core: GroundContactCore) => {
    if (disposed) return;
    try { current = build(core); }
    catch (error) { reason = `${name}-unavailable: ${errorMessage(error)}`; }
  };
  const release = () => {
    const kernel = current;
    current = null;
    try { kernel?.dispose(); }
    catch (error) { reason ??= `${name}-release-failed: ${errorMessage(error)}`; }
  };
  if (source && typeof (source as PromiseLike<GroundContactCore>).then === "function") {
    (source as PromiseLike<GroundContactCore>).then(adopt, error => {
      if (!disposed) reason = `ground-contact-core-unavailable: ${errorMessage(error)}`;
    });
  } else if (source) adopt(source as GroundContactCore);
  return {
    get current() { return current; },
    status: () => ({ requestedKernel: source ? "single" : "typescript", kernel: current ? "single" : "typescript", fallbackReason: reason }),
    refuse(why) { reason = `${name}-refused: ${why}`; },
    fail(error) {
      reason = `${name}-failed: ${errorMessage(error)}`;
      release();
    },
    dispose() {
      disposed = true;
      release();
    },
  };
}
