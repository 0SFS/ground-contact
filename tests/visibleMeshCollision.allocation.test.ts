import { getHeapSpaceStatistics } from "node:v8";
import { describe, expect, it } from "vitest";
import type { SurfaceQuery } from "foss-earth/runtime";
import type { BodyCollisionProbe } from "../src/collisionGeometry";
import { createVisibleMeshCollision } from "../src/visibleMeshCollision";

// A file of its own: the many surface doubles in the behaviour tests would
// make the collision's call sites megamorphic. V8 then calls the surface
// without inlining it and boxes each double argument, 16 bytes apiece, which
// measures the test doubles rather than the collision.

/** Young-generation bytes in use. Only allocation grows them; a scavenge empties them. */
function youngBytes(): number {
  let bytes = 0;
  for (const space of getHeapSpaceStatistics()) {
    if (space.space_name === "new_space" || space.space_name === "new_large_object_space") bytes += space.space_used_size;
  }
  return bytes;
}

/** The median growth over nine windows of `calls` calls: a window a scavenge cut short is an outlier. */
function youngGrowth(work: () => void, calls: number): number {
  const growth: number[] = [];
  for (let window = 0; window < 9; window++) {
    const before = youngBytes();
    for (let call = 0; call < calls; call++) work();
    growth.push(youngBytes() - before);
  }
  return growth.sort((a, b) => a - b)[4];
}

const probe = (name: string, xIn: number, yIn: number, zIn: number, insetIn = 0): BodyCollisionProbe => ({ name, xIn, yIn, zIn, insetIn });

describe("visible mesh body collision allocation", () => {
  it("allocates nothing on a step that sweeps every probe and hits nothing", () => {
    const live: Record<string, number> = {
      "position/h-sl-ft": 320 / 0.3048, "attitude/phi-deg": 10, "attitude/theta-deg": 3, "attitude/psi-deg": 40,
      "velocities/v-north-fps": 30, "velocities/v-down-fps": 2,
      "inertia/cg-x-in": 100, "inertia/mass-slugs": 100,
      "inertia/ixx-slugs_ft2": 1000, "inertia/iyy-slugs_ft2": 2000, "inertia/izz-slugs_ft2": 2500,
    };
    // The batch alternates between two altitudes, so every probe moves and is cast every step.
    const source = new Float64Array(64);
    let paths: readonly string[] = [], high = false, casts = 0;
    const native = {
      getPropertyValue: () => 0,
      createPropertyBatch: (requested: readonly string[]) => {
        paths = requested;
        paths.forEach((path, slot) => { source[slot] = live[path] ?? 0; });
        return {
          missing: [], dispose: () => {},
          read: (target: Float64Array) => {
            high = !high;
            for (let slot = 0; slot < paths.length; slot++) target[slot] = source[slot];
            target[2] = high ? source[2] + 1 : source[2];
            return target;
          },
        };
      },
    };
    const support = { heightMeters: 300 };
    const surface = { sample: () => support, raycast: () => { casts++; return null; } } as unknown as SurfaceQuery;
    const collision = createVisibleMeshCollision(native as never, surface, {
      bodyProbes: [probe("nose", 0, 0, 0), probe("tail", 220, 0, 0), probe("right-wing", 100, 120, 0, 3),
        probe("left-wing", 100, -120, 0, 3), probe("belly", 100, 0, -40, 3)],
    });
    const step = () => { collision.update(); };
    const nothing = () => {};
    // Past first-call setup, lazy feedback and tier-up: until the optimizing
    // compiler takes the step, near 20 000 calls, its doubles live in boxes.
    for (let call = 0; call < 30_000; call++) step();
    expect(paths[2]).toBe("position/h-sl-ft");
    expect(casts).toBe(29_999 * 5);
    const calls = 4_000;
    // Any allocation adds at least 8 bytes a call.
    expect(youngGrowth(step, calls) - youngGrowth(nothing, calls)).toBeLessThan(calls);
  });
});
