import type { SurfaceHit, SurfaceQuery } from "foss-earth/runtime";

/** Share exact height queries within one synchronous flight tick. Tiles can
 * change between ticks, so even a parked aircraft must start with fresh data.
 * Never round coordinates or reuse a nearby height: that could hide a kerb,
 * building edge, coverage miss, or a refinement beneath the wheels.
 *
 * beginFrame() also opens the surface's own query frame, when it has one, so
 * the tick's many samples and sweeps read the world transform and the surface
 * meshes' bounds once. Call endFrame() before anything moves the world, such as
 * the floating origin following the aircraft. */
export function createFrameSurfaceQuery(surface: SurfaceQuery) {
  const samples: { lat: number; lon: number; hit: SurfaceHit | null }[] = [];
  let open = false;
  return {
    beginFrame(): void {
      samples.length = 0;
      surface.beginFrame?.();
      open = true;
    },
    endFrame(): void {
      if (!open) return;
      open = false;
      surface.endFrame?.();
    },
    sample(lat: number, lon: number): SurfaceHit | null {
      for (const sample of samples) if (sample.lat === lat && sample.lon === lon) return sample.hit;
      const hit = surface.sample(lat, lon);
      // A normal frame has a few physics steps, each sampling under the
      // aircraft and its contact points, plus an interpolated view and
      // refinement anchors. Bound storage even for an unexpected caller.
      if (samples.length < 64) samples.push({ lat, lon, hit });
      return hit;
    },
    raycast: ((...args) => surface.raycast(...args)) as SurfaceQuery["raycast"],
  };
}
