import { Vector3 } from "@babylonjs/core";
import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import { describe, expect, it, vi } from "vitest";
import { flightAttitudeToQuaternion, readFlightState, releaseFlightStateReads } from "../src/ecefBridge";

describe("flightAttitudeToQuaternion", () => {
  const aircraftNose = new Vector3(0, 0, 1);

  it("points the aircraft nose north at heading zero", () => {
    const attitude = flightAttitudeToQuaternion(0, 0, 0);
    const forward = Vector3.Zero();
    aircraftNose.rotateByQuaternionToRef(attitude, forward);
    expect(forward.x).toBeCloseTo(0, 5);
    expect(forward.z).toBeCloseTo(-1, 5);
  });

  it("points the aircraft nose east at a 90 degree heading", () => {
    const attitude = flightAttitudeToQuaternion(0, 0, Math.PI / 2);
    const forward = Vector3.Zero();
    aircraftNose.rotateByQuaternionToRef(attitude, forward);
    expect(forward.x).toBeCloseTo(1, 5);
    expect(forward.z).toBeCloseTo(0, 5);
  });

  it("raises the nose for positive JSBSim pitch", () => {
    const attitude = flightAttitudeToQuaternion(0, Math.PI / 6, 0);
    const forward = Vector3.Zero();
    aircraftNose.rotateByQuaternionToRef(attitude, forward);
    expect(forward.y).toBeCloseTo(0.5, 5);
  });

  it("banks the model right for positive JSBSim roll", () => {
    const attitude = flightAttitudeToQuaternion(Math.PI / 6, 0, 0);
    const wingAxis = Vector3.Zero();
    new Vector3(-1, 0, 0).rotateByQuaternionToRef(attitude, wingAxis);
    expect(wingAxis.y).toBeCloseTo(-0.5, 5);
  });

  it("converts JSBSim down velocity to climb-positive vertical speed", () => {
    const sdk = {
      getPropertyValue: (property: string) => property === "velocities/v-down-fps" ? -12 : 0,
    } as JSBSimSdk;

    expect(readFlightState(sdk).verticalSpeedFps).toBe(12);
  });
});

describe("native flight-state reads", () => {
  it("resolves properties once, observes fresh values, and returns independent state records", () => {
    const properties: Record<string, number> = {
      "position/lat-geod-deg": 45, "position/long-gc-deg": -93, "position/h-sl-ft": 1000,
      "attitude/phi-deg": 30, "attitude/theta-deg": 10, "attitude/psi-deg": 90,
      "velocities/vc-kts": 120, "velocities/v-north-fps": 1, "velocities/v-east-fps": 2,
      "velocities/v-down-fps": -12, "fcs/throttle-cmd-norm": 0.6,
    };
    const dispose = vi.fn();
    const createPropertyBatch = vi.fn((paths: readonly string[]) => ({
      missing: [], dispose,
      read: vi.fn((target: Float64Array) => { target.set(paths.map(path => properties[path])); return target; }),
    }));
    const scalar = vi.fn((path: string) => properties[path]);
    const sdk = { createPropertyBatch, getPropertyValue: scalar } as unknown as JSBSimSdk;
    const first = readFlightState(sdk);
    expect(first).toEqual(readFlightState({ getPropertyValue: scalar } as unknown as JSBSimSdk));
    scalar.mockClear();
    properties["position/h-sl-ft"] = 2000;
    properties["velocities/v-down-fps"] = Number.NaN;
    const next = readFlightState(sdk);
    expect(first.altMeters).toBe(304.8);
    expect(next.altMeters).toBe(609.6);
    expect(next.verticalSpeedFps).toBeNaN();
    expect(scalar).not.toHaveBeenCalled();
    expect(createPropertyBatch).toHaveBeenCalledOnce();
    expect(createPropertyBatch.mock.calls[0]).toEqual([expect.any(Array), { create: false }]);
    releaseFlightStateReads(sdk);
    releaseFlightStateReads(sdk);
    expect(dispose).toHaveBeenCalledOnce();
    readFlightState(sdk);
    expect(createPropertyBatch).toHaveBeenCalledTimes(2);
    releaseFlightStateReads(sdk);
  });

  it("retains scalar behavior when a WASM build lacks the batch binding", () => {
    const createPropertyBatch = vi.fn(() => { throw new Error("No binding"); });
    const sdk = { createPropertyBatch, getPropertyValue: () => 1 } as unknown as JSBSimSdk;
    expect(readFlightState(sdk).latDeg).toBe(1);
    readFlightState(sdk);
    expect(createPropertyBatch).toHaveBeenCalledOnce();
    releaseFlightStateReads(sdk);
  });
});
