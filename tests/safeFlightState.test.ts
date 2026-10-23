import { describe, expect, it, vi } from "vitest";
import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import { captureSimulation, releaseSimulationCapture, restoreSimulation } from "../src/safeFlightState";

const thermal = (index?: number) => `propulsion/engine${index === undefined ? "" : `[${index}]`}/thermal/`;
const statePath = (index?: number) => `${thermal(index)}metal-temperature-state-k`;
const initializedPath = (index?: number) => `${thermal(index)}initialized`;

function fixture(catalog: readonly string[] = [
  `${statePath()} (RW)`, `${initializedPath()} (R)`,
  `${statePath(3)} (RW)`, `${initializedPath(3)} (R)`,
]) {
  const values: Record<string, number> = {
    "propulsion/engine/set-running": 1, "simulation/sim-time-sec": 42,
    [statePath()]: 840, [initializedPath()]: 1,
    [statePath(3)]: 620, [initializedPath(3)]: 1,
  };
  const startupStates: Record<string, number>[] = [];
  const getPropertyValue = vi.fn((path: string) => values[path] ?? 0);
  const queryPropertyCatalog = vi.fn((query: string) => catalog.filter(line => line.startsWith(query)).join("\n"));
  const setPropertyValue = vi.fn((path: string, value: number) => {
    values[path] = value;
    if (path.endsWith("/thermal/metal-temperature-state-k")) {
      values[path.replace(/metal-temperature-state-k$/, "initialized")] = 1;
    }
    if (path === "propulsion/set-running") {
      startupStates.push({ [statePath()]: values[statePath()], [statePath(3)]: values[statePath(3)] });
      for (const index of [undefined, 3]) {
        if (!values[initializedPath(index)]) {
          values[statePath(index)] = 1100;
          values[initializedPath(index)] = 1;
        }
      }
    }
  });
  const resetToInitialConditions = vi.fn(() => {
    for (const index of [undefined, 3]) {
      values[statePath(index)] = 273.15;
      values[initializedPath(index)] = 0;
    }
  });
  const sdk = {
    getPropertyValue, queryPropertyCatalog, setPropertyValue, resetToInitialConditions,
    runIc: vi.fn(() => true), setSimTime: vi.fn(),
  };
  return { sdk, native: sdk as unknown as JSBSimSdk, values, startupStates };
}

describe("brake commands in simulation snapshots", () => {
  it("preserves the pilot brake independently of differential wheel commands", () => {
    const t = fixture();
    Object.assign(t.values, { "fcs/brake-cmd-norm": 0.4, "fcs/left-brake-cmd-norm": 0.2, "fcs/right-brake-cmd-norm": 0.7 });
    const snapshot = captureSimulation(t.native);
    expect(snapshot.controls["fcs/brake-cmd-norm"]).toBe(0.4);
    t.values["fcs/brake-cmd-norm"] = 0;
    restoreSimulation(t.native, snapshot);
    expect(t.values["fcs/brake-cmd-norm"]).toBe(0.4);
  });

  it("restores the equal brake command from snapshots made before differential braking", () => {
    const t = fixture();
    const snapshot = captureSimulation(t.native);
    delete snapshot.controls["fcs/brake-cmd-norm"];
    snapshot.controls["fcs/left-brake-cmd-norm"] = snapshot.controls["fcs/right-brake-cmd-norm"] = 0.6;
    restoreSimulation(t.native, snapshot);
    expect(t.values["fcs/brake-cmd-norm"]).toBe(0.6);
  });
});

describe("native engine thermal state in simulation snapshots", () => {
  it.each(["core", "2nd-wall"])("captures and restores named solid %s using its own initialization flag", region => {
    const core = `${thermal()}${region}/metal-temperature-state-k`;
    const coreReady = `${thermal()}${region}/initialized`;
    const pending = `${thermal()}pending/metal-temperature-state-k`;
    const pendingReady = `${thermal()}pending/initialized`;
    const t = fixture([`${statePath()} (RW)`, `${initializedPath()} (R)`,
      `${core} (RW)`, `${coreReady} (R)`, `${pending} (RW)`, `${pendingReady} (R)`]);
    t.values[core] = 1200;
    t.values[coreReady] = 1;
    t.values[pending] = 300;
    t.values[pendingReady] = 0;
    const snapshot = captureSimulation(t.native);
    expect(snapshot.controls[core]).toBe(1200);
    expect(snapshot.controls[statePath()]).toBe(840);
    expect(snapshot.controls[pending]).toBeUndefined();
    t.values[core] = 500;
    restoreSimulation(t.native, snapshot);
    expect(t.values[core]).toBe(1200);
    const write = t.sdk.setPropertyValue.mock.calls.findIndex(([path]) => path === core);
    expect(t.sdk.setPropertyValue.mock.invocationCallOrder[write]).toBeLessThan(t.sdk.runIc.mock.invocationCallOrder[0]);
  });

  it("discovers writable state once, preserves all native indices, and leaves pending or unavailable state alone", () => {
    const t = fixture([
      `${statePath()} (RW)`, `${initializedPath()} (R)`,
      `${statePath(3)} (RW)`, `${initializedPath(3)} (R)`,
      `${statePath(1)} (R)`, `${initializedPath(1)} (R)`,
      `${statePath(2)} (W)`, `${initializedPath(2)} (R)`,
      `${statePath(4)} (RW)`,
      `${statePath(5)} (RW)`, `${initializedPath(5)} (R)`,
      `${statePath(6)} (RW)`, `${initializedPath(6)} (R)`,
    ]);
    t.values[statePath(5)] = 300;
    t.values[initializedPath(5)] = 0;
    t.values[statePath(6)] = Number.NaN;
    t.values[initializedPath(6)] = 1;
    const first = captureSimulation(t.native);
    expect(first.controls[statePath()]).toBe(840);
    expect(first.controls[statePath(3)]).toBe(620);
    for (const index of [1, 2, 4, 5, 6]) expect(first.controls[statePath(index)]).toBeUndefined();
    for (const index of [1, 2, 4, 5]) expect(t.sdk.getPropertyValue).not.toHaveBeenCalledWith(statePath(index));
    expect(t.sdk.setPropertyValue).not.toHaveBeenCalled();

    // Capability discovery is cached, but initialized state is observed anew.
    t.values[statePath(5)] = 900;
    t.values[initializedPath(5)] = 1;
    expect(captureSimulation(t.native).controls[statePath(5)]).toBe(900);
    expect(t.sdk.queryPropertyCatalog.mock.calls.filter(([query]) => query === "propulsion/engine")).toHaveLength(1);
  });

  it("restores captured metal temperatures before warm initialization can replace them", () => {
    const t = fixture();
    const snapshot = captureSimulation(t.native);
    restoreSimulation(t.native, snapshot);
    expect(t.sdk.resetToInitialConditions).toHaveBeenCalledWith(2);
    expect(t.startupStates).toEqual([{ [statePath()]: 840, [statePath(3)]: 620 }]);
    expect(t.values[statePath()]).toBe(840);
    expect(t.values[statePath(3)]).toBe(620);
    expect(t.values[initializedPath()]).toBe(1);
    expect(t.values[initializedPath(3)]).toBe(1);
    const firstRun = t.sdk.runIc.mock.invocationCallOrder[0];
    for (const path of [statePath(), statePath(3)]) {
      const index = t.sdk.setPropertyValue.mock.calls.findIndex(([written]) => written === path);
      expect(t.sdk.setPropertyValue.mock.invocationCallOrder[index]).toBeLessThan(firstRun);
    }
  });

  it("does not replay foreign thermal nodes onto a model that has no matching native capability", () => {
    const t = fixture([]);
    const snapshot = captureSimulation(t.native);
    snapshot.controls[statePath(7)] = 950;
    snapshot.controls[`${thermal(7)}metal-temperature-k`] = 950;
    restoreSimulation(t.native, snapshot);
    expect(t.sdk.setPropertyValue.mock.calls.filter(([path]) => path.includes("/thermal/"))).toEqual([]);
    expect(t.sdk.getPropertyValue.mock.calls.filter(([path]) => path.includes("/thermal/"))).toEqual([]);
  });
});

describe("hung stores in simulation snapshots", () => {
  const storeCatalog = [
    "stores/external-tank[0]/attached (RW)",
    "stores/station[1]/occupied (RW)", "stores/station[1]/store-id (RW)",
    "stores/station[1]/mass-lbs (RW)", "stores/station[1]/drag-area-ft2 (RW)",
    "stores/station[1]/drag-lbs (R)",
    "stores/bay[0]/door-cmd-norm (RW)", "stores/bay[0]/door-pos-norm (RW)",
    "stores/gun/rounds (RW)", "stores/master-arm (RW)",
  ];

  it("keeps the hung load and leaves derived drag out", () => {
    const t = fixture(storeCatalog);
    Object.assign(t.values, {
      "stores/station[1]/occupied": 1, "stores/station[1]/store-id": 3,
      "stores/station[1]/mass-lbs": 1013, "stores/station[1]/drag-area-ft2": 1,
      "stores/station[1]/drag-lbs": 40, "stores/bay[0]/door-cmd-norm": 1,
      "stores/bay[0]/door-pos-norm": 0.4, "stores/gun/rounds": 220, "stores/master-arm": 0,
    });
    const snapshot = captureSimulation(t.native);
    expect(snapshot.controls["stores/station[1]/mass-lbs"]).toBe(1013);
    expect(snapshot.controls["stores/station[1]/store-id"]).toBe(3);
    expect(snapshot.controls["stores/bay[0]/door-pos-norm"]).toBe(0.4);
    expect(snapshot.controls["stores/gun/rounds"]).toBe(220);
    expect(snapshot.controls["stores/master-arm"]).toBe(0);
    expect(snapshot.controls["stores/external-tank[0]/attached"]).toBe(0);
    expect(snapshot.controls).not.toHaveProperty("stores/station[1]/drag-lbs");
    snapshot.controls["stores/station[1]/drag-lbs"] = 40;
    t.values["stores/station[1]/mass-lbs"] = 0;
    restoreSimulation(t.native, snapshot);
    expect(t.values["stores/station[1]/mass-lbs"]).toBe(1013);
    expect(t.sdk.setPropertyValue).not.toHaveBeenCalledWith("stores/station[1]/drag-lbs", 40);
  });

  it("does not invent weapon stations on a model that has none", () => {
    const t = fixture([]);
    const snapshot = captureSimulation(t.native);
    snapshot.controls["stores/station[1]/occupied"] = 1;
    snapshot.controls["stores/station[1]/mass-lbs"] = 1013;
    snapshot.controls["stores/master-arm"] = 1;
    restoreSimulation(t.native, snapshot);
    expect(t.sdk.setPropertyValue).not.toHaveBeenCalledWith("stores/station[1]/occupied", 1);
    expect(t.sdk.setPropertyValue).not.toHaveBeenCalledWith("stores/master-arm", 1);
    expect(t.values["stores/station[1]/occupied"]).toBeUndefined();
  });
});

describe("batched simulation capture", () => {
  function withBatch(t: ReturnType<typeof fixture>, missing: string[] = []) {
    const batches: { read: ReturnType<typeof vi.fn>; dispose: ReturnType<typeof vi.fn> }[] = [];
    const createPropertyBatch = vi.fn((paths: readonly string[]) => {
      const batch = {
        missing, dispose: vi.fn(),
        read: vi.fn((target: Float64Array) => {
          target.set(paths.map(path => missing.includes(path) ? Number.NaN : t.values[path] ?? 0));
          return target;
        }),
      };
      batches.push(batch);
      return batch;
    });
    Object.assign(t.sdk, { createPropertyBatch });
    return { batches, createPropertyBatch };
  }

  it("matches scalar snapshots including native plant state and pending thermal state", () => {
    const prefix = "propulsion/engine[2]/plant/state/";
    const catalog = [`${statePath()} (RW)`, `${initializedPath()} (R)`,
      `${statePath(3)} (RW)`, `${initializedPath(3)} (R)`,
      "fcs/stovl-pos-norm (RW)", "stores/external-tank[4]/attached (RW)", "propulsion/tank[5]/contents-lbs (RW)",
      `${prefix}spool-rpm (RW)`, `${prefix}fuel-mass (RW)`, `${prefix}commit (RW)`];
    const scalar = fixture(catalog), batched = fixture(catalog);
    Object.assign(scalar.values, { [initializedPath(3)]: 0, [prefix + "spool-rpm"]: 6500,
      [prefix + "fuel-mass"]: 0.08, "fcs/stovl-pos-norm": 0.6,
      "stores/external-tank[4]/attached": 1, "propulsion/tank[5]/contents-lbs": 500 });
    Object.assign(batched.values, scalar.values);
    const b = withBatch(batched);
    const first = captureSimulation(batched.native);
    expect(first).toEqual(captureSimulation(scalar.native));
    expect(first.controls["stores/external-tank[4]/attached"]).toBe(1);
    expect(first.controls).not.toHaveProperty(prefix + "commit");
    expect(first.controls).not.toHaveProperty(statePath(3));
    expect(batched.sdk.getPropertyValue).not.toHaveBeenCalled();
    expect(b.createPropertyBatch.mock.calls[0]).toEqual([expect.any(Array), { create: false }]);
    batched.values[prefix + "spool-rpm"] = 7000;
    batched.values[initializedPath(3)] = 1;
    const next = captureSimulation(batched.native);
    expect(first.controls[prefix + "spool-rpm"]).toBe(6500);
    expect(next.controls[prefix + "spool-rpm"]).toBe(7000);
    expect(next.controls[statePath(3)]).toBe(620);
    expect(b.createPropertyBatch).toHaveBeenCalledOnce();
    expect(b.batches[0].read).toHaveBeenCalledTimes(2);
    releaseSimulationCapture(batched.native);
    releaseSimulationCapture(batched.native);
    expect(b.batches[0].dispose).toHaveBeenCalledOnce();
    captureSimulation(batched.native);
    expect(b.createPropertyBatch).toHaveBeenCalledTimes(2);
    releaseSimulationCapture(batched.native);
  });

  it("uses scalar values for missing nodes and preserves live non-finite values for rejection", () => {
    const t = fixture();
    const missing = "fcs/mixture-cmd-norm";
    withBatch(t, [missing]);
    t.values[missing] = 0.25;
    t.values["position/h-sl-ft"] = Number.NaN;
    const snapshot = captureSimulation(t.native);
    expect(snapshot.controls[missing]).toBe(0.25);
    expect(t.sdk.getPropertyValue.mock.calls).toEqual([[missing]]);
    expect(snapshot.initial["ic/h-sl-ft"]).toBeNaN();
    expect(() => restoreSimulation(t.native, snapshot)).toThrow("Invalid simulation snapshot");
    expect(t.sdk.resetToInitialConditions).not.toHaveBeenCalled();
    releaseSimulationCapture(t.native);
  });

  it("does not return a partial snapshot when a batch read fails", () => {
    const t = fixture();
    const b = withBatch(t);
    captureSimulation(t.native);
    b.batches[0].read.mockImplementationOnce(() => { throw new Error("Native read failed"); });
    expect(() => captureSimulation(t.native)).toThrow("Native read failed");
    releaseSimulationCapture(t.native);
  });
});
