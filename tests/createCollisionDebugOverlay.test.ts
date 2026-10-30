import { NullEngine, Quaternion, Scene, TransformNode, Vector3 } from "@babylonjs/core";
import { afterEach, describe, expect, it, vi } from "vitest";
import { C172_BODY_COLLISION_PROBES, C172_GROUND_CONTACTS, groundContactBodyPosition } from "../src/collisionGeometry";
import { collisionMarkerDiameter, createCollisionDebugOverlay } from "../src/createCollisionDebugOverlay";

const INCHES_PER_METER = 1 / 0.0254;

/** The C172's contacts as JSBSim reports them: BOGEY units under gear/, STRUCTURE under contact/. */
function createSdk(cg = { xIn: 42.117, yIn: -1.34, zIn: 37.739 }, completeCatalog = true) {
  const values: Record<string, number> = {
    "inertia/cg-x-in": cg.xIn,
    "inertia/cg-y-in": cg.yIn,
    "inertia/cg-z-in": cg.zIn,
    "gear/num-units": C172_GROUND_CONTACTS.length,
  };
  const catalog: string[] = [];
  C172_GROUND_CONTACTS.forEach((contact, index) => {
    const prefix = `${contact.kind === "wheel" ? "gear" : "contact"}/unit${index === 0 ? "" : `[${index}]`}/`;
    values[`${prefix}x-position`] = contact.xIn;
    values[`${prefix}y-position`] = contact.yIn;
    values[`${prefix}z-position`] = contact.zIn;
    if (completeCatalog || index > 0) catalog.push(`${prefix}x-position (RW)`, `${prefix}y-position (RW)`, `${prefix}z-position (RW)`);
  });
  return {
    values,
    sdk: {
      getPropertyValue: vi.fn((property: string) => values[property]),
      queryPropertyCatalog: vi.fn(() => catalog.join("\n")),
    },
  };
}

const position = (scene: Scene, name: string) => scene.getMeshByName(`collision-debug/${name}`)?.position.asArray();

describe("collision debug overlay", () => {
  const engines: NullEngine[] = [];
  afterEach(() => { for (const engine of engines.splice(0)) engine.dispose(); });

  function createScene() {
    const engine = new NullEngine();
    engines.push(engine);
    return new Scene(engine);
  }

  it("allocates and reads nothing until enabled, then renders the aircraft's probes and model contacts", () => {
    const scene = createScene();
    const aircraft = new TransformNode("aircraft", scene);
    const { sdk, values } = createSdk();
    const overlay = createCollisionDebugOverlay(scene, aircraft, sdk as never, C172_BODY_COLLISION_PROBES);

    expect(scene.meshes).toHaveLength(0);
    expect(sdk.getPropertyValue).not.toHaveBeenCalled();
    expect(sdk.queryPropertyCatalog).not.toHaveBeenCalled();

    overlay.setEnabled(true);
    const cg = { xIn: values["inertia/cg-x-in"], yIn: values["inertia/cg-y-in"], zIn: values["inertia/cg-z-in"] };
    // The nose probe is a hull point; the belly probe sweeps a point three inches inside the hull.
    const nose = groundContactBodyPosition(C172_BODY_COLLISION_PROBES[0], cg);
    expect(position(scene, "body/nose")).toEqual([nose.left, nose.up, nose.forward]);
    const belly = groundContactBodyPosition(C172_BODY_COLLISION_PROBES[1], cg);
    expect(position(scene, "body/belly")).toEqual([belly.left, belly.up + 3 * 0.0254, belly.forward]);
    expect(position(scene, "cg/center-of-gravity")).toEqual([0, 0, 0]);
    // Every unit the model reports, wheels and structure, under its property prefix.
    C172_GROUND_CONTACTS.forEach((contact, index) => {
      const name = `${contact.kind === "wheel" ? "wheel/gear" : "structure/contact"}/unit${index === 0 ? "" : `[${index}]`}`;
      const expected = groundContactBodyPosition(contact, cg);
      expect(position(scene, name)).toEqual([expected.left, expected.up, expected.forward]);
    });
    expect(scene.getMeshByName("collision-debug/guides")?.isPickable).toBe(false);
    expect(scene.meshes).toHaveLength(1 + C172_BODY_COLLISION_PROBES.length + C172_GROUND_CONTACTS.length + 1);
    expect(scene.meshes.every((mesh) => mesh.isPickable === false)).toBe(true);
    expect(scene.meshes.every((mesh) => mesh.metadata?.collisionDebug)).toBe(true);

    const reads = sdk.getPropertyValue.mock.calls.length;
    overlay.update();
    // An unchanged CG reads the CG and nothing else.
    expect(sdk.getPropertyValue).toHaveBeenCalledTimes(reads + 3);
    overlay.setEnabled(false);
    overlay.update();
    expect(sdk.getPropertyValue).toHaveBeenCalledTimes(reads + 3);
    overlay.dispose();
    expect(scene.meshes).toHaveLength(0);
  });

  it("follows the aircraft transform and uses the current JSBSim CG for probes and contacts", () => {
    const scene = createScene();
    const aircraft = new TransformNode("aircraft", scene);
    aircraft.position.set(10, 20, 30);
    aircraft.rotationQuaternion = Quaternion.RotationAxis(Vector3.Up(), Math.PI / 2);
    const { sdk, values } = createSdk();
    const overlay = createCollisionDebugOverlay(scene, aircraft, sdk as never, C172_BODY_COLLISION_PROBES);
    overlay.setEnabled(true);

    const cg = { xIn: values["inertia/cg-x-in"], yIn: values["inertia/cg-y-in"], zIn: values["inertia/cg-z-in"] };
    const local = groundContactBodyPosition(C172_BODY_COLLISION_PROBES[0], cg);
    const nose = scene.getMeshByName("collision-debug/body/nose")!;
    nose.computeWorldMatrix(true);
    const nosePosition = nose.getAbsolutePosition();
    // A quarter turn about up takes forward to +x and left to -z. Babylon's
    // world transform is Float32, enough for centimetre-level geometry.
    expect(nosePosition.x).toBeCloseTo(10 + local.forward, 5);
    expect(nosePosition.y).toBeCloseTo(20 + local.up, 5);
    expect(nosePosition.z).toBeCloseTo(30 - local.left, 5);

    const marker = scene.getMeshByName("collision-debug/wheel/gear/unit[1]")!;
    const expected = groundContactBodyPosition(C172_GROUND_CONTACTS[1], cg);
    values["inertia/cg-x-in"] += INCHES_PER_METER;
    overlay.update();
    expect(marker.position.z).toBeCloseTo(expected.forward + 1, 12);
    expect(nose.position.z).toBeCloseTo(local.forward + 1, 12);
  });

  it("hides every marker without a CG, and shows the probes alone for a model without complete contacts", () => {
    const scene = createScene();
    const aircraft = new TransformNode("aircraft", scene);
    const { sdk, values } = createSdk(undefined, false);
    values["inertia/cg-x-in"] = NaN;
    const overlay = createCollisionDebugOverlay(scene, aircraft, sdk as never, C172_BODY_COLLISION_PROBES);
    overlay.setEnabled(true);
    expect(scene.meshes).toHaveLength(1 + C172_BODY_COLLISION_PROBES.length + 1);
    expect(scene.getMeshByName("collision-debug/body/nose")?.isVisible).toBe(false);
    expect(position(scene, "body/nose")).toEqual([0, 0, 0]);
    values["inertia/cg-x-in"] = 42;
    overlay.update();
    expect(scene.getMeshByName("collision-debug/body/nose")?.isVisible).toBe(true);
    overlay.dispose();
  });

  it("tells what it shows and sizes its markers to the aircraft", () => {
    const scene = createScene();
    const { sdk } = createSdk();
    const plane = createCollisionDebugOverlay(scene, new TransformNode("plane", scene), sdk as never, C172_BODY_COLLISION_PROBES);
    expect(plane.legend()).toBeNull();
    plane.setEnabled(true);
    expect(plane.legend()).toEqual({ bodyProbes: C172_BODY_COLLISION_PROBES.length,
      wheels: C172_GROUND_CONTACTS.filter(contact => contact.kind === "wheel").length,
      structures: C172_GROUND_CONTACTS.filter(contact => contact.kind === "structure").length });
    expect(scene.getMeshByName("collision-debug/wheel/gear/unit")?.scaling.x).toBe(0.24);
    plane.dispose();

    // A small drone's four feet, about twelve centimetres from its CG, as JSBSim STRUCTURE units.
    const feet = [[-2.76, -3.76, -1.57], [-2.76, 3.76, -1.57], [1.55, -0.62, -1.21], [1.55, 0.62, -1.21]];
    const values: Record<string, number> = { "inertia/cg-x-in": -0.087, "inertia/cg-y-in": 0, "inertia/cg-z-in": -0.197, "gear/num-units": 4 };
    const catalog: string[] = [];
    feet.forEach(([x, y, z], unit) => {
      const prefix = `contact/unit[${unit}]/`;
      Object.assign(values, { [`${prefix}x-position`]: x, [`${prefix}y-position`]: y, [`${prefix}z-position`]: z });
      catalog.push(`${prefix}x-position (RW)`);
    });
    const drone = createCollisionDebugOverlay(scene, new TransformNode("drone", scene),
      { getPropertyValue: (property: string) => values[property], queryPropertyCatalog: () => catalog.join("\n") } as never, []);
    drone.setEnabled(true);
    expect(drone.legend()).toEqual({ bodyProbes: 0, wheels: 0, structures: 4 });
    const foot = scene.getMeshByName("collision-debug/structure/contact/unit[0]")!;
    expect(foot.scaling.x).toBeCloseTo(collisionMarkerDiameter(foot.position.length()), 12);
    expect(foot.scaling.x).toBeLessThan(0.015);
    expect(scene.getMeshByName("collision-debug/cg/center-of-gravity")?.scaling.x).toBeCloseTo(foot.scaling.x * 0.75, 12);
    drone.dispose();
  });
});
