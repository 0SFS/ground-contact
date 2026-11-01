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
    if (contact.kind === "wheel") {
      values[`${prefix}compression-ft`] = 0;
      values[`${prefix}WOW`] = 0;
    }
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
    // The scalar fallback reads CG plus compression/WOW, but no static
    // contact geometry. The native backend refreshes these in one batch.
    expect(sdk.getPropertyValue).toHaveBeenCalledTimes(reads + 9);
    overlay.setEnabled(false);
    overlay.update();
    expect(sdk.getPropertyValue).toHaveBeenCalledTimes(reads + 9);
    overlay.dispose();
    expect(scene.meshes).toHaveLength(0);
  });

  it("moves wheel dots with native strut compression even when CG is unchanged", () => {
    const scene = createScene();
    const { sdk, values } = createSdk();
    const overlay = createCollisionDebugOverlay(scene, new TransformNode("aircraft", scene), sdk as never, C172_BODY_COLLISION_PROBES);
    overlay.setEnabled(true);
    const wheel = scene.getMeshByName("collision-debug/wheel/gear/unit[1]")!;
    const structure = scene.getMeshByName("collision-debug/structure/contact/unit[3]")!;
    const wheelRest = wheel.position.clone(), structureRest = structure.position.clone();
    const guides = scene.getMeshByName("collision-debug/guides")!;
    const upload = vi.spyOn(guides, "updateVerticesData");
    const readGeometry = () => sdk.getPropertyValue.mock.calls.filter(([path]) => /-position$/.test(path)).length;
    const geometryReads = readGeometry();

    overlay.update();
    expect(upload).not.toHaveBeenCalled();
    values["gear/unit[1]/WOW"] = 1;
    values["gear/unit[1]/compression-ft"] = 0.2 / 0.3048;
    values["contact/unit[3]/compression-ft"] = 10;
    overlay.update();
    expect(wheel.position.asArray()).toEqual([wheelRest.x, wheelRest.y + 0.2, wheelRest.z]);
    expect(structure.position.equals(structureRest)).toBe(true);
    expect(readGeometry()).toBe(geometryReads);
    expect(upload).toHaveBeenCalledOnce();

    overlay.update();
    expect(upload).toHaveBeenCalledOnce();
    // No accumulated offset on further compression or rebound.
    values["gear/unit[1]/compression-ft"] = 0.1 / 0.3048;
    overlay.update();
    expect(wheel.position.y).toBeCloseTo(wheelRest.y + 0.1, 12);
    values["gear/unit[1]/WOW"] = 0;
    overlay.update();
    expect(wheel.position.equals(wheelRest)).toBe(true);
    overlay.dispose();
  });

  it("rides a retractable wheel's dot with its tire while the gear moves and hides it once stowed", () => {
    const scene = createScene();
    const { sdk, values } = createSdk();
    // The main wheels retract; the nose wheel stays fixed.
    const catalog = sdk.queryPropertyCatalog() + "\ngear/unit[1]/pos-norm (RW)\ngear/unit[2]/pos-norm (RW)";
    sdk.queryPropertyCatalog.mockReturnValue(catalog);
    values["gear/gear-pos-norm"] = 1;
    const aircraft = new TransformNode("aircraft", scene);
    aircraft.position.set(10, 20, 30);
    aircraft.rotationQuaternion = Quaternion.RotationAxis(Vector3.Up(), 0.4);
    // Unit 1's leg hinges on the aircraft; unit 2 has no wheel loaded.
    const leg = new TransformNode("leftSuspension", scene);
    leg.parent = aircraft;
    leg.position.set(1.2, -0.4, -0.6);
    let loaded = false;
    const carrier = vi.fn((unit: number) => loaded && unit === 1 ? leg : null);
    const overlay = createCollisionDebugOverlay(scene, aircraft, sdk as never, [], { carrier });
    overlay.setEnabled(true);
    const nose = scene.getMeshByName("collision-debug/wheel/gear/unit")!;
    const left = scene.getMeshByName("collision-debug/wheel/gear/unit[1]")!;
    const right = scene.getMeshByName("collision-debug/wheel/gear/unit[2]")!;
    const noseRest = nose.position.clone(), leftRest = left.position.clone(), rightRest = right.position.clone();
    expect(carrier).toHaveBeenCalledWith(1);
    expect(carrier).not.toHaveBeenCalledWith(0);
    // The rig loads after the dot is placed, with the gear down.
    loaded = true;
    overlay.update();

    // Mid-transit, after this frame's rig pose but before Babylon rendered it.
    leg.computeWorldMatrix(true);
    const swing = (angle: number) => {
      leg.rotationQuaternion = Quaternion.RotationAxis(Vector3.Left(), angle);
      return leftRest.subtract(leg.position).applyRotationQuaternion(leg.rotationQuaternion).addInPlace(leg.position);
    };
    values["gear/gear-pos-norm"] = 0.6;
    let expected = swing(0.7);
    overlay.update();
    expect(left.isVisible).toBe(true);
    for (const axis of [0, 1, 2]) expect(left.position.asArray()[axis]).toBeCloseTo(expected.asArray()[axis], 5);
    expect(left.position.subtract(leftRest).length()).toBeGreaterThan(0.1);
    // A unit without a wheel keeps its extended contact; the fixed wheel never moves.
    expect(right.isVisible).toBe(true);
    expect(right.position.equals(rightRest)).toBe(true);
    expect(nose.position.equals(noseRest)).toBe(true);

    // Stowed, the retractable dots go; the fixed wheel's stays.
    values["gear/gear-pos-norm"] = 0;
    swing(1.5);
    overlay.update();
    expect([left.isVisible, right.isVisible, nose.isVisible]).toEqual([false, false, true]);
    values["gear/gear-pos-norm"] = 0.3;
    expected = swing(1.2);
    overlay.update();
    expect(left.isVisible).toBe(true);
    for (const axis of [0, 1, 2]) expect(left.position.asArray()[axis]).toBeCloseTo(expected.asArray()[axis], 5);

    // Down and locked again, it is the compressed contact once more.
    values["gear/gear-pos-norm"] = 1;
    values["gear/unit[1]/WOW"] = 1;
    values["gear/unit[1]/compression-ft"] = 0.15 / 0.3048;
    leg.rotationQuaternion = Quaternion.Identity();
    overlay.update();
    expect(left.position.x).toBe(leftRest.x);
    expect(left.position.y).toBeCloseTo(leftRest.y + 0.15, 12);
    expect(left.position.z).toBe(leftRest.z);
    overlay.dispose();
  });

  it("rides from the first extension when the rig can pose the gear down, and leaves its pose as it was", () => {
    const scene = createScene();
    const { sdk, values } = createSdk();
    const catalog = sdk.queryPropertyCatalog() + "\ngear/unit[1]/pos-norm (RW)";
    sdk.queryPropertyCatalog.mockReturnValue(catalog);
    const aircraft = new TransformNode("aircraft", scene);
    aircraft.rotationQuaternion = Quaternion.RotationAxis(Vector3.Up(), -0.3);
    const leg = new TransformNode("leftSuspension", scene);
    leg.parent = aircraft;
    leg.position.set(1.2, -0.4, -0.6);
    const swing = (angle: number) => { leg.rotationQuaternion = Quaternion.RotationAxis(Vector3.Left(), angle); };
    // The view opens with the gear stowed; the rig poses it down only on request.
    values["gear/gear-pos-norm"] = 0;
    swing(1.5);
    let stowedAngle = 1.5;
    const withGearDown = vi.fn((read: () => void) => {
      swing(0);
      try { read(); } finally { swing(stowedAngle); }
    });
    const overlay = createCollisionDebugOverlay(scene, aircraft, sdk as never, [], { carrier: unit => unit === 1 ? leg : null, withGearDown });
    overlay.setEnabled(true);
    const left = scene.getMeshByName("collision-debug/wheel/gear/unit[1]")!;
    expect(left.isVisible).toBe(false);
    expect(withGearDown).toHaveBeenCalledOnce();
    // The stowed pose is back, and so is its world matrix.
    const stowed = leg.getWorldMatrix().clone();
    leg.computeWorldMatrix(true);
    expect(stowed.asArray()).toEqual(leg.getWorldMatrix().asArray());

    // The first extension: the dot is on the tire from the start.
    values["gear/gear-pos-norm"] = 0.2;
    stowedAngle = 1.2;
    swing(stowedAngle);
    overlay.update();
    expect(left.isVisible).toBe(true);
    const riding = left.position.clone();
    const turn = leg.rotationQuaternion!.clone();
    // Down and locked, it is at the contact, and the tire got there with it.
    values["gear/gear-pos-norm"] = 1;
    swing(0);
    overlay.update();
    const expected = left.position.subtract(leg.position).applyRotationQuaternion(turn).addInPlace(leg.position);
    for (const axis of [0, 1, 2]) expect(riding.asArray()[axis]).toBeCloseTo(expected.asArray()[axis], 5);
    expect(riding.subtract(left.position).length()).toBeGreaterThan(0.1);
    expect(withGearDown).toHaveBeenCalledOnce();
    overlay.dispose();
  });

  it("reuses and disposes one native batch for the live CG and compression properties", () => {
    const scene = createScene();
    const { sdk, values } = createSdk();
    const dispose = vi.fn();
    const read = vi.fn<(target: Float64Array) => void>();
    const createPropertyBatch = vi.fn((paths: readonly string[]) => {
      read.mockImplementation(target => { paths.forEach((path, index) => { target[index] = values[path]; }); });
      return { missing: [], read, dispose };
    });
    const overlay = createCollisionDebugOverlay(scene, new TransformNode("aircraft", scene),
      { ...sdk, createPropertyBatch } as never, []);
    expect(createPropertyBatch).not.toHaveBeenCalled();
    overlay.setEnabled(true);
    expect(createPropertyBatch).toHaveBeenCalledOnce();
    expect(read).toHaveBeenCalledOnce();
    sdk.getPropertyValue.mockClear();
    overlay.update();
    expect(read).toHaveBeenCalledTimes(2);
    expect(sdk.getPropertyValue).not.toHaveBeenCalled();
    overlay.dispose(); overlay.dispose();
    expect(dispose).toHaveBeenCalledOnce();
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
