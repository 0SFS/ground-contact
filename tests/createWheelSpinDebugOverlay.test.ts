import { Constants, Matrix, Mesh, MeshBuilder, NullEngine, Quaternion, Scene, TransformNode, Vector3, VertexBuffer } from "@babylonjs/core";
import { expect, it, vi } from "vitest";
import { WHEEL_SPIN_CONFIGS, createWheelSpinState } from "../src/wheelSpin";
import { C172_GROUND_CONTACTS, groundContactBodyPosition } from "../src/collisionGeometry";
import { createAircraftWheelDebugOverlay, createWheelSpinDebugOverlay, measureTireOutline } from "../src/createWheelSpinDebugOverlay";

it("places spinning outlines above the contact references and disposes their resources", () => {
  const engine = new NullEngine();
  const scene = new Scene(engine);
  try {
    const parent = new TransformNode("aircraft", scene);
    const cg = { xIn: 42, yIn: 0, zIn: 38 };
    const sdk = { getPropertyValue: vi.fn((key: string) => ({ "inertia/cg-x-in": cg.xIn,
      "inertia/cg-y-in": cg.yIn, "inertia/cg-z-in": cg.zIn })[key]) };
    const states = WHEEL_SPIN_CONFIGS.map(() => createWheelSpinState());
    states[0].compressionMeters = 0.03;
    states[0].steeringRad = 0.1;
    states[0].angleRad = 1.2;
    const overlay = createWheelSpinDebugOverlay(scene, parent, sdk as never, () => states);
    expect(scene.meshes).toHaveLength(0);
    expect(sdk.getPropertyValue).not.toHaveBeenCalled();
    const materialCount = scene.materials.length;
    overlay.setEnabled(true);
    expect(scene.meshes).toHaveLength(3);
    expect(scene.meshes.every(mesh => !mesh.isPickable && mesh.metadata.collisionDebug.kind === "wheel-spin")).toBe(true);
    const axle = scene.getTransformNodeByName("wheel-spin-debug/NOSE/axle")!;
    const rotor = scene.getTransformNodeByName("wheel-spin-debug/NOSE/rotor")!;
    const contact = groundContactBodyPosition(C172_GROUND_CONTACTS[0], cg);
    expect(axle.position.y).toBeCloseTo(contact.up + 0.18 + 0.03);
    expect(axle.position.z).toBeCloseTo(contact.forward);
    expect(axle.rotation.y).toBe(-0.1);
    expect(rotor.rotation.x).toBe(1.2);
    states[0].angleRad = 2.3;
    overlay.update();
    expect(rotor.rotation.x).toBe(2.3);
    overlay.setEnabled(false);
    sdk.getPropertyValue.mockClear();
    overlay.update();
    expect(sdk.getPropertyValue).not.toHaveBeenCalled();
    expect(axle.isEnabled()).toBe(false);
    overlay.dispose();
    overlay.dispose();
    expect(scene.meshes).toHaveLength(0);
    expect(scene.materials).toHaveLength(materialCount);
    expect(scene.transformNodes).toEqual([parent]);
  } finally { engine.dispose(); }
});

it("anchors rings and spokes to real wheel geometry through model scale, spin and gear motion", () => {
  const engine = new NullEngine();
  const scene = new Scene(engine);
  try {
    const aircraft = new TransformNode("aircraft", scene);
    aircraft.position.set(12, -4, 8);
    aircraft.scaling.setAll(0.833);
    aircraft.rotationQuaternion = Quaternion.FromEulerAngles(0.2, 0.8, -0.1);
    const leg = new TransformNode("leg", scene);
    leg.parent = aircraft;
    leg.position.set(-2, 1.2, 0.7);
    const wheel = MeshBuilder.CreateCylinder("actual-wheel", { diameter: 1, height: 0.2, tessellation: 32 }, scene);
    wheel.rotation.z = Math.PI / 2;
    wheel.bakeCurrentTransformIntoVertices();
    // Authored mesh geometry need not be centred exactly on its node origin.
    wheel.bakeTransformIntoVertices(Matrix.Translation(0.03, 0.04, -0.02));
    wheel.parent = leg;
    wheel.position.set(0.1, -0.5, 0.2);
    const getWheels = vi.fn(() => [{ node: wheel }]);
    const overlay = createAircraftWheelDebugOverlay(scene, getWheels);
    expect(getWheels).not.toHaveBeenCalled();
    overlay.setEnabled(true);
    const drawing = scene.getMeshByName("wheel-spin-debug/actual-wheel/tire")!;
    expect(drawing.parent).toBe(wheel);
    expect(drawing.isPickable).toBe(false);
    expect(drawing.material!.depthFunction).toBe(Constants.ALWAYS);
    const vertices = drawing.getVerticesData(VertexBuffer.PositionKind)!;
    const initialWorld = Vector3.TransformCoordinates(Vector3.FromArray(vertices, 0), drawing.computeWorldMatrix(true));
    // The first ring's top: the tire's port face, centred on its own geometry.
    const expectedLocal = new Vector3(-0.1 + 0.03, 0.04 + 0.5, -0.02);
    expect(Vector3.Distance(initialWorld, Vector3.TransformCoordinates(expectedLocal, wheel.computeWorldMatrix(true)))).toBeLessThan(1e-6);
    const meshCount = scene.meshes.length;
    const materialCount = scene.materials.length;
    wheel.rotationQuaternion = Quaternion.RotationAxis(Vector3.Right(), 0.7);
    leg.rotationQuaternion = Quaternion.FromEulerAngles(0.4, 0.3, -0.2);
    overlay.update();
    const posedWorld = Vector3.TransformCoordinates(Vector3.FromArray(vertices, 0), drawing.computeWorldMatrix(true));
    expect(Vector3.Distance(posedWorld, Vector3.TransformCoordinates(expectedLocal, wheel.computeWorldMatrix(true)))).toBeLessThan(1e-6);
    expect(Vector3.Distance(posedWorld, initialWorld)).toBeGreaterThan(0.1);
    expect(drawing.getVerticesData(VertexBuffer.PositionKind)).toBe(vertices);
    expect(scene.meshes).toHaveLength(meshCount);
    expect(scene.materials).toHaveLength(materialCount);
    overlay.dispose();
    expect(wheel.isDisposed()).toBe(false);
    expect(scene.meshes).toEqual([wheel]);
  } finally { scene.dispose(); engine.dispose(); }
});

it("rings the tire, not a hub or axle stub that reaches past one sidewall", () => {
  const engine = new NullEngine();
  const scene = new Scene(engine);
  try {
    // As on AF267's F-35B wheels: a 0.2-wide tire with a narrow hub reaching
    // 0.5 to starboard, so the bounding box is 0.6 wide and off-centre.
    const tire = MeshBuilder.CreateCylinder("tire", { diameter: 1, height: 0.2, tessellation: 32 }, scene);
    const hub = MeshBuilder.CreateCylinder("hub", { diameter: 0.4, height: 0.6, tessellation: 16 }, scene);
    hub.position.y = 0.2;
    const wheel = Mesh.MergeMeshes([tire, hub], true)!;
    wheel.name = "hub-wheel";
    wheel.rotation.z = -Math.PI / 2;
    wheel.bakeCurrentTransformIntoVertices();
    expect(wheel.getBoundingInfo().boundingBox.maximum.x).toBeCloseTo(0.5, 6);
    expect(measureTireOutline(wheel)).toEqual({
      centerY: expect.closeTo(0, 6), centerZ: expect.closeTo(0, 6), radius: expect.closeTo(0.5, 6),
      minX: expect.closeTo(-0.1, 6), maxX: expect.closeTo(0.1, 6),
    });
    const overlay = createAircraftWheelDebugOverlay(scene, () => [{ node: wheel }]);
    overlay.setEnabled(true);
    const lines = scene.getMeshByName("wheel-spin-debug/hub-wheel/tire")!.getVerticesData(VertexBuffer.PositionKind)!;
    const xs = Array.from({ length: lines.length / 3 }, (_, index) => lines[index * 3]);
    expect(Math.min(...xs)).toBeCloseTo(-0.1, 6);
    expect(Math.max(...xs)).toBeCloseTo(0.1, 6);
    overlay.dispose();

    // No readable vertices: draw nothing rather than an outline guessed from bounds.
    const empty = new Mesh("empty-wheel", scene);
    expect(measureTireOutline(empty)).toBeNull();
    const none = createAircraftWheelDebugOverlay(scene, () => [{ node: empty }]);
    none.setEnabled(true);
    expect(scene.getMeshByName("wheel-spin-debug/empty-wheel/tire")).toBeNull();
    none.dispose();
  } finally { scene.dispose(); engine.dispose(); }
});

it("follows wheel visibility, rebinds a replaced LOD and releases its own resources", () => {
  const engine = new NullEngine();
  const scene = new Scene(engine);
  try {
    const oldWheel = MeshBuilder.CreateCylinder("old-wheel", { diameter: 0.4, height: 0.1 }, scene);
    const nextWheel = MeshBuilder.CreateCylinder("next-wheel", { diameter: 0.8, height: 0.2 }, scene);
    oldWheel.material = nextWheel.material = scene.defaultMaterial;
    let wheels = [{ node: oldWheel }];
    const getWheels = vi.fn(() => wheels);
    const materialCount = scene.materials.length;
    const overlay = createAircraftWheelDebugOverlay(scene, getWheels);
    overlay.setEnabled(true);
    const oldDrawing = scene.getMeshByName("wheel-spin-debug/old-wheel/tire")!;
    oldWheel.isVisible = false;
    overlay.update();
    expect(oldDrawing.isEnabled()).toBe(false);
    oldWheel.isVisible = true;
    overlay.update();
    expect(oldDrawing.isEnabled()).toBe(true);
    overlay.setEnabled(false);
    getWheels.mockClear();
    overlay.update();
    expect(getWheels).not.toHaveBeenCalled();
    expect(oldDrawing.isEnabled()).toBe(false);
    oldWheel.dispose();
    wheels = [{ node: nextWheel }];
    overlay.setEnabled(true);
    expect(oldDrawing.isDisposed()).toBe(true);
    const nextDrawing = scene.getMeshByName("wheel-spin-debug/next-wheel/tire")!;
    expect(nextDrawing.parent).toBe(nextWheel);
    expect(nextDrawing.isEnabled()).toBe(true);
    expect(scene.materials.length - materialCount).toBe(1);
    wheels = [];
    overlay.update();
    expect(nextDrawing.isDisposed()).toBe(true);
    expect(scene.materials).toHaveLength(materialCount);
    overlay.dispose();
    overlay.dispose();
    expect(nextWheel.isDisposed()).toBe(false);
    expect(scene.meshes).toEqual([nextWheel]);
  } finally { scene.dispose(); engine.dispose(); }
});
