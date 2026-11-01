import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import { AbstractMesh, Color3, Constants, CreateLineSystem, TransformNode, Vector3, VertexBuffer, type LinesMesh, type Material, type Scene } from "@babylonjs/core";
import { C172_GROUND_CONTACTS, groundContactBodyPosition } from "./collisionGeometry";
import { WHEEL_SPIN_CONFIGS, type WheelSpinState } from "./wheelSpin";

/** Shared drawing style; each outline owns its line material. */
function createTireOutline(scene: Scene, parent: TransformNode, name: string, lines: Vector3[][]): LinesMesh {
  const mesh = CreateLineSystem(`wheel-spin-debug/${name}/tire`, { lines }, scene);
  mesh.parent = parent;
  mesh.color = Color3.FromHexString("#ffbb46");
  mesh.alpha = 0.95;
  mesh.alphaIndex = 10002;
  mesh.isPickable = false;
  mesh.doNotSerialize = true;
  mesh.alwaysSelectAsActiveMesh = true;
  mesh.metadata = { collisionDebug: { kind: "wheel-spin", name } };
  if (mesh.material) {
    mesh.material.depthFunction = Constants.ALWAYS;
    mesh.material.disableDepthWrite = true;
  }
  return mesh;
}

/** C172-only illustration for the standalone wheel-feedback benchmark.
 * Not used by the app's Collision geometry view: these outlines are not collision
 * shapes and their hardcoded C172 coordinates are invalid for other aircraft.
 * Axles use the uncompressed contact references plus radius and vertical stroke.
 * Sloped ground/strut flex are intentionally not reconstructed here.
 */
export function createWheelSpinDebugOverlay(scene: Scene, parent: TransformNode, sdk: JSBSimSdk,
  getStates: () => readonly WheelSpinState[]) {
  let enabled = false;
  let disposed = false;
  let root: TransformNode | null = null;
  const axles: TransformNode[] = [];
  const rotors: TransformNode[] = [];

  function createGeometry(): void {
    root = new TransformNode("wheel-spin-debug", scene);
    root.parent = parent;
    root.doNotSerialize = true;
    for (const config of WHEEL_SPIN_CONFIGS) {
      const axle = new TransformNode(`wheel-spin-debug/${config.name}/axle`, scene);
      axle.parent = root;
      axle.doNotSerialize = true;
      axles.push(axle);
      const rotor = new TransformNode(`wheel-spin-debug/${config.name}/rotor`, scene);
      rotor.parent = axle;
      rotor.doNotSerialize = true;
      rotors.push(rotor);
      const lines: Vector3[][] = [];
      for (const side of [-1, 1]) {
        const x = side * config.widthMeters / 2;
        const ring = Array.from({ length: 33 }, (_, i) => new Vector3(x,
          Math.cos(i * Math.PI / 16) * config.radiusMeters, Math.sin(i * Math.PI / 16) * config.radiusMeters));
        lines.push(ring, [new Vector3(x, 0, 0), ring[0]]);
      }
      lines.push([new Vector3(-config.widthMeters / 2, 0, 0), new Vector3(config.widthMeters / 2, 0, 0)]);
      createTireOutline(scene, rotor, config.name, lines);
    }
  }

  function update(): void {
    if (!enabled || disposed || !root) return;
    const cg = { xIn: sdk.getPropertyValue("inertia/cg-x-in"), yIn: sdk.getPropertyValue("inertia/cg-y-in"),
      zIn: sdk.getPropertyValue("inertia/cg-z-in") };
    const valid = Object.values(cg).every(Number.isFinite);
    const states = getStates();
    for (let i = 0; i < axles.length; i++) {
      const state = states[i];
      const usable = valid && state && [state.compressionMeters, state.angleRad, state.steeringRad].every(Number.isFinite);
      axles[i].setEnabled(!!usable);
      if (!usable) continue;
      const contact = groundContactBodyPosition(C172_GROUND_CONTACTS[i], cg);
      axles[i].position.set(contact.left, contact.up + WHEEL_SPIN_CONFIGS[i].radiusMeters + state.compressionMeters, contact.forward);
      // JSBSim positive steering turns right; body +X here points left.
      axles[i].rotation.y = -state.steeringRad;
      rotors[i].rotation.x = state.angleRad;
    }
  }

  return {
    setEnabled(value: boolean): void {
      if (disposed || enabled === value) return;
      enabled = value;
      if (enabled && !root) createGeometry();
      root?.setEnabled(enabled);
      if (enabled) update();
    },
    update,
    dispose(): void {
      if (disposed) return;
      disposed = true;
      root?.dispose(false, true);
      root = null;
    },
  };
}

/** The portion of an aircraft rig's rolling wheel needed by its debug drawing. */
export interface AircraftWheelDebugSource {
  readonly node: TransformNode;
}

/** A tire's enclosing cylinder about the wheel's local X axis, in mesh units. */
export interface TireOutline {
  readonly centerY: number;
  readonly centerZ: number;
  readonly radius: number;
  readonly minX: number;
  readonly maxX: number;
}

/** Vertices nearer the axle than this fraction of the outer radius are hub. */
const TIRE_BAND = 0.5;

/**
 * Measure the tire from the wheel mesh's own vertices: its outer radius about
 * the axle and the side faces of the tire band. A wheel mesh can include a hub
 * or axle stub that reaches far past one sidewall (AF267's F-35B wheels do),
 * so the mesh's bounding box would put one ring beside the tire. Null without
 * readable vertices, rather than an outline guessed from the bounds.
 */
export function measureTireOutline(mesh: AbstractMesh): TireOutline | null {
  const positions = mesh.getVerticesData(VertexBuffer.PositionKind);
  if (!positions || positions.length < 3) return null;
  let minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (let at = 0; at < positions.length; at += 3) {
    minY = Math.min(minY, positions[at + 1]); maxY = Math.max(maxY, positions[at + 1]);
    minZ = Math.min(minZ, positions[at + 2]); maxZ = Math.max(maxZ, positions[at + 2]);
  }
  const centerY = (minY + maxY) / 2, centerZ = (minZ + maxZ) / 2;
  let radiusSquared = 0;
  for (let at = 0; at < positions.length; at += 3) {
    const y = positions[at + 1] - centerY, z = positions[at + 2] - centerZ;
    radiusSquared = Math.max(radiusSquared, y * y + z * z);
  }
  const bandSquared = radiusSquared * TIRE_BAND * TIRE_BAND;
  let minX = Infinity, maxX = -Infinity;
  for (let at = 0; at < positions.length; at += 3) {
    const y = positions[at + 1] - centerY, z = positions[at + 2] - centerZ;
    if (y * y + z * z < bandSquared) continue;
    minX = Math.min(minX, positions[at]); maxX = Math.max(maxX, positions[at]);
  }
  const radius = Math.sqrt(radiusSquared);
  return [centerY, centerZ, radius, minX, maxX].every(Number.isFinite) && radius > 0 && maxX > minX
    ? { centerY, centerZ, radius, minX, maxX } : null;
}

/**
 * Draw rings and spokes on the actual wheel mesh, in that mesh's own units.
 * Parenting carries its axle position, spin, authored rest rotation, suspension,
 * gear retraction and model scale without reconstructing any of them from a
 * second contact/aircraft definition. Measure and rebuild only when the rig/LOD
 * changes; nothing is scanned per frame.
 */
export function createAircraftWheelDebugOverlay(scene: Scene,
  getWheels: () => readonly AircraftWheelDebugSource[]) {
  let enabled = false;
  let disposed = false;
  let boundNodes: TransformNode[] = [];
  const drawings: { wheel: AbstractMesh; mesh: LinesMesh; material: Material | null }[] = [];

  function clear(): void {
    for (const drawing of drawings) {
      drawing.mesh.dispose();
      drawing.material?.dispose();
    }
    drawings.length = 0;
    boundNodes = [];
  }

  function bind(wheels: readonly AircraftWheelDebugSource[]): void {
    clear();
    for (const { node } of wheels) {
      boundNodes.push(node);
      if (!(node instanceof AbstractMesh) || node.isDisposed()) continue;
      const tire = measureTireOutline(node);
      if (!tire) continue;
      const { centerY, centerZ, radius } = tire;
      const lines: Vector3[][] = [];
      for (const x of [tire.minX, tire.maxX]) {
        const ring = Array.from({ length: 33 }, (_, index) => new Vector3(x,
          centerY + radius * Math.cos(index * Math.PI / 16),
          centerZ + radius * Math.sin(index * Math.PI / 16)));
        lines.push(ring, [new Vector3(x, centerY, centerZ), ring[0]]);
      }
      lines.push([new Vector3(tire.minX, centerY, centerZ), new Vector3(tire.maxX, centerY, centerZ)]);
      const mesh = createTireOutline(scene, node, node.name, lines);
      drawings.push({ wheel: node, mesh, material: mesh.material });
    }
  }

  function update(): void {
    if (!enabled || disposed) return;
    const wheels = getWheels();
    let changed = wheels.length !== boundNodes.length;
    for (let index = 0; !changed && index < wheels.length; index++) changed = wheels[index].node !== boundNodes[index];
    if (changed) bind(wheels);
    for (const { wheel, mesh } of drawings) {
      if (mesh.isDisposed()) continue;
      // Babylon inherits parent enablement, but not a mesh's isVisible flag.
      // Gear visibility uses that flag, so mirror it explicitly.
      mesh.setEnabled(!wheel.isDisposed() && wheel.isEnabled() && wheel.isVisible && wheel.visibility > 0);
    }
  }

  return {
    setEnabled(value: boolean): void {
      if (disposed || value === enabled) return;
      enabled = value;
      if (enabled) update();
      else for (const { mesh } of drawings) if (!mesh.isDisposed()) mesh.setEnabled(false);
    },
    update,
    dispose(): void {
      if (disposed) return;
      disposed = true;
      enabled = false;
      clear();
    },
  };
}
