import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import {
  Color3, Constants, Material, Matrix, CreateLineSystem, CreateSphere, StandardMaterial, TransformNode, Vector3, VertexBuffer,
  type LinesMesh, type Mesh, type Node, type Scene,
} from "@babylonjs/core";
import { groundContactBodyPosition, type BodyCollisionProbe } from "./collisionGeometry";
import { groundContactUnits, type GroundContactUnit } from "./groundContactClearance";
import { createPropertyReadBatch } from "./propertyReadBatch";

type MarkerKind = "body" | "wheel" | "structure" | "cg";
const COLORS: Record<MarkerKind, string> = {
  body: "#39dfff", wheel: "#ffbb46", structure: "#cc82ff", cg: "#ffffff",
};

/** How many points of each kind the overlay shows. */
export interface CollisionDebugLegend { bodyProbes: number; wheels: number; structures: number }

/** How the aircraft model carries its retractable wheels, for their dots to ride along. */
export interface WheelCarriers {
  /**
   * The node that carries JSBSim gear unit `unit`'s wheel through retraction,
   * without the wheel's own spin, or null while none is loaded.
   */
  carrier(unit: number): Node | null | undefined;
  /**
   * Calls `read` with the gear posed down and locked and its struts unloaded,
   * then puts back the current pose. Without it, a dot rides only once its
   * gear has been seen down and locked.
   */
  withGearDown?(read: () => void): void;
}

// A JSBSim BOGEY prefix's unit index: `gear/unit/` is unit 0.
const GEAR_UNIT = /^gear\/unit(?:\[(\d+)\])?\/$/;

/**
 * Marker diameter in metres for an aircraft whose farthest shown point is
 * `reachM` from its CG: a dot beside a drone's foot, at most a quarter metre
 * on an airliner.
 */
export function collisionMarkerDiameter(reachM: number): number {
  return Math.min(0.24, Math.max(0.01, reachM * 0.08));
}

/** Shows the actual point contacts, not an invented solid collision hull:
 * the points the hull probes sweep, and the loaded model's JSBSim contacts.
 * Wheel dots follow the compression-adjusted tire contact patches. JSBSim
 * applies a retractable wheel only while its gear is down and locked; while the
 * gear moves, the dot rides with the tire `wheels` carries it on (or stays
 * where it was without one), and once the gear is stowed it goes. The
 * bundled aircraft use body-Z struts, so native compression moves a bogey upward in this
 * frame; oriented external struts need their compression axis exposed first.
 * Local coordinates are aircraft left/up/forward in metres, centred on its CG.
 * Marker sizes and spokes are visibility aids, not physical collision geometry.
 */
export function createCollisionDebugOverlay(scene: Scene, parent: TransformNode, sdk: JSBSimSdk,
  probes: readonly BodyCollisionProbe[], wheels?: WheelCarriers) {
  let enabled = false;
  let disposed = false;
  let root: TransformNode | null = null;
  let guides: LinesMesh | null = null;
  const materials: StandardMaterial[] = [];
  const bodyMarkers: Mesh[] = [];
  const groundMarkers: Mesh[] = [];
  let cgMarker: Mesh | null = null;
  let legend: CollisionDebugLegend | null = null;
  // Each contact's x, y and z position properties, three per contact.
  let contactPaths: string[] = [];
  let liveReader: ReturnType<typeof createPropertyReadBatch> | null = null;
  let compressionSlots = new Int32Array(0);
  // The common gear position's slot, -1 when no unit retracts.
  let gearSlot = -1;
  // A retractable wheel's JSBSim gear unit, -1 for every other contact.
  let gearUnits = new Int32Array(0);
  // How each marker was last shown: 0 hidden, 1 at its contact, 2 riding with its wheel.
  let lastMode = new Uint8Array(0);
  // A retractable wheel's dot in its carrier's frame, taken with the gear on
  // its down stop, and the carrier it was taken in.
  let carried = new Float64Array(0);
  let carriers: (Node | null)[] = [];
  // Each retractable wheel's carrier this frame.
  let current: (Node | null)[] = [];
  let lastCompression = new Float64Array(0);
  let contactBase = new Float64Array(0);
  let contactKnown = new Uint8Array(0);
  let guidePositions = new Float32Array(0);
  const lastCg = { xIn: NaN, yIn: NaN, zIn: NaN };
  const contact = { xIn: 0, yIn: 0, zIn: 0 };
  const offset = { left: 0, up: 0, forward: 0 };
  const chain: Node[] = [];
  const inverse = new Matrix();
  const transform = new Matrix();
  const point = new Vector3();
  const unloaded = new Vector3();

  function createGeometry(): void {
    // A model that does not report its contacts completely shows its probes alone.
    let contacts: readonly GroundContactUnit[] = [];
    try { contacts = groundContactUnits(sdk); } catch { /* None to show. */ }
    contactPaths = contacts.flatMap(({ prefix }) => ["x", "y", "z"].map(axis => `${prefix}${axis}-position`));
    const livePaths = ["inertia/cg-x-in", "inertia/cg-y-in", "inertia/cg-z-in"];
    compressionSlots = new Int32Array(contacts.length).fill(-1);
    lastCompression = new Float64Array(contacts.length).fill(Number.NaN);
    contactBase = new Float64Array(contacts.length * 3);
    contactKnown = new Uint8Array(contacts.length);
    gearUnits = Int32Array.from(contacts, ({ prefix, retractable }) => {
      const unit = retractable ? GEAR_UNIT.exec(prefix) : null;
      return unit ? Number(unit[1] ?? 0) : -1;
    });
    lastMode = new Uint8Array(contacts.length);
    carried = new Float64Array(contacts.length * 3);
    carriers = contacts.map(() => null);
    current = contacts.map(() => null);
    for (let index = 0; index < contacts.length; index++) {
      const { prefix } = contacts[index];
      if (!prefix.startsWith("gear/")) continue;
      compressionSlots[index] = livePaths.length;
      livePaths.push(`${prefix}compression-ft`, `${prefix}WOW`);
    }
    gearSlot = gearUnits.some(unit => unit >= 0) ? livePaths.push("gear/gear-pos-norm") - 1 : -1;
    // One native batch refreshes CG and wheel compression while this debug
    // view is visible. Static contact positions and body probes update only
    // when CG changes; no buffer uploads occur on an unchanged pose.
    liveReader = createPropertyReadBatch(sdk, livePaths);
    root = new TransformNode("collision-debug", scene);
    root.parent = parent;
    root.doNotSerialize = true;
    const palette = {} as Record<MarkerKind, StandardMaterial>;
    for (const kind of Object.keys(COLORS) as MarkerKind[]) {
      const material = new StandardMaterial(`collision-debug/${kind}-material`, scene);
      material.disableLighting = true;
      material.emissiveColor = Color3.FromHexString(COLORS[kind]);
      material.diffuseColor = material.emissiveColor;
      material.specularColor = Color3.Black();
      material.alpha = 0.95;
      material.transparencyMode = Material.MATERIAL_ALPHABLEND;
      material.depthFunction = Constants.ALWAYS;
      material.disableDepthWrite = true;
      material.doNotSerialize = true;
      materials.push(material);
      palette[kind] = material;
    }

    function marker(kind: MarkerKind, name: string): Mesh {
      // A unit sphere, sized to the aircraft once its points are placed.
      const mesh = CreateSphere(`collision-debug/${kind}/${name}`, { diameter: 1, segments: 4 }, scene);
      mesh.parent = root;
      mesh.material = palette[kind];
      mesh.isPickable = false;
      mesh.doNotSerialize = true;
      mesh.alwaysSelectAsActiveMesh = true;
      // Draw after the aircraft's transparent surfaces without changing the
      // scene's render groups or depth-buffer clearing policy.
      mesh.alphaIndex = 10001;
      mesh.metadata = { collisionDebug: { kind, name } };
      return mesh;
    }

    cgMarker = marker("cg", "center-of-gravity");
    // Both kinds follow the CG, so they stay hidden until update() reads one.
    for (const probe of probes) {
      const mesh = marker("body", probe.name);
      mesh.isVisible = false;
      bodyMarkers.push(mesh);
    }
    for (const { prefix } of contacts) {
      // BOGEY units are under gear/, STRUCTURE units under contact/.
      const mesh = marker(prefix.startsWith("gear/") ? "wheel" : "structure", prefix.slice(0, -1));
      mesh.isVisible = false;
      groundMarkers.push(mesh);
    }
    const markers = [...bodyMarkers, ...groundMarkers];
    legend = { bodyProbes: bodyMarkers.length, wheels: groundMarkers.filter(mesh => mesh.metadata.collisionDebug.kind === "wheel").length,
      structures: groundMarkers.filter(mesh => mesh.metadata.collisionDebug.kind === "structure").length };
    resize(0);
    guidePositions = new Float32Array(markers.length * 6);
    guides = CreateLineSystem("collision-debug/guides", {
      lines: markers.map(mesh => [Vector3.Zero(), mesh.position]), updatable: true,
    }, scene);
    guides.parent = root;
    guides.color = Color3.White();
    guides.alpha = 0.3;
    guides.alphaIndex = 10000;
    guides.isPickable = false;
    guides.alwaysSelectAsActiveMesh = true;
    guides.doNotSerialize = true;
    guides.metadata = { collisionDebug: { kind: "guides" } };
    // `CreateLineSystem` supplies a line material, but keep the nullable
    // public Babylon type honest rather than asserting it exists.
    const guideMaterial = guides.material;
    if (guideMaterial) {
      guideMaterial.depthFunction = Constants.ALWAYS;
      guideMaterial.disableDepthWrite = true;
    }
  }

  function place(mesh: Mesh, known: boolean, left: number, up: number, forward: number, line: number): void {
    mesh.isVisible = known;
    if (known) mesh.position.set(left, up, forward);
    else mesh.position.setAll(0);
    mesh.position.toArray(guidePositions, line * 6 + 3);
  }

  /**
   * `node`'s world matrix as the rig has posed it this frame. Babylon
   * refreshes world matrices while it renders, so between frames they hold
   * the last frame's pose; refresh the chain from the top down.
   */
  function worldMatrix(node: Node): Matrix {
    chain.length = 0;
    for (let at: Node | null = node; at; at = at.parent) chain.push(at);
    for (let index = chain.length - 1; index >= 0; index--) chain[index].computeWorldMatrix(true);
    return node.getWorldMatrix();
  }

  /** Keeps the dot at `index`, or this place for it, in `carrier`'s frame for it to ride with the tire. */
  function carry(index: number, carrier: Node | null, at: Vector3 = groundMarkers[index].position): void {
    carriers[index] = carrier;
    if (!carrier) return;
    worldMatrix(carrier).invertToRef(inverse);
    worldMatrix(root!).multiplyToRef(inverse, transform);
    Vector3.TransformCoordinatesToRef(at, transform, point);
    carried[index * 3] = point.x;
    carried[index * 3 + 1] = point.y;
    carried[index * 3 + 2] = point.z;
  }

  /**
   * Takes each retractable dot's place on its leg with the gear posed down,
   * so it rides from the first extension, not the first time the gear locks.
   */
  function carryDown(): void {
    if (!wheels?.withGearDown) return;
    let pending = false;
    for (let index = 0; index < current.length; index++) pending ||= current[index] !== null && current[index] !== carriers[index];
    if (!pending) return;
    wheels.withGearDown(() => {
      for (let index = 0; index < current.length; index++) {
        const carrier = current[index];
        if (!carrier || carrier === carriers[index]) continue;
        // The unloaded contact, as the posed struts are.
        unloaded.set(contactBase[index * 3], contactBase[index * 3 + 1], contactBase[index * 3 + 2]);
        carry(index, carrier, unloaded);
      }
    });
    // The rig has its pose back; this frame's render must not draw the other.
    for (const carrier of current) if (carrier) worldMatrix(carrier);
  }

  /** Places the dot at `index` where its carrier has taken the tire. */
  function ride(index: number, carrier: Node): void {
    worldMatrix(root!).invertToRef(inverse);
    worldMatrix(carrier).multiplyToRef(inverse, transform);
    point.set(carried[index * 3], carried[index * 3 + 1], carried[index * 3 + 2]);
    Vector3.TransformCoordinatesToRef(point, transform, point);
    place(groundMarkers[index], true, point.x, point.y, point.z, bodyMarkers.length + index);
  }

  /** Sizes every marker to an aircraft reaching `reachM` from its CG. */
  function resize(reachM: number): void {
    const diameter = collisionMarkerDiameter(reachM);
    cgMarker?.scaling.setAll(diameter * 0.75);
    for (const mesh of bodyMarkers) mesh.scaling.setAll(diameter);
    for (const mesh of groundMarkers) mesh.scaling.setAll(diameter);
  }

  function update(): void {
    if (!enabled || disposed || !guides) return;
    const live = liveReader!.read();
    const xIn = live[0], yIn = live[1], zIn = live[2];
    const valid = Number.isFinite(xIn) && Number.isFinite(yIn) && Number.isFinite(zIn);
    const cgChanged = !valid || xIn !== lastCg.xIn || yIn !== lastCg.yIn || zIn !== lastCg.zIn;
    let changed = cgChanged;
    if (cgChanged) {
      lastCg.xIn = xIn; lastCg.yIn = yIn; lastCg.zIn = zIn;
      // Hide what the CG cannot place and collapse its guide, instead of
      // showing NaNs or a plausible-looking position based on a guessed CG.
      for (let index = 0; index < bodyMarkers.length; index++) {
        const probe = probes[index];
        groundContactBodyPosition(probe, lastCg, offset);
        place(bodyMarkers[index], valid, offset.left, offset.up + probe.insetIn * 0.0254, offset.forward, index);
      }
      for (let index = 0; index < groundMarkers.length; index++) {
        contact.xIn = sdk.getPropertyValue(contactPaths[index * 3]);
        contact.yIn = sdk.getPropertyValue(contactPaths[index * 3 + 1]);
        contact.zIn = sdk.getPropertyValue(contactPaths[index * 3 + 2]);
        groundContactBodyPosition(contact, lastCg, offset);
        contactKnown[index] = valid && Number.isFinite(contact.xIn) && Number.isFinite(contact.yIn) && Number.isFinite(contact.zIn) ? 1 : 0;
        contactBase[index * 3] = offset.left;
        contactBase[index * 3 + 1] = offset.up;
        contactBase[index * 3 + 2] = offset.forward;
      }
    }
    const gear = gearSlot < 0 || !Number.isFinite(live[gearSlot]) ? 1 : live[gearSlot];
    for (let index = 0; index < groundMarkers.length; index++) {
      const carrier = gearUnits[index] >= 0 && contactKnown[index] !== 0 ? wheels?.carrier(gearUnits[index]) ?? null : null;
      current[index] = carrier?.isDisposed() ? null : carrier;
    }
    if (gear < 1) carryDown();
    for (let index = 0; index < groundMarkers.length; index++) {
      const unit = gearUnits[index];
      const mode = contactKnown[index] === 0 || (unit >= 0 && gear <= 0) ? 0 : 1;
      const carrier = mode === 0 ? null : current[index];
      if (carrier) {
        if (gear < 1 && carrier === carriers[index]) {
          // Off its down stop the dot goes where the tire goes. JSBSim stops
          // loading the wheel 0.01 of travel later, until it locks down again.
          ride(index, carrier);
          lastMode[index] = 2;
          changed = true;
          continue;
        }
        // On the stop, a wheel loaded since the dot was placed takes it from there.
        if (gear >= 1 && carrier !== carriers[index] && lastMode[index] === 1) carry(index, carrier);
      }
      const slot = compressionSlots[index];
      // The native strut displacement is mTGear * (0, 0, -compression-ft)
      // in body axes. Bundled bogeys have the identity mTGear. WOW gates the
      // value because retracting a unit can leave its last compression stored.
      const feet = slot >= 0 && live[slot + 1] > 0.5 ? live[slot] : 0;
      const compression = Number.isFinite(feet) ? Math.max(0, feet) * 0.3048 : 0;
      if (!cgChanged && compression === lastCompression[index] && mode === lastMode[index]) continue;
      lastCompression[index] = compression;
      lastMode[index] = mode;
      changed = true;
      place(groundMarkers[index], mode !== 0, contactBase[index * 3],
        contactBase[index * 3 + 1] + compression, contactBase[index * 3 + 2], bodyMarkers.length + index);
      if (unit >= 0 && mode !== 0 && gear >= 1) carry(index, carrier);
    }
    if (!changed) return;
    let reachM = 0;
    for (const mesh of bodyMarkers) if (mesh.isVisible) reachM = Math.max(reachM, mesh.position.length());
    for (const mesh of groundMarkers) if (mesh.isVisible) reachM = Math.max(reachM, mesh.position.length());
    resize(reachM);
    guides.updateVerticesData(VertexBuffer.PositionKind, guidePositions);
  }

  return {
    setEnabled(value: boolean): void {
      if (disposed || value === enabled) return;
      enabled = value;
      if (enabled && !root) createGeometry();
      root?.setEnabled(enabled);
      if (enabled) update();
    },
    update,
    /** What the overlay shows, once it has been enabled. */
    legend(): CollisionDebugLegend | null {
      return legend;
    },
    dispose(): void {
      if (disposed) return;
      disposed = true;
      enabled = false;
      root?.dispose();
      liveReader?.dispose();
      liveReader = null;
      for (const material of materials) material.dispose();
      root = null;
      guides = null;
      cgMarker = null;
      legend = null;
    },
  };
}
