import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import {
  Color3, Constants, Material, CreateLineSystem, CreateSphere, StandardMaterial, TransformNode, Vector3, VertexBuffer,
  type LinesMesh, type Mesh, type Scene,
} from "@babylonjs/core";
import { groundContactBodyPosition, type BodyCollisionProbe } from "./collisionGeometry";
import { groundContactUnits, type GroundContactUnit } from "./groundContactClearance";

type MarkerKind = "body" | "wheel" | "structure" | "cg";
const COLORS: Record<MarkerKind, string> = {
  body: "#39dfff", wheel: "#ffbb46", structure: "#cc82ff", cg: "#ffffff",
};

/** Shows the actual point contacts, not an invented solid collision hull:
 * the points the hull probes sweep, and the loaded model's JSBSim contacts.
 * Local coordinates are aircraft left/up/forward in metres, centred on its CG.
 * Marker sizes and spokes are visibility aids, not physical collision geometry.
 */
export function createCollisionDebugOverlay(scene: Scene, parent: TransformNode, sdk: JSBSimSdk,
  probes: readonly BodyCollisionProbe[]) {
  let enabled = false;
  let disposed = false;
  let root: TransformNode | null = null;
  let guides: LinesMesh | null = null;
  const materials: StandardMaterial[] = [];
  const bodyMarkers: Mesh[] = [];
  const groundMarkers: Mesh[] = [];
  // Each contact's x, y and z position properties, three per contact.
  let contactPaths: string[] = [];
  let guidePositions = new Float32Array(0);
  const lastCg = { xIn: NaN, yIn: NaN, zIn: NaN };
  const contact = { xIn: 0, yIn: 0, zIn: 0 };
  const offset = { left: 0, up: 0, forward: 0 };

  function createGeometry(): void {
    // A model that does not report its contacts completely shows its probes alone.
    let contacts: readonly GroundContactUnit[] = [];
    try { contacts = groundContactUnits(sdk); } catch { /* None to show. */ }
    contactPaths = contacts.flatMap(({ prefix }) => ["x", "y", "z"].map(axis => `${prefix}${axis}-position`));
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
      const mesh = CreateSphere(`collision-debug/${kind}/${name}`,
        { diameter: kind === "cg" ? 0.18 : 0.24, segments: 4 }, scene);
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

    marker("cg", "center-of-gravity");
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

  function update(): void {
    if (!enabled || disposed || !guides) return;
    const xIn = sdk.getPropertyValue("inertia/cg-x-in");
    const yIn = sdk.getPropertyValue("inertia/cg-y-in");
    const zIn = sdk.getPropertyValue("inertia/cg-z-in");
    const valid = Number.isFinite(xIn) && Number.isFinite(yIn) && Number.isFinite(zIn);
    if (valid && xIn === lastCg.xIn && yIn === lastCg.yIn && zIn === lastCg.zIn) return;
    lastCg.xIn = xIn; lastCg.yIn = yIn; lastCg.zIn = zIn;
    // Hide what the CG cannot place and collapse its guide, instead of
    // showing NaNs or a plausible-looking position based on a guessed CG.
    for (let index = 0; index < bodyMarkers.length; index++) {
      const probe = probes[index];
      groundContactBodyPosition(probe, lastCg, offset);
      // The point the probe sweeps sits its inset above the hull point.
      place(bodyMarkers[index], valid, offset.left, offset.up + probe.insetIn * 0.0254, offset.forward, index);
    }
    for (let index = 0; index < groundMarkers.length; index++) {
      contact.xIn = sdk.getPropertyValue(contactPaths[index * 3]);
      contact.yIn = sdk.getPropertyValue(contactPaths[index * 3 + 1]);
      contact.zIn = sdk.getPropertyValue(contactPaths[index * 3 + 2]);
      groundContactBodyPosition(contact, lastCg, offset);
      const known = valid && Number.isFinite(contact.xIn) && Number.isFinite(contact.yIn) && Number.isFinite(contact.zIn);
      place(groundMarkers[index], known, offset.left, offset.up, offset.forward, bodyMarkers.length + index);
    }
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
    dispose(): void {
      if (disposed) return;
      disposed = true;
      enabled = false;
      root?.dispose();
      for (const material of materials) material.dispose();
      root = null;
      guides = null;
    },
  };
}
