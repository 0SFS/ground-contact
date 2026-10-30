# Inside vs. Outside, Bridges, and Automatic Float-Out over Streamed Photogrammetry

**Proposal:** underground and bridges, sampling below the aircraft. Responds to [the brief](brief.md); see the [README](README.md) for its status and review.  
**Date:** 2026-10-30  
**Scope:** `@0sfs/ground-contact` (`terrainContact`, `visibleMeshCollision`, `groundContactCore`), `@foss-earth/engine` (`SurfaceQuery`, `groundAcceleration`), and `0sfs/fly` (`createFlightSimApp`, `droneController`, `fixedStepLoop`)

---

## 0. Executive Summary & Verified Codebase Findings

Today's ground-contact system reduces the 3D world at `(lat, lon)` to a single number: the height returned by `SurfaceQuery.sample(lat, lon)`, which casts a ray straight down from $20\text{ km}$ above the WGS84 ellipsoid (`foss-earth/engine/src/babylon/surfaceQuery.ts:132`). That single architectural choice—defining "the ground" as the **first surface seen from the sky** rather than the **supporting surface below the aircraft**—causes two systemic failures:

1. **The Bridge Catastrophe:** When an aircraft flies at $3\text{ m}$ AGL under a $9\text{ m}$ bridge deck, `sample()` returns $9\text{ m}$. `terrainContact` feeds $9\text{ m}$ (or a support plane anchored by it) to JSBSim's `position/terrain-elevation-asl-ft` (`ground-contact/src/terrainContact.ts:431, 1085`), or `fixedStepLoop.ts:139` catches the resulting $6\text{ m}$ penetration (`> MAX_GROUND_PENETRATION_METERS = 0.5`) and faults with `"Hard ground impact"`. Simultaneously, `createFlightSimApp.ts:4162` feeds that same sky-down sample to the drone controller and HUD.
2. **The `"moving you 0.0 m (surface refinement)"` Card:** In `terrainContact.ts:1030–1068`, any map revision that changes the support plane by `Math.abs(refinement) > 0.00001` ($10\text{ }\mu\text{m}$!) under a standing aircraft (`followGround`) creates a `TerrainRelocationProposal`. Because photogrammetry tiles refine by $1\text{–}4\text{ cm}$ as the camera settles, and `createFlightSimApp.ts:3023` formats `proposal.deltaMeters.toFixed(1)`, a $2\text{ cm}$ refinement produces a modal card asking permission to move `"0.0 m"`. Worse, even a *downward* refinement ($\Delta h < 0$, which leaves the aircraft in free air *above* the new surface) triggers the `"Terrain overlaps your aircraft"` card (`terrainRelocation.test.ts:145–158`).

### Key Findings from Inspecting the Current Codebase

Before stating assumptions, four implementation facts verified directly in the repository shape this design:

1. **`SurfaceHit.normal` is currently ray-flipped, not raw vertex winding:**  
   Although [the brief](brief.md) notes that `normal` comes from vertex winding, inspecting `foss-earth/engine/src/babylon/surfaceQuery.ts:118`, `groundAcceleration.ts:255`, and `foss-earth/engine/ground-WASM/crates/earth-ground/src/lib.rs:843–847` reveals that both Babylon's `PickingInfo.getNormal(true, false)` (when `pick.ray` is attached) and the Rust WASM picker explicitly execute:
   ```rust
   if normal.dot(world_direction) > 0.0 {
       normal = normal.scale(-1.0);
   }
   ```
   Consequently, `SurfaceHit.normal` today **always faces against the ray** ($\hat{n} \cdot \hat{d}_{\text{ray}} \le 0$). Our design therefore does **not** depend on raw vertex winding in Phase 1, and specifies the exact 3-line change in `surfaceQuery.ts` / `earth-ground` if raw winding sign (`windingSign: -1 | 1`) is added as an optional diagnostic signal.
2. **`fixedStepLoop.ts` hard-faults at `0.5 m` of floor penetration:**  
   `fly/src/flight/physics/fixedStepLoop.ts:15, 137–148` aborts integration before `sdk.run()` whenever `terrainMeters + groundContactClearanceMeters(...) - before.altMeters > 0.5` (`MAX_GROUND_PENETRATION_METERS`). Therefore, floating an aircraft out from $d \in (0.5\text{ m}, 1.0\text{ m}]$ **cannot** simply set `position/terrain-elevation-asl-ft` to the true surface height—doing so would immediately trip the $0.5\text{ m}$ hard-impact fault and fire massive oleo spring forces. Instead, we introduce an **Escort Floor** that rises with the aircraft while maintaining normal static contact compression.
3. **CPU BVH raycasts cost $\sim 0.17\text{ }\mu\text{s}$ per ray once indexed:**  
   Benchmarks in `ground-contact/docs/collision-compute-comparison.md:46–53` show that on a $32,768$-triangle tile, unaccelerated linear triangle scanning takes $0.44\text{ ms}$ per ray, whereas the resident CPU/WASM BVH (`earth-ground`) takes **$0.168\text{ }\mu\text{s}$ per ray** ($0.81\text{ }\mu\text{s}$ for 5 rays). Our real-time design keeps the per-step ray count identical to today when `groundAcceleration` is still building, and exploits short finite-segment bounds pruning so local downward rays are even cheaper than $40\text{ km}$ sky-down rays.
4. **Drones currently have zero `bodyProbes`:**  
   `visibleMeshCollision.ts` sweeps 5–6 hull probes for the C172, SF-50, and F-35B, but drones (`dji-mini-2`, `dji-air-2s`) have none and their CG sits only $2.3\text{–}4.0\text{ cm}$ above their leg contacts (`terrainRelocation.test.ts:161`). Our floor-origin formula and depth solver explicitly account for this $2.3\text{ cm}$ CG-to-contact offset.

### Explicit Assumptions

1. **No semantic tags on meshes:** Google Photorealistic 3D Tiles provide unlabelled triangle soups with `googleGeometricErrorMeters` metadata per mesh. Bridges, roads, trees, water, and buildings are indistinguishable except by their 3D geometry and `geometricErrorMeters`.
2. **Open-bottomed 2.5D-plus-overhangs topology:** Photogrammetry meshes represent only the visible outer skin seen from aerial cameras. Terrain tiles and buildings have **no bottom cap** (below the road or inside a building is empty void down to the Earth's center, except where a coarse parent LOD tile temporarily overlaps).
3. **Unverified triangle winding:** Even if the ray-facing flip in `PickingInfo.getNormal` is bypassed, photogrammetry meshes can contain inconsistent winding on thin trusses, vegetation, or double-sided curtains. All primary inside/outside decisions must be **winding-agnostic**, using ray-interval topology, temporal coherence, and sky-reachability.
4. **Ephemeral derived state only:** No Google Tile geometry or derived spatial structure is persisted to disk/IndexedDB; all spatial indices (`earth-ground` BVHs) and contact state live strictly in volatile memory for currently resident meshes.
5. **Consent boundary:** Automatic velocity-limited floating up to threshold $T$ (default $T = 1.0\text{ m}$ for airplanes, tunable or scaled for drones) is classified as continuous contact depenetration (exempt from modal confirmation). Any nonphysical relocation exceeding $T$ requires the existing overlap card and explicit `FlightPlacementCoordinator` Yes/No confirmation.

---

## 1. Principled Definition of "Inside" vs. "Outside"

### 1.1 Critique of Standard Candidate Definitions on Streamed Photogrammetry

Why do textbook inside/outside definitions fail when applied naively to Google Photorealistic 3D Tiles?

1. **3D Generalized Winding Number (GWN, Jacobson et al. 2013; Barill et al. 2018):**
   $$w(\mathbf{x}) = \frac{1}{4\pi} \sum_{f \in \mathcal{M}} \Omega_f(\mathbf{x})$$
   - *Fatal flaw on open terrain sheets:* A terrain tile is an open topological disk, not a closed 2-manifold bounding a volume. For a point $\mathbf{x}$ located $0.5\text{ m}$ below a flat road of tile radius $R \gg 0.5\text{ m}$, the road subtends a solid angle $\Omega(\mathbf{x}) \approx -2\pi$, giving $|w(\mathbf{x})| \approx 0.5$—the exact same magnitude as a point $0.5\text{ m}$ *above* the road ($+0.5$), and the same magnitude as a point halfway inside an open doorway! Worse,Inside a building whose floor is missing (because photogrammetry only captures the roof and walls), the missing floor subtracts $2\pi$ of solid angle. Finally, overlapping coarse and fine LOD tiles double-count $\Omega_f(\mathbf{x})$, yielding $w(\mathbf{x}) \approx 1.0$ in free air between two overlapping LOD sheets.
2. **3D Ray-Parity Voting (Nooruddin & Turk 2003):**
   - *Fatal flaw:* If $\mathbf{x}$ is $0.5\text{ m}$ below a flat road on an open terrain sheet, any ray cast horizontally or downward from $\mathbf{x}$ travels through the empty void beneath the terrain skin and exits the tile boundary with **zero intersections** (even parity = "outside"). Over $50\%$ of the sphere of directions from a point underground never intersects the terrain mesh at all.
3. **Front/Back-Face of the First Hit (or Angle-Weighted Pseudo-Normals, Bærentzen & Aanæs 2005):**
   - *Fatal flaw:* First, as noted in Section 0, `SurfaceQuery` currently flips normals toward the ray. Second, even with unflipped normals, a bridge deck modeled as a **single-layer ribbon of triangles** has its back-face pointing downward toward the road underneath. A point $3\text{ m}$ above the road and $6\text{ m}$ under a single-layer bridge deck sees the *back-face* of the bridge when looking up—indistinguishable by upward-normal sign alone from a point $6\text{ m}$ underground looking up at the back-face of a road!

### 1.2 The Principled Definition: LOD-Filtered Plumb-Line Topology + Clearance Sky-Reachability

To match human physical intuition across every case, we must recognize that a planetary photogrammetry scene is **anisotropic (gravity-aligned)** and **multi-resolution**. It consists of:
- A continuous planetary base sheet (the **Earth Base**), and
- Bounded 3D structures (bridges, buildings, trees, overhangs) protruding above that base sheet.

We define "inside" vs. "outside" through three exact mathematical layers:

#### Step 1: LOD Dominance Filtering ($\mathcal{M}^*$)
Let $\mathcal{M} = \bigcup_i M_i$ be the set of currently enabled, visible surface meshes, where each mesh $M_i$ has an advertised geometric error $\epsilon_i = \text{geometricErrorMeters}(M_i) \ge 0$ (with $\epsilon_i = 0$ for raster heightfields).
A point $\mathbf{p} \in M_i$ is **LOD-shadowed** if there exists a finer mesh $M_j$ with $\epsilon_j < \epsilon_i$ and a point $\mathbf{q} \in M_j$ along the same local vertical plumb line $\ell(\phi, \lambda)$ such that:
$$|h(\mathbf{p}) - h(\mathbf{q})| \le \epsilon_i$$
Removing all LOD-shadowed points yields the **effective surface $\mathcal{M}^*$**. (This generalizes the existing downward rule in `surfaceQuery.ts:140–144` to any vertical query: a coarse tile within its geometric error of a finer tile is a low-resolution proxy of the *same* surface, whereas a surface separated by more than $\epsilon_i$ is a distinct physical layer such as a bridge over a road.)

#### Step 2: Crack-Bridged Free Space and Sky-Reachability ($\Omega_{\text{free}}$)
Let $r_{\text{crack}} = 0.15\text{ m}$ be the maximum photogrammetry tile-seam half-width, and define the crack-closed surface via morphological dilation $\widetilde{\mathcal{M}} = \mathcal{M}^* \oplus B(\mathbf{0}, r_{\text{crack}})$.
Let $r_{\text{nav}} = 0.50\text{ m}$ be the minimum navigable clearance radius (with minimum vertical headroom $H_{\text{nav}} = \max(2.0\text{ m},\; H_{\text{aircraft}} + 0.5\text{ m})$).

> **Definition 1 (Outside / Free Air $\Omega_{\text{free}}$):**  
> A point $\mathbf{x} = (\phi, \lambda, z)$ is **Outside ($\mathbf{x} \in \Omega_{\text{free}}$)** if and only if:
> 1. **Supported from Below (Not Subterranean):** Along the vertical plumb line $\ell(\phi, \lambda)$ through $\mathbf{x}$ (bridged across seams of width $\le 2 r_{\text{crack}}$), there exists at least one intersection with $\mathcal{M}^*$ at height $h_{\text{below}}(\mathbf{x}) \le z$ (the **supporting floor** below $\mathbf{x}$), AND
> 2. **Clear Headroom:** Either there is no intersection with $\mathcal{M}^*$ above $\mathbf{x}$ ($h_{\text{above}}(\mathbf{x}) = +\infty$), OR the vertical clearance between the immediate ceiling $h_{\text{above}}(\mathbf{x}) > z$ and the floor $h_{\text{below}}(\mathbf{x}) \le z$ satisfies $h_{\text{above}}(\mathbf{x}) - h_{\text{below}}(\mathbf{x}) \ge H_{\text{nav}}$, AND
> 3. **Sky-Reachability (Not Enclosed in a Hollow Blob):** If $h_{\text{above}}(\mathbf{x}) < +\infty$, there exists a continuous path $\gamma: [0, 1] \to \mathbb{R}^3$ from $\mathbf{x}$ to the open sky $\{h \ge h_{\text{sky}}(\phi', \lambda')\}$ along which a clearance cylinder of radius $r_{\text{nav}}$ and height $H_{\text{nav}}$ never intersects $\widetilde{\mathcal{M}}$.
>
> **Definition 2 (Inside Solid Matter $\Omega_{\text{solid}}$):**  
> A point $\mathbf{x}$ is **Inside ($\mathbf{x} \in \Omega_{\text{solid}}$)** if and only if $\mathbf{x} \notin \Omega_{\text{free}}$.

#### Step 3: Penetration Depth $d(\mathbf{x})$ and Exit Vector $\mathbf{d}_{\text{exit}}(\mathbf{x})$
For any point $\mathbf{x} \in \Omega_{\text{solid}}$, the **minimum exit vector** is the shortest displacement (weighted by a gentle horizontal-vs-vertical preference factor $\alpha_{\text{lat}} = 1.2$ so vertical float-up wins ties on slopes) that places the aircraft's contact envelope in $\Omega_{\text{free}}$:
$$\mathbf{d}_{\text{exit}}(\mathbf{x}) = \arg\min_{\mathbf{d} \in \mathbb{R}^3:\; \mathbf{x} + \mathbf{d} \in \Omega_{\text{free}}} \sqrt{d_{\text{up}}^2 + \alpha_{\text{lat}}^2 (d_{\text{north}}^2 + d_{\text{east}}^2)}$$
and the **penetration depth** is $d(\mathbf{x}) = \|\mathbf{d}_{\text{exit}}(\mathbf{x})\|_2$.
- If $0 < d(\mathbf{x}) \le T$: $\mathbf{x}$ is **Shallow Inside** (automatic velocity-limited float-out).
- If $d(\mathbf{x}) > T$: $\mathbf{x}$ is **Deep Inside** (triggers underground relocation proposal or hard-impact fault depending on entry cause).

### 1.3 Evaluation on the Six Required Scenarios & Ambiguous Cases

| Scenario | Plumb-Line & Reachability Geometry | Verdict & Floor | Justification & Ambiguity Resolution |
| :--- | :--- | :--- | :--- |
| **1. Under a bridge** | $h_{\text{above}} = 9\text{ m}$ (deck), $h_{\text{below}} = 0\text{ m}$ (road/water). Headroom $9 - 0 = 9\text{ m} \ge H_{\text{nav}}$. Lateral path exits span to open sky. | **Outside** ($\Omega_{\text{free}}$). Floor = $0\text{ m}$ (road). | Has both a valid floor below and navigable headroom + lateral sky access. Works whether the bridge deck is a 2-sided slab or a 1-sided sheet. |
| **2. Sunk into a road** | $h_{\text{above}} = 0\text{ m}$ (road), $h_{\text{below}} = \text{none}$ (`null`). Violates Condition 1 (no floor below). | **Inside** ($\Omega_{\text{solid}}$). Depth $d = h_{\text{road}} - z_{\text{contact}}$. | Because terrain sheets have no bottom cap, a point below the lowest LOD-filtered surface has $h_{\text{below}} = \text{null}$ and is unambiguously subterranean. |
| **3. Inside a building's photogrammetry blob** | $h_{\text{above}} = 25\text{ m}$ (roof). Either $h_{\text{below}} = \text{none}$ (photogrammetry skins have no interior ground floor), OR if a stray/coarse floor exists, lateral escape paths are blocked in all directions by the building walls (violates Condition 3). | **Inside** ($\Omega_{\text{solid}}$). Depth $d = \min(h_{\text{roof}} - z_{\text{contact}}, d_{\text{wall}})$. | If near the wall ($d_{\text{wall}} \le T$), exits laterally through the wall; if near the roof ($d_{\text{up}} \le T$), floats up to the roof; if deep in the core ($d > T$), shows the underground card. |
| **4. In a tree canopy** | **Case 4a (Under overhanging boughs above a street):** $h_{\text{above}} = 10\text{ m}$ (canopy), $h_{\text{below}} = 0\text{ m}$ (street), $H = 10\text{ m} \ge H_{\text{nav}}$, laterally open $\to$ **Outside**, floor = street.<br>**Case 4b (Inside a solid un-hollowed "broccoli" tree mound):** $h_{\text{above}} = 10\text{ m}$, $h_{\text{below}} = \text{none}$ $\to$ **Inside**. | **4a: Outside** (street floor).<br>**4b: Inside** (mound top/side). | **Justification:** Google tiles lack semantic labels. Treating overhanging canopy with ground underneath as an overhang lets drones fly down tree-lined streets; treating solid un-hollowed tree blobs as solid prevents aircraft landing on dense forest photogrammetry from falling into an unmeshed void. Planes hitting canopies in flight bounce via `visibleMeshCollision`. |
| **5. In a tunnel mouth or underpass** | **Open portal / underpass:** $h_{\text{above}} = \text{arch}$, $h_{\text{below}} = \text{road}$, $H \ge H_{\text{nav}}$, open portal exit to sky $\to$ **Outside**, floor = road.<br>**Sealed tunnel curtain:** Behind the photogrammetry curtain sealing a deep tunnel, $h_{\text{below}} = \text{none}$, $h_{\text{above}} = \text{mountain}$. | **Portal: Outside**.<br>**Behind curtain: Inside**. | Photogrammetry meshes the open mouth of an underpass (both ceiling and road exist) up to where camera visibility ends. Crossing a vertical curtain into the unmeshed mountain interior transitions to Inside. |
| **6. In a crack between tiles** | A single point ray at $(\phi, \lambda)$ misses due to a $5\text{ cm}$ seam, but the morphological dilation $\widetilde{\mathcal{M}}$ of radius $r_{\text{crack}} = 0.15\text{ m}$ bridges the seam. | **Same state as adjacent surface** (floor held across crack). | Cracks narrower than $2 r_{\text{crack}} = 30\text{ cm}$ are never navigable passages; bridging them prevents 1-frame floor dropouts and false subterranean reclassification. |

#### Inherently Ambiguous Cases & Deterministic Behavior
1. **Closed Courtyard / Stadium vs. Hollow Building With a Stray Floor Triangle:**  
   If a coarse floor triangle happens to remain inside a building, how big must an opening to the sky be before an enclosed space counts as a "courtyard" (Outside) rather than a "building interior" (Inside)?  
   *Rule:* In the conical sky-escape test (Section 2 & 3), if at least **2 of 8 azimuthal rays** (separated by $\ge 90^\circ$) or an upward $45^\circ$ cone of radius $L_{\text{escape}} = 40\text{ m}$ reaches unobstructed sky ($h_{\text{sky}} \le z_{\text{ray}}$), it is an open courtyard/canyon/underpass (**Outside**); otherwise it is an enclosed blob (**Inside**).
2. **Bridge Deck Thickness vs. Low Crawlspace ($H < H_{\text{nav}}$):**  
   A double-sided bridge deck ($h_{\text{top}} = 9.0\text{ m}$, $h_{\text{soffit}} = 8.0\text{ m}$) or a low culvert ($H = 1.0\text{ m} < H_{\text{nav}} = 2.0\text{ m}$) has both an upper and lower surface, but the vertical gap is too thin to fly through.  
   *Rule:* Any slab with $h_{\text{above}} - h_{\text{below}} < H_{\text{nav}}$ is treated as **solid interior matter** ($\Omega_{\text{solid}}$), so an aircraft sinking into the top of a $1\text{ m}$-thick bridge deck is classified as **Inside** the deck and floats up to $h_{\text{top}} = 9.0\text{ m}$.

---

## 2. The Ideal Algorithm (Ignoring Compute Cost)

Given unlimited compute per physics step, the exact state of an aircraft with centre of gravity $\mathbf{p}_{\text{CG}} = (\phi_0, \lambda_0, z_{\text{CG}})$, contact points $\{\mathbf{c}_i\}_{i=1}^m$, and hull probes $\{\mathbf{b}_j\}_{j=1}^p$ against resident meshes $\mathcal{M}$ is computed as follows:

1. **Full Vertical Column Extraction (`columnHits`):**  
   For the query coordinate $(\phi, \lambda)$, intersect the vertical line $\ell(\phi, \lambda)$ from $h_{\max} = +20\text{ km}$ to $h_{\min} = -1\text{ km}$ with **all** triangles of all enabled visible meshes $\mathcal{M}$, returning the sorted list of hits $H = [(h_1, \epsilon_1, \mathbf{n}_1), \dots, (h_k, \epsilon_k, \mathbf{n}_k)]$ with $h_1 \ge h_2 \ge \dots \ge h_k$.
2. **LOD Dominance Pruning:**  
   Remove any hit $a \in H$ if there exists $b \in H$ with $\epsilon_b < \epsilon_a$ and $|h_a - h_b| \le \epsilon_a$. Merge hits within $\delta_{\text{weld}} = 0.05\text{ m}$ of each other. Denote the surviving heights $S = [s_1 > s_2 > \dots > s_r]$.
3. ** Crack-Bridged Column Disk Sampling:**  
   Repeat Steps 1–2 on a disk of radius $r_{\text{crack}} = 0.15\text{ m}$ around $(\phi, \lambda)$ (e.g., center + 4 cardinal offsets at $0.15\text{ m}$) and take the morphological closing of the height intervals so tile seams of width $\le 30\text{ cm}$ vanish.
4. **3D Free-Space Flood Fill (Configuration-Space Sky-Reachability):**  
   Voxelize a local domain $[\phi_0 \pm 100\text{ m}, \lambda_0 \pm 100\text{ m}, z_{\min} \dots z_{\max}]$ at resolution $\Delta x = 0.10\text{ m}$. Mark all voxels within distance $r_{\text{crack}}$ of $\mathcal{M}^*$ as occupied. Erode free space by the aircraft's minimum clearance cylinder $(r_{\text{nav}}, H_{\text{nav}})$, then flood-fill reachability from the upper boundary $z = z_{\max}$ (the sky).
   - Every voxel reached by the flood fill belongs to $\Omega_{\text{free}}$ (**Outside**).
   - Every unreached voxel belongs to $\Omega_{\text{solid}}$ (**Inside**).
5. **Signed Distance & Exit Vector via Euclidean Distance Transform (EDT):**  
   For $\mathbf{x} \in \Omega_{\text{solid}}$, compute the exact 3D Euclidean Distance Transform to $\Omega_{\text{free}}$:
   $$\mathbf{d}_{\text{exit}}(\mathbf{x}) = \mathbf{y}^*(\mathbf{x}) - \mathbf{x}, \quad \mathbf{y}^*(\mathbf{x}) = \arg\min_{\mathbf{y} \in \partial\Omega_{\text{free}}} \|\mathbf{y} - \mathbf{x}\|_{\alpha}$$
   and depth $d(\mathbf{x}) = \|\mathbf{d}_{\text{exit}}(\mathbf{x})\|_2$.
6. **Floor Selection:**  
   At each contact $\mathbf{c}_i$, let $\mathbf{c}_i^* = \mathbf{c}_i + \mathbf{d}_{\text{exit}}(\mathbf{c}_i) \in \Omega_{\text{free}}$ be its projected free-space point, and set its floor $g_i$ to the highest surface $s \in S(\phi_i, \lambda_i)$ satisfying $s \le z(\mathbf{c}_i^*) + \varepsilon_{\text{stick}}$. Combine $\{g_i\}$ into JSBSim's single horizontal support plane via the existing max-support rule.

---

## 3. The Real-Time Algorithm (120 Hz Budget on Fanless Laptop & Phone)

We now compress the Ideal Algorithm into a real-time algorithm that runs in **1–3 rays per step** in steady flight (identical to today's budget!) by exploiting **temporal coherence** and **plumb-line interval queries**.

### 3.1 Minimal Extension to `SurfaceQuery`

Today, `SurfaceQuery.sample(lat, lon)` in `foss-earth/engine/src/babylon/surfaceQuery.ts:126–146` casts from $20,000\text{ m}$ down to $-20,000\text{ m}$ with the `finerThanMeters` LOD filter. We add one optional method to `SurfaceQuery` (with a zero-cost fallback to `raycast` if a mock does not implement it):

```ts
export interface SurfaceQuery {
  raycast(origin: EcefCoord, direction: EcefCoord, lengthMeters: number): SurfaceHit | null;
  sample(latDeg: number, lonDeg: number): SurfaceHit | null;
  /**
   * Like sample(latDeg, lonDeg), including the Google LOD geometric-error
   * dominance filter, but starts at `maxAltitudeMeters` above the ellipsoid
   * instead of 20 km. In raster mode, returns the heightfield sample if its
   * height is <= maxAltitudeMeters, else null.
   */
  sampleBelow?(latDeg: number, lonDeg: number, maxAltitudeMeters: number): SurfaceHit | null;
  revision?(): number;
  beginFrame?(): void;
  endFrame?(): void;
}
```

In `foss-earth/engine/src/babylon/surfaceQuery.ts`, `sampleBelow` is **12 lines of code** sharing `cast()`:
```ts
function sampleBelow(latDeg: number, lonDeg: number, maxAltitudeMeters: number): SurfaceHit | null {
  if (!Number.isFinite(latDeg) || !Number.isFinite(lonDeg) || !Number.isFinite(maxAltitudeMeters) || Math.abs(latDeg) > 90) return null;
  const overridden = sampleOverride?.(latDeg, lonDeg);
  if (overridden !== undefined) {
    return overridden && overridden.heightMeters <= maxAltitudeMeters ? overridden : null;
  }
  const lat = latDeg * DEG_TO_RAD, lon = lonDeg * DEG_TO_RAD;
  const length = Math.max(10, maxAltitudeMeters + 1000); // Down to -1000 m ASL
  const origin = geodeticToEcef(lat, lon, maxAltitudeMeters);
  const down = { x: -Math.cos(lat) * Math.cos(lon), y: -Math.cos(lat) * Math.sin(lon), z: -Math.sin(lat) };
  let hit = cast(origin, down, length);
  for (let error = hit?.geometricErrorMeters; hit && error !== undefined; error = hit.geometricErrorMeters) {
    const finer = cast(origin, down, length, error);
    if (!finer || hit.heightMeters - finer.heightMeters > error) break;
    hit = finer;
  }
  return hit;
}
```
*Why `sampleBelow` is even faster than `sample`:* When the aircraft is at $h = 100\text{ m}$, `sampleBelow` casts a $1.1\text{ km}$ segment instead of a $40\text{ km}$ segment, so `intersectsSegmentBounds` (`surfaceQuery.ts:31`) immediately prunes overhead bridge meshes and high bounding boxes without touching their triangles!

### 3.2 Temporal Coherence Invariant & What Runs When

An aircraft in flight obeys a strict topological invariant:
> **Coherence Invariant:** If the aircraft is **Outside** ($\Omega_{\text{free}}$) at step $t - 1$ above an established floor $h_{\text{floor}, t-1}$, it can only become **Inside** ($\Omega_{\text{solid}}$) at step $t$ in two ways:
> 1. **Self-Motion Crossing:** Its trajectory segment $[\mathbf{p}_{t-1}, \mathbf{p}_t]$ crosses a surface (either sinking below its supporting floor $h_{\text{floor}, t}$ or sweeping through a wall/roof/hill), OR
> 2. **Map Mutation:** The map revision changes (`revision !== prevRevision`) or regains coverage after a dropout, moving a surface across the aircraft's position.

This splits the work cleanly into two paths:

| Path | Trigger Condition | Rays Cast | Purpose |
| :--- | :--- | :--- | :--- |
| **A. Every-Step Hot Path** | Every 120 Hz step | **1 ray** (`sampleBelow` at CG) + **0–2 rays** (`sampleBelow` at overdue contacts when `< 10 m` AGL) | Tracks the supporting floor below the aircraft and its contacts; advances any active shallow float (`SHALLOW_FLOAT`); ignores overhead bridges completely. |
| **B. Reclassification Path** | Only on:<br>1. `placement` (first step)<br>2. `revisionChanged` where local floor jumped $> 0.05\text{ m}$ or disappeared<br>3. `sampleBelow` returns `null` near ground<br>4. Regain after `blindSteps >= 60` | **1 sky ray** (`sample`) + **1 upward ray** (`raycast` $+\hat{u}$) + (only if $d_{\text{up}} > T$) **up to 4–8 short horizontal rays** | Determines whether the aircraft is Outside (under a bridge/overhang), Shallow Inside ($d \le T$), Deep Inside ($d > T$), or inside a Provisional Coarse tile (`PROVISIONAL_COARSE`), and computes the 3D exit vector $\mathbf{d}_{\text{exit}}$. |

### 3.3 Computing Penetration Depth $d$ and 3D Exit Direction $\mathbf{d}_{\text{exit}}$

When the aircraft is classified as **Inside** ($\Omega_{\text{solid}}$), we need the scalar depth $d$ (for the threshold rule $d \le T$ vs. $d > T$) and the exit direction:

1. **Vertical Floor Penetration ($d_{\text{up}}$):**  
   Let $z_{\text{contact}} = z_{\text{CG}} - c(\phi, \theta)$ be the lowest active contact height, and let $h_{\text{surf}}$ be the enclosing surface above $z_{\text{contact}}$ (specifically, the immediate upward surface $h_{\text{above}}$ if the space above $h_{\text{above}}$ is clear by $\ge H_{\text{nav}}$, otherwise $h_{\text{sky}}$). Then:
   $$d_{\text{up}} = h_{\text{surf}} - z_{\text{contact}}$$
   - **Fast acceptance:** If $0 < d_{\text{up}} \le T$, the exit vector is purely vertical:
     $$\mathbf{d}_{\text{exit}} = +d_{\text{up}} \, \hat{\mathbf{u}}, \qquad d = d_{\text{up}} \le T$$
     This covers $99\%$ of real-world shallow overlaps (sinking slightly into a road, runway, or bridge deck, or a centimetre/decimetre tile refinement under a parked or taxiing aircraft).

2. **Wall / Cliff Lateral Penetration ($d_{\text{lat}}$ when $d_{\text{up}} > T$):**  
   Suppose a drone hovers next to a $30\text{ m}$ cliff or building wall, and a map refinement shifts the wall $0.4\text{ m}$ outward so the drone is now $0.4\text{ m}$ inside the wall, while the roof/cliff-top is $25\text{ m}$ above it ($d_{\text{up}} = 25\text{ m} > T$). Measuring only $d_{\text{up}}$ would falsely report a $25\text{ m}$ burial!  
   Therefore, **whenever $d_{\text{up}} > T$**, we test 4 cardinal (or 8 octantal) horizontal rays of length $L_{\text{wall}} = T + r_{\text{aircraft}} + 0.5\text{ m}$ from $\mathbf{p}_{\text{CG}}$ along unit horizontal directions $\{\hat{\mathbf{e}}_k\}$:
   - If ray $k$ hits a wall face at distance $r_k \le T + r_{\text{aircraft}}$, we probe a candidate exit point $\mathbf{q}_k = \mathbf{p}_{\text{CG}} + (r_k + r_{\text{aircraft}} + 0.1\text{ m}) \hat{\mathbf{e}}_k$.
   - If $\mathbf{q}_k$ has a valid floor below it (`sampleBelow(q_k) !== null`) and no low ceiling within $H_{\text{nav}}$ above it, then $\hat{\mathbf{e}}_k$ is a valid **lateral exit into free air** with penetration depth:
     $$d_{\text{lat}, k} = r_k + r_{\text{aircraft}}$$
   - Choosing the minimum $d_{\text{lat}} = \min_k d_{\text{lat}, k}$: if $d_{\text{lat}} \le T$, the aircraft is **Shallow Inside a Wall/Cliff** with $d = d_{\text{lat}} \le T$ and horizontal exit vector $\mathbf{d}_{\text{exit}} = (d_{\text{lat}} + 0.05\text{ m}) \hat{\mathbf{e}}_{k^*}$!

### 3.4 TypeScript Reference Pseudocode (with 1:1 Rust/WASM Kernel Structure)

```ts
export type EnclosureKind =
  | "outside"            // In free air (open sky or under a navigable bridge/overhang)
  | "shallow-inside"     // Inside solid matter with 0 < depth <= T: automatic smooth float-out
  | "deep-inside"        // Inside solid matter with depth > T: requires pilot relocation card
  | "provisional-coarse";// Inside a coarse LOD tile while finer detail is actively loading

export interface ColumnClassification {
  kind: EnclosureKind;
  /** Authoritative supporting floor height (m ASL) for JSBSim and AGL consumers. */
  floorMeters: number;
  /** Known overhead ceiling height (m ASL) when under a bridge/overhang; +Infinity in open sky. */
  ceilingMeters: number;
  /** Positive penetration depth (m) when inside solid matter; 0 when outside. */
  depthMeters: number;
  /** Exit displacement in local ENU metres (east, north, up) when inside. */
  exitEastMeters: number;
  exitNorthMeters: number;
  exitUpMeters: number;
  /** Topmost sky-down surface height (m ASL), for deep-teleport fallback. */
  skySurfaceMeters: number;
  revision: number;
}

const DEFAULT_FLOAT_THRESHOLD_METERS = 1.0;
const FLOAT_UP_SPEED_MPS = 1.8;          // Smooth vertical float speed (1 m in 0.55 s)
const FLOAT_DOWN_SETTLE_MPS = 2.5;       // Max floor drop speed under a standing aircraft
const CRACK_BRIDGE_STEPS = 8;            // Hold floor across <= 67 ms / 25 cm seam dropouts
const CRACK_BRIDGE_DISTANCE_METERS = 0.30;
const MIN_NAVIGABLE_HEADROOM_METERS = 2.0;
const COARSE_BRIDGE_ERROR_METERS = 2.0;  // Geometric error above which a newly enclosing tile is provisional
const COARSE_GRACE_STEPS = 180;          // 1.5 s grace at 120 Hz while finer child tiles stream in

/**
 * Compute the downward ray origin altitude (m ASL) for floor tracking.
 *
 * Key properties:
 * 1. When flying under a bridge (zContact = 107.2 m, prevFloor = 100 m),
 *    min(zContact, prevFloor) + T = 101.2 m, so max(zCg, 101.2) = zCg (107.22 m),
 *    which stays strictly below the overhead bridge soffit (108 m).
 * 2. When standing or hovering low on a bridge deck (zContact = 109 m, prevFloor = 109 m)
 *    that refines up to 109.5 m, min(zContact, prevFloor) + T = 110.2 m, catching the
 *    refined bridge deck instead of dropping 9 m to the road below.
 */
export function computeFloorSearchOrigin(
  zCg: number,
  zContact: number,
  prevFloor: number,
  knownCeiling: number,
  floatThresholdMeters: number,
): number {
  const searchMargin = floatThresholdMeters + 0.20;
  const trackedBase = Number.isFinite(prevFloor) ? Math.min(zContact, prevFloor) : zContact;
  const rawOrigin = Math.max(zCg, trackedBase + searchMargin);
  if (Number.isFinite(knownCeiling) && knownCeiling > zCg) {
    return Math.min(rawOrigin, knownCeiling - 0.10);
  }
  return rawOrigin;
}

/**
 * Reclassify a 3D pose when placement, map revision, or floor loss requires
 * distinguishing "under a bridge" from "inside solid terrain/building".
 */
export function classifyPoseInColumn(
  surface: SurfaceQuery,
  latDeg: number,
  lonDeg: number,
  zCg: number,
  zContact: number,
  prevFloor: number,
  knownCeiling: number,
  clearanceMeters: number,
  floatThresholdMeters: number,
): ColumnClassification {
  const headroomRequired = Math.max(MIN_NAVIGABLE_HEADROOM_METERS, clearanceMeters * 2 + 0.5);
  const searchOrigin = computeFloorSearchOrigin(zCg, zContact, prevFloor, knownCeiling, floatThresholdMeters);
  const belowHit = surface.sampleBelow
    ? surface.sampleBelow(latDeg, lonDeg, searchOrigin)
    : sampleBelowFallback(surface, latDeg, lonDeg, searchOrigin);
  const skyHit = surface.sample(latDeg, lonDeg);
  const revision = belowHit?.revision ?? skyHit?.revision ?? 0;

  // Case 1: No sky hit at all -> missing coverage, caller handles via blindSteps.
  if (!skyHit || !Number.isFinite(skyHit.heightMeters)) {
    return {
      kind: "outside", floorMeters: belowHit?.heightMeters ?? prevFloor,
      ceilingMeters: Number.POSITIVE_INFINITY, depthMeters: 0,
      exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: 0,
      skySurfaceMeters: Number.NaN, revision,
    };
  }

  // Case 2: Contact is at or above the topmost sky surface -> unconditionally Outside in open air.
  if (zContact >= skyHit.heightMeters - 0.02) {
    return {
      kind: "outside", floorMeters: skyHit.heightMeters,
      ceilingMeters: Number.POSITIVE_INFINITY, depthMeters: 0,
      exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: 0,
      skySurfaceMeters: skyHit.heightMeters, revision,
    };
  }

  // Case 3: Contact is within shallow threshold T below the surface it was ALREADY standing on/near,
  // or within T below the topmost surface with no navigable cavity.
  const nearTopDepth = skyHit.heightMeters - zContact;
  const wasOnUpperSurface = Number.isFinite(prevFloor) && Math.abs(skyHit.heightMeters - prevFloor) <= floatThresholdMeters + 0.5;
  if (nearTopDepth > 0 && nearTopDepth <= floatThresholdMeters && (wasOnUpperSurface || !belowHit)) {
    return {
      kind: "shallow-inside", floorMeters: skyHit.heightMeters,
      ceilingMeters: Number.POSITIVE_INFINITY, depthMeters: nearTopDepth,
      exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: nearTopDepth,
      skySurfaceMeters: skyHit.heightMeters, revision,
    };
  }

  // Case 4: There IS a surface below the search origin AND a higher surface overhead (Bridge / Overhang candidate).
  if (belowHit && Number.isFinite(belowHit.heightMeters) && skyHit.heightMeters > belowHit.heightMeters + 0.05) {
    // Find the immediate ceiling above zCg by casting straight up (+u).
    const upHit = castVerticalUp(surface, latDeg, lonDeg, Math.max(zCg, belowHit.heightMeters + 0.1), skyHit.heightMeters - zCg + 10);
    const ceilingMeters = upHit ? upHit.heightMeters : skyHit.heightMeters;
    const cavityHeight = ceilingMeters - belowHit.heightMeters;

    // 4a. Check if the contact sank slightly into the lower floor (e.g. landing under a bridge).
    const lowerPenetration = belowHit.heightMeters - zContact;
    if (lowerPenetration > 0.02 && lowerPenetration <= floatThresholdMeters && cavityHeight >= headroomRequired) {
      return {
        kind: "shallow-inside", floorMeters: belowHit.heightMeters,
        ceilingMeters, depthMeters: lowerPenetration,
        exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: lowerPenetration,
        skySurfaceMeters: skyHit.heightMeters, revision,
      };
    }

    // 4b. Navigable headroom between floor below and ceiling above -> Under a bridge / overhang!
    // Verify lateral openness only if we did not enter this cavity coherently from outside.
    if (cavityHeight >= headroomRequired && zContact >= belowHit.heightMeters - 0.02) {
      const coherentUnderpass = Number.isFinite(prevFloor) && Math.abs(belowHit.heightMeters - prevFloor) <= 2.0;
      const openToSky = coherentUnderpass || hasLateralSkyEscape(surface, latDeg, lonDeg, zCg, ceilingMeters);
      if (openToSky) {
        return {
          kind: "outside", floorMeters: belowHit.heightMeters,
          ceilingMeters, depthMeters: 0,
          exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: 0,
          skySurfaceMeters: skyHit.heightMeters, revision,
        };
      }
    }
  }

  // Case 5: No valid floor below (or enclosed in a building/slab). The aircraft is Inside solid matter.
  // Check if a coarse tile (> 2 m geometric error) just caused this overlap over a previously valid floor.
  if (
    Number.isFinite(prevFloor) &&
    zContact >= prevFloor - 0.10 &&
    (skyHit.geometricErrorMeters ?? 0) >= COARSE_BRIDGE_ERROR_METERS
  ) {
    return {
      kind: "provisional-coarse", floorMeters: prevFloor,
      ceilingMeters: Number.POSITIVE_INFINITY, depthMeters: nearTopDepth,
      exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: 0,
      skySurfaceMeters: skyHit.heightMeters, revision,
    };
  }

  // If vertical depth > T, check if a lateral wall/cliff exit exists within <= T.
  if (nearTopDepth > floatThresholdMeters) {
    const lateral = findLateralWallExit(surface, latDeg, lonDeg, zCg, clearanceMeters, floatThresholdMeters);
    if (lateral && lateral.depthMeters <= floatThresholdMeters) {
      return {
        kind: "shallow-inside", floorMeters: lateral.floorMeters,
        ceilingMeters: Number.POSITIVE_INFINITY, depthMeters: lateral.depthMeters,
        exitEastMeters: lateral.eastMeters, exitNorthMeters: lateral.northMeters, exitUpMeters: 0,
        skySurfaceMeters: skyHit.heightMeters, revision,
      };
    }
  }

  return {
    kind: nearTopDepth <= floatThresholdMeters ? "shallow-inside" : "deep-inside",
    floorMeters: skyHit.heightMeters,
    ceilingMeters: Number.POSITIVE_INFINITY,
    depthMeters: Math.max(0, nearTopDepth),
    exitEastMeters: 0, exitNorthMeters: 0, exitUpMeters: Math.max(0, nearTopDepth),
    skySurfaceMeters: skyHit.heightMeters,
    revision,
  };
}
```

### 3.5 Per-Step Update Loop Pseudocode (`terrainContact.update`)

```ts
// Inside createTerrainContact.update():
const lat = sdk.getPropertyValue("position/lat-geod-deg");
const lon = sdk.getPropertyValue("position/long-gc-deg");
const altitude = sdk.getPropertyValue("position/h-sl-ft") * 0.3048;
const roll = sdk.getPropertyValue("attitude/phi-deg") * Math.PI / 180;
const pitch = sdk.getPropertyValue("attitude/theta-deg") * Math.PI / 180;
const clearance = aircraftClearanceMeters(clearanceProfile, roll, pitch);
const cDepth = lowestContactMeters(roll, pitch, clearance);
const zContact = altitude - cDepth;

// 1. Query the floor below the aircraft using the bounded search origin.
const searchOrigin = computeFloorSearchOrigin(
  altitude, zContact, previous?.ground ?? Number.NaN, knownCeiling, floatThresholdMeters
);
let hit = surface.sampleBelow
  ? surface.sampleBelow(lat, lon, searchOrigin)
  : surface.sample(lat, lon);

// 2. Crack bridging: if `sampleBelow` missed or dropped sharply (> 2 m) for <= CRACK_BRIDGE_STEPS
//    over a short horizontal travel (< 0.30 m), hold the previous floor while verifying.
const movedMeters = previous ? horizontalDistanceMeters(lat, lon, previous.lat, previous.lon) : 0;
if (
  previous !== null &&
  (!hit || (previous.ground - hit.heightMeters > 2.0 && hit.revision === previous.revision)) &&
  crackSteps < CRACK_BRIDGE_STEPS &&
  crackDistance + movedMeters < CRACK_BRIDGE_DISTANCE_METERS
) {
  crackSteps += 1;
  crackDistance += movedMeters;
  hit = { heightMeters: previous.height, revision: previous.revision, quality: 0 } as SurfaceHit;
} else {
  crackSteps = 0;
  crackDistance = 0;
}

// 3. Reclassify ONLY if placement, missing below-hit, blind regain, or significant revision jump.
const revisionChanged = previous !== null && hit !== null && hit.revision !== previous.revision;
let cgDelta = 0;
if (previous && revisionChanged) {
  const anchor = surface.sampleBelow
    ? surface.sampleBelow(previous.lat, previous.lon, previous.height + floatThresholdMeters + 1.0)
    : surface.sample(previous.lat, previous.lon);
  cgDelta = anchor && Number.isFinite(anchor.heightMeters) ? anchor.heightMeters - previous.height : Number.NaN;
}

const needsReclassify =
  placement ||
  blindSteps >= BLIND_STEPS ||
  hit === null ||
  (revisionChanged && (!Number.isFinite(cgDelta) || Math.abs(cgDelta) > 0.05));

if (needsReclassify) {
  const classified = classifyPoseInColumn(
    surface, lat, lon, altitude, zContact,
    previous?.ground ?? Number.NaN, knownCeiling, clearance, floatThresholdMeters
  );
  knownCeiling = classified.ceilingMeters;

  if (classified.kind === "provisional-coarse" && coarseGraceSteps < COARSE_GRACE_STEPS) {
    coarseGraceSteps += 1;
    setFloor(classified.floorMeters); // Keep previous valid road floor while fine tile loads!
    return true;
  }
  coarseGraceSteps = 0;

  if (classified.kind === "deep-inside") {
    // Enforce the hill-impact rule: only propose relocation if caused by placement,
    // blind regain, or a map revision raising the surface through the aircraft.
    const worldCaused = placement || blindSteps >= BLIND_STEPS || (revisionChanged && cgDelta > 0);
    if (worldCaused) {
      return issueRelocationProposal(classified, altitude, zContact, clearance);
    }
    // Otherwise the pilot flew into a hill in full view: let physics/collision fault naturally.
  }
}

// 4. Compute multi-contact support plane using `sampleBelow` at each contact's footprint.
const ground = measureMultiContactSupport(hit.heightMeters, cgDelta, revisionChanged);

// 5. Apply Escort Floor & Velocity-Limited Float-Out (Section 5).
return applyFloorAndFloat(sdk, ground, altitude, zContact, cDepth, floatThresholdMeters, dtSeconds);
```

---

## 4. Choosing the Floor (`chooseFloor`)

JSBSim accepts a single horizontal scalar `position/terrain-elevation-asl-ft` each step. Choosing that scalar accurately across bridges, slopes, drones, and jets requires coordinating the **centre-of-gravity floor query** and the **per-contact footprint queries**.

### 4.1 The Downward-Cast Origin Law

Instead of casting from $20\text{ km}$ above the ellipsoid, every vertical floor sample (both under the CG and under each contact unit $i$) casts downward from:
$$z_{\text{search}, i} = \min\Big(h_{\text{ceil,known}} - 0.10,\; \max\big(z_{\text{CG}},\; \min(z_{i}, g_{i,\text{prev}}) + T + 0.20\big)\Big)$$
where $z_i = z_{\text{CG}} - \text{down}_i$ is contact $i$'s world altitude and $g_{i,\text{prev}}$ is the previous step's ground height under contact $i$.

### 4.2 How Every Transition Works

1. **Passing Under a Bridge Deck:**
   - As the aircraft approaches and passes under a bridge deck at $109\text{ m}$ while flying at $103\text{ m}$ over a $100\text{ m}$ road ($g_{i,\text{prev}} = 100\text{ m}$):
   - $\min(z_i, g_{i,\text{prev}}) + 1.2 = \min(103, 100) + 1.2 = 101.2\text{ m}$.
   - $z_{\text{search}, i} = \max(103, 101.2) = 103\text{ m}$.
   - Every downward ray starts at $103\text{ m}$—**$5\text{–}6\text{ m}$ below the bridge deck**—and hits the road at $100\text{ m}$.
   - Even if the aircraft climbs to $107.5\text{ m}$ ($0.5\text{ m}$ below a $108\text{ m}$ soffit), $z_{\text{search}, i} = 107.5\text{ m} < 108\text{ m}$, so it **still** misses the soffit and tracks the $100\text{ m}$ road!
2. **Flying Onto and Off a Bridge Deck:**
   - **Flying onto a deck (airborne above deck height):** Suppose the aircraft approaches at $112\text{ m}$ ASL over water ($0\text{ m}$) and crosses over a bridge deck at $109\text{ m}$. Because $z_{\text{CG}} = 112\text{ m} > 109\text{ m}$, $z_{\text{search}, i} = 112\text{ m}$ is above the deck. As each contact's footprint crosses onto the deck, its `sampleBelow` transitions from $0\text{ m}$ to $109\text{ m}$ while the wheels are $3\text{ m}$ in the air (`WHEELS_CLEAR_METERS` / zero spring compression).
   - **Rolling or flying off a deck edge:** While taxiing off a $109\text{ m}$ deck edge, `createReferenceSupport` (`terrainContact.ts:431–441`) computes `top = Math.max(top, ground[i] + down[i])`. As long as at least one wheel remains on the $109\text{ m}$ deck, `top` holds the plane at $109\text{ m}$ (preventing an immediate drop while still supported); once the last supporting contact passes the edge, `ground[i]` drops to the surface below ($0\text{ m}$) and gravity accelerates the aircraft downward naturally.
   - **Approaching a deck edge slightly below deck level (e.g. $108.7\text{ m}$):**
     - For planes, the swept `bodyProbes` in `visibleMeshCollision.ts` intersect the bridge fascia/edge and trigger a rigid-body impact.
     - For drones (or if a deck edge is bevelled), once inside the top $T$ of the deck with $h_{\text{above}} - h_{\text{below}} < H_{\text{nav}}$, the shallow float lifts the contact onto the deck if it entered within the top bevel, or `visibleMeshCollision` bounces it off the vertical fascia.
3. **Slopes:**
   - The existing multi-contact support law (`terrainContact.ts:148–161`) sets the horizontal plane to:
     $$h_{\text{plane}} = \max_{i \in \text{supported}} (g_i + \text{down}_i) - \max_{j \in \text{active}} \text{down}_j$$
     so that no contact penetrates its own local ground $g_i$ deeper than JSBSim's lowest contact penetrates $h_{\text{plane}}$.
   - Structure contacts (wingtips, tail, belly) keep the `FOOTPRINT_REACH_METERS = 1.0` filter (`terrainContact.ts:435`): a building wall or overhang $> 1\text{ m}$ above where a wingtip would rest is ignored by the support plane and left to `visibleMeshCollision`.
4. **Scale: DJI Mini 2 Drone ($0.2\text{ m}$ span, $2.3\text{ cm}$ leg depth) vs. Jet ($10\text{ m}$ span, $1.5\text{ m}$ gear depth):**
   - On a **drone**, $z_{\text{CG}} - z_{\text{contact}} \approx 0.023\text{ m}$. Because $z_{\text{search}}$ includes $\min(z_{\text{contact}}, h_{\text{floor,prev}}) + T + 0.20\text{ m}$, a parked or low-hovering drone tracks upward surface refinements up to $T$ without losing its floor, even though its CG is only $2.3\text{ cm}$ above its skids.
   - We also scale the automatic float threshold $T$ optionally by aircraft class (or use a unified default $T = 1.0\text{ m}$, since photogrammetry vertical refinement jumps of $0.2\text{–}0.8\text{ m}$ are property of the *map tile*, not the aircraft size!). Keeping $T = 1.0\text{ m}$ across all aircraft guarantees that a $0.5\text{ m}$ Google tile refinement under a landed DJI Mini 2 floats the drone up smoothly rather than popping up a modal card.

### 4.3 Fixing the Other Four Consumers of "Ground Height Here"

In `fly/src/flight/createFlightSimApp.ts`, we add `terrainContact.getEffectiveGroundMeters(): number | null` (which returns the filtered aircraft-relative floor `previous?.ground ?? null`) and update the four consumers:

1. **Drone Controller (`createFlightSimApp.ts:4058, 4149`):** Pass `terrainContact.getEffectiveGroundMeters()` instead of `sampledSurfaceHeight`. Under a $9\text{ m}$ bridge at $3\text{ m}$ above the road, `droneController` sees `aglM = 3.0 m`, so altitude hold, landing descent damping (`droneController.ts:79`), and 1-second down-stick disarm on the road under the bridge all work seamlessly.
2. **HUD AGL Readout (`createFlightSimApp.ts:4222`):** Pass `terrainContact.getEffectiveGroundMeters() ?? sampledSurfaceHeight` to `hudBar.update(...)`. The HUD shows height above the road when under a bridge, and height above the bridge deck when over the bridge.
3. **Overlap Card "Teleport above ground" (`createFlightSimApp.ts:3014–3038`):** Use `proposal.altitudeMeters` (which `classifyPoseInColumn` sets to `enclosingSurfaceMeters + clearance`), or `terrainContact.getEnclosingSurfaceMeters()` when `ignoreOverlap` is active.
4. **Spawn Placement:** Initial location spawn (`prepareTerrain` / airport spawn) continues to use sky-down `sample(lat, lon)` so spawning at a bridge coordinate places the aircraft on the bridge deck; restoring a saved flight (`savedFlight.ts`) at a known `(lat, lon, alt)` runs `classifyPoseInColumn` so a flight saved under a bridge resumes under the bridge.

---

## 5. Floating Out Smoothly (`applyFloorAndFloat`)

### 5.1 Why Naive Floor Assignment Fails in JSBSim

If an aircraft's contact is $d = 0.7\text{ m}$ below the true surface $h_{\text{target}}$ and we write $h_{\text{target}}$ directly to `position/terrain-elevation-asl-ft`:
1. `fixedStepLoop.ts:139` sees `groundPenetration = 0.7 m > MAX_GROUND_PENETRATION_METERS (0.5 m)` and **immediately halts the simulator with a `"Hard ground impact"` fault**.
2. Even for $d = 0.35\text{ m}$, JSBSim's gear springs see an instantaneous $35\text{ cm}$ step compression and fire a violent upward impulse, launching the aircraft into the air.
3. Calling `restoreSimulation(sdk, snapshot)` (`runIc`) every step to move the aircraft resets JSBSim's multi-step Adams-Bashforth integrator history, zeroing out angular accelerations and exciting pitch/oleo transients.

### 5.2 The Escort Floor + Velocity-Limited Altitude Slew Algorithm

We solve all three problems simultaneously using **direct altitude slewing (`position/h-sl-ft`) paired with a rising Escort Floor (`position/terrain-elevation-asl-ft`)**:

Let $h_{\text{target}}$ be the true support plane height (`ground`), $z_{\text{contact}} = h_{\text{CG}} - c(\phi, \theta)$ be the lowest active contact altitude, and $d_{\text{eq}}$ be the aircraft's current resting spring compression on its previous floor:
$$d_{\text{eq}} = \text{clamp}\big(h_{\text{floor,prev}} - z_{\text{contact}},\; 0,\; d_{\text{max\_eq}}\big), \qquad d_{\text{max\_eq}} = 0.08\text{ m}$$
The excess penetration depth below the target equilibrium pose is:
$$d_{\text{excess}} = (h_{\text{target}} - d_{\text{eq}}) - z_{\text{contact}}$$

#### Case A: Upward Float ($0 < d_{\text{excess}} \le T$)
When $0 < d_{\text{excess}} \le T$ (and the penetration was not caused by flying at high speed into a hill in full view):
1. **Compute the velocity-limited upward step:**
   $$\delta h = \min\big(d_{\text{excess}},\; v_{\text{float}} \cdot \Delta t\big), \qquad v_{\text{float}} = 1.8\text{ m/s}, \quad \Delta t = \frac{1}{120}\text{ s}$$
   ($\delta h \le 1.5\text{ cm}$ per $120\text{ Hz}$ step).
2. **Slew the aircraft altitude directly (no `runIc`, integrator history preserved):**
   $$h_{\text{CG,new}} = h_{\text{CG}} + \delta h$$
   ```ts
   sdk.setPropertyValue("position/h-sl-ft", hCgNew / 0.3048);
   ```
3. **Set JSBSim's floor to the Escort Floor:**
   $$z_{\text{contact,new}} = z_{\text{contact}} + \delta h, \qquad h_{\text{escort}} = \min\big(h_{\text{target}},\; z_{\text{contact,new}} + d_{\text{eq}}\big)$$
   ```ts
   sdk.setPropertyValue("position/terrain-elevation-asl-ft", hEscort / 0.3048);
   ```

#### Why the Escort Floor Works So Well
- **Zero spring launch:** At every single $120\text{ Hz}$ tick during the float, JSBSim's gear springs see a penetration of **at most $d_{\text{eq}} \le 8\text{ cm}$** (the exact resting compression the aircraft already had!). The springs support the aircraft's weight normally without ever firing a launch impulse.
- **Never trips `fixedStepLoop.ts`:** Because `hEscort - zContactNew = d_eq <= 0.08 m < 0.50 m`, `fixedStepLoop.ts:139` never faults.
- **Centimetre refinements finish in 1–2 ticks:** A $2\text{ cm}$ refinement ($\delta h = 1.5\text{ cm}$ on tick 1, $0.5\text{ cm}$ on tick 2) completes within a single $60\text{ Hz}$ render frame, completely imperceptible and with zero UI cards.
- **What happens if the pilot pushes back down while floating?**
  - Suppose a drone pilot holds full down-stick (`vd > 0`) or a plane pilot pushes nose-down while floating out of a $0.6\text{ m}$ overlap.
  - Because `hEscort` sits right under the contacts at $z_{\text{contact,new}} + d_{\text{eq}}$, JSBSim's normal contact springs and dampers resist downward motion into `hEscort` just as if the aircraft were sliding up a gentle ramp!
  - To prevent extreme downward velocity (e.g., if the aircraft entered the float with $v_{\text{down}} = 4\text{ m/s}$) from compressing the springs past $0.25\text{ m}$ into `hEscort`, if $z_{\text{contact}}$ at the start of the next step has dropped below $h_{\text{escort,prev}} - 0.15\text{ m}$, we clamp $h_{\text{CG}} \ge h_{\text{escort,prev}} + c(\phi, \theta) - 0.08\text{ m}$ before applying the next $+\delta h$ upward float step. Thus the pilot cannot push the aircraft deeper underground during a float, and the moment the float reaches $h_{\text{target}}$, the aircraft rests stably on $h_{\text{target}}$ (where holding down-stick on a drone disarms it normally after $1.0\text{ s}$!).

#### Case B: Downward Refinement Under a Standing Aircraft ($\Delta h < 0$)
When the ground refines **downward** ($\Delta h < 0$) under a standing aircraft:
- The aircraft is **Outside** (in the air above the new ground), **never** "underground"!
- **Never show the underground card for $\Delta h < 0$.**
- To prevent a parked aircraft from dropping harshly onto its oleos if the ground refines downward by e.g. $-0.3\text{ m}$, if the aircraft was `standing()` on $h_{\text{floor,prev}}$ and $h_{\text{target}} < h_{\text{floor,prev}}$ with $h_{\text{floor,prev}} - h_{\text{target}} \le T$:
  - We slew both `hCg` and `hEscort` downward together by $\max(\Delta h, -v_{\text{settle}} \Delta t)$ while `standing()`, OR simply let `hEscort` descend at $v_{\text{settle}} = 2.5\text{ m/s}$ so gravity settles the aircraft smoothly onto the lower surface with zero bounce and zero modal cards.

#### Case C: Lateral Wall / Cliff Push-Out ($d_{\text{lat}} \le T < d_{\text{up}}$)
When `classifyPoseInColumn` finds a shallow lateral wall exit ($\mathbf{d}_{\text{exit}} = (\Delta E, \Delta N, 0)$ with $\|\mathbf{d}_{\text{exit}}\| \le T$):
- Because JSBSim's `run()` recomputes `position/lat-geod-deg` and `position/long-gc-deg` from its internal integrator state, lateral position shifts use `captureSimulation` / `restoreSimulation` (`visibleMeshCollision.ts:395–415`) with the horizontal velocity component into the wall zeroed out, moving the aircraft out of the wall by $\mathbf{d}_{\text{exit}}$ in one clean depenetration step (just like `visibleMeshCollision` already does for probe hits).

---

## 6. Handling Map Changes & Streaming Artifacts

### 6.1 Summary of Map-Change Rules

| Map Event | Detection | System Response |
| :--- | :--- | :--- |
| **1. Small refinement under a parked/standing aircraft** ($|\Delta h| \le T \approx 1\text{ m}$) | `revisionChanged && Math.abs(cgDelta) <= T` | **Silent automatic adjustment:** Escort floor + altitude slew moves the aircraft smoothly with the surface at $\le 1.8\text{ m/s}$, keeping exact oleo compression. **Zero cards shown.** |
| **2. Large upward refinement** ($\Delta h > T$) | `revisionChanged && cgDelta > T` putting contact $> T$ inside solid matter | **Deep overlap proposal:** Shows the `"Terrain overlaps your aircraft"` card. If the aircraft was **standing**, it keeps its old floor (`keptFloor`) and **physics keeps running** (Requirement 6). If airborne, physics holds until answered. |
| **3. Coarse LOD tile temporarily fills the space under a bridge** | `revisionChanged`, aircraft was Outside on `prevFloor`, new `skyHit` has `geometricErrorMeters >= 2.0` and `sampleBelow` loses the cavity | **Provisional Coarse Grace (`PROVISIONAL_COARSE`):** Hold `prevFloor` (the road) and suppress reclassification for up to `COARSE_GRACE_STEPS = 180` ($1.5\text{ s}$) while finer child tiles stream in. Only classify as Inside if the solid mesh persists after the tile settles (`geometricErrorMeters < 2.0` or grace expires). |
| **4. LOD overlap (coarse + fine tiles coexist)** | Both meshes visible in `scene.meshes` during tile transition | **Handled per-query by LOD Dominance:** `sampleBelow` and `sample` ignore any coarse hit whose height lies within its `geometricErrorMeters` of a finer tile hit. Additionally, if a coarse ground tile lies *below* a fine building roof within `coarse.geometricErrorMeters`, it is rejected as a false floor. |
| **5. Tile dropout and regain** | `hit === null` for $N$ steps, then `hit !== null` | - **$N \le 8$ steps ($67\text{ ms}$) & $< 0.30\text{ m}$ travel:** Crack-bridging holds the last floor.<br>- **$8 < N < 60$ steps ($0.5\text{ s}$) on Google tiles:** Continues on previous floor (`terrainContact.ts:936`).<br>- **$N \ge 60$ steps (`BLIND_STEPS`):** On regain, runs `classifyPoseInColumn`. If Outside or under a bridge, adopts the floor below; if Shallow Inside ($\le T$), floats out silently; if Deep Inside ($> T$), shows `"surface regained above the aircraft"` card. |
| **6. First placement at a new location** | `placement === true` on first valid surface sample | Wait until `prepareTerrainWithRecovery` finishes (or first valid surface arrives). Run `classifyPoseInColumn`. If within $T$ of the surface, snap/float to the surface without a card; if deeper than $T$ inside solid matter, show the `"placement"` card. |

### 6.2 When Should the System Wait for More Detail Before Deciding?

The system should defer a "Deep Inside" verdict (and keep the aircraft flying on its established floor) in **exactly two bounded situations**:
1. **In-Flight Coarse Tile Swap (`PROVISIONAL_COARSE`):** The aircraft already had a valid floor $h_{\text{floor,prev}}$, and a newly visible mesh with `geometricErrorMeters >= 2.0 m` suddenly encloses the aircraft. Because 3D Tiles traversal often makes a coarse parent or sibling visible for $0.2\text{–}1.0\text{ s}$ while child glTFs download and parse, waiting up to $1.5\text{ s}$ (`180` steps) or until `geometricErrorMeters < 2.0 m` prevents false underground cards under bridges during LOD transitions.
2. **Initial Spawn Preparation:** Handled before flight starts by `prepareTerrainWithRecovery`, which already waits for usable terrain progress before unpausing physics.

---

## 7. Failure-Mode Table

| # | Scenario | What the Algorithm Does | What the User Sees |
| :- | :--- | :--- | :--- |
| **1** | **Flying under a bridge** (e.g. $3\text{ m}$ above road, deck at $9\text{ m}$) | `computeFloorSearchOrigin` starts `sampleBelow` at $z_{\text{CG}} = 3\text{ m}$ (below the $9\text{ m}$ deck). `sampleBelow` hits the $0\text{ m}$ road. JSBSim floor = $0\text{ m}$. Drone controller & HUD receive $0\text{ m}$ floor ($3\text{ m}$ AGL). | Smooth normal flight under the bridge; HUD reads `3 m AGL`; no spring launch; no overlap card. |
| **2** | **Landing under a bridge** | As the aircraft descends to $z_{\text{contact}} = 0\text{ m}$ under the $9\text{ m}$ deck, `sampleBelow` from $\max(z_{\text{CG}}, 1.2\text{ m})$ hits the $0\text{ m}$ road. Contacts compress normally against the $0\text{ m}$ road. Drone disarms after $1\text{ s}$ down-stick. | Aircraft touches down, rolls/settles, and parks on the road or water beneath the bridge. |
| **3** | **Landing on a bridge** | While approaching above $9\text{ m}$, `sampleBelow` from $z_{\text{CG}} > 9\text{ m}$ hits the $9\text{ m}$ bridge deck. Multi-contact support plane tracks the $9\text{ m}$ deck. Even if contacts sink $5\text{ cm}$ into the deck, $\min(z_{\text{contact}}, h_{\text{prev}}) + 1.2 = 10.15\text{ m}$ keeps `sampleBelow` above the $9\text{ m}$ deck. | Normal touchdown and rollout on top of the bridge deck. |
| **4** | **Sunk in slightly ($\le T$, e.g. $0.4\text{ m}$ into road or deck)** | `classifyPoseInColumn` / hot loop sees $0 < d_{\text{excess}} = 0.4\text{ m} \le T$. Enters `SHALLOW_FLOAT`: slews `position/h-sl-ft` up at $1.8\text{ m/s}$ while holding `hEscort` at $z_{\text{contact}} + d_{\text{eq}}$. Completes in $0.22\text{ s}$. | Aircraft smoothly rises $0.4\text{ m}$ onto the surface over $\sim 0.2\text{ s}$ with no bounce, no freeze, and no card. |
| **5** | **Sunk in deeply ($> T$, e.g. $5\text{ m}$ underground from placement or big refinement)** | `classifyPoseInColumn` finds $h_{\text{below}} = \text{null}$, $d_{\text{up}} = 5\text{ m} > T$, no lateral exit $\le T$. Creates `TerrainRelocationProposal` ($+5.0\text{ m}$). | Underground card appears (`"The current map suggests moving you 5.0 m..."`) with **Resume flight here** and **Teleport above ground**. |
| **6** | **Refinement under a parked aircraft** (e.g. $\pm 0.02\text{ m}$ to $\pm 0.8\text{ m}$) | `cgDelta` is within $[-T, +T]$. Escort floor + altitude slew adjusts aircraft height smoothly at $1.8\text{ m/s}$ without resetting integrators (`runIc` not called). | Completely silent; parked aircraft stays resting on the refined surface; **no `"moving you 0.0 m"` card ever appears**. |
| **7** | **Drone hovering $5\text{ cm}$ over the ground while a tile refines** | - If ground refines to $+3\text{ cm}$ (still $2\text{ cm}$ below legs): floor updates to $+3\text{ cm}$; drone stays hovering.<br>- If ground refines to $+15\text{ cm}$ ($10\text{ cm}$ above legs, $\le T$): escort floor floats drone up by $10\text{ cm}$ smoothly. | Drone continues hovering smoothly without any card or spring kick. |
| **8** | **LOD swap temporarily fills space under a bridge** | `classifyPoseInColumn` sees `geometricErrorMeters >= 2.0 m` over an established road floor `prevFloor`. Enters `PROVISIONAL_COARSE` for up to $1.5\text{ s}$, retaining `prevFloor`. When fine tile arrives, normal bridge tracking resumes. | Aircraft flies through the temporary coarse tile without jumping to the bridge deck or pausing. |
| **9** | **Tile dropout and regain** | - Short miss ($\le 8$ steps): crack filter holds floor.<br>- Medium miss ($< 60$ steps on Google): holds floor.<br>- Long miss ($\ge 60$ steps): on regain, classifies column; only shows card if deeply buried ($> T$) inside solid matter. | Transient streaming misses are invisible; regaining a bridge overhead while over a road adopts the road floor with no card. |
| **10** | **First placement at a new location** | Runs `classifyPoseInColumn` on first surface sample. If within $\le T$ of surface, snaps/floats onto surface; if $> T$ inside solid matter, withholds floor and proposes relocation. | Clean spawn on roads, runways, and bridges; card shown only if spawned $> 1\text{ m}$ underground. |
| **11** | **Fast jet at low level** ($0.5\text{ m}$ per step at $60\text{ m/s}$, or $2.0\text{ m}$ per step at $240\text{ m/s}$) | Swept `bodyProbes` in `visibleMeshCollision.ts` raycast the full step segment $[\mathbf{p}_{t-1}, \mathbf{p}_t]$ ($0.5\text{–}2.0\text{ m}$) against visible triangles. Passing under a bridge hits nothing; hitting a hill/pier triggers rigid-body bounce or hard-impact fault (never lifted through the hill). | High-speed under-bridge flight works at any speed; flying into terrain impacts physically. |
| **12** | **Photogrammetry "curtains" that permanently fill the space under a bridge** | In finest LOD (`geometricErrorMeters < 2.0 m`), the curtain is real visible mesh with no open space inside. Planes hitting the curtain wall bounce via `visibleMeshCollision`; if a drone flies through a gap into the hollow curtain interior (no floor below), it floats out if $\le T$ from the wall/top, or gets the overlap card if $> T$ deep. | Matches the visible mesh: solid curtains act as solid obstacles rather than invisible pass-throughs or vertical launch pads. |
| **13** | **Thin geometry (poles, railings, lamp posts, bridge cables)** | A single contact ray hitting a $10\text{ cm}$ railing top while other contacts see the deck $1.0\text{ m}$ below is rejected for structure contacts by `FOOTPRINT_REACH_METERS`, and for the CG ray by the crack/spike width check if narrower than $2 r_{\text{crack}}$ across consecutive steps, or floated over if $\le T$. | Taxing past a thin pole or railing on a bridge does not teleport the floor to the top of the pole. |
| **14** | **Steep cliffs and building walls** | Structure contacts ignore surfaces $> 1\text{ m}$ above their rest height (`FOOTPRINT_REACH_METERS`). Hull probes bounce off walls along the horizontal normal. If a map refinement pushes a wall $\le T$ over a hovering aircraft, `findLateralWallExit` pushes it out horizontally rather than lifting it $30\text{ m}$ to the roof. | Flying near cliffs or walls never snaps the floor to the cliff top; shallow wall overlaps push out horizontally. |

---

## 8. Headless Test Suite Specification

Using the existing Vitest harness (`ground-contact/tests/terrainRelocation.test.ts` and `fly/src/flight/physics/terrainContact.test.ts` with real JSBSim WASM + scripted multi-layer `SurfaceQuery` mocks), we define 10 deterministic test scenarios:

1. **T1 — Drone & C172 Flight Under a Bridge Deck (`bridge-underpass`):**
   - *Setup:* Mock surface with road slab at $h = 100\text{ m}$ and bridge slab at $h \in [108\text{ m}, 109\text{ m}]$ for $\text{lat} \in [34.000, 34.001]$. Aircraft flies at $h_{\text{CG}} = 103\text{ m}$ (and in a second sub-test at $107.4\text{ m}$, just $0.6\text{ m}$ below the soffit).
   - *Pass criteria:* Across 240 steps, `position/terrain-elevation-asl-ft` remains $100\text{ m}$ ($\pm 0.01\text{ m}$), `getRelocationProposal()` is `null`, `loop.getFault()` is `null`, and `getEffectiveGroundMeters()` is $100\text{ m}$.
2. **T2 — Landing Under a Bridge and Disarming (`bridge-under-landing`):**
   - *Setup:* Same bridge geometry ($100\text{ m}$ road, $109\text{ m}$ deck). DJI Mini 2 starts at $102\text{ m}$ and commands full down-stick (`vertical = -1`).
   - *Pass criteria:* Drone touches down smoothly at $h_{\text{contact}} \approx 100\text{ m}$, disarms after $1.0\text{ s}$ on the ground (`armed === false`), no relocation proposal is created.
3. **T3 — Landing on Top of a Bridge and Refining (`bridge-top-landing`):**
   - *Setup:* C172 and DJI Mini 2 land on the $109\text{ m}$ bridge deck (road at $100\text{ m}$ below). Revision increments and bridge deck refines to $109.25\text{ m}$.
   - *Pass criteria:* Floor stays on the upper layer ($109\text{ m} \to 109.25\text{ m}$), never drops to $100\text{ m}$, aircraft floats smoothly to $109.25\text{ m}$ + clearance within $30$ steps, `getRelocationProposal()` is `null`.
4. **T4 — Silent Automatic Float-Out for Shallow Penetration (`shallow-float-up`):**
   - *Setup:* Aircraft standing at $100\text{ m}$; map refines to $100.8\text{ m}$ ($d = 0.8\text{ m} \le T = 1.0\text{ m}$).
   - *Pass criteria:* `getRelocationProposal()` is `null`; `sdk.runIc` is **not** called; over $60$ steps ($0.5\text{ s}$) `altMeters` rises monotonically at $\le 1.85\text{ m/s}$ to $100.8\text{ m} + \text{clearance}$; peak vertical velocity $v_{\text{up}} < 2.2\text{ m/s}$ (no spring launch); `loop.getFault()` is `null`.
5. **T5 — Pilot Pushing Down During Shallow Float (`shallow-float-pilot-pushdown`):**
   - *Setup:* Same $0.8\text{ m}$ shallow overlap, but aircraft starts with downward velocity $v_{\text{down}} = 2.0\text{ m/s}$ and full down-command.
   - *Pass criteria:* Aircraft never sinks below initial $z_{\text{contact}} - 0.08\text{ m}$, `loop.getFault()` is `null`, and contact reaches $100.8\text{ m}$ within $90$ steps.
6. **T6 — Deep Penetration Triggers Proposal Without Freezing a Standing Aircraft (`deep-overlap-proposal`):**
   - *Setup:* Standing aircraft at $100\text{ m}$; map refines to $102.5\text{ m}$ ($d = 2.5\text{ m} > T$).
   - *Pass criteria:* `getRelocationProposal()` is non-null (`deltaMeters ≈ 2.5`), `contact.update()` returns `true` (physics not frozen for standing aircraft), and `floor()` remains $100\text{ m}$ (`keptFloor`) while pending. For an *airborne* aircraft at $101.5\text{ m}$ when ground refines to $104\text{ m}$ ($d > T$), `contact.update()` returns `false` (`holding === true`) and withholds support.
7. **T7 — Centimetre & Downward Refinements Never Produce the `0.0 m` Card (`silent-cm-and-downward-refinement`):**
   - *Setup:* Landed drone on $100.00\text{ m}$ ground. Revisions step through $[100.005, 100.02, 99.97, 99.60]\text{ m}$.
   - *Pass criteria:* `getRelocationProposal()` is `null` at every step; aircraft settles smoothly onto each height.
8. **T8 — Coarse LOD Bridge Curtain Grace Period (`provisional-coarse-bridge`):**
   - *Setup:* Aircraft flying at $103\text{ m}$ over $100\text{ m}$ road. At step 10, a coarse tile (`geometricErrorMeters = 8.0`) places a single surface at $109\text{ m}$ with no surface at $100\text{ m}$. At step 60 ($0.5\text{ s}$ later), the fine tile (`geometricErrorMeters = 0.5`) restores the $100\text{ m}$ road beneath the $109\text{ m}$ bridge.
   - *Pass criteria:* Floor remains $100\text{ m}$ throughout steps 10–60; no relocation proposal; no altitude jump.
9. **T9 — Tile Crack Immunity (`tile-seam-crack`):**
   - *Setup:* Aircraft taxing at $10\text{ m/s}$ on a $100\text{ m}$ runway crosses a $15\text{ cm}$ wide gap (2 physics steps where `sampleBelow` returns `null` or $-50\text{ m}$).
   - *Pass criteria:* Floor stays at $100\text{ m}$ across the 2-step seam; zero pitch jolt.
10. **T10 — Lateral Wall Refinement Push-Out (`wall-lateral-exit`):**
    - *Setup:* Drone hovering at $h = 105\text{ m}$ beside a $130\text{ m}$ building wall. Revision shifts the wall $0.5\text{ m}$ east so the drone is $0.3\text{ m}$ inside the building horizontally ($d_{\text{up}} = 25\text{ m} > T$, $d_{\text{lat}} = 0.3\text{ m} \le T$).
    - *Pass criteria:* Reclassification selects the $0.3\text{ m}$ lateral exit rather than proposing a $25\text{ m}$ teleport to the roof.

---

## 9. Migration Path, Cost Budget, and Risk / Measurement Plan

### 9.1 Phased Migration Path from Current Code

#### Phase 1: Smallest Safe First Step (Fixes Bridges + Eliminates the `0.0 m` Card Immediately)
1. **`foss-earth` (`surfaceQuery.ts` & `contracts.ts`):** Add `sampleBelow(latDeg, lonDeg, maxAltitudeMeters)` (12 lines, reusing `cast` and the existing `geometricErrorMeters` loop).
2. **`ground-contact` (`terrainContact.ts` & `frameSurfaceQuery.ts`):**
   - Use `computeFloorSearchOrigin` + `sampleBelow` for the centre sample and contact footprint samples, falling back to sky `sample(lat, lon)` only when `sampleBelow` returns `null` (to detect deep subterranean burial).
   - Replace the `Math.abs(refinement) > 0.00001` relocation trigger (`terrainContact.ts:1030`) with the **$T = 1.0\text{ m}$ Escort Floor + Velocity-Limited Float** (`applyFloorAndFloat`):
     - Refinements and placements with $0 < d_{\text{excess}} \le T$ float out automatically via `position/h-sl-ft` + `hEscort` with no proposal card.
     - Downward refinements ($\Delta h < 0$) never create a relocation proposal; the escort floor settles at $\le 2.5\text{ m/s}$.
     - Only penetrations with $d_{\text{excess}} > T$ (caused by placement, blind regain, or upward refinement) create a `TerrainRelocationProposal`.
   - Export `getEffectiveGroundMeters()` from `createTerrainContact`.
3. **`fly` (`createFlightSimApp.ts`):** Pass `terrainContact.getEffectiveGroundMeters()` to `droneHooks` and `hudBar.update`.

*Why Phase 1 is the ideal first PR:* It requires **zero new ray types** (only a shorter ray origin in `sampleBelow`), **zero changes to WASM memory layouts**, fixes flying and landing under bridges, fixes the drone controller and HUD under bridges, and completely eliminates the `"moving you 0.0 m"` card.

#### Phase 2: Full Column Reclassification, Provisional Coarse Grace, and Lateral Wall Exit
1. Add `classifyPoseInColumn` (upward ceiling ray + `PROVISIONAL_COARSE` grace window + 4-ray `findLateralWallExit` when $d_{\text{up}} > T$).
2. Add crack-bridging hysteresis (`CRACK_BRIDGE_STEPS = 8`, `CRACK_BRIDGE_DISTANCE_METERS = 0.30`).
3. Add 4 lightweight hull probes for drones in `collisionGeometry.ts` (`front-left`, `front-right`, `rear-left`, `rear-right` rotor guards) so drones flying into bridge piers or tree crowns get a clean rigid-body bounce instead of passing into the mesh.

#### Phase 3: Rust/WASM Kernel Parity (`WasmContactSupport`)
1. Update `WasmContactSupport` in `ground-contact/wasm` (`REQUEST_STRIDE` from 6 to 7 to include `maxAltitudeMeters` per footprint sample request) so the WASM contact-support kernel requests `sampleBelow` with exact parity with `createReferenceSupport`.

### 9.2 Per-Step Cost Analysis (at 120 Hz)

Using the measured timings from `ground-contact/docs/collision-compute-comparison.md` ($0.168\text{ }\mu\text{s}$/ray with CPU BVH, $0.44\text{ ms}$/ray worst-case unindexed linear scan, $0.9\text{ }\mu\text{s}$/ray raster):

| Regime | Frequency | Rays / Step | CPU Cost (with BVH) | CPU Cost (Unindexed Fallback) | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Steady Cruise (`> 10 m` AGL, Outside)** | $99\%$ of flight steps | **1** (`sampleBelow` at CG) | **$\sim 0.2\text{ }\mu\text{s}$** | $\sim 0.2\text{–}0.4\text{ ms}$ | Shorter ray segment ($z_{\text{CG}} \to -1\text{ km}$) prunes high mesh AABBs faster than $20\text{ km}$ sky ray. |
| **Near Ground / Under Bridge (`< 10 m` AGL, Outside or Floating)** | Every step near ground | **1** CG + **up to 2** contacts = **3 rays** | **$\sim 0.5\text{ }\mu\text{s}$** | $\sim 0.6\text{–}1.2\text{ ms}$ | Identical ray count to today's `GOOGLE_FOOTPRINT_SAMPLES_PER_STEP = 2`. |
| **Reclassification (`revisionChanged` with jump, or `sampleBelow === null`)** | Rare ($\le 1\text{–}2\times$ per tile load or bridge entry) | **2–3 vertical rays** (+ **4 short horizontal rays** only if $d_{\text{up}} > T$) | **$\sim 0.5\text{–}1.2\text{ }\mu\text{s}$** | $\sim 0.8\text{–}2.0\text{ ms}$ (once on revision change) | Horizontal rays have length $\le 2\text{ m}$, intersecting at most 1–2 mesh AABBs. |
| **Display/HUD/Drone Query (`createFlightSimApp.ts`)** | Once per render frame | **0 extra rays** (reuses `terrainContact.getEffectiveGroundMeters()`) | **$0\text{ }\mu\text{s}$** | **$0\text{ ms}$** | Saves 1 sky-down `sample()` per render frame compared to today! |

### 9.3 Risks & What to Measure First in the Real App

1. **Risk 1: Single-Sheet vs. Double-Sided Bridge Meshes in Google 3D Tiles**
   - *Risk:* Some smaller bridges in Google 3D Tiles have thick "curtains" hanging down from the deck sides or no road mesh rendered directly beneath the span at coarse LODs.
   - *What to measure first:* Log `{ skyHeight, belowHeight, cavityHeight, skyGeometricError, belowGeometricError }` in `flightLog` when flying under 5 real bridges (e.g., Golden Gate Bridge, Bay Bridge, a highway overpass, a narrow footbridge, and a tree-lined street). Verify the distribution of `cavityHeight` and `geometricErrorMeters` during approach.
2. **Risk 2: Float-Up Speed $v_{\text{float}}$ vs. Suspension Feel**
   - *Risk:* If $v_{\text{float}}$ is too fast ($> 4\text{ m/s}$), a $0.8\text{ m}$ float looks like a pop; if too slow ($< 0.5\text{ m/s}$), a fast-taxiing plane could stay partially sunk for $2\text{ s}$.
   - *What to measure first:* Test $v_{\text{float}} = 1.8\text{ m/s}$ across C172, SF-50, F-35B, and DJI Mini 2 on scripted $0.05\text{ m}$, $0.30\text{ m}$, and $0.90\text{ m}$ step refinements during stationary parking and $30\text{ kt}$ taxi rollout, logging peak gear force and pitch rate.
3. **Risk 3: False Floor from Overlapping Coarse Parent Tile Inside a Building**
   - *Risk:* While a coarse tile is still enabled alongside a finer building tile, `sampleBelow` inside the building could hit the coarse tile's ground below the building and mistake the building interior for a bridge span.
   - *Mitigation built into the design & what to verify:* `sampleBelow` must pass the sky hit's `geometricErrorMeters` as `finerThanMeters` (or reject any `belowHit` whose `geometricErrorMeters > skyHit.geometricErrorMeters` when `skyHit.heightMeters - belowHit.heightMeters <= belowHit.geometricErrorMeters`). Verify in the diagnostic overlay that `belowHit` is always `null` inside fine-LOD buildings.
