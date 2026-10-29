export interface StructuralPosition { xIn: number; yIn: number; zIn: number }

/**
 * A point on the visible hull, in JSBSim structural inches (aft, right, up),
 * swept against visible terrain and building triangles each physics step.
 * It is converted with the live CG, so fuel and payload move it as they move
 * the aircraft. These are point probes, not the vertices of a solid hull.
 */
export interface BodyCollisionProbe extends StructuralPosition {
  name: string;
  /**
   * Inches the swept point sits above the hull point, inside the airframe.
   * Over a STRUCTURE contact, JSBSim's spring carries an ordinary scrape and
   * the probe is only crossed once the hull has gone further into the ground
   * than that spring holds it. An impact then lifts the hull point, not the
   * swept point, clear of the surface.
   */
  insetIn: number;
}

/** How far a probe over a STRUCTURE contact sits inside the hull: more than its static compression. */
const STRUCTURE_PROBE_INSET_IN = 3;

/**
 * Each aircraft's hull probes, measured on the model the app draws: its
 * spinner or radome, the belly between its belly contacts, the tail, both
 * wingtips and, on the SF50, the top of the engine nacelle. The belly, tail
 * and wingtip probes sit over the aircraft's STRUCTURE contacts.
 */
export const C172_BODY_COLLISION_PROBES: readonly BodyCollisionProbe[] = [
  { name: "nose", xIn: -58.7, yIn: 0, zIn: 34.6, insetIn: 0 },
  { name: "belly", xIn: 42, yIn: 0, zIn: 11.9, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "tail", xIn: 255.9, yIn: 0, zIn: 35.5, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "left-wing", xIn: 53.7, yIn: -216.5, zIn: 68.5, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "right-wing", xIn: 53.7, yIn: 216.5, zIn: 68.5, insetIn: STRUCTURE_PROBE_INSET_IN },
];

export const SF50_BODY_COLLISION_PROBES: readonly BodyCollisionProbe[] = [
  { name: "nose", xIn: 0, yIn: 0, zIn: -37, insetIn: 0 },
  { name: "belly", xIn: 175.2, yIn: 0, zIn: -60.1, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "tail", xIn: 357.9, yIn: 0, zIn: -38.4, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "left-wing", xIn: 173.9, yIn: -226.4, zIn: -35.7, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "right-wing", xIn: 173.9, yIn: 226.4, zIn: -35.7, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "nacelle", xIn: 197.7, yIn: 0, zIn: 20.6, insetIn: 0 },
];

export const F35B_BODY_COLLISION_PROBES: readonly BodyCollisionProbe[] = [
  { name: "nose", xIn: 0, yIn: 0, zIn: 18, insetIn: 0 },
  { name: "belly", xIn: 357.4, yIn: 0, zIn: -21.6, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "tail", xIn: 515.6, yIn: 0, zIn: 0, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "left-wing", xIn: 415.9, yIn: -208.4, zIn: 16, insetIn: STRUCTURE_PROBE_INSET_IN },
  { name: "right-wing", xIn: 415.9, yIn: 208.4, zIn: 16, insetIn: STRUCTURE_PROBE_INSET_IN },
];

export const MAX_BODY_COLLISION_ALTITUDE_METERS = 150;

/** JSBSim c172p.xml ground contacts, in structural inches (aft, right, up).
 * Wheel locations are uncompressed; suspension compression is handled by JSBSim.
 * The test checks these against the aircraft XML used by the simulation.
 */
export const C172_GROUND_CONTACTS = [
  { name: "NOSE", kind: "wheel", xIn: -6.8, yIn: 0, zIn: -19.5 },
  { name: "LEFT_MAIN", kind: "wheel", xIn: 58.2, yIn: -43, zIn: -15.5 },
  { name: "RIGHT_MAIN", kind: "wheel", xIn: 58.2, yIn: 43, zIn: -15.5 },
  { name: "NOSE_SKID", kind: "structure", xIn: -37.7, yIn: 0, zIn: 26.6 },
  { name: "TAIL_SKID", kind: "structure", xIn: 175.2, yIn: 0, zIn: 20.6 },
  { name: "LEFT_TIP", kind: "structure", xIn: 53.7, yIn: -216.5, zIn: 68.5 },
  { name: "RIGHT_TIP", kind: "structure", xIn: 53.7, yIn: 216.5, zIn: 68.5 },
  { name: "BELLY", kind: "structure", xIn: 42, yIn: 0, zIn: 11.9 },
  { name: "TAIL", kind: "structure", xIn: 255.9, yIn: 0, zIn: 35.5 },
] as const;

/** Metres from the CG: left, up, forward, the renderer's aircraft axes. */
export interface BodyOffset { left: number; up: number; forward: number }

/** Metres from the CG in the local level frame: north, east, down. */
export interface LocalOffset { north: number; east: number; down: number }

/** Convert structural inches to renderer/probe metres using the current CG.
 * Pass `out` to reuse one object on every physics step. */
export function groundContactBodyPosition(contact: StructuralPosition, cg: StructuralPosition,
  out: BodyOffset = { left: 0, up: 0, forward: 0 }): BodyOffset {
  out.left = (cg.yIn - contact.yIn) * 0.0254;
  out.up = (contact.zIn - cg.zIn) * 0.0254;
  out.forward = (cg.xIn - contact.xIn) * 0.0254;
  return out;
}

/**
 * JSBSim's body-to-local rotation for its Euler angles, row-major into `out`:
 * (north, east, down) = M (forward, right, down).
 */
export function bodyToLocalRotation(rollRad: number, pitchRad: number, headingRad: number,
  out: Float64Array = new Float64Array(9)): Float64Array {
  const sr = Math.sin(rollRad), cr = Math.cos(rollRad);
  const sp = Math.sin(pitchRad), cp = Math.cos(pitchRad);
  const sh = Math.sin(headingRad), ch = Math.cos(headingRad);
  out[0] = cp * ch; out[1] = sr * sp * ch - cr * sh; out[2] = cr * sp * ch + sr * sh;
  out[3] = cp * sh; out[4] = sr * sp * sh + cr * ch; out[5] = cr * sp * sh - sr * ch;
  out[6] = -sp; out[7] = sr * cp; out[8] = cr * cp;
  return out;
}

/** Where a body offset lies in the local level frame, for a rotation from bodyToLocalRotation. */
export function bodyOffsetLocal(offset: BodyOffset, rotation: ArrayLike<number>,
  out: LocalOffset = { north: 0, east: 0, down: 0 }): LocalOffset {
  const forward = offset.forward, right = -offset.left, down = -offset.up;
  out.north = rotation[0] * forward + rotation[1] * right + rotation[2] * down;
  out.east = rotation[3] * forward + rotation[4] * right + rotation[5] * down;
  out.down = rotation[6] * forward + rotation[7] * right + rotation[8] * down;
  return out;
}

const WGS84_A = 6378137;
const WGS84_E2 = 6.69437999014e-3;

/**
 * Metres per degree of latitude and of longitude at a geodetic latitude, on
 * WGS84. Exact for offsets of a few metres, such as a contact from the CG.
 */
export function metresPerDegree(latDeg: number, out: { lat: number; lon: number } = { lat: 0, lon: 0 }): { lat: number; lon: number } {
  const lat = latDeg * Math.PI / 180;
  const sine = Math.sin(lat);
  const w = 1 - WGS84_E2 * sine * sine;
  const meridian = WGS84_A * (1 - WGS84_E2) / (w * Math.sqrt(w));
  const normal = WGS84_A / Math.sqrt(w);
  out.lat = meridian * Math.PI / 180;
  out.lon = normal * Math.cos(lat) * Math.PI / 180;
  return out;
}
