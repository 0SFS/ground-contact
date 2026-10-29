import type { JSBSimSdk } from "@felipegalind0/jsbsim";

/** One JSBSim contact unit, BOGEY or STRUCTURE, in the native unit numbering. */
export interface GroundContactUnit {
  /** Property prefix with its trailing slash, such as `gear/unit[1]/` or `contact/unit[4]/`. */
  prefix: string;
  /** A retractable BOGEY: it supports the aircraft only while the gear is down. */
  retractable: boolean;
}

/** Native BOGEY support is active only above this gear position for retractable units. */
export const GEAR_DOWN_POSITION = 0.99;

// Cache only property availability, never native nodes or geometry. Positions
// and CG stay live so fuel/payload changes and supported geometry edits apply.
const contactCapabilities = new WeakMap<JSBSimSdk, {
  count: number;
  contacts: readonly GroundContactUnit[];
}>();

/**
 * The loaded model's contact units. Throws when the model does not report its
 * contacts completely; an empty list is a model without any.
 */
export function groundContactUnits(sdk: JSBSimSdk): readonly GroundContactUnit[] {
  const count = sdk.getPropertyValue("gear/num-units");
  if (!Number.isInteger(count) || count < 0) throw new Error("JSBSim contact count is unavailable.");
  if (count === 0) return [];
  let capabilities = contactCapabilities.get(sdk);
  if (!capabilities || capabilities.count !== count) {
    // BOGEY units are under gear/unit; STRUCTURE units are under contact/unit.
    // Both use the same native unit numbering, including an unindexed unit 0.
    const catalog = sdk.queryPropertyCatalog("/unit");
    const contacts: GroundContactUnit[] = [];
    for (const match of catalog.matchAll(/^((?:gear|contact)\/unit(?:\[\d+\])?)\/x-position\s/gm)) {
      const prefix = match[1] + "/";
      contacts.push({ prefix, retractable: catalog.includes(prefix + "pos-norm ") });
    }
    if (contacts.length !== count) throw new Error("JSBSim contact geometry is incomplete.");
    capabilities = { count, contacts };
    contactCapabilities.set(sdk, capabilities);
  }
  return capabilities.contacts;
}

/** Lowest active contact in the loaded FDM below the current fuel/payload CG.
 * Placement clearance is deliberately conservative and must not decide impact:
 * adding wing extent to wheel height can stop an intact aircraft above ground.
 */
export function groundContactClearanceMeters(sdk: JSBSimSdk, roll: number, pitch: number): number {
  const contacts = groundContactUnits(sdk);
  if (contacts.length === 0) return 0;
  const gearPosition = sdk.getPropertyValue("gear/gear-pos-norm");
  const cgX = sdk.getPropertyValue("inertia/cg-x-in");
  const cgY = sdk.getPropertyValue("inertia/cg-y-in");
  const cgZ = sdk.getPropertyValue("inertia/cg-z-in");
  const downX = -Math.sin(pitch);
  const downY = Math.sin(roll) * Math.cos(pitch);
  const downZ = Math.cos(roll) * Math.cos(pitch);
  let lowest = -Infinity;
  for (const { prefix, retractable } of contacts) {
    // Both bundled aircraft use the common FCS gear position. Native BOGEY
    // support is active only above 0.99 for retractable units; fixed contacts
    // still participate regardless of the pilot's gear lever.
    if (retractable && gearPosition <= GEAR_DOWN_POSITION) continue;
    const xIn = sdk.getPropertyValue(prefix + "x-position");
    const yIn = sdk.getPropertyValue(prefix + "y-position");
    const zIn = sdk.getPropertyValue(prefix + "z-position");
    // Structural coordinates point aft/right/up; body coordinates point
    // forward/right/down. JSBSim's propagated location is its current CG.
    lowest = Math.max(lowest, downX * (cgX - xIn) + downY * (yIn - cgY) + downZ * (cgZ - zIn));
  }
  return lowest === -Infinity ? 0 : lowest * 0.0254;
}
