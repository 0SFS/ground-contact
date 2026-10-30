import { describe, expect, it, vi } from "vitest";
import type { SurfaceHit, SurfaceQuery } from "foss-earth/runtime";
import { createTerrainContact } from "../src/terrainContact";

/**
 * `contactMeters` gives the model one contact that far below its centre of
 * gravity, as a drone's legs are; without it the model reports no contacts and
 * its stance stands in for them.
 */
function fixture(altitude = 50, options: { contactMeters?: number; staticMeters?: number } = {}) {
  const values: Record<string, number> = {
    "position/lat-geod-deg": 34, "position/long-gc-deg": -118,
    "position/h-sl-ft": altitude / 0.3048,
    "position/terrain-elevation-asl-ft": 0,
  };
  const contacts = options.contactMeters !== undefined;
  if (contacts) {
    values["gear/num-units"] = 1;
    values["contact/unit/z-position"] = -options.contactMeters! / 0.0254;
  }
  const sdk = {
    getPropertyValue: vi.fn((name: string) => values[name] ?? 0),
    setPropertyValue: vi.fn((name: string, value: number) => { values[name] = value; }),
    queryPropertyCatalog: () => contacts ? "contact/unit/x-position \ncontact/unit/y-position \ncontact/unit/z-position \n" : "",
    resetToInitialConditions: vi.fn(),
    runIc: vi.fn(() => {
      values["position/h-sl-ft"] = values["ic/h-sl-ft"];
      values["position/terrain-elevation-asl-ft"] = values["ic/terrain-elevation-ft"];
      return true;
    }),
  };
  let height: number | null = 100;
  let revision = 1;
  const surface: SurfaceQuery = {
    raycast: () => null,
    sample: () => height === null ? null : ({ heightMeters: height, revision } as SurfaceHit),
  };
  const contact = createTerrainContact(sdk as never, surface,
    { staticMeters: options.staticMeters ?? 1, staticPitchRad: 0, pitchArmMeters: 0, rollArmMeters: 0 });
  return {
    sdk, values, contact, surface,
    setHeight(next: number | null) { height = next; revision++; },
    floor: () => values["position/terrain-elevation-asl-ft"] * 0.3048,
    altitude: () => values["position/h-sl-ft"] * 0.3048,
  };
}

describe("consent for terrain relocation", () => {
  it("proposes placement without changing altitude or integrating against a penetrating floor", () => {
    const { sdk, values, contact } = fixture();
    expect(contact.update()).toBe(false);
    expect(contact.getBlockReason()).toBe("relocation");
    expect(contact.getRelocationProposal()).toMatchObject({ reason: "placement", currentAltitudeMeters: 50, altitudeMeters: 101 });
    expect(values["position/h-sl-ft"] * 0.3048).toBeCloseTo(50);
    expect(values["position/terrain-elevation-asl-ft"] * 0.3048).toBeLessThan(-9000);
    expect(sdk.runIc).not.toHaveBeenCalled();
  });

  it("only accepts the current proposal after confirmation", () => {
    const { values, contact } = fixture();
    contact.update();
    const proposal = contact.getRelocationProposal()!;
    expect(contact.acceptRelocation(proposal.id + 1)).toBe(false);
    expect(contact.acceptRelocation(proposal.id)).toBe(true);
    expect(values["position/h-sl-ft"] * 0.3048).toBeCloseTo(101);
    expect(contact.getRelocationProposal()).toBeNull();
    expect(contact.update()).toBe(true);
  });

  it("keeps a declined pose flyable through missing terrain and tile revisions without asking again", () => {
    const { contact, setHeight, values } = fixture();
    contact.update();
    contact.declineRelocation(contact.getRelocationProposal()!.id);
    expect(contact.isTerrainIgnored()).toBe(true);
    for (const height of [110, null, 90, 100]) {
      setHeight(height);
      expect(contact.update()).toBe(true);
      expect(contact.getRelocationProposal()).toBeNull();
      expect(values["position/h-sl-ft"] * 0.3048).toBeCloseTo(50);
      expect(values["position/terrain-elevation-asl-ft"] * 0.3048).toBeLessThan(-9000);
    }
    setHeight(40);
    expect(contact.update()).toBe(true);
    expect(contact.isTerrainIgnored()).toBe(false);
    expect(values["position/terrain-elevation-asl-ft"] * 0.3048).toBeCloseTo(40);
  });

  it("can waive a completely missing map and requires the hull footprint to be outside before ending", () => {
    const { sdk, values, surface, setHeight } = fixture(110);
    const contact = createTerrainContact(sdk as never, surface,
      { staticMeters: 1, staticPitchRad: 0, pitchArmMeters: 0, rollArmMeters: 0 },
      { bodyProbes: [{ name: "belly", xIn: 0, yIn: 0, zIn: -500, insetIn: 0 }] });
    setHeight(null);
    contact.ignoreTerrainOverlap();
    expect(contact.update()).toBe(true);
    setHeight(100);
    expect(contact.update()).toBe(true);
    // CG is above ground but this probe remains underneath it.
    expect(contact.isTerrainIgnored()).toBe(true);
    values["position/h-sl-ft"] = 115 / 0.3048;
    expect(contact.update()).toBe(true);
    expect(contact.isTerrainIgnored()).toBe(false);
  });

  it("asks about a refinement under a standing aircraft without moving, dropping or freezing it", () => {
    const { sdk, contact, setHeight, floor, altitude } = fixture(101);
    expect(contact.update()).toBe(true);
    setHeight(110);
    expect(contact.update()).toBe(true);
    expect(contact.getBlockReason()).toBeNull();
    const proposal = contact.getRelocationProposal();
    expect(proposal).toMatchObject({ reason: "surface refinement", deltaMeters: 10 });
    expect(altitude()).toBeCloseTo(101);
    expect(floor()).toBeCloseTo(100);
    expect(sdk.runIc).not.toHaveBeenCalled();
    // Physics carries on while the pilot reads the question, on the same floor.
    setHeight(111);
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toBe(proposal);
    expect(floor()).toBeCloseTo(100);
  });

  it("keeps a standing aircraft on its floor after Resume until the surface comes back down to it", () => {
    const { contact, setHeight, floor, altitude } = fixture(101);
    contact.update();
    setHeight(100.03);
    contact.update();
    contact.declineRelocation(contact.getRelocationProposal()!.id);
    expect(contact.isTerrainIgnored()).toBe(true);
    expect(floor()).toBeCloseTo(100);
    for (const height of [100.05, null, 100.5, 100.02]) {
      setHeight(height);
      expect(contact.update()).toBe(true);
      expect(contact.getRelocationProposal()).toBeNull();
      expect(floor()).toBeCloseTo(100);
      expect(altitude()).toBeCloseTo(101);
    }
    setHeight(100);
    expect(contact.update()).toBe(true);
    expect(contact.isTerrainIgnored()).toBe(false);
    expect(contact.getRelocationProposal()).toBeNull();
    expect(floor()).toBeCloseTo(100);
  });

  it("lets a standing aircraft whose pilot declined a lower ground settle onto it", () => {
    const { contact, setHeight, floor, altitude } = fixture(101);
    contact.update();
    setHeight(99.5);
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toMatchObject({ reason: "surface refinement", deltaMeters: -0.5 });
    expect(floor()).toBeCloseTo(100);
    contact.declineRelocation(contact.getRelocationProposal()!.id);
    expect(contact.update()).toBe(true);
    // Not moved: the ground under it is lower now, and physics settles it there.
    expect(contact.isTerrainIgnored()).toBe(false);
    expect(floor()).toBeCloseTo(99.5);
    expect(altitude()).toBeCloseTo(101);
  });

  it("does not ask about a finer map under a drone hovering a few centimetres up to land", () => {
    const { values, contact, setHeight, floor } = fixture(100.15, { contactMeters: 0.023, staticMeters: 0.04 });
    values["velocities/v-down-fps"] = 1;
    expect(contact.update()).toBe(true);
    expect(floor()).toBeCloseTo(100);
    setHeight(100.04);
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toBeNull();
    expect(floor()).toBeCloseTo(100.04);
  });

  it("asks once a drone's legs are on the ground, and keeps it standing there meanwhile", () => {
    const { values, contact, setHeight, floor } = fixture(100.013, { contactMeters: 0.023, staticMeters: 0.04 });
    values["position/terrain-elevation-asl-ft"] = 100 / 0.3048;
    values["velocities/v-down-fps"] = 1;
    // Resting 1 cm into its legs' springs, below its nominal 4 cm stance: not misplaced.
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toBeNull();
    setHeight(100.03);
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toMatchObject({ reason: "surface refinement" });
    expect(contact.getRelocationProposal()!.deltaMeters).toBeCloseTo(0.03);
    expect(floor()).toBeCloseTo(100);
  });

  it("gives a held flying aircraft its floor back when the pilot stays, so it can still land", () => {
    const { contact, setHeight, floor } = fixture(101.5);
    contact.update();
    setHeight(102);
    expect(contact.update()).toBe(false);
    expect(contact.getRelocationProposal()).toMatchObject({ reason: "surface refinement", deltaMeters: 2 });
    expect(floor()).toBeLessThan(-9000);
    contact.declineRelocation(contact.getRelocationProposal()!.id);
    expect(contact.isTerrainIgnored()).toBe(true);
    expect(floor()).toBeCloseTo(100);
    expect(contact.update()).toBe(true);
    expect(floor()).toBeCloseTo(100);
  });

  it("accepts a refinement by moving only the height, so the gear keeps its compression", () => {
    const { sdk, values, contact, setHeight } = fixture(101);
    contact.update();
    setHeight(110);
    contact.update();
    expect(contact.acceptRelocation(contact.getRelocationProposal()!.id)).toBe(true);
    expect(values["position/h-sl-ft"] * 0.3048).toBeCloseTo(111);
    expect(values["position/terrain-elevation-asl-ft"] * 0.3048).toBeCloseTo(110);
    expect(sdk.resetToInitialConditions).not.toHaveBeenCalled();
    expect(sdk.runIc).not.toHaveBeenCalled();
    expect(contact.update()).toBe(true);
    expect(contact.getRelocationProposal()).toBeNull();
  });
});
