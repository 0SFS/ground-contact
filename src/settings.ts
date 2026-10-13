import type { ParameterSpec } from "foss-earth/settings";
import {
  catalogueDefaults,
  catalogueSpec,
  catalogueStore,
  type CatalogueParameters,
  type CatalogueRegistry,
  type CatalogueStore,
  type CatalogueValues,
} from "@0sfs/engines/parameterCatalogue";
import {
  DEFAULT_GROUND_INTERACTION_SETTINGS,
  GROUND_CHOICE_LABELS,
  GROUND_CHOICES,
  GROUND_FIELD_LABELS,
  GROUND_LOCKABLE_KEYS,
  type GroundLockableKey,
} from "./groundInteractionSettings";

/**
 * Ground interaction's flight parameters: its part of the flight's settings, with
 * their homes, section titles and defaults. The host composes the parts into
 * its catalogue in this order and registers them. Spec:
 * docs/proposals/flight-settings.md.
 */

const GROUND = { tab: "aircraft", section: "ground" } as const;

const all = <T extends { tab: string; section: string }>(home: T) => ({ ...home, level: "all" as const });
const within = (min: number, max: number) => () => ({ min, max });

export const GROUND_CONTACT_PARAMETERS = [
  // Aircraft → Ground handling: requests; what runs is resolved against what
  // is implemented, and reported beside each choice.
  {
    id: "osfs.ground.selection",
    label: "Ground interaction selection",
    description: "Auto substitutes a cheaper implemented choice when a request cannot run, never during a landing; Manual leaves it inactive.",
    unit: "none",
    kind: "choice",
    choices: [
      { id: "auto", label: "Auto within my choices" },
      { id: "manual", label: "Manual" },
    ],
    default: DEFAULT_GROUND_INTERACTION_SETTINGS.selection,
    defaultReason: "Nothing runs that the pilot did not choose.",
    home: all(GROUND),
    appliesLive: true,
    source: "ground-contact/src/groundInteractionSettings.ts",
  },
  groundChoice("rotation", "How the wheels spin up and roll; applies when paused or reset."),
  groundChoice("forceModel", "Which model computes the forces between wheels and ground; applies when paused or reset."),
  groundChoice("contactModel", "Where each wheel meets the ground; applies when paused or reset."),
  groundChoice("backend", "Where ground interaction is computed; applies when paused or reset."),
  groundChoice("tireAudio", "Which tyre sounds play."),
  groundChoice("haptics", "Which ground events vibrate a gamepad or phone."),
  groundChoice("wheelVisuals", "Whether the aircraft's wheel meshes turn."),
  groundLevel("tireAudioVolume", "Tire audio volume", "Scales the tyre sounds.", DEFAULT_GROUND_INTERACTION_SETTINGS.tireAudioVolume),
  groundLevel("hapticStrength", "Haptic strength", "Scales every ground vibration.", DEFAULT_GROUND_INTERACTION_SETTINGS.hapticStrength),
  ...GROUND_LOCKABLE_KEYS.map(groundLock),
] as const satisfies readonly ParameterSpec[];

/** The effective value of each of ground interaction's parameters, by id. */
export type GroundContactParameterValues = CatalogueValues<typeof GROUND_CONTACT_PARAMETERS>;

export type GroundContactParameterId = keyof GroundContactParameterValues & string;

/** What ground interaction code reads its parameters through: the registry, or its defaults in a test. */
export type GroundContactParameters = CatalogueParameters<GroundContactParameterValues>;

/** Reads, writes and watches ground interaction's parameters, typed by id. */
export type GroundContactParameterStore = CatalogueStore<GroundContactParameterValues>;

/** The registry, typed for ground interaction's parameters. */
export function groundContactParameterStore(registry: CatalogueRegistry): GroundContactParameterStore {
  return catalogueStore(registry);
}

/** The spec of one of ground interaction's parameters. */
export const groundContactParameterSpec = catalogueSpec(GROUND_CONTACT_PARAMETERS);

/**
 * Ground interaction's parameter defaults in memory, for code that runs without a
 * registry, such as a unit test. Overrides stand in for values the test sets.
 */
export function groundContactParameterDefaults(overrides: Partial<GroundContactParameterValues> = {}): GroundContactParameterStore {
  return catalogueDefaults(groundContactParameterSpec, overrides);
}

function groundChoice<Key extends GroundLockableKey>(key: Key, description: string) {
  const labels = GROUND_CHOICE_LABELS[key] as Record<string, string>;
  return {
    id: `osfs.ground.${key}` as const,
    label: GROUND_FIELD_LABELS[key],
    description,
    unit: "none" as const,
    kind: "choice" as const,
    choices: (GROUND_CHOICES[key] as readonly string[]).map(id => ({ id, label: labels[id] })),
    default: DEFAULT_GROUND_INTERACTION_SETTINGS[key] as string,
    defaultReason: "The Minimal profile: the ground as JSBSim models it, with nothing added.",
    home: all(GROUND),
    appliesLive: true,
    source: "ground-contact/src/groundInteractionSettings.ts",
  };
}

function groundLevel<Field extends string>(field: Field, label: string, description: string, value: number) {
  return {
    id: `osfs.ground.${field}` as const,
    label,
    description,
    unit: "fraction" as const,
    kind: "number" as const,
    step: 0.05,
    bounds: within(0, 1),
    default: value,
    defaultReason: "The original level.",
    home: all(GROUND),
    appliesLive: true,
    source: "src/flight/createFlightSimApp.ts",
  };
}

function groundLock<Key extends GroundLockableKey>(key: Key) {
  return {
    id: `osfs.ground.lock.${key}` as const,
    label: `Lock ${GROUND_FIELD_LABELS[key].toLowerCase()}`,
    description: "Auto cannot substitute another choice for a locked one.",
    unit: "none" as const,
    kind: "boolean" as const,
    default: false,
    defaultReason: "Nothing locked: Auto may substitute any choice that cannot run.",
    home: all(GROUND),
    appliesLive: true,
    source: "ground-contact/src/groundInteractionSettings.ts",
  };
}
