import { ChoiceTrack, HelpTip, Resettable } from "foss-earth/shell";
import {
  GROUND_CHOICE_LABELS, GROUND_CHOICES, GROUND_FIELD_LABELS, groundChoiceUnavailable,
  type GroundCapabilities, type GroundInteractionSettingsV1, type GroundLockableKey, type GroundResolution,
} from "./groundInteractionSettings";

export interface GroundInteractionPanelState {
  settings: GroundInteractionSettingsV1;
  resolution: GroundResolution;
  capabilities: GroundCapabilities;
  readOnlyReason: string | null;
  /** Result of the latest action. */
  message: { text: string; error: boolean } | null;
  /** Session-only Debug A/B override, described for the pilot. */
  experimentOverride: string | null;
  hapticDevices: { gamepad: string; phone: string };
}

export type GroundInteractionAction =
  | { type: "set"; patch: Partial<Omit<GroundInteractionSettingsV1, "version" | "locked">> }
  | { type: "lock"; key: GroundLockableKey; locked: boolean }
  | { type: "keep-experiment" }
  | { type: "discard-experiment" };

const ROW_LABELS = GROUND_FIELD_LABELS;

/** Compact segment text for rows drawn as a track; the full name stays the accessible name. */
const SHORT_LABELS: { [K in GroundLockableKey]?: Partial<Record<GroundInteractionSettingsV1[K], string>> } = {
  rotation: { instant: "Instant", inertia: "Inertia" },
  forceModel: { "jsbsim": "JSBSim", "coupled-rigid": "Rigid", "combined-slip": "Slip", "compliant-soil": "Soil" },
  contactModel: { "shared": "Shared", "per-wheel-point": "Per wheel", "footprint": "Footprint", "swept": "Swept" },
  haptics: { landing: "Landing" },
  wheelVisuals: { asset: "Asset" },
};

/** Rows with up to four choices are a track; longer lists stay a dropdown as wide as its text. */
const TRACK_LIMIT = 4;

/** Structural unavailability only; dependencies such as audio → inertia stay selectable. */
function unimplemented<K extends GroundLockableKey>(key: K, value: GroundInteractionSettingsV1[K], capabilities: GroundCapabilities) {
  return groundChoiceUnavailable(key, value, { rotation: "inertia" }, capabilities);
}

function ChoiceRow<K extends GroundLockableKey>({ field, state, onAction }: {
  field: K; state: GroundInteractionPanelState; onAction(action: GroundInteractionAction): void;
}) {
  const entry = state.resolution.entries[field];
  const labels = GROUND_CHOICE_LABELS[field] as Record<string, string>;
  const short = (SHORT_LABELS[field] ?? {}) as Record<string, string | undefined>;
  const values = GROUND_CHOICES[field] as readonly string[];
  const choices = values.map(value => ({ id: value, label: labels[value], shortLabel: short[value], unavailable: unimplemented(field, value as never, state.capabilities) }));
  const set = (value: string) => onAction({ type: "set", patch: { [field]: value } });
  // Why a choice cannot run, behind ? rather than inside the option text.
  const unavailable = choices.filter(choice => choice.unavailable);
  const help = unavailable.length ? <ul>{unavailable.map(choice => <li key={choice.id}>{choice.label}: {choice.unavailable}</li>)}</ul> : undefined;
  // "Wheel response [Off / Instant / Inertia] ?  Lock [switch]"
  return <div className="flight-panel__ground-row">
    {values.length <= TRACK_LIMIT
      ? <ChoiceTrack heading={ROW_LABELS[field]} value={state.settings[field]} parameter={`osfs.ground.${field}`} onChange={set} choices={choices} help={help}
        helpLabel={`Explain ${ROW_LABELS[field]} choices`} />
      : <>
        <label className="flight-panel__field">
          <span>{ROW_LABELS[field]}</span>
          <Resettable parameter={`osfs.ground.${field}`} name={ROW_LABELS[field]}>
            <select aria-label={ROW_LABELS[field]} value={state.settings[field]} onChange={event => set(event.target.value)}>
              {choices.map(choice => <option key={choice.id} value={choice.id} disabled={choice.unavailable !== null}>{choice.label}</option>)}
            </select>
          </Resettable>
        </label>
        {help && <HelpTip label={`Explain ${ROW_LABELS[field]} choices`}>{help}</HelpTip>}
      </>}
    <label className="flight-panel__switch flight-panel__ground-lock" title="Auto cannot change a locked choice">
      <span>Lock</span>
      <Resettable parameter={`osfs.ground.lock.${field}`} name={`Lock ${ROW_LABELS[field]}`}>
        <span className="foss-earth-choice foss-earth-switch">
          <input type="checkbox" aria-label={`Lock ${ROW_LABELS[field]}`} checked={state.settings.locked[field] === true}
            onChange={event => onAction({ type: "lock", key: field, locked: event.target.checked })} />
          <span className="foss-earth-switch__knob" aria-hidden="true" />
        </span>
      </Resettable>
    </label>
    {(entry.reason || entry.requested !== entry.active) && <p className={`flight-panel__hint${entry.pending ? "" : " is-error"}`}
      role="status" aria-label={`${ROW_LABELS[field]} status`}>
      Requested: {labels[entry.requested]} · Active: {labels[entry.active]}{entry.reason ? ` · ${entry.reason}` : ""}
    </p>}
  </div>;
}

export function GroundInteractionSettingsPanel({ state, onAction }: {
  state: GroundInteractionPanelState; onAction(action: GroundInteractionAction): void;
}) {
  const settings = state.settings;
  return (
    <fieldset className="flight-panel__fieldset" aria-label="Ground interaction">
      <legend>Ground interaction</legend>
      {/* "Selection [Auto / Manual] ?" */}
      <ChoiceTrack heading="Selection" value={settings.selection} parameter="osfs.ground.selection"
        onChange={selection => onAction({ type: "set", patch: { selection: selection as GroundInteractionSettingsV1["selection"] } })}
        choices={[{ id: "auto", label: "Auto within my choices", shortLabel: "Auto" }, { id: "manual", label: "Manual" }]}
        helpLabel="Explain ground interaction selection"
        help={<>
          <p>
            Auto only substitutes a cheaper implemented choice when a request cannot run; it never changes forces,
            contact or the wheel model during a landing. A locked choice is never substituted. Wheel response and
            force/contact changes apply when paused or reset.
          </p>
          <p>Named sets of these choices, such as Ground: Minimal and Ground: Landing feedback, are presets, in Settings → Presets.</p>
          <p>
            Performance: not profiled on this device, so no live cost is shown. Recorded terminal benchmarks
            cover one Apple M5 machine and are not a hardware policy.
          </p>
        </>} />
      <ChoiceRow field="rotation" state={state} onAction={onAction} />
      <ChoiceRow field="forceModel" state={state} onAction={onAction} />
      <ChoiceRow field="contactModel" state={state} onAction={onAction} />
      <ChoiceRow field="tireAudio" state={state} onAction={onAction} />
      <label className="flight-panel__field">
        <span>Tire audio volume<output>{Math.round(settings.tireAudioVolume * 100)}%</output></span>
        <Resettable parameter="osfs.ground.tireAudioVolume" name="Tire audio volume">
          <input type="range" aria-label="Tire audio volume" min={0} max={1} step={0.05} value={settings.tireAudioVolume}
            onChange={event => onAction({ type: "set", patch: { tireAudioVolume: Number(event.target.value) } })} />
        </Resettable>
      </label>
      <ChoiceRow field="haptics" state={state} onAction={onAction} />
      <label className="flight-panel__field">
        <span>Haptic strength<output>{Math.round(settings.hapticStrength * 100)}%</output></span>
        <Resettable parameter="osfs.ground.hapticStrength" name="Haptic strength">
          <input type="range" aria-label="Haptic strength" min={0} max={1} step={0.05} value={settings.hapticStrength}
            onChange={event => onAction({ type: "set", patch: { hapticStrength: Number(event.target.value) } })} />
        </Resettable>
      </label>
      <p className="flight-panel__hint" aria-label="Haptic devices">
        Gamepad: {state.hapticDevices.gamepad} · Phone: {state.hapticDevices.phone}
      </p>
      <ChoiceRow field="wheelVisuals" state={state} onAction={onAction} />
      <ChoiceRow field="backend" state={state} onAction={onAction} />
      {/* "Debug experiment, this session: Instant rolling. [Keep] [Discard]" */}
      {state.experimentOverride && <div className="flight-panel__row" role="status" aria-label="Wheel experiment override">
        <p className="flight-panel__hint">Debug experiment, this session: {state.experimentOverride}.</p>
        <button className="flight-panel__command" type="button" aria-label="Keep experiment choices"
          onClick={() => onAction({ type: "keep-experiment" })}>
          Keep
        </button>
        <button className="flight-panel__command" type="button" aria-label="Discard: return to saved settings"
          onClick={() => onAction({ type: "discard-experiment" })}>
          Discard
        </button>
      </div>}
      {state.message && <p className={`flight-panel__hint${state.message.error ? " is-error" : ""}`} role="status">{state.message.text}</p>}
      {state.readOnlyReason && <p className="flight-panel__hint is-error" role="status">{state.readOnlyReason}</p>}
    </fieldset>
  );
}
