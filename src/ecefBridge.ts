import { Quaternion } from "@babylonjs/core";
import type { JSBSimSdk } from "@felipegalind0/jsbsim";
import { degreesToRadians, feetToMeters, type FlightState } from "./flightState";
import { createPropertyReadBatch } from "./propertyReadBatch";

const stateProperties = [
  "position/lat-geod-deg", "position/long-gc-deg", "position/h-sl-ft",
  "attitude/phi-deg", "attitude/theta-deg", "attitude/psi-deg",
  "velocities/vc-kts", "velocities/v-north-fps", "velocities/v-east-fps",
  "velocities/v-down-fps", "fcs/throttle-cmd-norm",
] as const;
const stateReaders = new WeakMap<JSBSimSdk, ReturnType<typeof createPropertyReadBatch>>();

/** Release before the owning SDK; subsequent reads resolve a fresh model view. */
export function releaseFlightStateReads(sdk: JSBSimSdk): void {
  const reader = stateReaders.get(sdk);
  stateReaders.delete(sdk);
  reader?.dispose();
}

export function readFlightState(sdk: JSBSimSdk): FlightState {
  let reader = stateReaders.get(sdk);
  if (!reader) { reader = createPropertyReadBatch(sdk, stateProperties); stateReaders.set(sdk, reader); }
  const values = reader.read();
  return {
    latDeg: values[0], lonDeg: values[1], altMeters: feetToMeters(values[2]),
    rollRad: degreesToRadians(values[3]), pitchRad: degreesToRadians(values[4]), headingRad: degreesToRadians(values[5]),
    airspeedKts: values[6], northVelocityFps: values[7], eastVelocityFps: values[8],
    verticalSpeedFps: -values[9], throttleNorm: values[10],
  };
}

/**
 * Convert JSBSim NED body attitude (phi, theta, psi) to a Babylon quaternion.
 * The aircraft model uses +X left, +Y up, and +Z nose-forward.
 * World axes are east/up/south, so north is -Z and headings turn toward +X.
 */
export function flightAttitudeToQuaternion(rollRad: number, pitchRad: number, headingRad: number): Quaternion {
  return Quaternion.RotationYawPitchRoll(Math.PI - headingRad, -pitchRad, rollRad);
}
