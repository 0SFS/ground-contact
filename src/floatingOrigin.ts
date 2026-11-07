import { Quaternion, TransformNode, Vector3 } from "@babylonjs/core";
import { DEG_TO_RAD, geodeticToEcef, type EcefCoord } from "foss-earth/cameraMath";
import { buildWorldShiftFrame, buildWorldShiftMatrix } from "./enuFrame";
import { flightAttitudeToQuaternion } from "./ecefBridge";
import type { FlightState } from "./flightState";

/** Where the aircraft is and how it is turned: the part of a flight state the origin follows. */
export type FloatingOriginPose = Pick<FlightState, "latDeg" | "lonDeg" | "altMeters" | "rollRad" | "pitchRad" | "headingRad">;

/**
 * Keeps the aircraft at the scene's origin and moves the world around it, so
 * the renderer's single-precision numbers stay small wherever on Earth it is.
 */
export interface FloatingOriginHandle {
  worldShift: TransformNode;
  aircraftRoot: TransformNode;
  apply(state: FloatingOriginPose): void;
  /** Where the origin, and so the aircraft, is: ECEF metres of the last applied state; null before one. */
  getOriginEcef(): EcefCoord | null;
  /** Cached double-precision ECEF -> east/up/south frame for flight trails. */
  getWorldFromEcef(): { readonly m: ArrayLike<number> } | null;
  dispose(): void;
}

export function createFloatingOrigin(scene: import("@babylonjs/core").Scene, worldContent: TransformNode): FloatingOriginHandle {
  const worldShift = new TransformNode("world-shift", scene);
  worldContent.parent = worldShift;
  worldContent.position = Vector3.Zero();
  worldContent.rotationQuaternion = Quaternion.Identity();

  const aircraftRoot = new TransformNode("aircraft-root", scene);
  let originEcef: EcefCoord | null = null;
  let worldFromEcef: ReturnType<typeof buildWorldShiftFrame> | null = null;
  let appliedLatRad = 0, appliedLonRad = 0, appliedAltMeters = 0;

  return {
    worldShift,
    aircraftRoot,
    apply(state: FloatingOriginPose): void {
      const latRad = state.latDeg * DEG_TO_RAD;
      const lonRad = state.lonDeg * DEG_TO_RAD;
      originEcef = geodeticToEcef(latRad, lonRad, state.altMeters);
      appliedLatRad = latRad; appliedLonRad = lonRad; appliedAltMeters = state.altMeters;
      worldFromEcef = null;
      const shift = buildWorldShiftMatrix(latRad, lonRad, state.altMeters);

      const scale = new Vector3();
      const rotation = new Quaternion();
      const position = new Vector3();
      shift.decompose(scale, rotation, position);
      worldShift.scaling.copyFrom(scale);
      worldShift.rotationQuaternion = rotation;
      worldShift.position = position;

      aircraftRoot.rotationQuaternion = flightAttitudeToQuaternion(
        state.rollRad,
        state.pitchRad,
        state.headingRad,
      );
      aircraftRoot.position = Vector3.Zero();
    },
    getOriginEcef: () => originEcef,
    getWorldFromEcef: () => {
      if (!originEcef) return null;
      return worldFromEcef ??= buildWorldShiftFrame(appliedLatRad, appliedLonRad, appliedAltMeters, originEcef);
    },
    dispose(): void {
      worldShift.position.set(0, 0, 0);
      worldShift.rotationQuaternion = Quaternion.Identity();
      worldShift.scaling.set(1, 1, 1);
      // The streamed map belongs to the renderer, which may outlive this flight.
      worldContent.parent = null;
      aircraftRoot.dispose();
      worldShift.dispose();
      worldFromEcef = null;
      originEcef = null;
    },
  };
}
