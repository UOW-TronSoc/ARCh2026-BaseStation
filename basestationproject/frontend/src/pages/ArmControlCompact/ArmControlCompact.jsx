import React, { useState, useEffect, useRef } from "react";
import { postCmd, isTimeoutError } from "utils/api";

import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import barStyles from "components/DrivetrainCard/DrivetrainCard.module.css";
import enableSwitchStyles from "components/EnableSwitch/EnableSwitch.module.css";
import { getArmApiBase, getCommandUrl } from "../../config";
import pageStyles from "./ArmControlCompact.module.css";

const DISPLAY_THROTTLE_MS = 50;
/** Match Dashboard drivetrain scale (postCmd → :8080/command). */
const MAX_DRIVE_TWIST = 20;

function toBarPercent(value) {
  const p = Math.round(Number(value) * 100);
  return Math.max(-100, Math.min(100, p));
}

function VerticalBarMeter({ label, value }) {
  const clamped = toBarPercent(value);
  const fillH = Math.abs(clamped) * 0.5;
  const top = clamped >= 0 ? 50 - fillH : 50;
  return (
    <div className={barStyles.drivebar}>
      <div className={barStyles.value}>
        {clamped > 0 ? `+${clamped}` : clamped}%
      </div>
      <div className={barStyles.wrapper}>
        <div className={barStyles.line} />
        <div
          className={barStyles.fill}
          style={{ height: `${fillH}px`, top: `${top}px` }}
        />
      </div>
      <div className={barStyles.label}>{label}</div>
    </div>
  );
}

/**
 * D-pad drive (buttons 12–15):
 *   12 → forward (+linear.x), 13 → back (−linear.x)
 *   14 → ωz left (+angular.z), 15 → ωz right (−angular.z) — same signs as Dashboard W/S/A/D
 * Hat fallback: axes[6]/[7] when those buttons are unused.
 */
function readDpad(gp) {
  let forward = 0;
  let yaw = 0;

  const btnDown = (i) => {
    const b = gp.buttons[i];
    return !!(b && (b.pressed || (b.value ?? 0) > 0.5));
  };

  if (btnDown(12)) forward += 1; // forward
  if (btnDown(13)) forward -= 1; // back
  if (btnDown(14)) yaw += 1; // rotate left (+ωz)
  if (btnDown(15)) yaw -= 1; // rotate right (−ωz)

  if (forward !== 0 || yaw !== 0) {
    return { forward, yaw };
  }

  const ax6 = gp.axes[6] ?? 0;
  const ax7 = gp.axes[7] ?? 0;
  const t = 0.35;
  if (Math.abs(ax6) > t) yaw = -Math.sign(ax6);
  if (Math.abs(ax7) > t) forward = -Math.sign(ax7);
  return { forward, yaw };
}

export default function ArmControlCompact() {
  document.title = "Arm Control";
  const ARM_API = getArmApiBase();

  const [armControlEnabled, setArmControlEnabled] = useState(false);
  const armControlEnabledRef = useRef(armControlEnabled);
  useEffect(() => {
    armControlEnabledRef.current = armControlEnabled;
  }, [armControlEnabled]);

  const [controlMode, setControlMode] = useState("joint");
  const controlModeRef = useRef(controlMode);
  useEffect(() => { controlModeRef.current = controlMode; }, [controlMode]);
  const velocityRequestInFlight = useRef(false);

  const [eeScale, setEeScale] = useState(100);
  const [driveSpeed, setDriveSpeed] = useState(100);
  const [deadzone, setDeadzone] = useState(0.3);
  const eeScaleRef = useRef(eeScale);
  const driveSpeedRef = useRef(driveSpeed);
  const deadzoneRef = useRef(deadzone);
  const button0PrevRef = useRef(false);
  const toggleControlModeRef = useRef(null);
  const lastDisplayTsRef = useRef(0);
  useEffect(() => { eeScaleRef.current = eeScale; }, [eeScale]);
  useEffect(() => { driveSpeedRef.current = driveSpeed; }, [driveSpeed]);
  useEffect(() => { deadzoneRef.current = deadzone; }, [deadzone]);

  const [liveDisplay, setLiveDisplay] = useState({
    connected: false,
    mode: "joint",
    joint: [0, 0, 0, 0, 0, 0],
    ee: { vy: 0, vz: 0, wx: 0, j1: 0, j5: 0, j6: 0 },
    drive: { forward: 0, yaw: 0 },
  });

  useEffect(() => {
    if (!armControlEnabled) {
      setLiveDisplay((prev) => ({
        ...prev,
        joint: [0, 0, 0, 0, 0, 0],
        ee: { vy: 0, vz: 0, wx: 0, j1: 0, j5: 0, j6: 0 },
        drive: { forward: 0, yaw: 0 },
      }));
    }
  }, [armControlEnabled]);

  // ─── POST velocity command to FastAPI /arm/velocity ───
  // Gamepad polls at ~100 Hz: use `stream: true` so one in-flight request at a time.
  // UI (incremental card) omits stream so clicks are never dropped behind gamepad traffic.
  const sendVelocityCommand = async (velocities, { stream = false } = {}) => {
    if (!armControlEnabledRef.current) return;
    if (stream) {
      if (velocityRequestInFlight.current) return;
      velocityRequestInFlight.current = true;
    }
    try {
      const cmd = [...velocities.slice(0, 6)];
      while (cmd.length < 6) cmd.push(0);
      await postCmd(`${ARM_API}/velocity`, { joint_velocities: cmd });
    } catch (err) {
      if (!isTimeoutError(err)) console.error("Failed to send velocity command:", err.message);
    } finally {
      if (stream) velocityRequestInFlight.current = false;
    }
  };

  // ─── EE command: POST to FastAPI /arm/ee ────
  const eeRequestInFlight = useRef(false);
  const sendEECommand = async (
    { linearY = 0, linearZ = 0, angularX = 0, j1 = 0, j5 = 0, j6 = 0 } = {},
    { stream = false } = {}
  ) => {
    if (!armControlEnabledRef.current) return;
    if (stream) {
      if (eeRequestInFlight.current) return;
      eeRequestInFlight.current = true;
    }
    try {
      await postCmd(`${ARM_API}/ee`, {
        linear_y: linearY,
        linear_z: linearZ,
        angular_x: angularX,
        j1_velocity: j1,
        j5_velocity: j5,
        j6_velocity: j6,
      });
    } catch (err) {
      if (!isTimeoutError(err)) console.error("Failed to send EE command:", err.message);
    } finally {
      if (stream) eeRequestInFlight.current = false;
    }
  };

  // ─── Toggle joint / EE mode via FastAPI /arm/mode ────────────────────────────────
  const toggleControlMode = async () => {
    const next = controlModeRef.current === "joint" ? "ee" : "joint";
    try {
      await postCmd(`${ARM_API}/mode`, { mode: next });
      setControlMode(next);
    } catch (err) {
      if (!isTimeoutError(err)) console.error("Failed to toggle mode:", err.message);
    }
  };
  toggleControlModeRef.current = toggleControlMode;

  const driveWasActiveRef = useRef(false);
  /** Drive posts are not serialized (no in-flight skip): HTTP often exceeds the 10 ms poll, and skipping made held D-pad feel like a toggle. */
  const sendDriveCommand = (linearX, angularZ) => {
    void postCmd(getCommandUrl(), {
      linear: { x: linearX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angularZ },
    }).catch((err) => {
      if (!isTimeoutError(err)) console.error("Failed to send drive command:", err.message);
    });
  };

  // ─── Handle IncrementalMovementCard clicks ─────────────────────────────────────
  const handleSimIncrement = (mode, target, value) => {
    if (mode === "ee") {
      const mult = eeScaleRef.current / 100;
      const params = { linearY: 0, linearZ: 0, angularX: 0 };
      if (target === "Vy") params.linearY = value * mult;
      else if (target === "Vz") params.linearZ = -value * mult;
      else if (target === "Pitch") params.angularX = value * mult;
      else return;
      sendEECommand(params);
      return;
    }

    if (mode !== "joint") return;

    const jointIndex = {
      Theta1: 0, Theta2: 1, Theta3: 2, Theta4: 3, Theta5: 4,
    }[target];
    if (jointIndex === undefined) return;

    const velCmd = [0, 0, 0, 0, 0];
    const raw = Math.max(-1, Math.min(1, value / 100));
    velCmd[jointIndex] = jointIndex < 4 ? -raw : raw;
    sendVelocityCommand(velCmd);
  };

  // ─── Gamepad polling ──────────────────────────────────────────────────────────
  // D-pad → base drive (:8080/command), same frame as Dashboard (linear.x, angular.z).
  // Joint mode:  axes[0]→J1, [1]→J2, [3]→J3, [2]→J4; buttons 4/5→J5; b6→J6
  // EE mode:     axes[1]→Vy, [3]→Vz, [2]→Wx (twist); axes[0]→J1, buttons→J5, b6→J6
  useEffect(() => {
    // All velocities normalized to -1..1; J6 position (deg) is integrated separately in backend
    const pollGamepad = () => {
      const d = deadzoneRef.current;
      const applyDead = (v) => {
        const a = Math.abs(v);
        if (a < d) return 0;
        return Math.sign(v) * (a - d) / (1 - d);
      };

      const gp = navigator.getGamepads()[0];
      const now = Date.now();
      const flushLive = (payload) => {
        if (now - lastDisplayTsRef.current < DISPLAY_THROTTLE_MS) return;
        lastDisplayTsRef.current = now;
        setLiveDisplay(payload);
      };

      if (!gp) {
        if (driveWasActiveRef.current) {
          driveWasActiveRef.current = false;
          sendDriveCommand(0, 0);
        }
        flushLive({
          connected: false,
          mode: controlModeRef.current,
          joint: [0, 0, 0, 0, 0, 0],
          ee: { vy: 0, vz: 0, wx: 0, j1: 0, j5: 0, j6: 0 },
          drive: { forward: 0, yaw: 0 },
        });
        return;
      }

      const { forward: dF, yaw: dY } = readDpad(gp);
      const driveMult = (driveSpeedRef.current / 100) * MAX_DRIVE_TWIST;
      const driveLinearX = dF * driveMult;
      const driveAngularZ = dY * driveMult;
      const driveNorm = {
        forward: dF,
        yaw: dY,
      };
      const hasDrive = driveLinearX !== 0 || driveAngularZ !== 0;
      if (hasDrive || driveWasActiveRef.current) {
        driveWasActiveRef.current = hasDrive;
        sendDriveCommand(driveLinearX, driveAngularZ);
      }

      const b0 = gp.buttons[0]?.pressed ?? false;
      if (b0 && !button0PrevRef.current) {
        button0PrevRef.current = true;
        toggleControlModeRef.current?.();
      } else if (!b0) {
        button0PrevRef.current = false;
      }

      if (!armControlEnabledRef.current) {
        flushLive({
          connected: true,
          mode: controlModeRef.current,
          joint: [0, 0, 0, 0, 0, 0],
          ee: { vy: 0, vz: 0, wx: 0, j1: 0, j5: 0, j6: 0 },
          drive: driveNorm,
        });
        return;
      }

      const axis0 = applyDead(gp.axes[0] ?? 0);
      const axis1 = applyDead(gp.axes[1] ?? 0);
      const axis2 = applyDead(gp.axes[2] ?? 0);
      const axis3 = applyDead(gp.axes[3] ?? 0);

      const lb = gp.buttons[4]?.pressed ? -1 : 0;
      const rb = gp.buttons[5]?.pressed ? 1 : 0;
      const j5 = rb + lb;

      const b6 = gp.buttons[6]?.pressed ? -1 : 0;
      const b7 = gp.buttons[7]?.pressed ? 1 : 0;
      const j6 = b6 + b7;

      const mult = eeScaleRef.current / 100;
      if (controlModeRef.current === "ee") {
        flushLive({
          connected: true,
          mode: "ee",
          joint: [0, 0, 0, 0, 0, 0],
          ee: {
            vy: axis1 * mult,
            vz: -axis3 * mult,
            wx: axis2 * mult,
            j1: -axis0 * mult,
            j5,
            j6,
          },
          drive: driveNorm,
        });
        sendEECommand(
          {
            linearY: axis1 * mult,
            linearZ: -axis3 * mult,
            angularX: axis2 * mult,
            j1: -axis0 * mult,
            j5,
            j6,
          },
          { stream: true }
        );
      } else {
        flushLive({
          connected: true,
          mode: "joint",
          joint: [-axis0, -axis1, -axis3, axis2, j5, j6],
          ee: { vy: 0, vz: 0, wx: 0, j1: 0, j5: 0, j6: 0 },
          drive: driveNorm,
        });
        sendVelocityCommand([-axis0, -axis1, -axis3, axis2, j5, j6], { stream: true });
      }
    };

    const id = setInterval(pollGamepad, 10);
    return () => {
      clearInterval(id);
      if (driveWasActiveRef.current) {
        driveWasActiveRef.current = false;
        postCmd(getCommandUrl(), {
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        }).catch(() => {});
      }
    };
  }, []);

  return (
    <div className="container-fluid px-3 py-4">
      <div className="row gx-2 gx-lg-3 gy-3 gy-lg-2 align-items-start">
        <div className="col-12 col-lg-5 col-xl-4">
          <div className="d-flex flex-column gap-3">
            <div className="card p-3 w-100">
              <h6 className="mb-2 header">Arm control</h6>
              <div
                className={`form-check form-switch ${armControlEnabled ? enableSwitchStyles.panelActive : enableSwitchStyles.panelInactive}`}
              >
                <input
                  className={`form-check-input ${armControlEnabled ? enableSwitchStyles.switchOn : enableSwitchStyles.switchOff}`}
                  type="checkbox"
                  role="switch"
                  id="armControlSwitch"
                  checked={armControlEnabled}
                  onChange={() => setArmControlEnabled((v) => !v)}
                  aria-checked={armControlEnabled}
                />
                <label className="form-check-label" htmlFor="armControlSwitch">
                  <span className={armControlEnabled ? enableSwitchStyles.titleOn : enableSwitchStyles.titleOff}>
                    {armControlEnabled ? "Arm control active" : "Arm control off — sticks idle"}
                  </span>
                  <span className="d-block small text-secondary mt-1">
                    When off, no arm motion commands are sent to the backend.
                  </span>
                </label>
              </div>
            </div>

            <button
              type="button"
              className={`btn w-100 ${controlMode === "joint" ? pageStyles.modeBtnJoint : "btn-warning"}`}
              onClick={toggleControlMode}
            >
              Mode: {controlMode === "joint" ? "Joint" : "End-Effector"}
            </button>

            <div className="card p-3 w-100">
              <h6 className="mb-3 header">Controller settings</h6>
              <div className="mb-3">
                <label className="form-label small mb-1">
                  Deadzone: {deadzone.toFixed(2)}
                </label>
                <input
                  type="range"
                  className="form-range"
                  min="0"
                  max="0.5"
                  step="0.05"
                  value={deadzone}
                  onChange={(e) => setDeadzone(parseFloat(e.target.value))}
                />
              </div>
              <div className="mb-3">
                <label className="form-label small mb-1">
                  EE speed: {eeScale}%
                </label>
                <input
                  type="range"
                  className="form-range"
                  min="0"
                  max="100"
                  step="5"
                  value={eeScale}
                  onChange={(e) => setEeScale(parseInt(e.target.value, 10))}
                />
              </div>

              <div className="mt-3 pt-3 border-top border-2 border-primary border-opacity-50">
                <div className="rounded-3 px-3 py-3 bg-primary bg-opacity-10 border border-primary border-opacity-25">
                  <div className="small fw-semibold text-primary text-uppercase mb-2">
                    Rover base (D-pad)
                  </div>
                  <label className="form-label small mb-1 text-primary">
                    Base drive speed: {driveSpeed}%{" "}
                    <span className="text-primary-emphasis fw-normal">
                      (linear.x / angular.z)
                    </span>
                  </label>
                  <input
                    type="range"
                    className="form-range"
                    min="0"
                    max="100"
                    step="5"
                    value={driveSpeed}
                    onChange={(e) => setDriveSpeed(parseInt(e.target.value, 10))}
                  />
                </div>
              </div>
            </div>
          </div>
        </div>

        <div className="col-12 col-lg-7 col-xl-8">
          <div className="d-flex flex-column gap-3">
            <div className="card p-3 w-100">
              <h5 className="text-center mb-2 header">Arm command</h5>
              <p className="text-center small text-secondary mb-3">
                {!liveDisplay.connected
                  ? "No gamepad"
                  : !armControlEnabled
                    ? "Gamepad (not sent — arm control off)"
                    : "Gamepad"}
              </p>
              <div className="d-flex justify-content-around flex-wrap gap-2">
                {liveDisplay.mode === "joint"
                  ? liveDisplay.joint.map((v, i) => (
                      <VerticalBarMeter
                        key={`j${i + 1}`}
                        label={`J${i + 1}`}
                        value={v}
                      />
                    ))
                  : [
                      ["Vy", liveDisplay.ee.vy],
                      ["Vz", liveDisplay.ee.vz],
                      ["Wx", liveDisplay.ee.wx],
                      ["J1", liveDisplay.ee.j1],
                      ["J5", liveDisplay.ee.j5],
                      ["J6", liveDisplay.ee.j6],
                    ].map(([label, v]) => (
                      <VerticalBarMeter key={label} label={label} value={v} />
                    ))}
              </div>
            </div>

            <IncrementalMovementCard
              mode={controlMode}
              onIncrement={handleSimIncrement}
              disabled={!armControlEnabled}
            />

            {liveDisplay.connected && (
              <div className="card p-3 w-100">
                <h6 className="text-center mb-2 header">Base drive (D-pad)</h6>
                <div className="d-flex justify-content-around flex-wrap gap-2">
                  <VerticalBarMeter label="Fwd (b12)" value={Math.max(0, liveDisplay.drive.forward)} />
                  <VerticalBarMeter label="Back (b13)" value={Math.max(0, -liveDisplay.drive.forward)} />
                  <VerticalBarMeter label="ωz L (b14)" value={Math.max(0, liveDisplay.drive.yaw)} />
                  <VerticalBarMeter label="ωz R (b15)" value={Math.max(0, -liveDisplay.drive.yaw)} />
                </div>
              </div>
            )}

            <p className="text-muted small text-center mb-0">
              D-pad drive: b12 forward, b13 back (linear.x); b14 ωz left, b15 ωz right (angular.z). Same :8080/command as Dashboard. Sticks: Left → J1/J2, Right → J3/J4.
              LB/RB → J5. Buttons 6/7 → J6. Button 0 → mode.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
