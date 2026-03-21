import React, { useState, useEffect, useRef } from "react";
import axios from "axios";

import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import { getApiBase } from "../../config";

export default function ArmControlCompact() {
  document.title = "Arm Control";
  const API_BASE = getApiBase();

  const [controlMode, setControlMode] = useState("joint");
  const controlModeRef = useRef(controlMode);
  useEffect(() => { controlModeRef.current = controlMode; }, [controlMode]);
  const velocityRequestInFlight = useRef(false);

  const [eeScale, setEeScale] = useState(100);
  const [deadzone, setDeadzone] = useState(0.3);
  const eeScaleRef = useRef(eeScale);
  const deadzoneRef = useRef(deadzone);
  const button0PrevRef = useRef(false);
  const toggleControlModeRef = useRef(null);
  useEffect(() => { eeScaleRef.current = eeScale; }, [eeScale]);
  useEffect(() => { deadzoneRef.current = deadzone; }, [deadzone]);

  // ─── POST `/api/arm-velocity-command/ { joint_velocities: [v1…v6] }` ───
  const sendVelocityCommand = async (velocities) => {
    if (velocityRequestInFlight.current) return;
    velocityRequestInFlight.current = true;
    try {
      const cmd = [...velocities.slice(0, 6)];
      while (cmd.length < 6) cmd.push(0);
      await axios.post(`${API_BASE}/arm-velocity-command/`, {
        joint_velocities: cmd,
      });
    } catch (err) {
      console.error("❌ Failed to send velocity command:", err.message);
    } finally {
      velocityRequestInFlight.current = false;
    }
  };

  // ─── EE command: POST Twist (Vy/Vz/Wx) + J1/J5/J6 to /api/arm-ee-command/ ────
  const eeRequestInFlight = useRef(false);
  const sendEECommand = async ({ linearY = 0, linearZ = 0, angularX = 0, j1 = 0, j5 = 0, j6 = 0 } = {}) => {
    if (eeRequestInFlight.current) return;
    eeRequestInFlight.current = true;
    try {
      await axios.post(`${API_BASE}/arm-ee-command/`, {
        linear_y: linearY,
        linear_z: linearZ,
        angular_x: angularX,
        j1_velocity: j1,
        j5_velocity: j5,
        j6_velocity: j6,
      });
    } catch (err) {
      console.error("Failed to send EE command:", err.message);
    } finally {
      eeRequestInFlight.current = false;
    }
  };

  // ─── Toggle joint / EE mode ────────────────────────────────────────────────────
  const toggleControlMode = async () => {
    const next = controlModeRef.current === "joint" ? "ee" : "joint";
    try {
      await axios.post(`${API_BASE}/arm-mode/`, { mode: next });
      setControlMode(next);
    } catch (err) {
      console.error("Failed to toggle mode:", err.message);
    }
  };
  toggleControlModeRef.current = toggleControlMode;

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
      if (!gp) return;

      const b0 = gp.buttons[0]?.pressed ?? false;
      if (b0 && !button0PrevRef.current) {
        button0PrevRef.current = true;
        toggleControlModeRef.current?.();
      } else if (!b0) {
        button0PrevRef.current = false;
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

      if (controlModeRef.current === "ee") {
        const mult = eeScaleRef.current / 100;
        sendEECommand({
          linearY: axis1 * mult,
          linearZ: -axis3 * mult,
          angularX: axis2 * mult,
          j1: -axis0 * mult,
          j5,
          j6,
        });
      } else {
        sendVelocityCommand([-axis0, -axis1, -axis3, axis2, j5, j6]);
      }
    };

    const id = setInterval(pollGamepad, 50);
    return () => clearInterval(id);
  }, []);

  return (
    <div className="container-fluid px-3 py-4">
      <div className="row justify-content-center">
        <div className="col-12 col-md-8 col-lg-6">
          <div className="d-flex flex-column gap-3">
            <button
              className={`btn w-100 ${controlMode === "joint" ? "btn-info" : "btn-warning"}`}
              onClick={toggleControlMode}
            >
              Mode: {controlMode === "joint" ? "Joint" : "End-Effector"}
            </button>

            <div className="card p-3">
              <h6 className="mb-3">Controller settings</h6>
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
              <div>
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
            </div>

            <IncrementalMovementCard mode={controlMode} onIncrement={handleSimIncrement} />

            <p className="text-muted small text-center mb-0">
              Gamepad: Left stick → J1/J2, Right stick X/Y → J3/J4. LB/RB → J5. b6 → J6. Button 0 → Mode.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
