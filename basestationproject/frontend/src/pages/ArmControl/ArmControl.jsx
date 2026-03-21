import React, { useState, useEffect, useRef } from "react";
import axios from "axios";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import ArmFeedbackCard from "components/ArmFeedbackCard/ArmFeedbackCard";
import ArmSim from "components/ArmSim/ArmSim";
// import LocationPresetCard from "components/LocationPresetCard/LocationPresetCard";
import { getApiBase } from "../../config";

export default function ArmControl() {
  document.title = "Arm Control"
  const API_BASE = getApiBase();
  const NUM_CAMS = 5;

  // ─── State ─────────────────────────────────────────────────────────────────────
  const [camId, setCamId] = useState(0);

  // jointAngles ← read from /api/arm-feedback/ (from /joint_states topic)
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides] = useState([1, 1, 1]); // deg/s for first three joints

  // Hybrid mode: "joint" or "ee" — mirrors joy_to_hybrid_control
  const [controlMode, setControlMode] = useState("joint");

  // Lock/horiz flags for joint 4 (index 3)
  const [isLocked, setIsLocked] = useState(false);
  const [isHorizontal, setIsHorizontal] = useState(false);
  const holdIntervalRef = useRef(null);
  const gripperHoldRef = useRef(null);

  // Gamepad edge‐detection refs
  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);
  const button0PrevRef = useRef(false);

  // ─── Poll feedback (actual positions) every 50 ms ────────────────────────────────
  useEffect(() => {
    let isMounted = true;
    const fetchFeedback = async () => {
      try {
        const res = await axios.get(`${API_BASE}/arm-feedback/`);
        if (res.status === 200 && Array.isArray(res.data.joints)) {
          // Take first 5 positions (J1-J5 from /joint_states)
          const positions = res.data.joints.map((j) => j.position).slice(0, 5);
          if (isMounted && positions.length === 5) {
            setJointAngles(positions);
          }
        }
      } catch (err) {
        if (err.response && err.response.status === 204) return;
        console.error("Failed to fetch arm feedback:", err.message);
      }
    };

    fetchFeedback();
    const interval = setInterval(fetchFeedback, 50);
    return () => {
      isMounted = false;
      clearInterval(interval);
    };
  }, []);

  // ─── Helper: POST `/api/arm-velocity-command/ { joint_velocities: [v1…v6] }` ───
  const sendVelocityCommand = async (velocities) => {
    try {
      const cmd = [...velocities.slice(0, 6)];
      while (cmd.length < 6) cmd.push(0);
      await axios.post(`${API_BASE}/arm-velocity-command/`, {
        joint_velocities: cmd,
      });
    } catch (err) {
      console.error("❌ Failed to send velocity command:", err.message);
    }
  };

  // ─── Gripper: send j6 velocity (-1/0/1) while button held ─
  const startGripper = (j6vel) => {
    if (gripperHoldRef.current) return;
    const send = () => sendVelocityCommand([0, 0, 0, 0, 0, j6vel]);
    send();
    gripperHoldRef.current = setInterval(send, 50);
  };
  const stopGripper = () => {
    if (gripperHoldRef.current) {
      clearInterval(gripperHoldRef.current);
      gripperHoldRef.current = null;
    }
    sendVelocityCommand([0, 0, 0, 0, 0, 0]);
  };

  // ─── EE command: Twist Vy/Vz/pitch (EE frame) + J1/J5/J6 to joint_control ────
  const sendEECommand = async ({ linearY = 0, linearZ = 0, pitch = 0, j1 = 0, j5 = 0, j6 = 0 } = {}) => {
    try {
      await axios.post(`${API_BASE}/arm-ee-command/`, {
        linear_y: linearY,
        linear_z: linearZ,
        angular_pitch: pitch,
        j1_velocity: j1,
        j5_velocity: j5,
        j6_velocity: j6,
      });
    } catch (err) {
      console.error("Failed to send EE command:", err.message);
    }
  };

  // ─── Toggle joint / EE mode (publishes Bool to kanga_arm/control_mode_joint) ────
  const toggleControlMode = async () => {
    const next = controlMode === "joint" ? "ee" : "joint";
    try {
      await axios.post(`${API_BASE}/arm-mode/`, { mode: next });
      setControlMode(next);
    } catch (err) {
      console.error("Failed to toggle mode:", err.message);
    }
  };

  // ─── When locked/horizontal is active, continuously hold joint 4 at fixed angle ─
  useEffect(() => {
    if (holdIntervalRef.current) {
      clearInterval(holdIntervalRef.current);
      holdIntervalRef.current = null;
    }

    if (controlMode === "joint" && (isLocked || isHorizontal)) {
      holdIntervalRef.current = setInterval(() => {
        const velCmd = [0, 0, 0, 0, 0]; // 5 joints only
        // To hold at a fixed angle, we repeatedly send zero velocity on joint 4 (index 3)
        // Joint 4’s actual position remains unchanged by the integrator/hardware.
        sendVelocityCommand(velCmd);
      }, 50);
    }

    return () => {
      if (holdIntervalRef.current) {
        clearInterval(holdIntervalRef.current);
        holdIntervalRef.current = null;
      }
    };
  }, [controlMode, isLocked, isHorizontal, jointAngles]);

  // ─── Handle “Incremental Movement” clicks ────────────────────────────────────────
  const handleSimIncrement = (mode, target, value) => {
    if (mode === "ee") {
      const params = { linearY: 0, linearZ: 0, pitch: 0 };
      if (target === "Vy") params.linearY = value;
      else if (target === "Vz") params.linearZ = -value;
      else if (target === "Pitch") params.pitch = value;
      else return;
      sendEECommand(params);
      return;
    }

    if (mode !== "joint") return;
    if ((isLocked || isHorizontal) && target === "Theta4") return;

    // Map “Theta#” → index 0..5 (including EE at index 5)
    const jointIndex = {
      Theta1: 0, Theta2: 1, Theta3: 2, Theta4: 3, Theta5: 4,
    }[target];
    if (jointIndex === undefined) return;

    const velCmd = [0, 0, 0, 0, 0];
    const raw = Math.max(-1, Math.min(1, value / 100));
    velCmd[jointIndex] = jointIndex < 4 ? -raw : raw;
    sendVelocityCommand(velCmd);
    setSelectedJoint(null);
  };

  // ─── Handle preset (commented out for now) ────────────────────────────────────────
  // const handlePresetTriggered = (presetAngles) => {
  //   const SPEED = 20;
  //   const velCmd = jointAngles.map((cur, i) => {
  //     if ((isLocked && i === 3) || (isHorizontal && i === 3)) return 0;
  //     if (i < 5) {
  //       if (cur < presetAngles[i]) return SPEED;
  //       if (cur > presetAngles[i]) return -SPEED;
  //       return 0;
  //     }
  //     return 0;
  //   }).slice(0, 5);
  //   sendVelocityCommand(velCmd);
  //   setTimeout(() => sendVelocityCommand([0, 0, 0, 0, 0]), 1000);
  //   setSelectedJoint(null);
  // };

  // ─── Lock / Horizontal pitch toggles ───────────────────────────────────────────
  const handleLockPitch = () => {
    if (isLocked) {
      setIsLocked(false);
    } else {
      setIsLocked(true);
      setIsHorizontal(false);
      // Immediately send zero velocity to hold joint 4
      sendVelocityCommand([0, 0, 0, 0, 0]);
    }
  };

  const handleHorizontalPitch = () => {
    if (isHorizontal) {
      setIsHorizontal(false);
    } else {
      setIsHorizontal(true);
      setIsLocked(false);
      sendVelocityCommand([0, 0, 0, 0, 0]);
    }
  };

  // ─── Gamepad polling ───────────────────────────────────────────────────────────
  // Joint mode: axes[0]→J1, [1]→J2, [3]→J3, [2]→J4; buttons[4/5]→J5; axes[5/6]→J6
  // EE mode: axis[0]→J1, [1]→Vy, [2]→pitch, [3]→Vz; buttons[4/5]→J5; axes[5/6]→J6
  useEffect(() => {
    const DEADZONE = 0.5;
    const applyDead = (v) => Math.abs(v) < DEADZONE ? 0 : v;

    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) return;

      const b0 = gp.buttons[0]?.pressed ?? false;
      if (b0 && !button0PrevRef.current) {
        button0PrevRef.current = true;
        toggleControlMode();
      } else if (!b0) {
        button0PrevRef.current = false;
      }

      const j1 = applyDead(gp.axes[0] ?? 0);
      const lb = gp.buttons[4]?.pressed ? -1 : 0;
      const rb = gp.buttons[5]?.pressed ? 1 : 0;
      const j5 = rb + lb;
      const trigNeg = gp.axes[5] ?? 1;
      const trigPos = gp.axes[6] ?? 1;
      const j6neg = trigNeg < 0.5 ? -1 : 0;
      const j6pos = trigPos < 0.5 ? 1 : 0;
      const j6 = (j6neg === 0 && j6pos === 0) ? 0 : (j6pos + j6neg);

      if (controlMode === "ee") {
        const vy = applyDead(gp.axes[1] ?? 0);
        const vz = applyDead(gp.axes[3] ?? 0);
        const pitch = -applyDead(gp.axes[2] ?? 0);
        sendEECommand({ linearY: vy, linearZ: -vz, pitch, j1: -j1, j5, j6 });
      } else {
        const j2 = applyDead(gp.axes[1] ?? 0);
        const j3 = applyDead(gp.axes[3] ?? 0);
        const j4 = applyDead(gp.axes[2] ?? 0);
        sendVelocityCommand([-j1, -j2, -j3, j4, j5, j6]);
      }
    };

    const id = setInterval(pollGamepad, 20);
    return () => clearInterval(id);
  }, [controlMode]);

  return (
    <div className="container-fluid px-3 armPage">
      {/* ROW 1 */}
      <div className="row gx-4">
        <div className="col-lg-8">
          <VideoFeedCard
            api={API_BASE}
            camId={camId}
            setCamId={setCamId}
            showDropdown
          />
        </div>
        <div className="col-lg-4 mt-3 mt-lg-0">
          <IncrementalMovementCard mode={controlMode} onIncrement={handleSimIncrement} />
        </div>
      </div>

      <div className="mt-2" />

      {/* ROW 2 */}
      <div className="row gx-4 pb-5">
        <div className="col-lg-6 mt-3">
          <VideoFeedCard
            api={API_BASE}
            camId={(camId + 1) % NUM_CAMS}
            setCamId={() => {}}
            showDropdown={false}
          />
        </div>

        <div className="col-lg-3 mt-3">
          <ArmFeedbackCard jointAngles={jointAngles} />
        </div>

        <div className="col-lg-3 mt-3 d-flex flex-column justify-content-between">
          {/* <LocationPresetCard onPresetTriggered={handlePresetTriggered} /> */}

          <button
            className={`btn w-100 mt-3 ${controlMode === "joint" ? "btn-info" : "btn-warning"}`}
            onClick={toggleControlMode}
          >
            Mode: {controlMode === "joint" ? "Joint" : "End-Effector"}
          </button>

          <button
            className="btn btn-secondary w-100 mt-2"
            onClick={handleLockPitch}
          >
            {isLocked ? "Unlock Pitch" : "Lock Pitch"}
          </button>

          <button
            className="btn btn-secondary w-100 mt-2"
            onClick={handleHorizontalPitch}
          >
            {isHorizontal ? "Disable Horizontal" : "Horizontal Pitch"}
          </button>

          <div className="mt-3 pt-3 border-top">
            <div className="text-muted small mb-2">Gripper (J6)</div>
            <div className="d-flex gap-2">
              <button
                className="btn btn-success flex-grow-1"
                onMouseDown={() => startGripper(1)}
                onMouseUp={stopGripper}
                onMouseLeave={stopGripper}
                onTouchStart={() => startGripper(1)}
                onTouchEnd={stopGripper}
              >
                Open
              </button>
              <button
                className="btn btn-danger flex-grow-1"
                onMouseDown={() => startGripper(-1)}
                onMouseUp={stopGripper}
                onMouseLeave={stopGripper}
                onTouchStart={() => startGripper(-1)}
                onTouchEnd={stopGripper}
              >
                Close
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* ROW 3 (ArmSim) - commented out for now */}
      {/* <div className="row gx-4">
        <div className="col-12 my-4">
          <div
            style={{ height: "500px", background: "#111", borderRadius: "10px" }}
          >
            <ArmSim jointAngles={jointAngles} />
          </div>
        </div>
      </div> */}

      {/* ROW 4 (Select Joint) - commented out for now */}
      {/* <div className="row">
        <div className="card mt-3">
          <div className="card-header bg-primary text-white">Select Arm Joint</div>
          <div className="card-body text-center">
            {[0, 1, 2, 3, 4].map((joint) => (
              <button
                key={joint}
                className={`btn mx-1 ${
                  selectedJoint === joint ? "btn-primary" : "btn-outline-primary"
                }`}
                onClick={() => setSelectedJoint(joint)}
              >
                {`J${joint + 1}`}
              </button>
            ))}
          </div>
        </div>
      </div> */}
    </div>
  );
}
