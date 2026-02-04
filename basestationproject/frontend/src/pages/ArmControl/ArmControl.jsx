import React, { useState, useEffect, useRef } from "react";
import axios from "axios";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import ArmFeedbackCard from "components/ArmFeedbackCard/ArmFeedbackCard";
import ArmSim from "components/ArmSim/ArmSim";
import LocationPresetCard from "components/LocationPresetCard/LocationPresetCard";


export default function ArmControl() {
  document.title = "Arm Control"
  const API_BASE = "http://127.0.0.1:8000/api";
  const NUM_CAMS = 5;

  // ─── State ─────────────────────────────────────────────────────────────────────
  const [camId, setCamId] = useState(0);

  // jointAngles ← read from /api/arm-feedback/ (from /joint_states topic)
  // Now length 5, for joints J1–J5 (no gripper/EE in feedback)
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides] = useState([1, 1, 1]); // deg/s for first three joints

  // Lock/horiz flags for joint 4 (index 3)
  const [isLocked, setIsLocked] = useState(false);
  const [isHorizontal, setIsHorizontal] = useState(false);
  const holdIntervalRef = useRef(null);

  // Gamepad edge‐detection refs
  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);

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
      // For now we only send 5 joints (ignore gripper at index 5 if present).
      const fiveJointVel = velocities.slice(0, 5);
      await axios.post(`${API_BASE}/arm-velocity-command/`, {
        // joint_velocities: velocities, // original 6‑joint command (kept for reference)
        joint_velocities: fiveJointVel,
      });
    } catch (err) {
      console.error("❌ Failed to send velocity command:", err.message);
    }
    // The ROS2‐side must integrate these into positions and republish feedback
  };

  // ─── When locked/horizontal is active, continuously hold joint 4 at fixed angle ─
  useEffect(() => {
    if (holdIntervalRef.current) {
      clearInterval(holdIntervalRef.current);
      holdIntervalRef.current = null;
    }

    if (isLocked || isHorizontal) {
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
  }, [isLocked, isHorizontal, jointAngles]);

  // ─── Handle “Incremental Movement” clicks ────────────────────────────────────────
  const handleSimIncrement = (mode, target, value) => {
    if (mode !== "joint") return;

    // If joint 4 is locked/horizontal, ignore requests to move it
    if ((isLocked || isHorizontal) && target === "Theta4") {
      return;
    }

    // Map “Theta#” → index 0..5 (including EE at index 5)
    const jointIndex = {
      Theta1: 0,
      Theta2: 1,
      Theta3: 2,
      Theta4: 3,
      Theta5: 4,
      EE:     5,
    }[target];
    if (jointIndex === undefined) return;

    // value is interpreted as deg/s for joints J1-J5
    const velCmd = [0, 0, 0, 0, 0]; // 5 joints only
    if (jointIndex !== undefined) {
      velCmd[jointIndex] = value;
    }
    sendVelocityCommand(velCmd);
    setSelectedJoint(null);
  };

  // ─── Handle preset: send a brief velocity until the arm reaches target ───────────
  const handlePresetTriggered = (presetAngles) => {
    const SPEED = 20; // deg/s for joints J1–J5
    const velCmd = jointAngles.map((cur, i) => {
      if ((isLocked && i === 3) || (isHorizontal && i === 3)) {
        return 0;
      }
      if (i < 5) {
        if (cur < presetAngles[i]) return SPEED;
        if (cur > presetAngles[i]) return -SPEED;
        return 0;
      } else {
        // No EE/gripper in feedback, so keep zero
        return 0;
      }
    }).slice(0, 5); // Only take first 5 joints
    sendVelocityCommand(velCmd);

    // After a fixed time, stop all motion
    setTimeout(() => {
      sendVelocityCommand([0, 0, 0, 0, 0]);
    }, 1000);

    setSelectedJoint(null);
  };

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

  // ─── Gamepad polling: send per‐joint velocity commands ─────────────────────────
  useEffect(() => {
    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) return;

      const rt = gp.axes[5] > 0.5;
      const lt = gp.axes[5] < -0.5;

      const y = gp.buttons[3]?.pressed;
      const x = gp.buttons[2]?.pressed;
      const a = gp.buttons[0]?.pressed;

      // Cycle joint selection with Y/X/A among 5 joints (0..4, J1-J5)
      if (y && !yButtonRef.current) {
        setSelectedJoint((prev) => (prev === null ? 0 : (prev + 1) % 5));
      }
      if (x && !xButtonRef.current) {
        setSelectedJoint((prev) => (prev === null ? 4 : (prev - 1 + 5) % 5));
      }
      if (a && !aButtonRef.current) {
        setSelectedJoint(0);
      }

      yButtonRef.current = y;
      xButtonRef.current = x;
      aButtonRef.current = a;

      // Build a 5‐element velocity command (J1-J5 only)
      const velCmd = [0, 0, 0, 0, 0];

      if (selectedJoint !== null && selectedJoint < 5) {
        // If joint 4 is locked/horizontal, ignore attempts to move it
        if ((isLocked || isHorizontal) && selectedJoint === 3) {
          sendVelocityCommand([0, 0, 0, 0, 0]);
          return;
        }

        // Determine speed: first three joints use overrides, joints 4 & 5 use 1 deg/s
        let speed = 0;
        if (selectedJoint < 3) {
          speed = jointSpeedOverrides[selectedJoint];
        } else if (selectedJoint < 5) {
          // Joint 4 (index 3) and Joint 5 (index 4)
          speed = 1;
        }

        if (rt) {
          velCmd[selectedJoint] = speed;
        } else if (lt) {
          velCmd[selectedJoint] = -speed;
        }
      }

      // Always send something, even zeros, to stop movement when triggers release
      sendVelocityCommand(velCmd);
    };

    const id = setInterval(pollGamepad, 50);
    return () => clearInterval(id);
  }, [selectedJoint, jointAngles, jointSpeedOverrides, isLocked, isHorizontal, sendVelocityCommand]);

  return (
    <div className="container my-4">
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
          <IncrementalMovementCard onIncrement={handleSimIncrement} />
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
          <LocationPresetCard onPresetTriggered={handlePresetTriggered} />

          <button
            className="btn btn-secondary w-100 mt-3"
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
        </div>
      </div>

      {/* ROW 3 (ArmSim) */}
      <div className="row gx-4">
        <div className="col-12 my-4">
          <div
            style={{ height: "500px", background: "#111", borderRadius: "10px" }}
          >
            <ArmSim jointAngles={jointAngles} />
          </div>
        </div>
      </div>

      {/* ROW 4 (Select Joint) */}
      <div className="row">
        <div className="card mt-3">
          <div className="card-header bg-primary text-white">Select Arm Joint</div>
          <div className="card-body text-center">
            {/* Only show 5 joints (J1-J5), no gripper in feedback */}
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
      </div>
    </div>
  );
}
