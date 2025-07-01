import React, { useState, useEffect, useRef } from "react";
import axios from "axios";
import "./ArmControl.css";
import yaml from "js-yaml";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import ArmFeedbackCard from "components/ArmFeedbackCard/ArmFeedbackCard";
import ArmSim from "components/ArmSim/ArmSim";
import LocationPresetCard from "components/LocationPresetCard/LocationPresetCard";


export default function ArmControl() {
  const [config, setConfig] = useState(null);

  useEffect(() => {
    fetch("/setup.yaml")
      .then((res) => res.text())
      .then((text) => setConfig(yaml.load(text)));
  }, []);

  document.title = "Arm Control"
  const API_BASE = "http://127.0.0.1:8000/api";
  const NUM_CAMS = 5;

  // ─── State ─────────────────────────────────────────────────────────────────────
  const [camId, setCamId] = useState(0);

  // jointAngles ← read from /api/arm-feedback/ (physical or fake‐integrated positions)
  // Now length 6, for joints 1–5 and EE
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides] = useState([5, 5, 5]); // deg/s for first three joints

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
          // Take first 6 positions
          const positions = res.data.joints.map((j) => j.position).slice(0, 6);
          if (isMounted && positions.length === 6) {
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
      await axios.post(`${API_BASE}/arm-velocity-command/`, {
        joint_velocities: velocities,
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
        const velCmd = [0, 0, 0, 0, 0, 0];
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

    // value is interpreted as deg/s or percent for gripper (EE)
    const velCmd = [0, 0, 0, 0, 0, 0];
    velCmd[jointIndex] = value;
    sendVelocityCommand(velCmd);
    setSelectedJoint(null);
  };

  // ─── Handle preset: send a brief velocity until the arm reaches target ───────────
  const handlePresetTriggered = (presetAngles) => {
    const SPEED = 20; // deg/s for joints 1–5
    const velCmd = jointAngles.map((cur, i) => {
      if ((isLocked && i === 3) || (isHorizontal && i === 3)) {
        return 0;
      }
      if (i < 5) {
        if (cur < presetAngles[i]) return SPEED;
        if (cur > presetAngles[i]) return -SPEED;
        return 0;
      } else {
        // For EE (index 5), presets generally don't set gripper, so keep zero
        return 0;
      }
    });
    sendVelocityCommand(velCmd);

    // After a fixed time, stop all motion
    setTimeout(() => {
      sendVelocityCommand([0, 0, 0, 0, 0, 0]);
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
      sendVelocityCommand([0, 0, 0, 0, 0, 0]);
    }
  };

  const handleHorizontalPitch = () => {
    if (isHorizontal) {
      setIsHorizontal(false);
    } else {
      setIsHorizontal(true);
      setIsLocked(false);
      sendVelocityCommand([0, 0, 0, 0, 0, 0]);
    }
  };

  // ─── Gamepad polling: send per‐joint velocity commands ─────────────────────────
  useEffect(() => {
    if (!config) return; // Wait for config to load

    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) return;

      // Use YAML mappings
      const mappings = config.controllerKeyMappings;

      const rt = gp.axes[mappings.rightTrigger] > 0.5;
      const lt = gp.axes[mappings.leftTrigger] > 0.5;

      const y = gp.buttons[mappings.buttonY]?.pressed;
      const x = gp.buttons[mappings.buttonX]?.pressed;
      const a = gp.buttons[mappings.buttonA]?.pressed;

      if (y && !yButtonRef.current) {
        setSelectedJoint((prev) => (prev === null ? 0 : (prev + 1) % 6));
      }
      if (x && !xButtonRef.current) {
        setSelectedJoint((prev) => (prev === null ? 5 : (prev - 1 + 6) % 6));
      }
      if (a && !aButtonRef.current) {
        setSelectedJoint(0);
      }

      yButtonRef.current = y;
      xButtonRef.current = x;
      aButtonRef.current = a;

      // Build a 6‐element velocity command
      const velCmd = [0, 0, 0, 0, 0, 0];

      if (selectedJoint !== null) {
        // If joint 4 is locked/horizontal, ignore attempts to move it
        if ((isLocked || isHorizontal) && selectedJoint === 3) {
          sendVelocityCommand([0, 0, 0, 0, 0, 0]);
          return;
        }

        // Determine speed: first three joints use overrides, joints 4 & 5 use 10 deg/s,
        // and EE (index 5) can use a special value if needed (e.g. 50 for open/close).
        let speed = 0;
        if (selectedJoint < 3) {
          speed = jointSpeedOverrides[selectedJoint];
        } else if (selectedJoint < 5) {
          speed = 10;
        } else {
          // For EE (gripper), a different scale, e.g. ±50 for open/close %.
          speed = 50;
        }

        if (rt) {
          velCmd[selectedJoint] = speed;
        } else if (lt) {
          velCmd[selectedJoint] = -speed;
        }
  
        setJointAngles(newAngles);
        sendArmCommand(newAngles);
      }
    };
  
    const interval = setInterval(pollGamepad, 50);
    return () => clearInterval(interval);
  }, [selectedJoint, jointAngles, jointSpeedOverrides]);
  

  // ──────────────────────────────────────────────────────────────

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
            {[0, 1, 2, 3, 4, 5].map((joint) => (
              <button
                key={joint}
                className={`btn mx-1 ${
                  selectedJoint === joint ? "btn-primary" : "btn-outline-primary"
                }`}
                onClick={() => setSelectedJoint(joint)}
              >
                {joint === 5 ? "Gripper" : `Joint ${joint + 1}`}
              </button>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
