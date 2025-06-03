import React, { useState, useEffect, useRef } from "react";
import axios from "axios";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import ArmFeedbackCard from "components/ArmFeedbackCard/ArmFeedbackCard";
import ArmSim from "components/ArmSim/ArmSim";
import LocationPresetCard from "components/LocationPresetCard/LocationPresetCard";


export default function ArmControl() {
  const API_BASE = "http://127.0.0.1:8000/api";
  const NUM_CAMS = 5;

  // Camera state
  const [camId, setCamId] = useState(0);

  // jointAngles ← updated only via polling /api/arm-feedback/
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides] = useState([1, 1, 1]);

  // Track lock/horizontal state
  const [isLocked, setIsLocked] = useState(false);
  const [isHorizontal, setIsHorizontal] = useState(false);
  const holdIntervalRef = useRef(null);

  // Gamepad edge detection refs
  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);

  // Poll `/api/arm-feedback/` every 50ms
  useEffect(() => {
    let isMounted = true;

    const fetchFeedback = async () => {
      try {
        const res = await axios.get(`${API_BASE}/arm-feedback/`);
        if (res.status === 200 && Array.isArray(res.data.joints)) {
          const positions = res.data.joints.map((j) => j.position);
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

  // Helper: POST `/api/arm-command/` without updating state locally
  const sendArmCommand = async (positions) => {
    try {
      await axios.post(`${API_BASE}/arm-command/`, {
        joint_positions: positions,
      });
    } catch (err) {
      console.error("❌ Failed to send arm command:", err.message);
    }
    // Wait for feedback to update `jointAngles`
  };

  // If locked or horizontal, continuously send hold commands
  useEffect(() => {
    // Clear any existing interval
    if (holdIntervalRef.current) {
      clearInterval(holdIntervalRef.current);
      holdIntervalRef.current = null;
    }

    if (isLocked || isHorizontal) {
      holdIntervalRef.current = setInterval(() => {
        // Always use the latest jointAngles but override index 3
        const newAngles = [...jointAngles];
        newAngles[3] = isLocked ? 90 : 0;
        sendArmCommand(newAngles);
      }, 50);
    }

    return () => {
      if (holdIntervalRef.current) {
        clearInterval(holdIntervalRef.current);
        holdIntervalRef.current = null;
      }
    };
  }, [isLocked, isHorizontal, jointAngles]);

  // Handle IncrementalMovementCard clicks
  const handleSimIncrement = (mode, target, value) => {
    if (mode !== "joint") return;
    // If joint4 is locked/horizontal, ignore increments to it
    if (isLocked || isHorizontal) {
      if (target === "Theta4") return;
    }
    const jointIndex = {
      Theta1: 0,
      Theta2: 1,
      Theta3: 2,
      Theta4: 3,
      Theta5: 4,
      EE: 5,
    }[target];
    if (jointIndex === undefined) return;

    const newAngles = [...jointAngles];
    newAngles[jointIndex] += value;
    sendArmCommand(newAngles);
    setSelectedJoint(null);
  };

  // Handle preset clicks
  const handlePresetTriggered = (presetAngles) => {
    // If locked/horizontal, override preset's joint4
    const newAngles = [...presetAngles];
    if (isLocked) newAngles[3] = 90;
    if (isHorizontal) newAngles[3] = 0;
    sendArmCommand(newAngles);
    setSelectedJoint(null);
  };

  // Lock pitch toggle
  const handleLockPitch = () => {
    if (isLocked) {
      setIsLocked(false);
    } else {
      setIsLocked(true);
      setIsHorizontal(false);
      // Immediately send once to set joint4 to 90°
      const newAngles = [...jointAngles];
      newAngles[3] = 90;
      sendArmCommand(newAngles);
    }
  };

  // Horizontal pitch toggle
  const handleHorizontalPitch = () => {
    if (isHorizontal) {
      setIsHorizontal(false);
    } else {
      setIsHorizontal(true);
      setIsLocked(false);
      // Immediately send once to set joint4 to 0°
      const newAngles = [...jointAngles];
      newAngles[3] = 0;
      sendArmCommand(newAngles);
    }
  };

  // Gamepad polling: only send commands
  useEffect(() => {
    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) return;

      const rt = gp.axes[5] > 0.5;
      const lt = gp.axes[4] > 0.5;

      const y = gp.buttons[3]?.pressed;
      const x = gp.buttons[2]?.pressed;
      const a = gp.buttons[0]?.pressed;

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

      if (selectedJoint !== null && (rt || lt)) {
        // If locked/horizontal and trying to move joint4, ignore
        if ((isLocked || isHorizontal) && selectedJoint === 3) {
          return;
        }
        const newAngles = [...jointAngles];
        if (selectedJoint < 3) {
          const speed = jointSpeedOverrides[selectedJoint];
          newAngles[selectedJoint] += (rt ? speed : 0) - (lt ? speed : 0);
        } else if (selectedJoint < 5) {
          newAngles[selectedJoint] += (rt ? 5 : 0) - (lt ? 5 : 0);
        } else {
          newAngles[5] = rt ? 255 : lt ? 0 : newAngles[5];
        }
        sendArmCommand(newAngles);
      }
    };

    const id = setInterval(pollGamepad, 50);
    return () => clearInterval(id);
  }, [selectedJoint, jointAngles, jointSpeedOverrides, isLocked, isHorizontal]);

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
