import React, { useState, useEffect, useRef } from "react";
import axios from "axios";
import "./ArmControl.css";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import FeedbackCard from "components/ArmFeedbackCard/ArmFeedbackCard";
import LocationPresetCard from "components/LocationPresetCard/LocationPresetCard";
import EndEffectorPitchCard from "components/EndEffectorPitchCard/EndEffectorPitchCard";
import IncrementalMovementCard from "components/IncrementalMovementCard/IncrementalMovementCard";
import ArmSim from "components/ArmSim/ArmSim";

export default function ArmControl() {
  const API_BASE = "http://127.0.0.1:8000/api";
  const NUM_CAMS = 5;
  const [camId, setCamId] = useState(0);

  // Gamepad + Arm Control State
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides, setJointSpeedOverrides] = useState([1, 1, 1]);

  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);

  useEffect(() => {
    document.title = "Arm Control";
  }, []);

  // Arm feedback polling
  useEffect(() => {
    const fetchArmFeedback = async () => {
      try {
        const res = await axios.get(`${API_BASE}/arm-feedback/`);
        if (res.data?.joint_positions) {
          setJointAngles(res.data.joint_positions.slice(0, 6));
        }
      } catch (err) {
        console.error("❌ Failed to fetch arm feedback:", err.message);
      }
    };

    fetchArmFeedback();
    const interval = setInterval(fetchArmFeedback, 1000);
    return () => clearInterval(interval);
  }, []);

  // Arm command sender
  // const sendArmCommand = async (angles) => {
  //   const safe = angles.map((a) => parseFloat(a) || 0.0);
  //   try {
  //     await axios.post(`${API_BASE}/arm-command/`, {
  //       joint_positions: safe,
  //     });
  //   } catch (err) {
  //     console.error("❌ Failed to send arm command:", err.message);
  //   }
  // };

  const sendArmCommand = async (angles) => {
    try {
      await axios.post(`${API_BASE}/arm-command/`, {
        joint_positions: angles,
      });
    } catch (err) {
      console.error("❌ Failed to send arm command:", err.message);
    }
  };
  
  const handleSimIncrement = (mode, target, value) => {
    if (mode !== "joint") return;
  
    const jointIndex = {
      Theta1: 0,
      Theta2: 1,
      Theta3: 2,
      Theta4: 3,
      Theta5: 4,
      EE: 5,
    }[target];
  
    if (jointIndex === undefined) return;
  
    setJointAngles((prev) => {
      const updated = [...prev];
      updated[jointIndex] += value;
      return updated;
    });
  };
  

  // Gamepad polling logic
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
  
      const newAngles = [...jointAngles];
  
      if (selectedJoint !== null) {
        if (selectedJoint < 3) {
          const speed = jointSpeedOverrides[selectedJoint];
          newAngles[selectedJoint] += (rt ? speed : 0) - (lt ? speed : 0);
        } else if (selectedJoint < 5) {
          newAngles[selectedJoint] += (rt ? 5 : 0) - (lt ? 5 : 0);
        } else {
          newAngles[selectedJoint] = rt ? 255 : lt ? 0 : -127;
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
      {/* ────────────────────── ROW 1 ────────────────────── */}
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
          <IncrementalMovementCard api={API_BASE} onIncrement={handleSimIncrement}/>
        </div>
      </div>

      <div className="mt-2" />

      {/* ────────────────────── ROW 2 ────────────────────── */}
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
          <FeedbackCard api={API_BASE} />
        </div>

        <div className="col-lg-3 mt-3 d-flex flex-column justify-content-between">
          <LocationPresetCard api={API_BASE} />
          <div className="mt-3">
            <EndEffectorPitchCard api={API_BASE} />
          </div>
        </div>
      </div>

      <div className="row gx-4">
        <div className="col-12 my-4">
          <div style={{ height: "500px", background: "#111", borderRadius: "10px" }}>
            <ArmSim jointAngles={jointAngles} />
          </div>
        </div>
      </div>

      <div className="row">
        <div className="card mt-3">
          <div className="card-header bg-primary text-white">Select Arm Joint</div>
          <div className="card-body text-center">
            {[0, 1, 2, 3, 4, 5].map((joint) => (
              <button
                key={joint}
                className={`btn mx-1 ${selectedJoint === joint ? "btn-primary" : "btn-outline-primary"}`}
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
