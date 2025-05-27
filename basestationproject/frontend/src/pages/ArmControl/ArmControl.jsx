// src/components/ArmControl/ArmControl.jsx
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

  // ——————————————————————————————
  // these two states are now separate:
  //   feedbackAngles ← what comes back from ROS2
  //   commandedAngles ← what we send to ROS2
  // ——————————————————————————————
  const [feedbackAngles, setFeedbackAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [commandedAngles, setCommandedAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);
  const [jointSpeedOverrides] = useState([1, 1, 1]);

  // gamepad debounce refs
  const yRef = useRef(false), xRef = useRef(false), aRef = useRef(false);

  // set page title once
  useEffect(() => { document.title = "Arm Control"; }, []);

  // ——————————————————————————————
  // Poll ROS2 for feedback; update ONLY feedbackAngles
  // ——————————————————————————————
  useEffect(() => {
    const fetch = async () => {
      try {
        const res = await axios.get(`${API_BASE}/arm-feedback/`);
        if (res.data?.joint_positions) {
          setFeedbackAngles(res.data.joint_positions.slice(0, 6));
        }
      } catch (e) {
        console.error("Failed to fetch arm feedback:", e);
      }
    };
    fetch();
    const id = setInterval(fetch, 33);
    return () => clearInterval(id);
  }, []);

  // send any new commands to ROS2
  const sendArmCommand = async (angles) => {
    try {
      await axios.post(`${API_BASE}/arm-command/`, {
        joint_positions: angles,
      });
    } catch (e) {
      console.error("Failed to send arm command:", e);
    }
  };

  // helper for incremental moves from your sim-card UI
  const handleSimIncrement = (mode, target, value) => {
    if (mode !== "joint") return;
    const idx = { Theta1:0, Theta2:1, Theta3:2, Theta4:3, Theta5:4, EE:5 }[target];
    if (idx == null) return;
    setCommandedAngles(prev => {
      const upd = [...prev];
      upd[idx] += value;
      sendArmCommand(upd);
      return upd;
    });
  };

  // ——————————————————————————————
  // Gamepad → update commandedAngles & fire sendArmCommand
  // ——————————————————————————————
  useEffect(() => {
    const poll = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) return;

      const rt = gp.axes[5] > 0.5;
      const lt = gp.axes[4] > 0.5;
      const y  = gp.buttons[3]?.pressed;
      const x  = gp.buttons[2]?.pressed;
      const a  = gp.buttons[0]?.pressed;

      // cycle selection
      if (y && !yRef.current) setSelectedJoint(s => (s===null?0:(s+1)%6));
      if (x && !xRef.current) setSelectedJoint(s => (s===null?5:(s+5)%6));
      if (a && !aRef.current) setSelectedJoint(0);

      yRef.current = y; xRef.current = x; aRef.current = a;

      if (selectedJoint !== null) {
        // build a new commandedAngles array
        const newCmd = [...commandedAngles];

        if (selectedJoint < 3) {
          newCmd[selectedJoint] += (rt?jointSpeedOverrides[selectedJoint]:0) - (lt?jointSpeedOverrides[selectedJoint]:0);
        } else if (selectedJoint < 5) {
          newCmd[selectedJoint] += (rt?5:0) - (lt?5:0);
        } else {
          // gripper end-effector: full open/close
          newCmd[5] = rt?255:(lt?0:-127);
        }

        setCommandedAngles(newCmd);
        sendArmCommand(newCmd);
      }
    };

    const id = setInterval(poll, 50);
    return () => clearInterval(id);
  }, [selectedJoint, commandedAngles, jointSpeedOverrides]);

  // ——————————————————————————————
  // Render
  // ——————————————————————————————
  return (
    <div className="container my-4">
      <div className="row gx-4">
        <div className="col-lg-8">
          <VideoFeedCard api={API_BASE} camId={0} setCamId={()=>{}} showDropdown />
        </div>
        <div className="col-lg-4 mt-3 mt-lg-0">
          <IncrementalMovementCard api={API_BASE} onIncrement={handleSimIncrement}/>
        </div>
      </div>

      <div className="mt-2" />
      <div className="row gx-4 pb-5">
        <div className="col-lg-6 mt-3">
          <VideoFeedCard api={API_BASE} camId={1} setCamId={()=>{}} showDropdown={false}/>
        </div>
        <div className="col-lg-3 mt-3"><FeedbackCard api={API_BASE}/></div>
        <div className="col-lg-3 mt-3 d-flex flex-column justify-content-between">
          <LocationPresetCard api={API_BASE} 
          onPreset={(positions, velocities) => {
              // if you’re still using `jointAngles` in ArmControl:
              setJointAngles(positions);
              // if you’re now using separate commanded/feedback state,
              // update whichever one drives your sim.
            }}
          />
          <div className="mt-3"><EndEffectorPitchCard api={API_BASE}/></div>
        </div>
      </div>

      <div className="row gx-4">
        <div className="col-12 my-4">
          <div style={{ height: "500px", background: "#111", borderRadius: "10px" }}>
            {/* PASS FEEDBACK ANGLES to the sim, not the commanded ones */}
            <ArmSim jointAngles={feedbackAngles} />
          </div>
        </div>
      </div>

      <div className="row">
        <div className="card mt-3">
          <div className="card-header bg-primary text-white">Select Arm Joint</div>
          <div className="card-body text-center">
            {[0,1,2,3,4,5].map(j => (
              <button
                key={j}
                className={`btn mx-1 ${selectedJoint===j?"btn-primary":"btn-outline-primary"}`}
                onClick={()=>setSelectedJoint(j)}
              >
                {j===5 ? "Gripper" : `Joint ${j+1}`}
              </button>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
