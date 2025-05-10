import React, { useEffect, useState, useRef } from "react";
import axios from "axios";
import "bootstrap/dist/css/bootstrap.min.css";
import "./styles.css";
import MainNavbar from "../../components/MainNavbar";

const UnifiedControl = () => {
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);

  const [leftDrive, setLeftDrive] = useState(0);
  const [rightDrive, setRightDrive] = useState(0);
  const [speedMultiplier, setSpeedMultiplier] = useState(100);
  const [armFeedback, setArmFeedback] = useState([0, 0, 0, 0, 0, -127]);
  const [jointSpeedOverrides, setJointSpeedOverrides] = useState([1, 1, 1]);
  const [drivetrainFeedback, setDrivetrainFeedback] = useState({
    epoch_time: null,
    wheel_position: [],
    wheel_velocity: [],
    wheel_torque: [],
  });

  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);



  // Fetch Arm Feedback
  useEffect(() => {
    document.title = "Unified Control"
    const fetchArmFeedback = async () => {
      try {
        const response = await axios.get("http://localhost:8000/api/arm-feedback/");
        if (response.data.joint_positions?.length >= 6) {
          setArmFeedback(response.data.joint_positions.slice(0, 6));
        }
      } catch (error) {
        console.error("Failed to fetch arm feedback:", error.message);
      }
    };    
    const interval = setInterval(fetchArmFeedback, 1000);
    return () => clearInterval(interval);
  }, []);

  // Fetch Drivetrain Feedback
  useEffect(() => {
    const fetchDrivetrainFeedback = async () => {
      try {
        const response = await axios.get("http://localhost:8000/api/drivetrain-feedback/");
        setDrivetrainFeedback(response.data);
      } catch (error) {
        console.error("Failed to fetch drivetrain feedback:", error.message);
      }
    };
    const interval = setInterval(fetchDrivetrainFeedback, 1000);
    return () => clearInterval(interval);
  }, []);

  // Send Arm Command
  const sendArmCommand = async (angles) => {
    try {
      await axios.post("http://localhost:8000/api/arm-command/", {
        joint_positions: angles,
      });
    } catch (error) {
      console.error("Failed to send arm command:", error.message);
    }
  };

  // Send Drive Command
  const sendDriveCommand = async (left, right) => {
    try {
      await axios.post("http://localhost:8080/command", {
        left_drive: left,
        right_drive: right,
      });
    } catch (error) {
      console.error("Failed to send drive command:", error.message);
    }
  };

  // Unified Gamepad Polling
  useEffect(() => {
    const pollGamepad = () => {
      const gamepad = navigator.getGamepads()[0];
      if (!gamepad) return;
    
      // === ARM ===
      const joint1 = {}
      const jointSpeed = [
        jointSpeedOverrides[0],
        jointSpeedOverrides[1],
        jointSpeedOverrides[2],
        255.0,
        255.0
      ];      
      const rt = gamepad.axes[5] > 0.5;
      const lt = gamepad.axes[4] > 0.5;

      // X = next, Y = previous
      const yPressed = gamepad.buttons[3]?.pressed; // X
      const xPressed = gamepad.buttons[2]?.pressed; // Y
      const aPressed = gamepad.buttons[0]?.pressed; // A

      if (yPressed && !yButtonRef.current) {
        setSelectedJoint((prev) => (prev === null ? 0 : (prev + 1) % 6));
      }
      if (xPressed && !xButtonRef.current) {
        setSelectedJoint((prev) => {
          if (prev === null) return 5;
          return (prev - 1 + 6) % 6;
        });
      }
      if (aPressed && !aButtonRef.current) {
        setSelectedJoint(0); // Always reset to Joint 0
      }

      yButtonRef.current = yPressed;
      xButtonRef.current = xPressed;
      aButtonRef.current = aPressed;

    
      const updatedAngles = [...jointAngles];
      if (selectedJoint !== null) {
        if (selectedJoint < 5) {
          updatedAngles[selectedJoint] =
            (rt ? jointSpeed[selectedJoint] : 0) - (lt ? jointSpeed[selectedJoint] : 0);
        } else {
          updatedAngles[selectedJoint] = rt ? 255 : lt ? 0 : -127;
        }
        setJointAngles(updatedAngles);
        sendArmCommand(updatedAngles);
      }
    
      // === DRIVE ===
      const b12Pressed = gamepad.buttons[12]?.pressed;
      const b13Pressed = gamepad.buttons[13]?.pressed;
      const b14Pressed = gamepad.buttons[14]?.pressed;
      const b15Pressed = gamepad.buttons[15]?.pressed;
    
      let left = leftDrive;
      let right = rightDrive;
    
      if (b12Pressed) {
        left = speedMultiplier;
        right = speedMultiplier;
      } else if (b13Pressed) {
        left = -speedMultiplier;
        right = -speedMultiplier;
      } else if (b15Pressed) {
        left = speedMultiplier;
        right = -speedMultiplier;
      } else if (b14Pressed) {
        left = -speedMultiplier;
        right = speedMultiplier;
      } else {
        left = Math.round(gamepad.axes[1] * -speedMultiplier);
        right = Math.round(gamepad.axes[3] * -speedMultiplier);
      }
    
      if (left !== leftDrive || right !== rightDrive) {
        setLeftDrive(left);
        setRightDrive(right);
        sendDriveCommand(left, right);
      }
    };
    

    const interval = setInterval(pollGamepad, 50);
    return () => clearInterval(interval);
  }, [jointAngles, selectedJoint, leftDrive, rightDrive, speedMultiplier]);

  return (
    <>
      <MainNavbar />
      <div className="container-fluid mt-4">
        <div className="row">
          {/* LEFT PANEL */}
          <div className="col-lg-6 mb-4">
            <h2 className="text-white text-center">Unified Controls</h2>

            {/* Joint Selector */}
            <div className="card mb-3">
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

            {/* Joint Speed Multipliers (J1–J3) */}
            <div className="card mt-3 mb-3">
              <div className="card-header bg-warning text-dark text-center">
                Joint Speed Multipliers (J1–J3)
              </div>
              <div className="card-body">
                {[0, 1, 2].map((jointIndex) => (
                  <div key={jointIndex} className="mb-3">
                    <label className="form-label">
                      Joint {jointIndex + 1} - Speed: {jointSpeedOverrides[jointIndex]}
                    </label>
                    <input
                      type="range"
                      min={0}
                      max={5}
                      step={1}
                      value={jointSpeedOverrides[jointIndex]}
                      className="form-range"
                      onChange={(e) => {
                        const newSpeeds = [...jointSpeedOverrides];
                        newSpeeds[jointIndex] = parseInt(e.target.value);
                        setJointSpeedOverrides(newSpeeds);
                      }}
                    />
                  </div>
                ))}
              </div>
            </div>


            

            {/* Drive Multiplier */}
            <div className="card">
              <div className="card-header bg-primary text-white">Drive Speed Multiplier</div>
              <div className="card-body text-center">
                <input
                  type="range"
                  min="0"
                  max="100"
                  step="5"
                  value={speedMultiplier}
                  onChange={(e) => setSpeedMultiplier(parseInt(e.target.value))}
                  className="form-range"
                />
                <p className="mt-2">
                  <strong>Speed Multiplier:</strong> {speedMultiplier}
                </p>
              </div>
            </div>
          </div>

          {/* RIGHT PANEL */}
          <div className="col-lg-6">
            {/* Arm Feedback */}
            <div className="card mb-3">
              <div className="card-header bg-dark text-white text-center">
                Arm Joint Positions
              </div>
              <div className="card-body text-center">
                {jointAngles.slice(0, 5).map((angle, index) => (
                  <p key={index}>
                    <strong>Joint {index + 1}:</strong> {angle.toFixed(2)}°
                  </p>
                ))}
                <p>
                  <strong>Gripper:</strong> {jointAngles[5]}%
                </p>
              </div>
            </div>


            {/* Drivetrain Feedback */}
            <div className="card mb-3">
              <div className="card-header bg-secondary text-white text-center">Drivetrain Feedback</div>
              <div className="card-body">
                <p>
                  <strong>Timestamp:</strong> {drivetrainFeedback.epoch_time || "N/A"}
                </p>
                <p>
                  <strong>Wheel Positions:</strong>{" "}
                  {drivetrainFeedback.wheel_position?.join(", ") || " A"}
                </p>
                <p>
                  <strong>Wheel Velocities:</strong>{" "}
                  {drivetrainFeedback.wheel_velocity?.join(", ") || "N/A"}
                </p>
                <p>
                  <strong>Wheel Torques:</strong>{" "}
                  {drivetrainFeedback.wheel_torque?.join(", ") || "N/A"}
                </p>
              </div>
            </div>
            {/* Drive Values */}
            <div className="card mb-3">
              <div className="card-header bg-dark text-white text-center">
                Drive Values
              </div>
              <div className="card-body text-center">
                <p><strong>Left Drive:</strong> {leftDrive}</p>
                <p><strong>Right Drive:</strong> {rightDrive}</p>
              </div>
            </div>

          </div>
        </div>
      </div>
    </>
  );
};

export default UnifiedControl;
