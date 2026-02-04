import React, { useEffect, useState, useRef, useMemo, useCallback } from "react";
import axios from "axios";
import "bootstrap/dist/css/bootstrap.min.css";
import "./styles.css";
import MainNavbar from "../../components/MainNavbar";

const EPSILON = 0.01;

const vectorsAlmostEqual = (a, b, epsilon = EPSILON) =>
  Math.abs(a.x - b.x) < epsilon &&
  Math.abs(a.y - b.y) < epsilon &&
  Math.abs(a.z - b.z) < epsilon;

const twistAlmostEqual = (a, b, epsilon = EPSILON) =>
  vectorsAlmostEqual(a.linear, b.linear, epsilon) &&
  vectorsAlmostEqual(a.angular, b.angular, epsilon);

const CONTROL_KEYS = new Set(["w", "s", "a", "d", "q", "e"]);

const applyDeadzone = (value, threshold = 0.1) =>
  Math.abs(value) < threshold ? 0 : value;

const UnifiedControl = () => {
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, -127]);
  const [selectedJoint, setSelectedJoint] = useState(null);

  const [speedMultiplier, setSpeedMultiplier] = useState(100);
  const [armFeedback, setArmFeedback] = useState([0, 0, 0, 0, 0, -127]);
  const [jointSpeedOverrides, setJointSpeedOverrides] = useState([1, 1, 1]);
  const [drivetrainFeedback, setDrivetrainFeedback] = useState({
    epoch_time: null,
    wheel_position: [],
    wheel_velocity: [],
    wheel_torque: [],
  });

  const [keyboardLinear, setKeyboardLinear] = useState({ x: 0, y: 0, z: 0 });
  const [keyboardAngular, setKeyboardAngular] = useState({ x: 0, y: 0, z: 0 });
  const [gamepadLinear, setGamepadLinear] = useState({ x: 0, y: 0, z: 0 });
  const [gamepadAngular, setGamepadAngular] = useState({ x: 0, y: 0, z: 0 });

  const speedMultiplierRef = useRef(speedMultiplier);
  const pressedKeysRef = useRef(new Set());
  const lastSentTwistRef = useRef({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  });

  const recalcKeyboardTwist = useCallback(() => {
    const multiplier = speedMultiplierRef.current / 100;
    const keys = pressedKeysRef.current;

    const newLinear = { x: 0, y: 0, z: 0 };
    const newAngular = { x: 0, y: 0, z: 0 };

    if (keys.has("w")) newLinear.x += multiplier;
    if (keys.has("s")) newLinear.x -= multiplier;
    if (keys.has("q")) newLinear.y += multiplier;
    if (keys.has("e")) newLinear.y -= multiplier;
    if (keys.has("a")) newAngular.z += multiplier;
    if (keys.has("d")) newAngular.z -= multiplier;

    setKeyboardLinear((prev) => (vectorsAlmostEqual(prev, newLinear) ? prev : newLinear));
    setKeyboardAngular((prev) => (vectorsAlmostEqual(prev, newAngular) ? prev : newAngular));
  }, []);

  useEffect(() => {
    speedMultiplierRef.current = speedMultiplier;
    recalcKeyboardTwist();
  }, [speedMultiplier, recalcKeyboardTwist]);

  useEffect(() => {
    const handleKeyDown = (event) => {
      const key = event.key.toLowerCase();
      if (!CONTROL_KEYS.has(key)) return;
      event.preventDefault();
      const keys = pressedKeysRef.current;
      if (!keys.has(key)) {
        keys.add(key);
        recalcKeyboardTwist();
      }
    };

    const handleKeyUp = (event) => {
      const key = event.key.toLowerCase();
      if (!CONTROL_KEYS.has(key)) return;
      event.preventDefault();
      const keys = pressedKeysRef.current;
      if (keys.delete(key)) {
        recalcKeyboardTwist();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [recalcKeyboardTwist]);


  const yButtonRef = useRef(false);
  const xButtonRef = useRef(false);
  const aButtonRef = useRef(false);



  // Fetch Arm Feedback
  useEffect(() => {
    document.title = "Unified Control";
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
  const sendArmCommand = useCallback(async (angles) => {
    try {
      await axios.post("http://localhost:8000/api/arm-command/", {
        joint_positions: angles,
      });
    } catch (error) {
      console.error("Failed to send arm command:", error.message);
    }
  }, []);

  // Send Drive Command
  const sendTwistCommand = useCallback(async (payload) => {
    try {
      await axios.post("http://localhost:8080/command", payload);
    } catch (error) {
      console.error("Failed to send drive command:", error.message);
    }
  }, []);

  // Unified Gamepad Polling
  useEffect(() => {
    const zeroVector = { x: 0, y: 0, z: 0 };

    const pollGamepad = () => {
      const gamepad = navigator.getGamepads()[0];
      const scale = speedMultiplierRef.current / 100;

      if (!gamepad) {
        setGamepadLinear((prev) => (vectorsAlmostEqual(prev, zeroVector) ? prev : zeroVector));
        setGamepadAngular((prev) => (vectorsAlmostEqual(prev, zeroVector) ? prev : zeroVector));
        return;
      }

      // === ARM ===
      const jointSpeed = [
        jointSpeedOverrides[0],
        jointSpeedOverrides[1],
        jointSpeedOverrides[2],
        255.0,
        255.0,
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

      // === DRIVE (Twist) ===
      const linearVector = {
        x: applyDeadzone(-gamepad.axes[1] * scale),
        y: applyDeadzone(gamepad.axes[0] * scale),
        z: 0,
      };

      const angularVector = {
        x: 0,
        y: 0,
        z: applyDeadzone(gamepad.axes[2] * scale),
      };

      setGamepadLinear((prev) => (vectorsAlmostEqual(prev, linearVector) ? prev : linearVector));
      setGamepadAngular((prev) => (vectorsAlmostEqual(prev, angularVector) ? prev : angularVector));
    };

    const interval = setInterval(pollGamepad, 50);
    return () => clearInterval(interval);
  }, [jointAngles, selectedJoint, jointSpeedOverrides, sendArmCommand]);

  const combinedLinear = useMemo(
    () => ({
      x: keyboardLinear.x + gamepadLinear.x,
      y: keyboardLinear.y + gamepadLinear.y,
      z: keyboardLinear.z + gamepadLinear.z,
    }),
    [keyboardLinear, gamepadLinear]
  );

  const combinedAngular = useMemo(
    () => ({
      x: keyboardAngular.x + gamepadAngular.x,
      y: keyboardAngular.y + gamepadAngular.y,
      z: keyboardAngular.z + gamepadAngular.z,
    }),
    [keyboardAngular, gamepadAngular]
  );

  // Send drivetrain commands continuously at fixed rate (20Hz)
  useEffect(() => {
    const sendInterval = setInterval(() => {
      const payload = {
        linear: combinedLinear,
        angular: combinedAngular,
      };

      // Always send to maintain continuous command stream
      lastSentTwistRef.current = {
        linear: { ...payload.linear },
        angular: { ...payload.angular },
      };

      sendTwistCommand(payload);
    }, 50); // Send at 20Hz (50ms interval)

    return () => clearInterval(sendInterval);
  }, [combinedLinear, combinedAngular, sendTwistCommand]);

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
                    <strong>Joint {index + 1} Cmd:</strong> {angle.toFixed(2)}° |
                    <strong> Fb:</strong>{" "}
                    {typeof armFeedback[index] === "number"
                      ? `${armFeedback[index].toFixed(2)}°`
                      : "N/A"}
                  </p>
                ))}
                <p>
                  <strong>Gripper Cmd:</strong> {jointAngles[5]}% |
                  <strong> Fb:</strong>{" "}
                  {typeof armFeedback[5] === "number" ? `${armFeedback[5]}%` : "N/A"}
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
            {/* Twist Command Preview */}
            <div className="card mb-3">
              <div className="card-header bg-dark text-white text-center">
                Twist Command
              </div>
              <div className="card-body text-center">
                <p>
                  <strong>Linear:</strong> X {combinedLinear.x.toFixed(2)}, Y {combinedLinear.y.toFixed(2)}, Z {combinedLinear.z.toFixed(2)}
                </p>
                <p>
                  <strong>Angular:</strong> X {combinedAngular.x.toFixed(2)}, Y {combinedAngular.y.toFixed(2)}, Z {combinedAngular.z.toFixed(2)}
                </p>
              </div>
            </div>

          </div>
        </div>
      </div>
    </>
  );
};

export default UnifiedControl;
