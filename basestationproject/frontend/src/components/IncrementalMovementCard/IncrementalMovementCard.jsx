import React, { useState } from "react";
import styles from "./IncrementalMovementCard.module.css";

const jointTargets = ["Theta1", "Theta2", "Theta3", "Theta4", "Theta5", "EE"];
const worldTargets = ["X", "Y", "Z", "Alpha", "Beta", "EE"];

export default function IncrementalMovementCard({ onIncrement }) {
  const [mode, setMode] = useState("joint");
  const [selected, setSelected] = useState(null);
  const [value, setValue] = useState("");

  const handleModeChange = (newMode) => {
    setMode(newMode);
    setSelected(null);
    setValue("");
  };

  const handleSend = () => {
    if (!selected || value === "") return;

    const numericValue = parseFloat(value);
    if (isNaN(numericValue) || numericValue < -100 || numericValue > 100) {
      alert("Value must be between -100 and 100.");
      return;
    }

    // Invoke parent callback only; Django/ROS2 integration will happen there
    if (onIncrement) {
      onIncrement(mode, selected, numericValue);
    }
    setValue("");
  };

  const targets = mode === "joint" ? jointTargets : worldTargets;

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">Incremental Movement</h5>

      {/* Mode Switch */}
      <div className="d-flex justify-content-center gap-2 mb-3">
        <button
          className={`btn ${
            mode === "joint" ? styles.activeButton : styles.inactiveButton
          }`}
          onClick={() => handleModeChange("joint")}
        >
          Joint
        </button>
        <button
          className={`btn ${
            mode === "world" ? styles.activeButton : styles.inactiveButton
          }`}
          onClick={() => handleModeChange("world")}
        >
          World
        </button>
      </div>

      {/* Target Buttons */}
      <div className="d-flex flex-wrap justify-content-center gap-2 mb-3">
        {targets.map((t) => (
          <button
            key={t}
            className={`btn ${
              selected === t ? styles.selectedButton : styles.targetButton
            }`}
            onClick={() => setSelected(t)}
          >
            {t}
          </button>
        ))}
      </div>

      {/* Input + Send */}
      <div className="input-group">
        <input
          type="number"
          className="form-control bg-dark text-white"
          placeholder="Enter Value (-100 to 100)"
          value={value}
          onChange={(e) => setValue(e.target.value)}
          disabled={!selected}
        />
        <button
          className="btn btn-primary"
          onClick={handleSend}
          disabled={!selected || value === ""}
        >
          Send
        </button>
      </div>
    </div>
  );
}
