import React, { useState, useEffect } from "react";
import styles from "./IncrementalMovementCard.module.css";

const JOINT_TARGETS = ["Theta1", "Theta2", "Theta3", "Theta4", "Theta5"];
const EE_TARGETS = ["Vy", "Vz", "Pitch"];
const QUICK_VELS = [5, 10, 20];

export default function IncrementalMovementCard({ mode = "joint", onIncrement, disabled = false }) {
  const [selected, setSelected] = useState(null);
  const [value, setValue] = useState("");

  const targets = mode === "ee" ? EE_TARGETS : JOINT_TARGETS;
  const isEE = mode === "ee";

  useEffect(() => {
    setSelected(null);
    setValue("");
  }, [mode]);

  const sendVelocity = (vel) => {
    if (disabled || !selected || !onIncrement) return;
    onIncrement(mode, selected, vel);
  };

  const handleSend = () => {
    if (disabled || !selected || value === "") return;

    const numericValue = parseFloat(value);
    if (isNaN(numericValue) || numericValue < -100 || numericValue > 100) {
      alert("Value must be between -100 and 100.");
      return;
    }

    onIncrement(mode, selected, numericValue);
    setValue("");
  };

  const formatLabel = (t) => {
    if (isEE) return t;
    return t.replace("Theta", "J");
  };

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">
        {isEE ? "End-Effector Control" : "Joint Velocity"}
      </h5>
      <p className="text-center text-muted small mb-3">
        {isEE ? "EE frame: Vy, Vz, Pitch" : "deg/s per joint"}
      </p>

      <div className="d-flex flex-wrap justify-content-center gap-2 mb-3">
        {targets.map((t) => (
          <button
            key={t}
            type="button"
            className={`btn ${
              selected === t ? styles.selectedButton : styles.targetButton
            }`}
            disabled={disabled}
            onClick={() => setSelected(t)}
          >
            {formatLabel(t)}
          </button>
        ))}
      </div>

      {selected && (
        <div className="d-flex flex-wrap justify-content-center gap-2 mb-2">
          {QUICK_VELS.flatMap((v) => [
            <button
              key={`-${v}`}
              type="button"
              className="btn btn-outline-secondary btn-sm"
              disabled={disabled}
              onClick={() => sendVelocity(-v)}
            >
              -{v}
            </button>,
            <button
              key={`+${v}`}
              type="button"
              className="btn btn-outline-primary btn-sm"
              disabled={disabled}
              onClick={() => sendVelocity(v)}
            >
              +{v}
            </button>,
          ])}
        </div>
      )}

      <div className="input-group">
        <input
          type="number"
          className="form-control bg-dark text-white"
          placeholder={isEE ? "Value (-100 to 100)" : "Velocity deg/s (-100 to 100)"}
          value={value}
          onChange={(e) => setValue(e.target.value)}
          disabled={disabled || !selected}
        />
        <button
          type="button"
          className="btn btn-primary"
          onClick={handleSend}
          disabled={disabled || !selected || value === ""}
        >
          Send
        </button>
      </div>
    </div>
  );
}
