import React, { useState } from "react";
import styles from "./EndEffectorPitchCard.module.css";
import { postCmd, isTimeoutError } from "utils/api";


export default function EndEffectorPitchCard({ api }) {
  const [locked, setLocked] = useState(false);

  const handleLockToggle = async () => {
    try {
      await postCmd(`${api}/lock-end-effector-pitch/`, { locked: !locked });
      setLocked(!locked);
    } catch (err) {
      if (!isTimeoutError(err)) console.error("Failed to toggle pitch lock:", err.message);
    }
  };

  const handleHorizontal = async () => {
    const pose = {
      name: ["theta1", "theta2", "theta3", "theta4", "theta5"],
      position: [0.0, -1.2, 1.2, 0.0, 0.0],
    };
  
    try {
      await postCmd(`${api}/horizontal-end-effector-pitch/`, pose);
    } catch (err) {
      if (!isTimeoutError(err)) console.error("Failed to align pitch horizontally:", err.message);
    }
  };
  

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">End Effector Pitch</h5>

      <div className="d-flex flex-column gap-2">
        <button
          className={`btn ${styles.pitchButton} ${locked ? styles.locked : ""}`}
          onClick={handleLockToggle}
        >
          {locked ? "Unlock Pitch 🔓" : "Lock Pitch 🔒"}
        </button>

        <button className={`btn ${styles.pitchButton}`} onClick={handleHorizontal}>
          Horizontal Pitch
        </button>
      </div>
    </div>
  );
}
