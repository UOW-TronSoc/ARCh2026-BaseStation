// src/components/LocationPresetCard/LocationPresetCard.jsx
import React, { useState } from "react";
import styles from "./LocationPresetCard.module.css";
import axios from "axios";

const presets = [
  {
    id: "home",
    label: "Home",
    positions: [0, 0, 0, 0, 0, -127],
    velocities: [1, 1, 1, 1, 1, 1],
  },
  {
    id: "tiles",
    label: "Tiles",
    positions: [30, 15, -10, 0, 45, -127],
    velocities: [5, 5, 5, 5, 5, 1],
  },
  {
    id: "scoop",
    label: "Scoop",
    positions: [60, 30, -20, 10, 90, -127],
    velocities: [10, 10, 10, 5, 5, 1],
  },
  {
    id: "ground",
    label: "Ground",
    positions: [90, 45, -30, 20, 135, -127],
    velocities: [15, 15, 5, 5, 5, 1],
  },
];

export default function LocationPresetCard({ api, onPreset }) {
  const [disabled, setDisabled] = useState(false);

  const handleClick = async (preset) => {
    if (disabled) return;
    setDisabled(true);

    try {
      await axios.post(`${api}/arm-command/`, {
        joint_positions: preset.positions,
        joint_velocities: preset.velocities,
      });
      onPreset?.(preset.positions, preset.velocities);
    } catch (err) {
      console.error(`Failed to send preset "${preset.id}":`, err.message);
    } finally {
      // re-enable after 5s (or whatever makes sense)
      setTimeout(() => setDisabled(false), 5000);
    }
  };

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">Locations</h5>
      <div className="d-flex flex-wrap justify-content-between gap-2">
        {presets.map((p) => (
          <button
            key={p.id}
            className={`btn ${styles.presetButton}`}
            onClick={() => handleClick(p)}
            disabled={disabled}
          >
            {p.label}
          </button>
        ))}
      </div>
    </div>
  );
}
