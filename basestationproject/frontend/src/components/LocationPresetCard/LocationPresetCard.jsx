import React, { useState } from "react";
import styles from "./LocationPresetCard.module.css";
import axios from "axios";

const presets = [
  { id: "home", label: "Home" },
  { id: "tiles", label: "Tiles" },
  { id: "scoop", label: "Scoop" },
  { id: "ground", label: "Ground" },
];

export default function LocationPresetCard({ api }) {
  const [disabled, setDisabled] = useState(false);

  const handleClick = async (id) => {
    if (disabled) return;

    try {
      setDisabled(true);
      await axios.post(`${api}/arm-preset-command/${id}/`);
    } catch (err) {
      console.error(`Failed to trigger preset "${id}":`, err.message);
    } finally {
      setTimeout(() => setDisabled(false), 5000); // re-enable after 5s
    }
  };

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">Locations</h5>
      <div className="d-flex flex-wrap justify-content-between gap-2">
        {presets.map(({ id, label }) => (
          <button
            key={id}
            className={`btn ${styles.presetButton}`}
            onClick={() => handleClick(id)}
            disabled={disabled}
          >
            {label}
          </button>
        ))}
      </div>
    </div>
  );
}
