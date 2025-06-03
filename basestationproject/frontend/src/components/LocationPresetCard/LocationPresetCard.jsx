import React, { useState } from "react";
import "./LocationPresetCard.module.css"; // your styling file

// Map each preset name → six joint angles (degrees)
const presets = {
  home:   [0.0,   0.0,    0.0,   0.0,    0.0,   -127.0],
  tiles:  [30.0,  15.0,  -10.0,  0.0,   45.0,   -127.0],
  scoop:  [60.0,  30.0,  -20.0, 10.0,   90.0,   -127.0],
  ground: [90.0,  45.0,  -30.0, 20.0,  135.0,   -127.0],
};

export default function LocationPresetCard({ onPresetTriggered }) {
  const [disabled, setDisabled] = useState(false);

  const handleClick = (presetId) => {
    if (disabled) return;

    const angles = presets[presetId];
    if (!angles) {
      console.error(`Unknown preset "${presetId}"`);
      return;
    }

    setDisabled(true);
    // Invoke parent callback with the six‐element array
    if (onPresetTriggered) {
      onPresetTriggered(angles);
    }
    // Re‐enable buttons after 1 s (to avoid rapid repeats)
    setTimeout(() => setDisabled(false), 1000);
  };

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3">Location Presets</h5>
      <div className="d-flex flex-wrap justify-content-center gap-2">
        {Object.keys(presets).map((id) => (
          <button
            key={id}
            className="btn btn-outline-primary"
            onClick={() => handleClick(id)}
            disabled={disabled}
          >
            {id.charAt(0).toUpperCase() + id.slice(1)}
          </button>
        ))}
      </div>
    </div>
  );
}
