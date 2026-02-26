import React from "react";
import styles from "./DrivetrainCard.module.css";

/* Must match Dashboard's MAX_TWIST - used for twist scaling and percent display */
const MAX_TWIST = 20;
const clampPercent = (value) => {
  const percent = Math.round((value / MAX_TWIST) * 100);
  return Math.max(-100, Math.min(100, percent));
};

export default function DrivetrainCard({ timestamp, linear, angular }) {
  const bars = [
    { label: "LX", value: linear.x ?? 0 },
    { label: "LY", value: linear.y ?? 0 },
    { label: "AZ", value: angular.z ?? 0 },
  ];

  return (
    <div className="card p-3 w-100">
      <h5 className="text-center mb-2 header">Drivetrain Command</h5>
      <p className="text-center small mb-3">Timestamp: {timestamp || "N/A"}</p>

      <div className="d-flex justify-content-around">
        {bars.map(({ label, value }) => {
          const clamped = clampPercent(value);
          const fillH = Math.abs(clamped) * 0.5;
          const top = clamped >= 0 ? 50 - fillH : 50;

          return (
            <div key={label} className={styles.drivebar}>
              <div className={styles.value}>
                {clamped > 0 ? `+${clamped}` : clamped}%
              </div>
              <div className={styles.wrapper}>
                <div className={styles.line} />
                <div className={styles.fill} style={{ height: `${fillH}px`, top: `${top}px` }} />
              </div>
              <div className={styles.label}>{label}</div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
