import React from "react";
import styles from "./DrivetrainCard.module.css";

export default function DrivetrainCard({ timestamp, left, right }) {
  const bars = [
    { label: "LD", value: left },
    { label: "RD", value: right },
  ];

  return (
    <div className="card p-3 w-100">
      <h5 className="text-center mb-2 header">Drivetrain Feedback</h5>
      <p className="text-center small mb-3">Timestamp: {timestamp || "N/A"}</p>

      <div className="d-flex justify-content-around">
        {bars.map(({ label, value }) => {
          const clamped = Math.max(-100, Math.min(100, value));
          const fillH   = Math.abs(clamped) * 0.5;        // 100 px total
          const top     = clamped >= 0 ? 50 - fillH : 50;

          return (
            <div key={label} className={styles.drivebar}>
              <div className={styles.value}>{clamped>0?`+${clamped}`:clamped}%</div>
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
