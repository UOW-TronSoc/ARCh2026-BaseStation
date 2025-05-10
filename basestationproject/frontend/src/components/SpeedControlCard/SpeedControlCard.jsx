import React, { useRef, useState, useEffect } from "react";
import styles from "./SpeedControlCard.module.css";

export default function SpeedControlCard({ speed, setSpeed, enabled, setEnabled }) {
  const svgRef = useRef(null);
  const isDragging = useRef(false);

  const C = 120;     // Center
  const R = 100;     // Radius
  const START = 225; // Degrees
  const END = 495;   // Degrees
  const STROKE = 32;
  const arcRange = END - START;

  const polar = (cx, cy, r, deg) => {
    const rad = (deg - 90) * Math.PI / 180;
    return {
      x: cx + r * Math.cos(rad),
      y: cy + r * Math.sin(rad)
    };
  };

  const arc = (cx, cy, r, a1, a2) => {
    const start = polar(cx, cy, r, a1);
    const end = polar(cx, cy, r, a2);
    const largeArc = a2 - a1 > 180 ? 1 : 0;
    return `M ${start.x} ${start.y} A ${r} ${r} 0 ${largeArc} 1 ${end.x} ${end.y}`;
  };

  const angleFromValue = (v) => START + (v / 100) * arcRange;

  const valueFromAngle = (ang) => {
    let a = ang < 0 ? ang + 360 : ang;
    if (a < START) a += 360;
    a = Math.max(START, Math.min(END, a));
    return Math.round(((a - START) / arcRange) * 100);
  };

  const handleMouseMove = (e) => {
    if (!isDragging.current || !enabled) return;

    const rect = svgRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left - C;
    const y = e.clientY - rect.top - C;
    const angle = Math.atan2(y, x) * 180 / Math.PI + 90;
    const newVal = valueFromAngle(angle);
    setSpeed(newVal);
  };

  const handleMouseUp = () => {
    isDragging.current = false;
    window.removeEventListener("mousemove", handleMouseMove);
    window.removeEventListener("mouseup", handleMouseUp);
  };

  const handleMouseDown = (e) => {
    if (!enabled) return;
    e.preventDefault();
    isDragging.current = true;
    window.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", handleMouseUp);
  };

  const angle = angleFromValue(speed);
  const knobPos = polar(C, C, R, angle);
  const label0 = polar(C, C, R + 28, START);
  const label100 = polar(C, C, R + 28, END);

  return (
    <div className="card w-100 speedCard">
      <div className={`card-header d-flex justify-content-between align-items-center ${styles.cardHeader}`}>
        <h5 className="mb-0 header">Speed Control</h5>
        <div className="form-check form-switch">
          <input
            type="checkbox"
            className={`form-check-input ${styles.switch}`}
            checked={enabled}
            onChange={() => setEnabled(!enabled)}
          />
        </div>
      </div>

      <div className={`card-body ${styles.cardBody}`}>
        <svg ref={svgRef} className={styles.svg}>
          <path d={arc(C, C, R, START, END)} className={styles.arcBg} fill="none" />
          <path d={arc(C, C, R, START, angle)} className={styles.arcProgress} fill="none" />
          <g className={styles.knob} onMouseDown={handleMouseDown}>
            <circle cx={knobPos.x} cy={knobPos.y} r={STROKE / 2} fill="#2f2f2f" />
            <text
              x={knobPos.x}
              y={knobPos.y + 4}
              textAnchor="middle"
              fontWeight="700"
              fontSize="16"
              fill="#fff"
              transform={`rotate(${angle - 90} ${knobPos.x} ${knobPos.y})`}
            >
              ››
            </text>
          </g>
          <text x={C} y={C + 10} className={styles.value}>{speed}</text>
          <text x={label0.x} y={label0.y + 4} className={styles.label}>0</text>
          <text x={label100.x} y={label100.y + 4} className={styles.label}>100</text>
        </svg>
      </div>
    </div>
  );
}
