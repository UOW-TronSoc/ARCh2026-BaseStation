import React from "react";
import styles from "./ArmFeedbackCard.module.css";

export default function ArmFeedbackCard({ jointAngles }) {
  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">Feedback</h5>

      <table className={`table table-dark table-sm ${styles.table}`}>
        <thead>
          <tr>
            <th>Joint</th>
            <th>Pos (°)</th>
          </tr>
        </thead>
        <tbody>
          {jointAngles.every((a) => a === 0) ? (
            <tr>
              <td colSpan="2" className="text-center text-muted">
                No data
              </td>
            </tr>
          ) : (
            jointAngles.map((angle, i) => (
              <tr key={i}>
                <td>{`Joint ${i + 1}`}</td>
                <td>{angle.toFixed(1)}</td>
              </tr>
            ))
          )}
        </tbody>
      </table>
    </div>
  );
}
