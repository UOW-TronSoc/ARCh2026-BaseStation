import React, { useState, useEffect } from "react";
import styles from "./ArmFeedbackCard.module.css";
import axios from "axios";

export default function ArmFeedbackCard({ api }) {
  const [joints, setJoints] = useState([]);

  const fetchFeedback = async () => {
    try {
      const { data } = await axios.get(`${api}/arm-feedback/`);
      const positions  = data.joint_positions  || [];
      const names      = data.joint_names      || [];
      const velocities = data.joint_velocities || [];

      // build a unified “joints” array
      const combined = positions.map((pos, i) => ({
        name:     names[i]      || `θ${i + 1}`,
        position: pos,
        velocity: velocities[i] || 0,
      }));

      setJoints(combined);
    } catch (err) {
      console.error("Failed to fetch arm feedback:", err.message);
    }
  };

  useEffect(() => {
    fetchFeedback();
    const interval = setInterval(fetchFeedback, 33);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="card p-3">
      <h5 className="text-center mb-3 header">Feedback</h5>
      <table className={`table table-dark table-sm ${styles.table}`}>
        <thead>
          <tr>
            <th>Joint</th>
            <th>Pos</th>
            <th>Vel</th>
          </tr>
        </thead>
        <tbody>
          {joints.length === 0 ? (
            <tr>
              <td colSpan="3" className="text-center text-muted">
                No data
              </td>
            </tr>
          ) : (
            joints.map((joint, i) => (
              <tr key={i}>
                <td>{joint.name}</td>
                <td>{joint.position}°</td>
                <td>{joint.velocity}°/s</td>
              </tr>
            ))
          )}
        </tbody>
      </table>
    </div>
  );
}
