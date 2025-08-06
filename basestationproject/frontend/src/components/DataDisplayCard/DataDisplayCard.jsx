import React from "react";
import styles from "./DataDisplayCard.module.css";

export default function DataDisplayCard({ radio, battery, pitch, roll }) {
  return (
    <div className={`card p-3 ${styles.card}`}>
      <h4 className="mb-3 header">Data Display</h4>

      {/* Radio */}
      <section className={styles.section}>
        {[
          ["Connection", radio.connection],
          ["Strength",   radio.strength],
          ["Ping",       `${radio.ping} ms`],
          ["RX",         radio.received],
          ["TX",         radio.sent],
        ].map(([k, v]) => (
          <div key={k} className="d-flex justify-content-between">
            <span>{k}</span><strong>{v}</strong>
          </div>
        ))}
      </section>

      {/* Battery */}
      <section className={styles.section}>
        <div className="d-flex justify-content-between">
          <span>Battery Charge<br /><strong>{battery.charge_pct.toFixed(1)}%</strong></span>
          <span>Current Draw<br /><strong>{battery.current_draw.toFixed(1)} A</strong></span>
        </div>
        <div className="d-flex justify-content-between mt-3">
          <span>Temperature<br /><strong>{battery.temperature.toFixed(1)} °C</strong></span>
          <span>Total Voltage<br /><strong>{battery.total_voltage?.toFixed(2)} V</strong></span>
        </div>
        <div className="d-flex justify-content-between mt-3">
          <span>Measured Voltage<br /><strong>{battery.measured_voltage?.toFixed(2)} V</strong></span>
          <span>Capacity<br /><strong>{battery.capacity} mAh</strong></span>
        </div>
      </section>
    </div>
  );
}
