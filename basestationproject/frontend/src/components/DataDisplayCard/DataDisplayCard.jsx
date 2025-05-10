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
          <span>Last Update<br /><strong>{new Date(battery.timestamp*1000).toLocaleTimeString()}</strong></span>
        </div>
      </section>

      {/* Pitch / Roll */}
      <section className={`d-flex justify-content-between ${styles.section}`}>
        <span>Pitch<br /><strong>{pitch.toFixed(2)} °</strong></span>
        <span>Roll<br /><strong>{roll.toFixed(2)} °</strong></span>
      </section>
    </div>
  );
}
