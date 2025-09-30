import React from "react";
import styles from "./DataDisplayCard.module.css";

export default function DataDisplayCard({ radio, battery, pitch, roll }) {
  const safeNumber = (value, fractionDigits = 1) =>
    typeof value === "number" && !Number.isNaN(value)
      ? value.toFixed(fractionDigits)
      : "--";

  const formatCellVoltages = (cellVoltages) => {
    if (!Array.isArray(cellVoltages)) return "N/A";
    return cellVoltages.map((mv) => `${(mv / 1000).toFixed(2)}V`).join(" | ");
  };

  const formatFaultBits = (bits) => {
    if (!Array.isArray(bits)) return "N/A";
    const setBits = bits
      .map((b, i) => (b ? i : null))
      .filter((v) => v !== null);
    return setBits.length > 0 ? `Faults: ${setBits.join(", ")}` : "OK";
  };

  // console.log("Battery prop:", battery);
  return (
    
    <div className={`card p-3 ${styles.card}`}>
      <h4 className="mb-3 header">Data Display</h4>

      {/* Radio */}
      <section className={styles.section}>
        {[
          ["Connection", radio.connection],
          ["Strength", radio.strength],
          ["Ping", `${radio.ping} ms`],
          ["RX", radio.received],
          ["TX", radio.sent],
        ].map(([k, v]) => (
          <div key={k} className="d-flex justify-content-between">
            <span>{k}</span>
            <strong>{v}</strong>
          </div>
        ))}
      </section>

      {/* Orientation */}
      <section className={styles.section}>
        <div className="d-flex justify-content-between">
          <span>Pitch</span>
          <strong>{safeNumber(pitch, 2)}°</strong>
        </div>
        <div className="d-flex justify-content-between mt-2">
          <span>Roll</span>
          <strong>{safeNumber(roll, 2)}°</strong>
        </div>
      </section>

      {/* Battery */}
      <section className={styles.section}>
        {[
          [
            "Battery Charge",
            `${safeNumber(battery.charge_pct)}%`,
            "Current Draw",
            `${safeNumber(battery.current_draw)} A`,
          ],
          [
            "Avg Temp",
            `${safeNumber(battery.temperature)} °C`,
            "Max Temp",
            `${safeNumber(battery.temperature_max)} °C`,
          ],
          [
            "Min Temp",
            `${safeNumber(battery.temperature_min)} °C`,
            "Charge State",
            `${battery.charge_state ?? "--"}`,
          ],
          [
            "Measured Voltage",
            `${safeNumber(battery.measured_voltage, 2)} V`,
            "Total Voltage",
            `${safeNumber(battery.total_voltage, 2)} V`,
          ],
          [
            "Capacity",
            typeof battery.capacity === "number"
              ? `${battery.capacity} mAh`
              : "--",
            "Source Stamp",
            battery.source_timestamp
              ? new Date(battery.source_timestamp * 1000).toLocaleTimeString()
              : "--",
          ],
        ].map(([labelLeft, valueLeft, labelRight, valueRight]) => (
          <div key={labelLeft} className="d-flex justify-content-between mt-3">
            <span>
              {labelLeft}
              <br />
              <strong>{valueLeft}</strong>
            </span>
            <span>
              {labelRight}
              <br />
              <strong>{valueRight}</strong>
            </span>
          </div>
        ))}

        <div className="mt-3">
          <span>Cell Voltages</span>
          <div className={`mt-2 ${styles.inlineList}`}>
            <small>{formatCellVoltages(battery.cell_voltages)}</small>
          </div>
        </div>

        <div className="mt-3">
          <span>Fault Bits</span>
          <div className={`mt-2 ${styles.inlineList}`}>
            <small>{formatFaultBits(battery.fault_bits)}</small>
          </div>
        </div>
      </section>
    </div>
  );
}
