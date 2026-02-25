import React from "react";
import styles from "./DataDisplayCard.module.css";

export default function DataDisplayCard({ battery, pitch, roll, linkLatencyMs, linkClientIp }) {
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

  return (
    <div className={`card p-3 ${styles.card}`}>
      <h4 className={styles.title}>Data Display</h4>

      {/* Link latency */}
      <section className={styles.section}>
        <h6 className={styles.sectionTitle}>Link latency</h6>
        {linkLatencyMs != null ? (
          <>
            <div className={styles.row}>
              <span className={styles.rowLabel}>RTT</span>
              <span className={styles.rowValue}>{linkLatencyMs} ms</span>
            </div>
            {linkClientIp && (
              <div className={styles.row}>
                <span className={styles.rowLabel}>Client</span>
                <span className={styles.rowValue}>{linkClientIp}</span>
              </div>
            )}
          </>
        ) : (
          <div className={styles.measuring}>Measuring…</div>
        )}
      </section>

      {/* Orientation */}
      <section className={styles.section}>
        <h6 className={styles.sectionTitle}>Orientation</h6>
        <div className={styles.row}>
          <span className={styles.rowLabel}>Pitch</span>
          <span className={styles.rowValue}>{safeNumber(pitch, 2)}°</span>
        </div>
        <div className={styles.row}>
          <span className={styles.rowLabel}>Roll</span>
          <span className={styles.rowValue}>{safeNumber(roll, 2)}°</span>
        </div>
      </section>

      {/* Battery — compact grid */}
      <section className={styles.section}>
        <h6 className={styles.sectionTitle}>Battery</h6>
        <div className={styles.batteryGrid}>
          {[
            ["Charge", `${safeNumber(battery.charge_pct)}%`],
            ["Current", `${safeNumber(battery.current_draw)} A`],
            ["Avg Temp", `${safeNumber(battery.temperature)} °C`],
            ["Max Temp", `${safeNumber(battery.temperature_max)} °C`],
            ["Min Temp", `${safeNumber(battery.temperature_min)} °C`],
            ["Charge State", `${battery.charge_state ?? "--"}`],
            ["Measured V", `${safeNumber(battery.measured_voltage, 2)} V`],
            ["Total V", `${safeNumber(battery.total_voltage, 2)} V`],
            [
              "Capacity",
              typeof battery.capacity === "number"
                ? `${battery.capacity} mAh`
                : "--",
            ],
            [
              "Stamp",
              battery.source_timestamp
                ? new Date(battery.source_timestamp * 1000).toLocaleTimeString()
                : "--",
            ],
          ].map(([label, value]) => (
            <div key={label} className={styles.batteryItem}>
              <span className={styles.rowLabel}>{label}</span>
              <span className={styles.rowValue}>{value}</span>
            </div>
          ))}
        </div>

        <div className={styles.subsection}>
          <div className={styles.subsectionTitle}>Cell Voltages</div>
          <div className={styles.inlineList}>
            {formatCellVoltages(battery.cell_voltages)}
          </div>
        </div>

        <div className={styles.subsection}>
          <div className={styles.subsectionTitle}>Fault Bits</div>
          <div className={styles.inlineList}>
            {formatFaultBits(battery.fault_bits)}
          </div>
        </div>
      </section>
    </div>
  );
}
