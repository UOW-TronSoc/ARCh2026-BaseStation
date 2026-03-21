import React, { createContext, useContext, useState, useEffect } from "react";
import axios from "axios";
import { getApiBase } from "../config";

const BatteryContext = createContext(null);

const INITIAL_BATTERY = {
  charge_pct: 0,
  current_draw: 0,
  temperature: 0,
  timestamp: 0,
  temperature_max: 0,
  temperature_min: 0,
  temps: [],
  total_voltage: 0,
  measured_voltage: 0,
  capacity: 0,
  cell_voltages: [],
  cell_voltages_v: [],
  charge_state: 0,
  fault_bits: [],
  source_timestamp: null,
};

export function BatteryProvider({ children }) {
  const [batteryInfo, setBatteryInfo] = useState(INITIAL_BATTERY);

  useEffect(() => {
    const fetchBattery = async () => {
      try {
        const { data } = await axios.get(`${getApiBase()}/battery-feedback/`);
        setBatteryInfo({
          charge_pct: data.charge_pct ?? 0,
          current_draw: data.current_draw ?? 0,
          temperature: data.temperature ?? 0,
          timestamp: data.timestamp ?? 0,
          temperature_max: data.temperature_max ?? data.temperature ?? 0,
          temperature_min: data.temperature_min ?? data.temperature ?? 0,
          temps: data.temps ?? [],
          total_voltage: data.total_voltage ?? 0,
          measured_voltage: data.measured_voltage ?? 0,
          capacity: data.capacity ?? 0,
          cell_voltages: data.cell_voltages ?? [],
          cell_voltages_v: data.cell_voltages_v ?? [],
          charge_state: data.charge_state ?? 0,
          fault_bits: data.fault_bits ?? [],
          source_timestamp: data.source_timestamp ?? null,
        });
      } catch (err) {
        console.error("Failed to fetch battery status:", err.message);
      }
    };

    fetchBattery();
    const timer = setInterval(fetchBattery, 10000); // every 10 seconds — single shared poll for navbar + dashboard
    return () => clearInterval(timer);
  }, []);

  return (
    <BatteryContext.Provider value={batteryInfo}>
      {children}
    </BatteryContext.Provider>
  );
}

export function useBattery() {
  const ctx = useContext(BatteryContext);
  if (!ctx) throw new Error("useBattery must be used within BatteryProvider");
  return ctx;
}
