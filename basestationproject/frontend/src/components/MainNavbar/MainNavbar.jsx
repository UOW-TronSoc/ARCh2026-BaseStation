import React, { useState, useEffect } from "react";
import logo from "assets/logo.png";
import './MainNavbar.css';
import { getBackendBase } from "../../config";
import { useBattery } from "context/BatteryContext";

export default function MainNavbar() {
  const [backendConnected, setBackendConnected] = useState(null);
  const batteryInfo = useBattery();

  useEffect(() => {
    const checkBackend = async () => {
      try {
        const ctrl = new AbortController();
        const id = setTimeout(() => ctrl.abort(), 3000);
        const r = await fetch(`${getBackendBase()}/api/status/`, { credentials: 'include', signal: ctrl.signal });
        clearTimeout(id);
        const ok = r.ok && r.status < 500;
        setBackendConnected(ok);
      } catch {
        setBackendConnected(false);
      }
    };
    checkBackend();
    const timer = setInterval(checkBackend, 5000);
    return () => clearInterval(timer);
  }, []);

  const activeFaults = Array.isArray(batteryInfo.fault_bits)
    ? batteryInfo.fault_bits
        .map((bit, idx) => (bit ? idx : null))
        .filter((v) => v !== null)
    : [];

  const batteryTooltip = `Current: ${batteryInfo.current_draw}A\n` +
    `Temp: ${batteryInfo.temperature}°C\n` +
    `Max Temp: ${batteryInfo.temperature_max}°C\n` +
    `Charge State: ${batteryInfo.charge_state}\n` +
    (activeFaults.length ? `Faults: ${activeFaults.join(', ')}` : "Faults: OK");

  return (
    <nav className="navbar navbar-expand-lg navbar-dark bg-dark sticky-top floating-navbar">
      <div className="container-fluid">
        <a className="navbar-brand d-flex align-items-center" href="/">
          <span
            className={`backend-dot ${backendConnected === true ? "backend-dot--connected" : backendConnected === false ? "backend-dot--disconnected" : "backend-dot--unknown"}`}
            title={backendConnected === true ? "Django backend connected" : backendConnected === false ? "Django backend not reachable" : "Checking…"}
            aria-label={backendConnected === true ? "Backend connected" : backendConnected === false ? "Backend disconnected" : "Checking connection"}
          />
          <img src={logo} alt="Logo" height="40" className="me-2 navbar-logo" />
          <span className="navbar-brand-text">UOW Tronsoc</span>
        </a>
        <button
          className="navbar-toggler"
          type="button"
          data-bs-toggle="collapse"
          data-bs-target="#mainNav"
          aria-controls="mainNav"
          aria-expanded="false"
          aria-label="Toggle navigation"
        >
          <span className="navbar-toggler-icon" />
        </button>
        <div className="collapse navbar-collapse" id="mainNav">
          <ul className="navbar-nav ms-auto mb-2 mb-lg-0 align-items-lg-center flex-column flex-lg-row">
            {/* Battery status */}
            <li className="nav-item me-lg-3 mb-2 mb-lg-0">
              <div className="battery-wrapper">
                <div className="battery">
                  <div
                    className={
                      "battery-level " +
                      (batteryInfo.charge_pct > 50
                        ? "battery-green"
                        : batteryInfo.charge_pct > 20
                        ? "battery-yellow"
                        : "battery-red")
                    }
                    style={{ width: `${batteryInfo.charge_pct}%` }}
                  />
                </div>
                <small
                  className="text-light ms-2"
                  title={batteryTooltip}
                >
                  {batteryInfo.charge_pct}%
                </small>
              </div>
            </li>

            {/* Home link */}
            <li className="nav-item">
              <a className="nav-link active" href="/">Home</a>
            </li>

            {/* Control Pages dropdown */}
            <li className="nav-item dropdown">
              <a
                className="nav-link dropdown-toggle"
                href="#"
                id="controlDropdown"
                role="button"
                data-bs-toggle="dropdown"
                aria-expanded="false"
              >
                Control Pages
              </a>
              <ul className="dropdown-menu dropdown-menu-end" aria-labelledby="controlDropdown">
                <li>
                  <a className="dropdown-item" href="/arm-control/">Arm Control</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/dashboard/">Dashboard</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/script-manager/">Process Manager</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/cameras/">Cameras</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/logs/">Logs</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/checklist/">Checklist</a>
                </li>
                <li>
                  <a className="dropdown-item" href="/automap/">AutoMap</a>
                </li>
              </ul>
            </li>

            {/* Telemetry link */}
            <li className="nav-item">
              <a className="nav-link" href="/services">Telemetry</a>
            </li>
          </ul>
        </div>
      </div>
    </nav>
  );
}
