import React, { useState, useEffect } from "react";
import axios from "axios";
import logo from "assets/logo.png";
import './MainNavbar.css';

export default function MainNavbar() {
  const API = "http://127.0.0.1:8000/api";
  const [batteryInfo, setBatteryInfo] = useState({
    charge_pct: 0,
    current_draw: 0,
    temperature: 0,
  });

  const fetchBattery = async () => {
    try {
      const { data } = await axios.get(`${API}/battery-feedback`);
      setBatteryInfo({
        charge_pct: data.charge_pct ?? 0,
        current_draw: data.current_draw ?? 0,
        temperature: data.temperature ?? 0,
      });
    } catch (err) {
      console.error("Failed to fetch battery:", err);
    }
  };

  useEffect(() => {
    fetchBattery();
    const timer = setInterval(fetchBattery, 2000);
    return () => clearInterval(timer);
  }, []);

  return (
    <nav className="navbar navbar-expand-lg navbar-dark bg-dark sticky-top floating-navbar">
      <div className="container-fluid d-flex justify-content-between">
        <a className="navbar-brand d-flex align-items-center" href="/">
          <img src={logo} alt="Logo" height="40" className="me-2" />
          <span>UOW Tronsoc</span>
        </a>
        <button
          className="navbar-toggler"
          type="button"
          data-bs-toggle="collapse"
          data-bs-target="#mainNav"
        >
          <span className="navbar-toggler-icon" />
        </button>
        <div className="collapse navbar-collapse" id="mainNav">
          <ul className="navbar-nav ms-auto mb-2 mb-lg-0 align-items-center">
            {/* Battery status */}
            <li className="nav-item me-3">
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
                  title={`Current: ${batteryInfo.current_draw}A\nTemp: ${batteryInfo.temperature}°C`}
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
