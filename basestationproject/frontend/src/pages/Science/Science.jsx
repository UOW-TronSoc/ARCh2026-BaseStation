import React, { useState, useEffect, useCallback } from "react";
import axios from "axios";
import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import { getApiBase } from "../../config";
import "./Science.css";

export default function Science() {
  document.title = "Science";
  const API_BASE = getApiBase();

  const [feedback, setFeedback] = useState({
    temperatures: [],
    ultrasonic_cm: null,
    current_amps: null,
    spectrophotometer: [],
    heating_on: false,
    cooling_on: false,
    nir_on: false,
    drill_state: "stopped",
    linear_actuator_state: "stopped",
    servo_angle: 90,
  });
  const [controlPending, setControlPending] = useState(false);

  // Poll science feedback every 500 ms
  useEffect(() => {
    let isMounted = true;
    const fetchFeedback = async () => {
      try {
        const res = await axios.get(`${API_BASE}/science-feedback/`);
        if (res.status === 200 && isMounted) {
          setFeedback((prev) => ({
            ...prev,
            ...res.data,
            temperatures: res.data.temperatures ?? prev.temperatures,
            spectrophotometer: res.data.spectrophotometer ?? prev.spectrophotometer,
          }));
        }
      } catch (err) {
        if (err.response?.status === 204) return;
        console.error("Failed to fetch science feedback:", err.message);
      }
    };
    fetchFeedback();
    const interval = setInterval(fetchFeedback, 500);
    return () => {
      isMounted = false;
      clearInterval(interval);
    };
  }, [API_BASE]);

  const sendControl = useCallback(
    async (payload) => {
      if (controlPending) return;
      setControlPending(true);
      try {
        await axios.post(`${API_BASE}/science-control/`, payload);
      } catch (err) {
        console.error("Failed to send science control:", err.message);
      } finally {
        setControlPending(false);
      }
    },
    [API_BASE, controlPending]
  );

  const handleDrill = (cmd) => () => sendControl({ drill: cmd });
  const handleLinearActuator = (cmd) => () => sendControl({ linear_actuator: cmd });
  const handleHeating = () => sendControl({ heating_on: !feedback.heating_on });
  const handleCooling = () => sendControl({ cooling_on: !feedback.cooling_on });
  const handleNir = () => sendControl({ nir_on: !feedback.nir_on });
  const handleServo = (e) => {
    const val = parseInt(e.target.value, 10);
    setFeedback((prev) => ({ ...prev, servo_angle: val }));
    sendControl({ servo_angle: val });
  };

  const temps = Array.isArray(feedback.temperatures) ? feedback.temperatures : [];
  const spec = Array.isArray(feedback.spectrophotometer) ? feedback.spectrophotometer : [];

  return (
    <div className="Science">
      <h1 className="Science-title">Science Payload</h1>

      <div className="Science-grid">
        {/* Camera - top left */}
        <div className="card Science-card Science-card-camera">
          <div className="card-header Science-card-header">Camera</div>
          <div className="card-body p-0">
            <VideoFeedCard api={API_BASE} />
          </div>
        </div>

        {/* Drill control */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Drill</div>
          <div className="card-body">
            <div className="Science-btn-group">
              <button
                className="btn btn-outline-primary"
                onClick={handleDrill("up")}
                disabled={controlPending}
              >
                Up
              </button>
              <button
                className="btn btn-outline-warning"
                onClick={handleDrill("stop")}
                disabled={controlPending}
              >
                Stop
              </button>
              <button
                className="btn btn-outline-primary"
                onClick={handleDrill("down")}
                disabled={controlPending}
              >
                Down
              </button>
            </div>
            <small className="text-muted">State: {feedback.drill_state}</small>
          </div>
        </div>

        {/* Linear actuator control */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Linear Actuator</div>
          <div className="card-body">
            <div className="Science-btn-group">
              <button
                className="btn btn-outline-primary"
                onClick={handleLinearActuator("up")}
                disabled={controlPending}
              >
                Up
              </button>
              <button
                className="btn btn-outline-warning"
                onClick={handleLinearActuator("stop")}
                disabled={controlPending}
              >
                Stop
              </button>
              <button
                className="btn btn-outline-primary"
                onClick={handleLinearActuator("down")}
                disabled={controlPending}
              >
                Down
              </button>
            </div>
            <small className="text-muted">State: {feedback.linear_actuator_state}</small>
          </div>
        </div>

        {/* Temperatures (3–5 thermistors) */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Temperatures</div>
          <div className="card-body Science-temps">
            {temps.slice(0, 5).map((t, i) => (
              <span key={i} className="Science-temp-badge">
                T{i + 1}: {typeof t === "number" ? t.toFixed(1) : t}°C
              </span>
            ))}
            {temps.length === 0 && <span className="text-muted">—</span>}
          </div>
        </div>

        {/* Heating / Cooling */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Heating & Cooling</div>
          <div className="card-body Science-toggles">
            <button
              className={`btn ${feedback.heating_on ? "btn-danger" : "btn-outline-secondary"}`}
              onClick={handleHeating}
              disabled={controlPending}
            >
              Heating {feedback.heating_on ? "ON" : "OFF"}
            </button>
            <button
              className={`btn ${feedback.cooling_on ? "btn-info" : "btn-outline-secondary"}`}
              onClick={handleCooling}
              disabled={controlPending}
            >
              Cooling {feedback.cooling_on ? "ON" : "OFF"}
            </button>
          </div>
        </div>

        {/* Ultrasonic */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Ultrasonic</div>
          <div className="card-body">
            <span className="Science-value">
              {feedback.ultrasonic_cm != null ? `${feedback.ultrasonic_cm} cm` : "—"}
            </span>
          </div>
        </div>

        {/* Current (INA226) */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Current (INA226)</div>
          <div className="card-body">
            <span className="Science-value">
              {feedback.current_amps != null ? `${feedback.current_amps} A` : "—"}
            </span>
          </div>
        </div>

        {/* NIR sensor */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">NIR Sensor</div>
          <div className="card-body">
            <button
              className={`btn ${feedback.nir_on ? "btn-success" : "btn-outline-secondary"}`}
              onClick={handleNir}
              disabled={controlPending}
            >
              NIR {feedback.nir_on ? "ON" : "OFF"}
            </button>
          </div>
        </div>

        {/* Servo */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Servo (0–180°)</div>
          <div className="card-body">
            <input
              type="range"
              min="0"
              max="180"
              value={feedback.servo_angle ?? 90}
              onChange={handleServo}
              className="form-range Science-servo-slider"
            />
            <span className="Science-value">{feedback.servo_angle ?? 90}°</span>
          </div>
        </div>

        {/* Spectrophotometer (18 values) */}
        <div className="card Science-card Science-card-spec">
          <div className="card-header Science-card-header">Spectrophotometer (18 values)</div>
          <div className="card-body">
            <div className="Science-spec-grid">
              {spec.length > 0
                ? spec.slice(0, 18).map((v, i) => (
                    <span key={i} className="Science-spec-item">
                      {i + 1}: {typeof v === "number" ? v.toFixed(3) : v}
                    </span>
                  ))
                : Array.from({ length: 18 }, (_, i) => (
                    <span key={i} className="Science-spec-item text-muted">
                      {i + 1}: —
                    </span>
                  ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
