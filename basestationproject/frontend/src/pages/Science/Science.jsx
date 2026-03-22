import React, { useState, useEffect, useCallback, useRef } from "react";
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
    linear_actuator_speed: 0,
  });

  const feedbackRef = useRef(feedback);
  feedbackRef.current = feedback;

  const [nirDemo, setNirDemo] = useState({ status: "idle", message: "", detail: "" });

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
    return () => { isMounted = false; clearInterval(interval); };
  }, [API_BASE]);

  const sendControl = useCallback(
    async (payload) => {
      try {
        await axios.post(`${API_BASE}/science-control/`, payload);
      } catch (err) {
        console.error("Failed to send science control:", err.message);
      }
    },
    [API_BASE]
  );

  const handleLinearActuator = (speed) => () => sendControl({ linear_actuator_speed: speed });
  const handleHeating = () => sendControl({ heating: !feedback.heating_on });
  const handleCooling = () => sendControl({ cooling: !feedback.cooling_on });

  const runNirServoDemo = useCallback(async () => {
    setNirDemo({ status: "running", message: "Running NIR servo routine…", detail: "" });
    try {
      const { data } = await axios.post(`${API_BASE}/nir-servo-demo/`, {});
      if (data.ok) {
        setNirDemo({
          status: "success",
          message: data.message || "NIR servo routine finished.",
          detail: (data.stdout || "").trim(),
        });
      } else {
        setNirDemo({
          status: "error",
          message: data.message || data.error || "NIR servo script failed.",
          detail: [data.stderr, data.stdout].filter(Boolean).join("\n\n").trim(),
        });
      }
    } catch (err) {
      const d = err.response?.data;
      setNirDemo({
        status: "error",
        message: d?.error || d?.message || err.message || "Could not run NIR servo demo.",
        detail: [d?.stderr, d?.stdout].filter(Boolean).join("\n\n").trim(),
      });
    }
  }, [API_BASE]);

  // Gamepad: D-pad → linear actuator speed, A → heating toggle, B → cooling toggle
  const prevBtnsRef = useRef({ a: false, b: false });
  const lastSpeedRef = useRef(0);

  useEffect(() => {
    const poll = () => {
      const gp = navigator.getGamepads?.()?.[0];
      if (!gp) return;

      const up = gp.buttons[12]?.pressed;
      const down = gp.buttons[13]?.pressed;
      let speed = 0;
      if (up && !down) speed = 100;
      else if (down && !up) speed = -100;
      if (speed !== lastSpeedRef.current) {
        lastSpeedRef.current = speed;
        sendControl({ linear_actuator_speed: speed });
      }

      const aPressed = gp.buttons[0]?.pressed ?? false;
      if (aPressed && !prevBtnsRef.current.a) {
        sendControl({ heating: !feedbackRef.current.heating_on });
      }
      prevBtnsRef.current.a = aPressed;

      const bPressed = gp.buttons[1]?.pressed ?? false;
      if (bPressed && !prevBtnsRef.current.b) {
        sendControl({ cooling: !feedbackRef.current.cooling_on });
      }
      prevBtnsRef.current.b = bPressed;
    };

    const interval = setInterval(poll, 100);
    return () => clearInterval(interval);
  }, [sendControl]);

  const temps = Array.isArray(feedback.temperatures) ? feedback.temperatures : [];
  const spec = Array.isArray(feedback.spectrophotometer) ? feedback.spectrophotometer : [];

  return (
    <div className="sciencePage Science">
      <h1 className="Science-title">Science Payload</h1>

      <div className="Science-grid">
        {/* Camera */}
        <div className="card Science-card Science-card-camera">
          <div className="card-header Science-card-header">Camera</div>
          <div className="card-body p-0">
            <VideoFeedCard api={API_BASE} />
          </div>
        </div>

        {/* Linear Actuator */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Linear Actuator</div>
          <div className="card-body">
            <div className="Science-btn-group">
              <button className="btn btn-outline-primary" onClick={handleLinearActuator(100)}>
                Up (+100)
              </button>
              <button className="btn btn-outline-warning" onClick={handleLinearActuator(0)}>
                Stop
              </button>
              <button className="btn btn-outline-primary" onClick={handleLinearActuator(-100)}>
                Down (-100)
              </button>
            </div>
            <small className="text-muted">
              Speed: {feedback.linear_actuator_speed ?? 0}
              <span className="ms-2 text-secondary">D-pad</span>
            </small>
          </div>
        </div>

        {/* Temperatures */}
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

        {/* Heating & Cooling */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">Heating & Cooling</div>
          <div className="card-body Science-toggles">
            <button
              className={`btn ${feedback.heating_on ? "btn-danger" : "btn-outline-secondary"}`}
              onClick={handleHeating}
            >
              Heating {feedback.heating_on ? "ON" : "OFF"}
            </button>
            <button
              className={`btn ${feedback.cooling_on ? "btn-info" : "btn-outline-secondary"}`}
              onClick={handleCooling}
            >
              Cooling {feedback.cooling_on ? "ON" : "OFF"}
            </button>
            <small className="text-muted d-block mt-1">A = Heating, B = Cooling</small>
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

        {/* NIR Servo */}
        <div className="card Science-card">
          <div className="card-header Science-card-header">NIR Servo (Pin 15)</div>
          <div className="card-body">
            <p className="small text-muted mb-2">PWM sweep 0% → 60% → 0%</p>
            <button
              type="button"
              className="btn btn-primary btn-sm"
              disabled={nirDemo.status === "running"}
              onClick={runNirServoDemo}
            >
              {nirDemo.status === "running" ? "Running…" : "Run NIR Servo"}
            </button>
            {nirDemo.status !== "idle" && (
              <div
                className={`mt-2 small ${
                  nirDemo.status === "running"
                    ? "text-info"
                    : nirDemo.status === "success"
                      ? "text-success"
                      : "text-danger"
                }`}
              >
                <div className="fw-semibold">{nirDemo.message}</div>
                {nirDemo.detail && (
                  <pre className="Science-demo-detail small mt-1 mb-0">{nirDemo.detail}</pre>
                )}
              </div>
            )}
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
