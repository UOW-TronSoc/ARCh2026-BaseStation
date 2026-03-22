import React, { useState, useEffect, useMemo, useCallback, useRef } from "react";
import axios from "axios";
import "./Dashboard.css";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import DataDisplayCard from "components/DataDisplayCard/DataDisplayCard";
import DrivetrainCard from "components/DrivetrainCard/DrivetrainCard";
import SpeedControlCard from "components/SpeedControlCard/SpeedControlCard";
import { getApiBase, getCommandUrl } from "../../config";
import { useBattery } from "context/BatteryContext";

const EPSILON = 0.01;
const MAX_TWIST = 20; // absolute range for linear/Angular components
const CONTROL_KEYS = new Set(["w", "s", "a", "d", "q", "e"]);

const ZERO_VECTOR = { x: 0, y: 0, z: 0 };

const vectorsAlmostEqual = (a, b, epsilon = EPSILON) =>
  Math.abs(a.x - b.x) < epsilon &&
  Math.abs(a.y - b.y) < epsilon &&
  Math.abs(a.z - b.z) < epsilon;

const twistAlmostEqual = (a, b, epsilon = EPSILON) =>
  vectorsAlmostEqual(a.linear, b.linear, epsilon) &&
  vectorsAlmostEqual(a.angular, b.angular, epsilon);

// Apply 30% deadzone in axis space [-1,1], with rescale after the deadzone.
const applyAxisDeadzone = (value, deadzone = 0.3) => {
  const v = typeof value === 'number' ? value : 0;
  const a = Math.abs(v);
  if (a < deadzone) return 0;
  return Math.sign(v) * (a - deadzone) / (1 - deadzone);
};

const identifyControllerType = (id = "") => {
  const lower = id.toLowerCase();
  if (lower.includes("046d") && lower.includes("c215")) return "logitech-extreme-3d";
  if (lower.includes("logitech") && lower.includes("extreme") && lower.includes("3d")) {
    return "logitech-extreme-3d";
  }
  return "generic-gamepad";
};

export default function Dashboard() {
  /* ------------------------------------------------------------------ */
  /*  Constants & initial state                                         */
  /* ------------------------------------------------------------------ */
  const NUM_CAMS = 5;

  const [camId, setCamId] = useState(0);

  const [coreFeedback, setCoreFeedback] = useState({
    epoch_time: "N/A",
    pitch: 0,
    roll: 0,
  });

  const batteryInfo = useBattery();

  /** idle | running | success | error — feedback from POST /api/servo-demo/ */
  const [servoDemo, setServoDemo] = useState({
    status: "idle",
    message: "",
    detail: "",
  });

  const [linkLatencyMs, setLinkLatencyMs] = useState(null);
  const [linkClientIp, setLinkClientIp] = useState(null);

  const [driveEnabled, setDriveEnabled] = useState(false);

  const [keyboardLinear, setKeyboardLinear] = useState({ ...ZERO_VECTOR });
  const [keyboardAngular, setKeyboardAngular] = useState({ ...ZERO_VECTOR });
  const [gamepadLinear, setGamepadLinear] = useState({ ...ZERO_VECTOR });
  const [gamepadAngular, setGamepadAngular] = useState({ ...ZERO_VECTOR });
  const [controllerInfo, setControllerInfo] = useState({
    name: "None",
    type: null,
    throttle: null,
  });

  const pressedKeysRef = useRef(new Set());

  const recalcKeyboardTwist = useCallback(() => {
    const scale = MAX_TWIST;
    const keys = pressedKeysRef.current;

    const nextLinear = { ...ZERO_VECTOR };
    const nextAngular = { ...ZERO_VECTOR };

    if (keys.has("w")) nextLinear.x += scale;
    if (keys.has("s")) nextLinear.x -= scale;
    if (keys.has("q")) nextLinear.y += scale;
    if (keys.has("e")) nextLinear.y -= scale;
    if (keys.has("a")) nextAngular.z += scale;
    if (keys.has("d")) nextAngular.z -= scale;

    setKeyboardLinear((prev) => (vectorsAlmostEqual(prev, nextLinear) ? prev : nextLinear));
    setKeyboardAngular((prev) => (vectorsAlmostEqual(prev, nextAngular) ? prev : nextAngular));
  }, []);

  const updateControllerInfo = useCallback((info) => {
    setControllerInfo((prev) => {
      if (
        prev.name === info.name &&
        prev.type === info.type &&
        (prev.throttle ?? null) === (info.throttle ?? null)
      ) {
        return prev;
      }
      return info;
    });
  }, []);

  /* ------------------------------------------------------------------ */
  /*  REST fetchers (link-latency) — battery via BatteryContext         */
  /* ------------------------------------------------------------------ */
  const runServoDemo = useCallback(async () => {
    setServoDemo({
      status: "running",
      message: "Running servo routine on rover…",
      detail: "",
    });
    try {
      const { data } = await axios.post(`${getApiBase()}/servo-demo/`, {});
      if (data.ok) {
        setServoDemo({
          status: "success",
          message: data.message || "Servo routine finished.",
          detail: (data.stdout || "").trim(),
        });
      } else {
        setServoDemo({
          status: "error",
          message: data.message || data.error || "Servo script reported failure.",
          detail: [data.stderr, data.stdout].filter(Boolean).join("\n\n").trim(),
        });
      }
    } catch (err) {
      const d = err.response?.data;
      const msg =
        d?.error ||
        d?.message ||
        err.message ||
        "Could not run servo demo.";
      const detail = [d?.stderr, d?.stdout].filter(Boolean).join("\n\n").trim();
      setServoDemo({ status: "error", message: msg, detail });
    }
  }, []);

  const fetchLinkLatency = async () => {
    try {
      const t0 = performance.now();
      const { data } = await axios.get(`${getApiBase()}/link-latency/`);
      const t1 = performance.now();
      const rttMs = Math.round(t1 - t0);
      setLinkLatencyMs(rttMs);
      if (data.client_ip) setLinkClientIp(data.client_ip);
    } catch (err) {
      setLinkLatencyMs(null);
      console.error("Failed to measure link latency:", err.message);
    }
  };

  const LINK_LATENCY_MS = 3000;   // ~0.33 Hz — antenna RTT (when enabled)

  useEffect(() => {
    const handleKeyDown = (event) => {
      const key = event.key.toLowerCase();
      if (!CONTROL_KEYS.has(key)) return;
      event.preventDefault();
      const keys = pressedKeysRef.current;
      if (!keys.has(key)) {
        keys.add(key);
        recalcKeyboardTwist();
      }
    };

    const handleKeyUp = (event) => {
      const key = event.key.toLowerCase();
      if (!CONTROL_KEYS.has(key)) return;
      event.preventDefault();
      const keys = pressedKeysRef.current;
      if (keys.delete(key)) {
        recalcKeyboardTwist();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [recalcKeyboardTwist]);

  /* Battery: shared via BatteryContext (single poll for navbar + dashboard) */

  /* Link latency (antenna RTT): ~0.33 Hz */
  useEffect(() => {
    fetchLinkLatency();
    const latencyTimer = setInterval(fetchLinkLatency, LINK_LATENCY_MS);
    return () => clearInterval(latencyTimer);
  }, []);

  /* ------------------------------------------------------------------ */
  /*  Gamepad + keyboard input → Twist commands                         */
  /* ------------------------------------------------------------------ */

  useEffect(() => {
    document.title = "Dashboard";
  }, []);

  const sendTwistCommand = useCallback(async (payload) => {
    try {
      await axios.post(getCommandUrl(), payload);
    } catch (err) {
      console.error("Failed to send drive command:", err.message);
    }
  }, []);

  useEffect(() => {
    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp) {
        setGamepadLinear((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        setGamepadAngular((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        updateControllerInfo({ name: "None", type: null, throttle: null });
        return;
      }

      const controllerType = identifyControllerType(gp.id);

      if (!driveEnabled) {
        updateControllerInfo({ name: gp.id || "Unknown Controller", type: controllerType, throttle: 0 });
        setGamepadLinear((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        setGamepadAngular((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        return;
      }

      const baseScale = MAX_TWIST;

      let throttle = 1;
      let nextLinear = { ...ZERO_VECTOR };
      let nextAngular = { ...ZERO_VECTOR };

      if (controllerType === "logitech-extreme-3d") {
        const throttleAxis = gp.axes[3] ?? -1; // [-1,1], forward ≈ -1, back ≈ 1
        const normalized = Math.min(Math.max((1 - throttleAxis) / 2, 0), 1); // 0..1
        throttle = normalized;
        const scale = baseScale * throttle;

        const axisLX = applyAxisDeadzone(-(gp.axes[1] ?? 0), 0.3); // forward/back
        const axisLY = applyAxisDeadzone( (gp.axes[0] ?? 0), 0.3); // strafe
        const axisAZ = applyAxisDeadzone( (gp.axes[2] ?? 0), 0.3); // twist

        nextLinear = {
          x: axisLX * scale,
          y: axisLY * scale,
          z: 0,
        };

        nextAngular = {
          x: 0,
          y: 0,
          z: axisAZ * scale,
        };
      } else {
        const scale = baseScale;
        const axisLX2 = applyAxisDeadzone(-(gp.axes[1] ?? 0), 0.3);
        const axisLY2 = applyAxisDeadzone( (gp.axes[0] ?? 0), 0.3);
        const axisAZ2 = applyAxisDeadzone(-(gp.axes[2] ?? gp.axes[3] ?? 0), 0.3);

        nextLinear = {
          x: axisLX2 * scale,
          y: axisLY2 * scale,
          z: 0,
        };
        nextAngular = {
          x: 0,
          y: 0,
          z: axisAZ2 * scale,
        };
      }

      updateControllerInfo({
        name: gp.id || "Unknown Controller",
        type: controllerType,
        throttle,
      });

      setGamepadLinear((prev) => (vectorsAlmostEqual(prev, nextLinear) ? prev : nextLinear));
      setGamepadAngular((prev) => (vectorsAlmostEqual(prev, nextAngular) ? prev : nextAngular));
    };

    const interval = setInterval(pollGamepad, 50);
    return () => clearInterval(interval);
  }, [driveEnabled, updateControllerInfo]);

  const combinedLinear = useMemo(
    () => ({
      x: keyboardLinear.x + gamepadLinear.x,
      y: keyboardLinear.y + gamepadLinear.y,
      z: keyboardLinear.z + gamepadLinear.z,
    }),
    [keyboardLinear, gamepadLinear]
  );

  const combinedAngular = useMemo(
    () => ({
      x: keyboardAngular.x + gamepadAngular.x,
      y: keyboardAngular.y + gamepadAngular.y,
      z: keyboardAngular.z + gamepadAngular.z,
    }),
    [keyboardAngular, gamepadAngular]
  );

  const effectiveTwist = useMemo(
    () => (
      driveEnabled
        ? {
            linear: { ...combinedLinear },
            angular: { ...combinedAngular },
          }
        : {
            linear: { ...ZERO_VECTOR },
            angular: { ...ZERO_VECTOR },
          }
    ),
    [combinedLinear, combinedAngular, driveEnabled]
  );

  // Send drivetrain commands at ~60 Hz only while drive is enabled (no backend traffic when off)
  useEffect(() => {
    if (!driveEnabled) return undefined;

    const sendInterval = setInterval(() => {
      const payload = {
        linear: { ...effectiveTwist.linear },
        angular: { ...effectiveTwist.angular },
      };

      sendTwistCommand(payload);
    }, 16);

    return () => clearInterval(sendInterval);
  }, [driveEnabled, effectiveTwist, sendTwistCommand]);

  /* ---------------------------------------------------------------------------------- */
  /* (VideoFeedCard, DataDisplayCard, DrivetrainCard, SpeedControlCard + servo) */
  /* ---------------------------------------------------------------------------------- */
  return (
    <div className="dashboardPage">
      <div className="container-fluid px-3">
        {/* ────────────────────── ROW 1 ────────────────────── */}
        <div className="row dashboardRow1 gx-2 gy-1">
          <div className="col-lg-8">
            <VideoFeedCard
              api={getApiBase()}
              camId={camId}
              setCamId={setCamId}
              showDropdown
            />
          </div>
          <div className="col-lg-4">
            <DataDisplayCard
              battery={batteryInfo}
              pitch={coreFeedback.pitch}
              roll={coreFeedback.roll}
              linkLatencyMs={linkLatencyMs}
              linkClientIp={linkClientIp}
            />
          </div>
        </div>

        {/* ────────────────────── ROW 2 ────────────────────── */}
        <div className="row dashboardRow2 gx-2 gy-1 mt-1 pb-2">
          <div className="col-lg-6">
            <VideoFeedCard
              api={getApiBase()}
              camId={(camId + 1) % NUM_CAMS}
              setCamId={() => {}}
              showDropdown={false}
            />
          </div>
          <div className="col-lg-3">
            <DrivetrainCard
              timestamp={coreFeedback.epoch_time}
              linear={effectiveTwist.linear}
              angular={effectiveTwist.angular}
            />
          </div>
          <div className="col-lg-3">
            <SpeedControlCard
              enabled={driveEnabled}
              setEnabled={setDriveEnabled}
              controllerInfo={controllerInfo}
            >
              <div className="servoDemoCardInner">
                <div className="header small text-uppercase mb-2">Servo Activation</div>
                <p className="small text-secondary mb-2 mb-lg-3">
                  <code className="small">Takes several seconds.</code>
                </p>
                <button
                  type="button"
                  className="btn btn-primary btn-sm"
                  disabled={servoDemo.status === "running"}
                  onClick={runServoDemo}
                >
                  {servoDemo.status === "running" ? "Running…" : "Roo release"}
                </button>
                {servoDemo.status !== "idle" && (
                  <div
                    className={`servoDemoFeedback mt-2 small ${
                      servoDemo.status === "running"
                        ? "text-info"
                        : servoDemo.status === "success"
                          ? "text-success"
                          : "text-danger"
                    }`}
                    role="status"
                  >
                    <div className="fw-semibold">{servoDemo.message}</div>
                    {servoDemo.detail ? (
                      <pre className="servoDemoDetail small mt-1 mb-0">{servoDemo.detail}</pre>
                    ) : null}
                  </div>
                )}
              </div>
            </SpeedControlCard>
          </div>
        </div>
      </div>
    </div>
  );
}
