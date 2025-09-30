import React, { useState, useEffect, useMemo, useCallback, useRef } from "react";
import axios from "axios";
import "./Dashboard.css";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import DataDisplayCard from "components/DataDisplayCard/DataDisplayCard";
import DrivetrainCard from "components/DrivetrainCard/DrivetrainCard";
import SpeedControlCard from "components/SpeedControlCard/SpeedControlCard";

const EPSILON = 0.01;
const MAX_TWIST = 15; // absolute range for linear/Angular components
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
  const API_BASE = "http://127.0.0.1:8000/api";
  const NUM_CAMS = 5;

  const [camId, setCamId] = useState(0);

  const [coreFeedback, setCoreFeedback] = useState({
    epoch_time: "N/A",
    pitch: 0,
    roll: 0,
  });

  const [batteryInfo, setBatteryInfo] = useState({
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
  });

  const [radio, setRadio] = useState({
    connection: "N/A",
    strength: "N/A",
    ping: "N/A",
    received: "N/A",
    sent: "N/A",
  });

  const [speed, setSpeed] = useState(100);
  const [speedEnabled, setSpeedEnabled] = useState(true);

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
  const speedRef = useRef(speed);
  const lastSentTwistRef = useRef({
    linear: { ...ZERO_VECTOR },
    angular: { ...ZERO_VECTOR },
  });

  const recalcKeyboardTwist = useCallback(() => {
    const scale = (speedRef.current / 100) * MAX_TWIST;
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
  /*  REST fetchers (core, battery, radio)                              */
  /* ------------------------------------------------------------------ */
  const fetchCoreFeedback = async () => {
    try {
      const { data } = await axios.get(`${API_BASE}/core-feedback/`);
      setCoreFeedback((prev) => ({ ...prev, ...data }));
    } catch (err) {
      console.error("Failed to fetch core feedback:", err.message);
    }
  };

  const fetchBattery = async () => {
    try {
      const { data } = await axios.get(`${API_BASE}/battery-feedback/`);
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


  const fetchRadio = async () => {
    try {
      const { data } = await axios.get(`${API_BASE}/radio-feedback/`);
      setRadio({
        connection: data.connection ?? "N/A",
        strength: data.strength ?? "N/A",
        ping: data.ping ?? "N/A",
        received: data.received ?? "N/A",
        sent: data.sent ?? "N/A",
      });
    } catch (err) {
      console.error("Failed to fetch radio status:", err.message);
    }
  };

  const refresh_rate = 500; //ms

  useEffect(() => {
    speedRef.current = speed;
    recalcKeyboardTwist();
  }, [speed, recalcKeyboardTwist]);

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

  /* Poll every 2 s */
  useEffect(() => {
    fetchCoreFeedback();
    fetchBattery();
    fetchRadio();
    const timer = setInterval(() => {
      fetchCoreFeedback();
      fetchBattery();

      
      fetchRadio();
    }, refresh_rate);
    return () => clearInterval(timer);
  }, []);

  /* ------------------------------------------------------------------ */
  /*  Gamepad + keyboard input → Twist commands                         */
  /* ------------------------------------------------------------------ */

  useEffect(() => {
    document.title = "Dashboard";
  }, []);

  const sendTwistCommand = useCallback(async (payload) => {
    try {
      await axios.post("http://localhost:8080/command", payload);
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

      if (!speedEnabled) {
        updateControllerInfo({ name: gp.id || "Unknown Controller", type: controllerType, throttle: 0 });
        setGamepadLinear((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        setGamepadAngular((prev) => (vectorsAlmostEqual(prev, ZERO_VECTOR) ? prev : { ...ZERO_VECTOR }));
        return;
      }

      const baseScale = (speedRef.current / 100) * MAX_TWIST;

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
        const axisAZ = applyAxisDeadzone(-(gp.axes[2] ?? 0), 0.3); // twist

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
        const axisAZ2 = applyAxisDeadzone( (gp.axes[2] ?? gp.axes[3] ?? 0), 0.3);

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
  }, [speedEnabled, updateControllerInfo]);

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
      speedEnabled
        ? {
            linear: { ...combinedLinear },
            angular: { ...combinedAngular },
          }
        : {
            linear: { ...ZERO_VECTOR },
            angular: { ...ZERO_VECTOR },
          }
    ),
    [combinedLinear, combinedAngular, speedEnabled]
  );

  useEffect(() => {
    const payload = {
      linear: { ...effectiveTwist.linear },
      angular: { ...effectiveTwist.angular },
    };

    if (twistAlmostEqual(payload, lastSentTwistRef.current)) {
      return;
    }

    lastSentTwistRef.current = {
      linear: { ...payload.linear },
      angular: { ...payload.angular },
    };

    sendTwistCommand(payload);
  }, [effectiveTwist, sendTwistCommand]);

  /* ---------------------------------------------------------------------------------- */
  /* (VideoFeedCard, DataDisplayCard, VideoFeedCard, DrivetrainCard, SpeedControlCard ) */
  /* ---------------------------------------------------------------------------------- */
  return (
    <div className="container my-4">
      {/* ────────────────────── ROW 1 ────────────────────── */}
      <div className="row gx-4">
        <div className="col-lg-8">
          <VideoFeedCard
            api={API_BASE}
            camId={camId}
            setCamId={setCamId}
            showDropdown
          />
        </div>
        <div className="col-lg-4 mt-3 mt-md-0 mt-lg-0">
          <DataDisplayCard
            radio={radio}
            battery={batteryInfo}
            pitch={coreFeedback.pitch}
            roll={coreFeedback.roll}
          />
        </div>
      </div>

      <div className="mt-2" />

      {/* ────────────────────── ROW 2 ────────────────────── */}
      <div className="row gx-4 pb-5">
        <div className="col-lg-6 mt-3">
          {/* second camera: next ID, no dropdown */}
          <VideoFeedCard
            api={API_BASE}
            camId={(camId + 1) % NUM_CAMS}
            setCamId={() => {}}
            showDropdown={false}
          />
        </div>

        <div className="col-lg-3 mt-3">
          <DrivetrainCard
            timestamp={coreFeedback.epoch_time}
            linear={effectiveTwist.linear}
            angular={effectiveTwist.angular}
          />
        </div>

        <div className="col-lg-3 mt-3">
          <SpeedControlCard
            speed={speed}
            setSpeed={setSpeed}
            enabled={speedEnabled}
            setEnabled={setSpeedEnabled}
            controllerInfo={controllerInfo}
          />
        </div>
      </div>
    </div>
  );
}
