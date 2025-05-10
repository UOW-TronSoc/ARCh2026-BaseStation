import React, { useState, useEffect } from "react";
import axios from "axios";
import "./Dashboard.css";

import VideoFeedCard from "components/VideoFeedCard/VideoFeedCard";
import DataDisplayCard from "components/DataDisplayCard/DataDisplayCard";
import DrivetrainCard from "components/DrivetrainCard/DrivetrainCard";
import SpeedControlCard from "components/SpeedControlCard/SpeedControlCard";

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
  });

  const [radio, setRadio] = useState({
    connection: "N/A",
    strength: "N/A",
    ping: "N/A",
    received: "N/A",
    sent: "N/A",
  });

  const [leftDrive, setLeftDrive] = useState(0);
  const [rightDrive, setRightDrive] = useState(0);

  const [speed, setSpeed] = useState(100);
  const [speedEnabled, setSpeedEnabled] = useState(true);

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
  /*  Game-pad polling → drivetrain commands                            */
  /* ------------------------------------------------------------------ */
  useEffect(() => {
    document.title = "Dashboard";

    const sendCommand = async (left, right) => {
      try {
        await axios.post("http://localhost:5000/command", {
          left_drive: left,
          right_drive: right,
        });
      } catch (err) {
        console.error("Failed to send drive command:", err.message);
      }
    };

    const pollGamepad = () => {
      const gp = navigator.getGamepads()[0];
      if (!gp || !speedEnabled) return;

      /* D-pad overrides */
      const b12 = gp.buttons[12]?.pressed; // up
      const b13 = gp.buttons[13]?.pressed; // down
      const b14 = gp.buttons[14]?.pressed; // left
      const b15 = gp.buttons[15]?.pressed; // right

      let left = 0;
      let right = 0;

      if (b12) {
        left = right = speed;
      } else if (b13) {
        left = right = -speed;
      } else if (b15) {
        left = speed;
        right = -speed;
      } else if (b14) {
        left = -speed;
        right = speed;
      } else {
        /* Analogue sticks (axes 2 & 3) */
        left = Math.round(gp.axes[1] * -speed);
        right = Math.round(gp.axes[3] * -speed);
      }

      if (left !== leftDrive || right !== rightDrive) {
        setLeftDrive(left);
        setRightDrive(right);
        sendCommand(left, right);
      }
    };

    const interval = setInterval(pollGamepad, 50);
    return () => clearInterval(interval);
  }, [leftDrive, rightDrive, speed, speedEnabled]);

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
            left={leftDrive}
            right={rightDrive}
          />
        </div>

        <div className="col-lg-3 mt-3">
          <SpeedControlCard
            speed={speed}
            setSpeed={setSpeed}
            enabled={speedEnabled}
            setEnabled={setSpeedEnabled}
          />
        </div>
      </div>
    </div>
  );
}
