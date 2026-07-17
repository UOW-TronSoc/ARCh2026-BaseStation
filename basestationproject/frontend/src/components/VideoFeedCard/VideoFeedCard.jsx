import React, { useEffect, useState, useCallback, memo } from "react";
import styles from "./VideoFeedCard.module.css";
import "bootstrap/dist/js/bootstrap.bundle.min.js";

const makeFrameUrl = (api, cameraName, preset, t) =>
  `${api}/video_feed/${encodeURIComponent(cameraName)}/?single=1&preset=${preset}&t=${t}`;

const VideoFeedCard = ({ api }) => {
  const [cameras, setCameras] = useState([]);
  const [selectedCamera, setSelectedCamera] = useState("");
  const [live, setLive] = useState(false);
  const [feedEnabled, setFeedEnabled] = useState(false);
  const [displaySrc, setDisplaySrc] = useState("");
  const [rotation, setRotation] = useState(0);
  const rotateCW = () => setRotation((prev) => (prev + 90) % 360);
  const [preset, setPreset] = useState("normal");
  const cyclePreset = () => setPreset((p) => (p === "normal" ? "potato" : "normal"));

  // Fetch camera list from API on mount
  useEffect(() => {
    fetch(`${api}/cameras/`, { credentials: 'include' })
      .then((res) => res.ok ? res.json() : Promise.reject())
      .then((data) => {
        const list = data.cameras || [];
        setCameras(list);
        if (list.length > 0 && !selectedCamera) {
          setSelectedCamera(list[0]);
        }
      })
      .catch(() => setCameras([]));
  }, [api]);

  // When cameras load, default selection to first (or reset if current selection not in list)
  useEffect(() => {
    if (cameras.length > 0 && !cameras.includes(selectedCamera)) {
      setSelectedCamera(cameras[0]);
    }
  }, [cameras, selectedCamera]);

  const requestNextFrame = useCallback(() => {
    if (feedEnabled && selectedCamera) {
      setDisplaySrc(makeFrameUrl(api, selectedCamera, preset, Date.now()));
    }
  }, [api, selectedCamera, feedEnabled, preset]);

  useEffect(() => {
    if (feedEnabled && selectedCamera) {
      setLive(true);
      setDisplaySrc(makeFrameUrl(api, selectedCamera, preset, Date.now()));
    } else {
      setLive(false);
      setDisplaySrc("");
    }
  }, [api, selectedCamera, feedEnabled, preset]);

  const onFrameLoad = useCallback(() => {
    requestNextFrame();
  }, [requestNextFrame]);

  const onFrameError = useCallback(() => {
    requestNextFrame();
  }, [requestNextFrame]);

  return (
    <div className={`card bg-transparent rounded-3 p-0 ${styles.videoFeedCard}`}>
      <div className="card-header p-0 position-relative border-0 bg-transparent">
        {/* Video */}
        <div className="ratio ratio-16x9 overflow-hidden rounded-3">
          <img
            src={displaySrc || undefined}
            alt="Live Camera Feed"
            className="w-100 h-100"
            style={{ objectFit: "cover", transform: `rotate(${rotation}deg)` }}
            onLoad={feedEnabled ? onFrameLoad : undefined}
            onError={feedEnabled ? onFrameError : undefined}
          />
        </div>

        {/* Top-left: Live badge + rotation control */}
        <div className="position-absolute top-0 start-0 mt-3 ms-3 d-flex flex-column gap-1" style={{ zIndex: 2 }}>
          {live && feedEnabled && (
            <span className={`${styles.badge} badge bg-warning text-dark`}>
              Live
            </span>
          )}
          <button
            className={`btn btn-sm btn-outline-light ${styles.rotateBtn}`}
            onClick={rotateCW}
            title="Rotate 90° clockwise"
          >
            ↻
          </button>
        </div>

        {/* Camera Selector Dropdown */}
        <div className="position-absolute top-0 end-0 mt-3 me-3 dropdown">
          <button
            className="btn btn-sm btn-outline-light dropdown-toggle"
            type="button"
            id="cameraDropdown"
            data-bs-toggle="dropdown"
            aria-expanded="false"
            disabled={cameras.length === 0}
          >
            {selectedCamera ? selectedCamera.charAt(0).toUpperCase() + selectedCamera.slice(1) : "Camera"}
          </button>
          <ul className="dropdown-menu dropdown-menu-end" aria-labelledby="cameraDropdown">
            {cameras.map((name) => (
              <li key={name}>
                <button
                  className="dropdown-item"
                  onClick={() => setSelectedCamera(name)}
                >
                  {name.charAt(0).toUpperCase() + name.slice(1)}
                </button>
              </li>
            ))}
          </ul>
        </div>

        {/* Start/Stop Feed + Quality preset */}
        <div className="position-absolute bottom-0 start-0 mb-3 ms-3 d-flex gap-1">
          <button
            className="btn btn-sm btn-outline-light"
            onClick={() => setFeedEnabled(prev => !prev)}
          >
            {feedEnabled ? "Stop Feed" : "Start Feed"}
          </button>
          {feedEnabled && (
            <button
              className={`btn btn-sm ${preset === "potato" ? "btn-danger" : "btn-outline-info"}`}
              onClick={cyclePreset}
              title={preset === "potato" ? "Potato (240p)" : "Normal (640p)"}
            >
              {preset === "potato" ? "🥔" : "📷"}
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

export default memo(VideoFeedCard);
