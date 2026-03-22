import React, { useEffect, useState, useRef, useCallback, memo } from "react";
import styles from "./VideoFeedCard.module.css";
import "bootstrap/dist/js/bootstrap.bundle.min.js";

const makeFrameUrl = (api, cameraName, t) =>
  `${api}/video_feed/${encodeURIComponent(cameraName)}/?single=1&t=${t}`;

const VideoFeedCard = ({ api }) => {
  const [cameras, setCameras] = useState([]);
  const [selectedCamera, setSelectedCamera] = useState("");
  const [live, setLive] = useState(false);
  const [feedEnabled, setFeedEnabled] = useState(false);
  const [displaySrc, setDisplaySrc] = useState("");

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

  // onLoad-driven chain: request next frame only when current loads (avoids aborting requests)
  const requestNextFrame = useCallback(() => {
    if (feedEnabled && selectedCamera) {
      setDisplaySrc(makeFrameUrl(api, selectedCamera, Date.now()));
    }
  }, [api, selectedCamera, feedEnabled]);

  useEffect(() => {
    if (feedEnabled && selectedCamera) {
      setLive(true);
      setDisplaySrc(makeFrameUrl(api, selectedCamera, Date.now()));
    } else {
      setLive(false);
      setDisplaySrc("");
    }
  }, [api, selectedCamera, feedEnabled]);

  // Realtime FPS logging
  const fpsRef = useRef({ lastTs: 0, deltas: [], logTs: 0 });
  useEffect(() => {
    if (!feedEnabled) fpsRef.current = { lastTs: 0, deltas: [], logTs: 0 };
  }, [feedEnabled]);

  const onFrameLoad = useCallback(() => {
    const now = performance.now();
    const { lastTs, deltas, logTs } = fpsRef.current;
    if (lastTs > 0) {
      deltas.push(1000 / (now - lastTs));
      if (deltas.length > 10) deltas.shift();
      const fps = deltas.reduce((a, b) => a + b, 0) / deltas.length;
      if (now - logTs >= 1000) {
        console.log(`[VideoFeedCard] ${selectedCamera} FPS: ${fps.toFixed(1)}`);
        fpsRef.current.logTs = now;
      }
    }
    fpsRef.current.lastTs = now;
    requestNextFrame();
  }, [selectedCamera, requestNextFrame]);

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
            style={{ objectFit: "cover" }}
            onLoad={feedEnabled ? onFrameLoad : undefined}
            onError={feedEnabled ? onFrameError : undefined}
          />
        </div>

        {/* Live Badge */}
        {live && feedEnabled && (
          <span
            className={`${styles.badge} badge bg-warning text-dark position-absolute top-0 start-0 mt-3 ms-3`}
          >
            Live
          </span>
        )}

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

        {/* Start/Stop Feed Button */}
        <div className="position-absolute bottom-0 start-0 mb-3 ms-3">
          <button
            className="btn btn-sm btn-outline-light"
            onClick={() => setFeedEnabled(prev => !prev)}
          >
            {feedEnabled ? "Stop Feed" : "Start Feed"}
          </button>
        </div>
      </div>
    </div>
  );
};

export default memo(VideoFeedCard);
