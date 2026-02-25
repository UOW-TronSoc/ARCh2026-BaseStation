import React, { useEffect, useState, useRef } from "react";
import styles from "./VideoFeedCard.module.css";
import "bootstrap/dist/js/bootstrap.bundle.min.js";

export default function VideoFeedCard({ api }) {
  const [cameras, setCameras] = useState([]);
  const [selectedCamera, setSelectedCamera] = useState("");
  const [imageSrc, setImageSrc] = useState("");
  const [live, setLive] = useState(false);
  const [feedEnabled, setFeedEnabled] = useState(false);

  const intervalRef = useRef(null);

  // Fetch camera list from API on mount
  useEffect(() => {
    fetch(`${api}/cameras/`)
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

  // Update image src directly (no fetch). Same approach as CameraFeed: one request per frame
  // so the browser streams the MJPEG frame. Using fetch() here doubled requests and added latency.
  useEffect(() => {
    if (feedEnabled && selectedCamera) {
      setLive(true);
      intervalRef.current = setInterval(() => {
        setImageSrc(
          `${api}/video_feed/${encodeURIComponent(selectedCamera)}/?time=${Date.now()}`
        );
      }, 1000 / 15); // ~15 FPS to match CameraFeed and reduce load when multiple cards are shown
    } else {
      setLive(false);
      setImageSrc("");
      clearInterval(intervalRef.current);
    }

    return () => clearInterval(intervalRef.current);
  }, [api, selectedCamera, feedEnabled]);

  return (
    <div className={`card bg-transparent rounded-3 p-0`}>
      <div className="card-header p-0 position-relative border-0 bg-transparent overflow-hidden">
        {/* Video */}
        <div className="ratio ratio-16x9 overflow-hidden rounded-3">
          <img
            src={feedEnabled ? imageSrc : ""}
            alt="Live Camera Feed"
            className="w-100 h-100"
            style={{ objectFit: "cover" }}
          />
        </div>

        {/* Live Badge */}
        {live && feedEnabled && (
          <span
            className={`${styles.badge} badge bg-danger position-absolute top-0 start-0 mt-3 ms-3`}
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
            className="btn btn-sm btn-outline-warning"
            onClick={() => setFeedEnabled(prev => !prev)}
          >
            {feedEnabled ? "Stop Feed" : "Start Feed"}
          </button>
        </div>
      </div>
    </div>
  );
}
