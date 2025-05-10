import React, { useEffect, useState, useRef } from "react";
import styles from "./VideoFeedCard.module.css";
import "bootstrap/dist/js/bootstrap.bundle.min.js";

export default function VideoFeedCard({ api }) {
  const availableCameras = [0, 1, 2, 3, 4]; // 👈 Static for now
  const [cameraIndex, setCameraIndex] = useState(0);
  const [imageSrc, setImageSrc] = useState("");
  const [live, setLive] = useState(true);
  const [feedEnabled, setFeedEnabled] = useState(true);

  const intervalRef = useRef(null);

  useEffect(() => {
    if (feedEnabled) {
      intervalRef.current = setInterval(() => {
        const newSrc = `${api}/video_feed/${cameraIndex}/?time=${Date.now()}`;
        fetch(newSrc)
          .then((res) => {
            if (res.ok) {
              setImageSrc(newSrc);
              setLive(true);
            } else {
              setLive(false);
            }
          })
          .catch(() => setLive(false));
      }, 33); // ~30fps
    } else {
      setLive(false);
      clearInterval(intervalRef.current);
    }

    return () => clearInterval(intervalRef.current);
  }, [api, cameraIndex, feedEnabled]);

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
          >
            Cam {cameraIndex}
          </button>
          <ul className="dropdown-menu dropdown-menu-end" aria-labelledby="cameraDropdown">
            {availableCameras.map((id) => (
              <li key={id}>
                <button
                  className="dropdown-item"
                  onClick={() => setCameraIndex(id)}
                >
                  Cam {id}
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
