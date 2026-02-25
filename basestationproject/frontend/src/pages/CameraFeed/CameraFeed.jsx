import React, { useState, useEffect, useRef, Suspense } from "react";
import "bootstrap/dist/css/bootstrap.min.css";
import "./CameraFeed.css";

import * as THREE from "three";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import { getApiBase } from "../../config";

const API_BASE = import.meta.env.VITE_API_URL || getApiBase();

/**
 * Hook that polls MJPEG URLs and updates THREE.Textures.
 * @param {string[]} cameraNames - List of camera names (e.g. ["top", "back", "front", "left"])
 */
function useMJPEGTextures(cameraNames, fps = 10) {
  const texturesRef = useRef([]);

  useEffect(() => {
    if (!cameraNames.length) return;
    const intervals = cameraNames.map((name, i) => {
      const img = new Image();
      img.crossOrigin = "Anonymous";
      const tex = new THREE.Texture(img);
      tex.minFilter = THREE.LinearFilter;
      tex.magFilter = THREE.LinearFilter;
      texturesRef.current[i] = tex;

      const url = `${API_BASE}/video_feed/${encodeURIComponent(name)}/`;
      const update = () => {
        img.src = `${url}?t=${Date.now()}`;
      };

      update();
      return setInterval(update, 1000 / fps);
    });

    return () => intervals.forEach((i) => clearInterval(i));
  }, [cameraNames.join(","), fps]);

  // on each render frame, mark textures needing update
  useFrame(() => {
    texturesRef.current.forEach((tex) => {
      if (tex && tex.image && tex.image.complete) {
        tex.needsUpdate = true;
      }
    });
  });

  return texturesRef.current;
}

/**
 * Renders planes in a circle, each textured with its MJPEG feed.
 * Uses first 4 cameras from the list for the ring.
 */
function CamerasRing({ cameraNames }) {
  const ringCameras = cameraNames.slice(0, 4);
  const textures = useMJPEGTextures(ringCameras, 10);

  return (
    <group>
      {textures.map((tex, i) => {
        const angle = (i / textures.length) * Math.PI * 2;
        const radius = 3;
        const x = Math.sin(angle) * radius;
        const z = Math.cos(angle) * radius;
        const rotY = -angle + Math.PI;

        return (
          <mesh key={i} position={[x, 1.5, z]} rotation={[0, rotY, 0]}>
            <planeGeometry args={[3, 3]} />
            <meshBasicMaterial
              map={tex}
              toneMapped={false}
              side={THREE.DoubleSide}
            />
          </mesh>
        );
      })}

      {/* optional floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[20, 20]} />
        <meshStandardMaterial color="#222" />
      </mesh>
    </group>
  );
}

const CameraFeed = () => {
  const [cameras, setCameras] = useState([]);
  const [activeCameras, setActiveCameras] = useState([]);
  const [imageSrcs, setImageSrcs] = useState([]);
  const [focusedCameras, setFocusedCameras] = useState([]);
  const [showBirdsEye, setShowBirdsEye] = useState(false);

  // Fetch camera list from API
  useEffect(() => {
    fetch(`${API_BASE}/cameras/`)
      .then((res) => (res.ok ? res.json() : Promise.reject()))
      .then((data) => {
        const list = data.cameras || [];
        setCameras(list);
        setActiveCameras(Array(list.length).fill(false));
        setImageSrcs(Array(list.length).fill(""));
      })
      .catch(() => setCameras([]));
  }, []);

  // 2D polling for sidebar & focused images (JPEG frames)
  useEffect(() => {
    if (!cameras.length) return;
    const intervals = activeCameras.map((on, idx) => {
      if (!on) return null;
      const name = cameras[idx];
      return setInterval(() => {
        setImageSrcs((prev) => {
          const next = [...prev];
          next[idx] = `${API_BASE}/video_feed/${encodeURIComponent(name)}/?t=${Date.now()}`;
          return next;
        });
      }, 1000 / 15); // ~15 FPS
    });
    return () => intervals.forEach((i) => i != null && clearInterval(i));
  }, [cameras, activeCameras]);

  const toggleCamera = (i) =>
    setActiveCameras((prev) => prev.map((v, idx) => (idx === i ? !v : v)));
  const toggleFocus = (i) =>
    setFocusedCameras((prev) =>
      prev.includes(i) ? prev.filter((x) => x !== i) : [...prev, i]
    );
  const sidebar = cameras
    .map((_, i) => i)
    .filter((i) => !focusedCameras.includes(i));

  const displayName = (name) =>
    name ? name.charAt(0).toUpperCase() + name.slice(1) : "";

  return (
    <div className="container-fluid mt-4 main-container text-white">
      <h2 className="text-center mb-4">Live Camera Feeds</h2>

      <div className="text-center mb-4">
        <button
          className="btn btn-warning"
          onClick={() => setShowBirdsEye((v) => !v)}
        >
          {showBirdsEye ? "Exit 3D View" : "Show Bird's-Eye View"}
        </button>
      </div>

      {cameras.length === 0 && (
        <p className="text-center text-muted">Loading cameras...</p>
      )}

      {showBirdsEye && cameras.length > 0 ? (
        // === 3D MJPEG Ring ===
        <div
          style={{ width: "100%", height: "600px", border: "2px solid white" }}
        >
          <Canvas camera={{ position: [0, 1.5, 0], fov: 75 }}>
            <ambientLight intensity={0.6} />
            <directionalLight position={[5, 10, 5]} intensity={0.5} />
            <Suspense fallback={<Html>Loading feeds...</Html>}>
              <CamerasRing cameraNames={cameras} />
            </Suspense>
            <OrbitControls enablePan={false} enableZoom zoomSpeed={0.6} />
          </Canvas>
        </div>
      ) : cameras.length > 0 ? (
        // === 2D Polling Layout ===
        <div className="row">
          <div className="col-md-9">
            <div className="row">
              {focusedCameras.length > 0 ? (
                focusedCameras.map((id) => {
                  const col = focusedCameras.length === 1 ? 12 : 6;
                  return (
                    <div key={id} className={`col-md-${col} mb-4`}>
                      <button
                        className={`btn ${
                          activeCameras[id] ? "btn-danger" : "btn-success"
                        } mb-2 w-100`}
                        onClick={() => toggleCamera(id)}
                      >
                        {activeCameras[id] ? "Turn Off" : "Turn On"}{" "}
                        {displayName(cameras[id])}
                      </button>
                      {activeCameras[id] && (
                        <img
                          src={imageSrcs[id]}
                          alt={displayName(cameras[id])}
                          className="img-fluid border rounded w-100"
                          style={{
                            maxHeight: "70vh",
                            objectFit: "contain",
                            cursor: "pointer",
                          }}
                          onClick={() => toggleFocus(id)}
                        />
                      )}
                    </div>
                  );
                })
              ) : (
                <div className="focus-placeholder w-100">
                  Click a camera from the right to bring it into view.
                </div>
              )}
            </div>
          </div>
          <div className="col-md-3">
            {sidebar.map((id) => (
              <div key={id} className="mb-4">
                <button
                  className={`btn btn-sm ${
                    activeCameras[id] ? "btn-danger" : "btn-success"
                  } mb-1 w-100`}
                  onClick={() => toggleCamera(id)}
                >
                  {activeCameras[id] ? "Turn Off" : "Turn On"}{" "}
                  {displayName(cameras[id])}
                </button>
                {activeCameras[id] && (
                  <img
                    src={imageSrcs[id]}
                    alt={displayName(cameras[id])}
                    className="img-fluid border rounded"
                    style={{
                      cursor: "pointer",
                      maxHeight: "150px",
                      objectFit: "cover",
                    }}
                    onClick={() => toggleFocus(id)}
                  />
                )}
              </div>
            ))}
          </div>
        </div>
      ) : null}
    </div>
  );
};

export default CameraFeed;
