import React, { useState, useEffect, useRef, Suspense } from "react";
import "bootstrap/dist/css/bootstrap.min.css";
import "./CameraFeed.css";

import * as THREE from "three";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";

const CAMERA_IDS = [1, 2, 0, 3]; // front, left, right, rear
/**
 * Hook that polls MJPEG URLs and updates THREE.Textures.
 */
function useMJPEGTextures(ids, fps = 10) {
  const texturesRef = useRef([]);

  useEffect(() => {
    const intervals = ids.map((id, i) => {
      // create an Image and a THREE.Texture
      const img = new Image();
      img.crossOrigin = "Anonymous";
      const tex = new THREE.Texture(img);
      tex.minFilter = THREE.LinearFilter;
      tex.magFilter = THREE.LinearFilter;
      texturesRef.current[i] = tex;

      // polling function
      const url = `http://localhost:8000/api/video_feed/${id}/`;
      const update = () => {
        img.src = `${url}?t=${Date.now()}`;
      };

      update(); // first frame
      return setInterval(update, 1000 / fps);
    });

    return () => intervals.forEach((i) => clearInterval(i));
  }, [ids, fps]);

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
 * Renders the 4 planes in a circle, each textured with its MJPEG feed.
 */
function CamerasRing() {
  const textures = useMJPEGTextures(CAMERA_IDS, 10); // 10 FPS polling

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
  const CAMERA_COUNT = 5;
  const [activeCameras, setActiveCameras] = useState(
    Array(CAMERA_COUNT).fill(false)
  );
  const [imageSrcs, setImageSrcs] = useState(
    Array(CAMERA_COUNT).fill("")
  );
  const [focusedCameras, setFocusedCameras] = useState([]);
  const [showBirdsEye, setShowBirdsEye] = useState(false);

  // 2D polling for sidebar & focused images (JPEG frames)
  useEffect(() => {
    const intervals = activeCameras.map((on, idx) => {
      if (!on) return null;
      return setInterval(() => {
        setImageSrcs((prev) => {
          const next = [...prev];
          next[idx] = `http://localhost:8000/api/video_feed/${idx}/?t=${Date.now()}`;
          return next;
        });
      }, 1000 / 15); // ~15 FPS
    });
    return () => intervals.forEach((i) => clearInterval(i));
  }, [activeCameras]);

  const toggleCamera = (i) =>
    setActiveCameras((prev) => prev.map((v, idx) => (idx === i ? !v : v)));
  const toggleFocus = (i) =>
    setFocusedCameras((prev) =>
      prev.includes(i) ? prev.filter((x) => x !== i) : [...prev, i]
    );
  const sidebar = [...Array(CAMERA_COUNT).keys()].filter(
    (i) => !focusedCameras.includes(i)
  );

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

      {showBirdsEye ? (
        // === 3D MJPEG Ring ===
        <div
          style={{ width: "100%", height: "600px", border: "2px solid white" }}
        >
          <Canvas camera={{ position: [0, 1.5, 0], fov: 75 }}>
            <ambientLight intensity={0.6} />
            <directionalLight position={[5, 10, 5]} intensity={0.5} />
            <Suspense fallback={<Html>Loading feeds...</Html>}>
              <CamerasRing />
            </Suspense>
            <OrbitControls enablePan={false} enableZoom zoomSpeed={0.6} />
          </Canvas>
        </div>
      ) : (
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
                        {activeCameras[id] ? "Turn Off" : "Turn On"} Camera{" "}
                        {id}
                      </button>
                      {activeCameras[id] && (
                        <img
                          src={imageSrcs[id]}
                          alt={`Cam ${id}`}
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
                  {activeCameras[id] ? "Turn Off" : "Turn On"} Camera {id}
                </button>
                {activeCameras[id] && (
                  <img
                    src={imageSrcs[id]}
                    alt={`Cam ${id}`}
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
      )}
    </div>
  );
};

export default CameraFeed;
