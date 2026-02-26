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

const GRID_SIZE = 6; // 3 cols x 2 rows
const GRID_COLS = 3;
const GRID_ROWS = 2;
const MIN_SPAN = 1;

/** slotIndex -> null | { cameraIndex, colSpan, rowSpan } */
const defaultGridSlots = () => Array(GRID_SIZE).fill(null);

function slotToColRow(slotIndex) {
  return { col: slotIndex % GRID_COLS, row: Math.floor(slotIndex / GRID_COLS) };
}

function isSlotCovered(slotIndex, gridSlots) {
  const { col, row } = slotToColRow(slotIndex);
  for (let s = 0; s < slotIndex; s++) {
    const item = gridSlots[s];
    if (!item) continue;
    const { col: c, row: r } = slotToColRow(s);
    const cs = item.colSpan ?? 1;
    const rs = item.rowSpan ?? 1;
    if (col >= c && col < c + cs && row >= r && row < r + rs) return true;
  }
  return false;
}

/** Returns the primary slot index that occupies cell (cellCol, cellRow), or -1 if empty. */
function getPrimaryForCell(cellCol, cellRow, gridSlots) {
  for (let s = 0; s < GRID_SIZE; s++) {
    if (isSlotCovered(s, gridSlots)) continue;
    const item = gridSlots[s];
    if (!item) continue;
    const { col, row } = slotToColRow(s);
    const cs = item.colSpan ?? 1;
    const rs = item.rowSpan ?? 1;
    if (cellCol >= col && cellCol < col + cs && cellRow >= row && cellRow < row + rs) return s;
  }
  return -1;
}

function canExpandTo(slotIndex, gridSlots, newColSpan, newRowSpan) {
  const { col, row } = slotToColRow(slotIndex);
  if (col + newColSpan > GRID_COLS || row + newRowSpan > GRID_ROWS) return false;
  for (let dc = 0; dc < newColSpan; dc++) {
    for (let dr = 0; dr < newRowSpan; dr++) {
      const primary = getPrimaryForCell(col + dc, row + dr, gridSlots);
      if (primary >= 0 && primary !== slotIndex) return false;
    }
  }
  return true;
}

const CameraFeed = () => {
  const [cameras, setCameras] = useState([]);
  const [activeCameras, setActiveCameras] = useState([]);
  const [imageSrcs, setImageSrcs] = useState([]);
  const [focusedCameras, setFocusedCameras] = useState([]);
  const [showBirdsEye, setShowBirdsEye] = useState(false);
  const [gridMode, setGridMode] = useState(false);
  const [gridSlots, setGridSlots] = useState(defaultGridSlots);

  // Fetch camera list from API
  useEffect(() => {
    fetch(`${API_BASE}/cameras/`, { credentials: 'include' })
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

  const toggleCamera = (i) => {
    const isTurningOff = activeCameras[i];
    setActiveCameras((prev) => prev.map((v, idx) => (idx === i ? !v : v)));
    if (isTurningOff) {
      setFocusedCameras((prev) => prev.filter((x) => x !== i));
    }
  };
  const toggleFocus = (i) =>
    setFocusedCameras((prev) =>
      prev.includes(i) ? prev.filter((x) => x !== i) : [...prev, i]
    );
  const sidebar = cameras
    .map((_, i) => i)
    .filter((i) => !focusedCameras.includes(i));

  const displayName = (name) =>
    name ? name.charAt(0).toUpperCase() + name.slice(1) : "";

  const handleDragStart = (e, cameraIndex) => {
    e.dataTransfer.setData("cameraIndex", String(cameraIndex));
    e.dataTransfer.effectAllowed = "move";
  };

  const handleSlotDrop = (e, slotIndex) => {
    e.preventDefault();
    if (isSlotCovered(slotIndex, gridSlots)) return;
    const cameraIndex = parseInt(e.dataTransfer.getData("cameraIndex"), 10);
    if (isNaN(cameraIndex) || cameraIndex < 0 || cameraIndex >= cameras.length) return;
    setGridSlots((prev) => {
      const next = prev.map((s) => (s && s.cameraIndex === cameraIndex ? null : s));
      next[slotIndex] = { cameraIndex, colSpan: 1, rowSpan: 1 };
      return next;
    });
    setActiveCameras((prev) => {
      const next = [...prev];
      next[cameraIndex] = true;
      return next;
    });
  };

  const handleSlotDragOver = (e) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = "move";
  };

  const clearSlot = (slotIndex) => {
    setGridSlots((prev) => {
      const next = [...prev];
      next[slotIndex] = null;
      return next;
    });
  };

  const resizeSlotRef = useRef({ slotIndex: null, startX: 0, startY: 0, startColSpan: 1, startRowSpan: 1 });

  const handleResizeStart = (e, slotIndex) => {
    e.preventDefault();
    e.stopPropagation();
    const item = gridSlots[slotIndex];
    if (!item) return;
    const slotEl = e.currentTarget.closest(".cameraGridSlot");
    const cspan = item.colSpan ?? 1;
    const rspan = item.rowSpan ?? 1;
    const cellWidth = slotEl ? slotEl.offsetWidth / cspan : 80;
    const cellHeight = slotEl ? slotEl.offsetHeight / rspan : 60;
    resizeSlotRef.current = {
      slotIndex,
      startX: e.clientX,
      startY: e.clientY,
      startColSpan: cspan,
      startRowSpan: rspan,
      cellWidth,
      cellHeight,
    };
    const onMove = (ev) => {
      const dx = ev.clientX - resizeSlotRef.current.startX;
      const dy = ev.clientY - resizeSlotRef.current.startY;
      const { col, row } = slotToColRow(slotIndex);
      const { cellWidth: cw, cellHeight: ch } = resizeSlotRef.current;
      let newColSpan = resizeSlotRef.current.startColSpan + Math.round(dx / cw);
      let newRowSpan = resizeSlotRef.current.startRowSpan + Math.round(dy / ch);
      newColSpan = Math.max(MIN_SPAN, Math.min(GRID_COLS - col, newColSpan));
      newRowSpan = Math.max(MIN_SPAN, Math.min(GRID_ROWS - row, newRowSpan));
      setGridSlots((prev) => {
        if (!canExpandTo(slotIndex, prev, newColSpan, newRowSpan)) return prev;
        const next = [...prev];
        next[slotIndex] = { ...prev[slotIndex], colSpan: newColSpan, rowSpan: newRowSpan };
        return next;
      });
    };
    const onUp = () => {
      document.removeEventListener("mousemove", onMove);
      document.removeEventListener("mouseup", onUp);
    };
    document.addEventListener("mousemove", onMove);
    document.addEventListener("mouseup", onUp);
  };

  const camerasInGrid = new Set(gridSlots.filter(Boolean).map((s) => s.cameraIndex));
  const sidebarItems = gridMode
    ? cameras.map((_, i) => i).filter((i) => !camerasInGrid.has(i))
    : sidebar;

  return (
    <div className="cameraPage">
      <div className="container-fluid px-3 main-container text-white">
      <div className="cameraPageHeader">
        <h2 className="mb-0">Live Camera Feeds</h2>
        <div className="cameraPageHeaderButtons">
          <button
            className={`btn btn-sm ${gridMode ? "btn-warning" : "btn-outline-warning"}`}
            onClick={() => setGridMode((v) => !v)}
          >
            Grid Mode
          </button>
          <button
            className="btn btn-warning btn-sm"
            onClick={() => setShowBirdsEye((v) => !v)}
          >
            {showBirdsEye ? "Exit 3D View" : "Bird's-Eye"}
          </button>
        </div>
      </div>

      {cameras.length === 0 && (
        <p className="text-center text-muted">Loading cameras...</p>
      )}

      {showBirdsEye && cameras.length > 0 ? (
        // === 3D MJPEG Ring ===
        <div className="camera3DView">
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
        // === 2D Layout: Grid mode (3x3) or focused + sidebar ===
        <div className={`cameraLayout ${!gridMode && sidebar.length === 0 ? 'cameraLayout--fullWidth' : ''}`}>
          <div className="cameraMain">
            {gridMode ? (
              <div
                className="cameraGrid3x3"
                style={{
                  gridTemplateColumns: `repeat(${GRID_COLS}, 1fr)`,
                  gridTemplateRows: `repeat(${GRID_ROWS}, 1fr)`,
                }}
              >
                {gridSlots.map((item, slotIndex) => {
                  if (isSlotCovered(slotIndex, gridSlots)) return null;
                  const { col, row } = slotToColRow(slotIndex);
                  const colSpan = item?.colSpan ?? 1;
                  const rowSpan = item?.rowSpan ?? 1;
                  const cameraIndex = item?.cameraIndex ?? null;
                  const gridStyle = {
                    gridColumn: `${col + 1} / span ${colSpan}`,
                    gridRow: `${row + 1} / span ${rowSpan}`,
                  };
                  return (
                    <div
                      key={slotIndex}
                      className={`cameraGridSlot ${cameraIndex !== null ? "cameraGridSlot--filled" : ""}`}
                      style={gridStyle}
                      onDragOver={handleSlotDragOver}
                      onDrop={(e) => handleSlotDrop(e, slotIndex)}
                    >
                      {cameraIndex !== null ? (
                        <div className="cameraGridSlotContent">
                          <div
                            className="cameraGridSlotDraggable"
                            draggable
                            onDragStart={(e) => handleDragStart(e, cameraIndex)}
                          >
                            <div className="cameraGridSlotHeader">
                              <span>{displayName(cameras[cameraIndex])}</span>
                              <button
                                className="btn btn-sm btn-outline-danger"
                                onClick={(e) => { e.stopPropagation(); clearSlot(slotIndex); }}
                                aria-label="Clear slot"
                              >
                                ×
                              </button>
                            </div>
                            {activeCameras[cameraIndex] && (
                              <div className="cameraGridSlotView">
                                <img
                                  src={imageSrcs[cameraIndex]}
                                  alt={displayName(cameras[cameraIndex])}
                                  className="cameraImg"
                                />
                              </div>
                            )}
                          </div>
                          <div
                            className="cameraGridSlotResizeHandle"
                            onMouseDown={(e) => handleResizeStart(e, slotIndex)}
                            title="Drag to resize"
                            aria-label="Resize tile"
                          />
                        </div>
                      ) : (
                        <span className="cameraGridSlotPlaceholder">Drop camera</span>
                      )}
                    </div>
                  );
                })}
              </div>
            ) : focusedCameras.length > 0 ? (
              <div className={`cameraGrid cameraGrid--${Math.min(focusedCameras.length, 4)}`}>
                {focusedCameras.map((id) => (
                  <div key={id} className="cameraTile">
                    <div className="cameraTileHeader">
                      <span className="cameraTileName">{displayName(cameras[id])}</span>
                      <button
                        className={`btn btn-sm ${activeCameras[id] ? "btn-danger" : "btn-success"}`}
                        onClick={() => toggleCamera(id)}
                      >
                        {activeCameras[id] ? "Off" : "On"}
                      </button>
                    </div>
                    {activeCameras[id] && (
                      <div className="cameraTileView" onClick={() => toggleFocus(id)}>
                        <img
                          src={imageSrcs[id]}
                          alt={displayName(cameras[id])}
                          className="cameraImg"
                        />
                      </div>
                    )}
                  </div>
                ))}
              </div>
            ) : (
              <div className="focus-placeholder">
                <p>Click a camera from the sidebar to bring it into view.</p>
              </div>
            )}
          </div>
          {(gridMode || sidebarItems.length > 0) && (
            <aside className="cameraSidebar">
              {sidebarItems.map((id) => (
                  <div
                    key={id}
                    className={`cameraSidebarItem ${gridMode ? "cameraSidebarItem--draggable" : ""}`}
                    draggable={gridMode}
                    onDragStart={gridMode ? (e) => handleDragStart(e, id) : undefined}
                  >
                    <button
                      className={`btn btn-sm w-100 mb-1 ${activeCameras[id] ? "btn-danger" : "btn-success"}`}
                      onClick={() => toggleCamera(id)}
                    >
                      {displayName(cameras[id])}: {activeCameras[id] ? "Off" : "On"}
                    </button>
                    {activeCameras[id] && (
                      <div
                        className="cameraSidebarThumb"
                        onClick={!gridMode ? () => toggleFocus(id) : undefined}
                        draggable={gridMode}
                        onDragStart={gridMode ? (e) => handleDragStart(e, id) : undefined}
                      >
                        <img src={imageSrcs[id]} alt={displayName(cameras[id])} />
                      </div>
                    )}
                  </div>
              ))}
            </aside>
          )}
        </div>
      ) : null}
      </div>
    </div>
  );
};

export default CameraFeed;
CameraFeed;
