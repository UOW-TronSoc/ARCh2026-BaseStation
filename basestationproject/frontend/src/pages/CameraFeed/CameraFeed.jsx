import React, { useState, useEffect, useRef, useCallback, Suspense } from "react";
import "bootstrap/dist/css/bootstrap.min.css";
import "./CameraFeed.css";

import * as THREE from "three";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import { getApiBase } from "../../config";

const API_BASE = import.meta.env.VITE_API_URL || getApiBase();

/**
 * Hook that binds MJPEG stream URLs to THREE.Textures.
 * Uses native MJPEG streaming (one connection per camera, browser updates image continuously).
 */
function useMJPEGTextures(cameraNames, cameraPresets) {
  const texturesRef = useRef([]);

  useEffect(() => {
    if (!cameraNames.length) return;
    cameraNames.forEach((name, i) => {
      const preset = cameraPresets[i] ?? "normal";
      const img = new Image();
      img.crossOrigin = "Anonymous";
      const tex = new THREE.Texture(img);
      tex.minFilter = THREE.LinearFilter;
      tex.magFilter = THREE.LinearFilter;
      texturesRef.current[i] = tex;
      img.src = `${API_BASE}/video_feed/${encodeURIComponent(name)}/?preset=${preset}`;
    });
    return () => {
      texturesRef.current.forEach((tex) => tex?.image && (tex.image.src = ""));
    };
  }, [cameraNames.join(","), cameraPresets.slice(0, cameraNames.length).join(",")]);

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
function CamerasRing({ cameraNames, cameraPresets }) {
  const ringCameras = cameraNames.slice(0, 4);
  const ringPresets = cameraPresets.slice(0, ringCameras.length);
  const textures = useMJPEGTextures(ringCameras, ringPresets);

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

/** Backend query values; UI labels are HD / TS */
const PRESET_HD = "normal";
const PRESET_TS = "potato";

const LAYOUT_PRESET_STORAGE_KEY = "cameraFeed.layoutPresets.v1";
const NUM_LAYOUT_PRESETS = 5;

function loadAllLayoutPresets() {
  try {
    const raw = localStorage.getItem(LAYOUT_PRESET_STORAGE_KEY);
    if (!raw) return Array(NUM_LAYOUT_PRESETS).fill(null);
    const parsed = JSON.parse(raw);
    return Array.from({ length: NUM_LAYOUT_PRESETS }, (_, i) => parsed[i] ?? null);
  } catch {
    return Array(NUM_LAYOUT_PRESETS).fill(null);
  }
}

function saveAllLayoutPresets(arr) {
  try {
    const obj = {};
    arr.forEach((v, i) => {
      if (v != null) obj[i] = v;
    });
    localStorage.setItem(LAYOUT_PRESET_STORAGE_KEY, JSON.stringify(obj));
  } catch {
    /* ignore quota */
  }
}

function buildLayoutSnapshot(
  cameras,
  gridSlots,
  rotations,
  cameraPresets,
  activeCameras,
  gridMode
) {
  return {
    v: 1,
    cameraOrder: [...cameras],
    grid: gridSlots.map((item) => {
      if (!item) return null;
      const name = cameras[item.cameraIndex];
      if (!name) return null;
      return {
        cameraName: name,
        colSpan: item.colSpan ?? 1,
        rowSpan: item.rowSpan ?? 1,
      };
    }),
    rotationsByName: Object.fromEntries(
      cameras.map((n, i) => [n, rotations[i] ?? 0]).filter(([, deg]) => deg % 360 !== 0)
    ),
    cameraPresets: [...cameraPresets],
    activeCameras: [...activeCameras],
    gridMode: !!gridMode,
  };
}

function dedupeGridSlots(gridSlots) {
  const seen = new Set();
  const out = [...gridSlots];
  for (let i = 0; i < out.length; i++) {
    const it = out[i];
    if (!it) continue;
    if (seen.has(it.cameraIndex)) out[i] = null;
    else seen.add(it.cameraIndex);
  }
  return out;
}

function applyLayoutSnapshot(snapshot, cameras) {
  if (!snapshot || snapshot.v !== 1 || !cameras.length) return null;
  const {
    cameraOrder,
    grid,
    rotationsByName,
    cameraPresets: savedPresets,
    activeCameras: savedActive,
    gridMode,
  } = snapshot;
  const nameToSavedIdx = Object.fromEntries(
    (cameraOrder || []).map((n, i) => [n, i])
  );
  const nextPresets = cameras.map((n, i) => {
    const j = nameToSavedIdx[n];
    return j !== undefined && savedPresets[j] !== undefined ? savedPresets[j] : PRESET_HD;
  });
  const nextActive = cameras.map((n, i) => {
    const j = nameToSavedIdx[n];
    return j !== undefined ? !!savedActive[j] : false;
  });
  const nextRot = {};
  cameras.forEach((n, i) => {
    const deg = rotationsByName?.[n];
    if (deg != null && deg % 360 !== 0) nextRot[i] = deg;
  });
  const nextGrid = defaultGridSlots();
  const g = grid || [];
  for (let i = 0; i < GRID_SIZE; i++) {
    const cell = g[i];
    if (!cell?.cameraName) {
      nextGrid[i] = null;
      continue;
    }
    const idx = cameras.indexOf(cell.cameraName);
    if (idx < 0) nextGrid[i] = null;
    else {
      nextGrid[i] = {
        cameraIndex: idx,
        colSpan: cell.colSpan ?? 1,
        rowSpan: cell.rowSpan ?? 1,
      };
    }
  }
  return {
    gridSlots: dedupeGridSlots(nextGrid),
    rotations: nextRot,
    cameraPresets: nextPresets,
    activeCameras: nextActive,
    gridMode: !!gridMode,
  };
}

const CameraFeed = () => {
  const [cameras, setCameras] = useState([]);
  /** Maps api id e.g. usb_8 -> "/dev/video8" (from backend discovery, not a UI slot). */
  const [usbDevicePaths, setUsbDevicePaths] = useState({});
  const [activeCameras, setActiveCameras] = useState([]);
  const [imageSrcs, setImageSrcs] = useState([]);
  const [focusedCameras, setFocusedCameras] = useState([]);
  const [showBirdsEye, setShowBirdsEye] = useState(false);
  const [gridMode, setGridMode] = useState(true);
  const [gridSlots, setGridSlots] = useState(defaultGridSlots);
  const [layoutPresetSlot, setLayoutPresetSlot] = useState(0);
  const [layoutPresetHint, setLayoutPresetHint] = useState("");
  const [rotations, setRotations] = useState({});
  const rotateCamera = (id) =>
    setRotations((prev) => ({ ...prev, [id]: ((prev[id] || 0) + 90) % 360 }));
  /** Per-camera stream quality; indices align with `cameras` */
  const [cameraPresets, setCameraPresets] = useState([]);
  const cyclePresetFor = (cameraIndex) => {
    setCameraPresets((prev) => {
      const next = [...prev];
      if (cameraIndex < 0 || cameraIndex >= next.length) return prev;
      next[cameraIndex] = next[cameraIndex] === PRESET_HD ? PRESET_TS : PRESET_HD;
      return next;
    });
  };
  const presetLabel = (p) => (p === PRESET_TS ? "TS" : "HD");

  const presetBtnClass = (p) =>
    (p ?? PRESET_HD) === PRESET_TS ? "btn-secondary" : "btn-outline-warning";

  const hintTimerRef = useRef(null);
  const flashLayoutHint = useCallback((msg) => {
    if (hintTimerRef.current) clearTimeout(hintTimerRef.current);
    setLayoutPresetHint(msg);
    hintTimerRef.current = setTimeout(() => {
      setLayoutPresetHint("");
      hintTimerRef.current = null;
    }, 2500);
  }, []);

  useEffect(
    () => () => {
      if (hintTimerRef.current) clearTimeout(hintTimerRef.current);
    },
    []
  );

  const flashLayoutHintRef = useRef(flashLayoutHint);
  useEffect(() => {
    flashLayoutHintRef.current = flashLayoutHint;
  }, [flashLayoutHint]);

  const camerasRef = useRef(cameras);
  const gridSlotsRef = useRef(gridSlots);
  const rotationsRef = useRef(rotations);
  const cameraPresetsRef = useRef(cameraPresets);
  const activeCamerasRef = useRef(activeCameras);
  const gridModeRef = useRef(gridMode);
  const layoutPresetSlotRef = useRef(layoutPresetSlot);
  useEffect(() => {
    camerasRef.current = cameras;
    gridSlotsRef.current = gridSlots;
    rotationsRef.current = rotations;
    cameraPresetsRef.current = cameraPresets;
    activeCamerasRef.current = activeCameras;
    gridModeRef.current = gridMode;
    layoutPresetSlotRef.current = layoutPresetSlot;
  }, [cameras, gridSlots, rotations, cameraPresets, activeCameras, gridMode, layoutPresetSlot]);

  const camerasKey = cameras.join(",");
  useEffect(() => {
    if (!cameras.length) return;
    const all = loadAllLayoutPresets();
    const snap = all[layoutPresetSlot];
    if (!snap) {
      setGridSlots(defaultGridSlots());
      setRotations({});
      return;
    }
    const applied = applyLayoutSnapshot(snap, cameras);
    if (!applied) return;
    setGridSlots(applied.gridSlots);
    setRotations(applied.rotations);
    setCameraPresets(applied.cameraPresets);
    setActiveCameras(applied.activeCameras);
    setGridMode(applied.gridMode);
  }, [layoutPresetSlot, camerasKey]);

  useEffect(() => {
    const onKey = (e) => {
      if (e.defaultPrevented) return;
      const t = e.target;
      if (
        t?.tagName === "INPUT" ||
        t?.tagName === "TEXTAREA" ||
        t?.tagName === "SELECT" ||
        t?.isContentEditable
      ) {
        return;
      }
      const k = e.key.toLowerCase();
      if (k !== "c" && k !== "d") return;
      if (!camerasRef.current.length) return;
      e.preventDefault();
      const slot = layoutPresetSlotRef.current;
      if (k === "c") {
        const snap = buildLayoutSnapshot(
          camerasRef.current,
          gridSlotsRef.current,
          rotationsRef.current,
          cameraPresetsRef.current,
          activeCamerasRef.current,
          gridModeRef.current
        );
        const all = loadAllLayoutPresets();
        all[slot] = snap;
        saveAllLayoutPresets(all);
        flashLayoutHintRef.current(`Saved camera layout to preset ${slot + 1} (C)`);
      } else {
        const all = loadAllLayoutPresets();
        all[slot] = null;
        saveAllLayoutPresets(all);
        setGridSlots(defaultGridSlots());
        setRotations({});
        flashLayoutHintRef.current(`Cleared preset ${slot + 1} (D)`);
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // Poll /api/cameras/ continuously so newly plugged USB cameras appear without a restart.
  // The backend's _sync_new_usb_cameras() probes /dev/video* on each call, so polling here
  // is sufficient — no service restart needed for hotplug.
  useEffect(() => {
    let cancelled = false;
    let pollTimer = null;
    const POLL_INTERVAL_MS = 8000;

    const fetchCameras = () => {
      fetch(`${API_BASE}/cameras/`, { credentials: 'include' })
        .then((res) => (res.ok ? res.json() : Promise.reject()))
        .then((data) => {
          if (cancelled) return;
          const list = data.cameras || [];
          setCameras((prev) =>
            prev.join(",") === list.join(",") ? prev : list
          );
          setUsbDevicePaths(data.usb_device_paths || {});
          // Preserve active state of existing cameras; only extend/trim for count changes.
          setActiveCameras((prev) => {
            if (prev.length === list.length) return prev;
            if (list.length > prev.length)
              return [...prev, ...Array(list.length - prev.length).fill(false)];
            return prev.slice(0, list.length);
          });
          setImageSrcs((prev) => {
            if (prev.length === list.length) return prev;
            if (list.length > prev.length)
              return [...prev, ...Array(list.length - prev.length).fill("")];
            return prev.slice(0, list.length);
          });
          setCameraPresets((prev) => {
            if (prev.length === list.length) return prev;
            if (list.length > prev.length)
              return [...prev, ...Array(list.length - prev.length).fill(PRESET_HD)];
            return prev.slice(0, list.length);
          });
          pollTimer = setTimeout(fetchCameras, POLL_INTERVAL_MS);
        })
        .catch(() => {
          if (!cancelled) {
            setCameras([]);
            pollTimer = setTimeout(fetchCameras, POLL_INTERVAL_MS);
          }
        });
    };

    fetchCameras();
    return () => { cancelled = true; clearTimeout(pollTimer); };
  }, []);

  const makeFrameUrl = (cameraName, cameraIndex, t) => {
    const pr = cameraPresets[cameraIndex] ?? PRESET_HD;
    return `${API_BASE}/video_feed/${encodeURIComponent(cameraName)}/?single=1&preset=${pr}&t=${t}`;
  };
  useEffect(() => {
    if (!cameras.length) return;
    setImageSrcs(
      cameras.map((name, idx) =>
        activeCameras[idx] ? makeFrameUrl(name, idx, Date.now()) : ""
      )
    );
  }, [cameras, activeCameras, cameraPresets]);

  const requestNextFrame = (cameraId) => {
    if (!activeCameras[cameraId] || !cameras[cameraId]) return;
    setImageSrcs((prev) => {
      const next = [...prev];
      next[cameraId] = makeFrameUrl(cameras[cameraId], cameraId, Date.now());
      return next;
    });
  };

  const onFrameError = (cameraId) => {
    requestNextFrame(cameraId);
  };

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

  const displayName = (name) => {
    if (!name) return "";
    // Friendly labels for IP and USB cameras
    const ipMatch = name.match(/^ip_(\d+)$/);
    if (ipMatch) return `IP Camera ${ipMatch[1]}`;
    const usbMatch = name.match(/^usb_(\d+)$/);
    if (usbMatch) {
      const dev = usbDevicePaths[name];
      return dev ? `USB ${dev}` : `USB /dev/video${usbMatch[1]}`;
    }
    return name.charAt(0).toUpperCase() + name.slice(1);
  };

  const onFrameLoad = (cameraId) => {
    requestNextFrame(cameraId);
  };

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

  // --- NIR Servo arrow-key control ---
  const [nirServoEnabled, setNirServoEnabled] = useState(false);
  const [nirDuty, setNirDuty] = useState(0);
  const nirDutyRef = useRef(0);
  const nirKeyInterval = useRef(null);

  const sendNirDuty = useCallback(
    (duty) => {
      fetch(`${API_BASE}/nir-servo-control/`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ duty }),
      }).catch(() => {});
    },
    []
  );

  useEffect(() => {
    if (!nirServoEnabled) return;
    const STEP = 2;
    const INTERVAL_MS = 60;
    const keysHeld = { ArrowLeft: false, ArrowRight: false };

    const tick = (dir) => {
      const next = Math.max(0, Math.min(100, nirDutyRef.current + STEP * dir));
      if (next === nirDutyRef.current) return;
      nirDutyRef.current = next;
      setNirDuty(next);
      sendNirDuty(next);
    };

    const handleKeyDown = (e) => {
      if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
      if (keysHeld[e.key]) return;
      e.preventDefault();
      keysHeld[e.key] = true;
      const dir = e.key === "ArrowRight" ? 1 : -1;
      tick(dir);
      clearInterval(nirKeyInterval.current);
      nirKeyInterval.current = setInterval(() => tick(dir), INTERVAL_MS);
    };

    const handleKeyUp = (e) => {
      if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
      keysHeld[e.key] = false;
      if (!keysHeld.ArrowLeft && !keysHeld.ArrowRight) {
        clearInterval(nirKeyInterval.current);
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      clearInterval(nirKeyInterval.current);
    };
  }, [nirServoEnabled, sendNirDuty]);

  return (
    <div className="cameraPage">
      <div className="container-fluid px-3 main-container text-white">
      <div className="cameraPageHeaderBlock">
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
        <div className="cameraLayoutPresetBar">
          <span className="cameraLayoutPresetLabel">Camera layout preset</span>
          <div className="cameraLayoutPresetSlots" role="group" aria-label="Layout preset slot">
            {Array.from({ length: NUM_LAYOUT_PRESETS }, (_, i) => (
              <button
                key={i}
                type="button"
                className={`btn btn-sm ${layoutPresetSlot === i ? "btn-warning" : "btn-outline-secondary"}`}
                onClick={() => setLayoutPresetSlot(i)}
              >
                {i + 1}
              </button>
            ))}
          </div>
          <span className="cameraLayoutPresetKbd text-muted small">
            <kbd className="text-dark bg-light">C</kbd> save ·{" "}
            <kbd className="text-dark bg-light">D</kbd> clear slot
          </span>
          {layoutPresetHint ? (
            <span className="cameraLayoutPresetToast" role="status">
              {layoutPresetHint}
            </span>
          ) : null}
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
              <CamerasRing cameraNames={cameras} cameraPresets={cameraPresets} />
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
                              <div className="cameraGridSlotHeaderActions">
                                <button
                                  type="button"
                                  className={`btn btn-sm cameraPresetBtn ${presetBtnClass(cameraPresets[cameraIndex])}`}
                                  onClick={(e) => { e.stopPropagation(); cyclePresetFor(cameraIndex); }}
                                  title="HD: higher quality. TS: thumbnail stream, lower bandwidth."
                                >
                                  {presetLabel(cameraPresets[cameraIndex] ?? PRESET_HD)}
                                </button>
                                <button
                                  className="btn btn-sm btn-outline-danger"
                                  onClick={(e) => { e.stopPropagation(); clearSlot(slotIndex); }}
                                  aria-label="Clear slot"
                                >
                                  ×
                                </button>
                              </div>
                            </div>
                            {activeCameras[cameraIndex] && (
                              <div className="cameraGridSlotView">
                                <button
                                  className="cameraRotateBtn"
                                  onClick={(e) => { e.stopPropagation(); rotateCamera(cameraIndex); }}
                                  title="Rotate 90° clockwise"
                                >
                                  ↻
                                </button>
                                <img
                                  src={imageSrcs[cameraIndex] || undefined}
                                  alt={displayName(cameras[cameraIndex])}
                                  className="cameraImg"
                                  style={{ transform: `rotate(${rotations[cameraIndex] || 0}deg)` }}
                                  onLoad={() => onFrameLoad(cameraIndex)}
                                  onError={() => onFrameError(cameraIndex)}
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
                      <div className="cameraTileHeaderActions">
                        <button
                          type="button"
                          className={`btn btn-sm cameraPresetBtn ${presetBtnClass(cameraPresets[id])}`}
                          onClick={() => cyclePresetFor(id)}
                          title="HD: higher quality. TS: thumbnail stream, lower bandwidth."
                        >
                          {presetLabel(cameraPresets[id] ?? PRESET_HD)}
                        </button>
                        <button
                          className={`btn btn-sm ${activeCameras[id] ? "btn-danger" : "btn-success"}`}
                          onClick={() => toggleCamera(id)}
                        >
                          {activeCameras[id] ? "Off" : "On"}
                        </button>
                      </div>
                    </div>
                    {activeCameras[id] && (
                      <div className="cameraTileView" onClick={() => toggleFocus(id)}>
                        <button
                          className="cameraRotateBtn"
                          onClick={(e) => { e.stopPropagation(); rotateCamera(id); }}
                          title="Rotate 90° clockwise"
                        >
                          ↻
                        </button>
                        <img
                          src={imageSrcs[id] || undefined}
                          alt={displayName(cameras[id])}
                          className="cameraImg"
                          style={{ transform: `rotate(${rotations[id] || 0}deg)` }}
                          onLoad={() => onFrameLoad(id)}
                          onError={() => onFrameError(id)}
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
                    <button
                      type="button"
                      className={`btn btn-sm cameraPresetBtn mb-1 ${presetBtnClass(cameraPresets[id])}`}
                      onClick={(e) => { e.stopPropagation(); cyclePresetFor(id); }}
                      title="HD: higher quality. TS: thumbnail stream, lower bandwidth."
                    >
                      {presetLabel(cameraPresets[id] ?? PRESET_HD)}
                    </button>
                    {activeCameras[id] && (
                      <div
                        className="cameraSidebarThumb"
                        onClick={!gridMode ? () => toggleFocus(id) : undefined}
                        draggable={gridMode}
                        onDragStart={gridMode ? (e) => handleDragStart(e, id) : undefined}
                      >
                        <img src={imageSrcs[id] || undefined} alt={displayName(cameras[id])} style={{ transform: `rotate(${rotations[id] || 0}deg)` }} onLoad={() => onFrameLoad(id)} onError={() => onFrameError(id)} />
                      </div>
                    )}
                  </div>
              ))}
            </aside>
          )}
        </div>
      ) : null}

      {/* NIR Servo control bar */}
      <div className="nirServoCard">
        <div className="nirServoRow">
          <label className="nirServoToggle">
            <input
              type="checkbox"
              checked={nirServoEnabled}
              onChange={(e) => setNirServoEnabled(e.target.checked)}
            />
            <span className="nirServoToggleTrack">
              <span className="nirServoToggleThumb" />
            </span>
            <span className="nirServoLabel">NIR Servo</span>
          </label>
          {nirServoEnabled && (
            <>
              <div className="nirServoBarTrack">
                <div className="nirServoBarFill" style={{ width: `${nirDuty}%` }} />
                <span className="nirServoBarLabel">{nirDuty}%</span>
              </div>
              <span className="nirServoHint">
                <kbd>←</kbd> / <kbd>→</kbd>
              </span>
              <button
                type="button"
                className="btn btn-outline-warning btn-sm"
                onClick={() => { nirDutyRef.current = 0; setNirDuty(0); sendNirDuty(0); }}
              >
                Reset
              </button>
            </>
          )}
        </div>
      </div>

      </div>
    </div>
  );
};

export default CameraFeed;
