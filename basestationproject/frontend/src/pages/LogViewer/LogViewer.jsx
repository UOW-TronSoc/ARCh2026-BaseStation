import React, { useState, useEffect, useMemo, useCallback, useRef } from "react";
import axios from "axios";
import { getApiBase } from "../../config";

/** Split one CSV row respecting double-quoted fields. */
function parseCSVLine(line) {
  const result = [];
  let current = "";
  let inQuotes = false;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (c === '"') {
      inQuotes = !inQuotes;
    } else if (c === "," && !inQuotes) {
      result.push(current.trim());
      current = "";
    } else {
      current += c;
    }
  }
  result.push(current.trim());
  return result;
}

function parseCSV(text) {
  const lines = text.trim().split(/\r?\n/).filter((l) => l.length > 0);
  if (lines.length < 2) {
    return { columns: [], rows: [] };
  }
  const columns = parseCSVLine(lines[0]);
  const rows = lines.slice(1).map((line) => {
    const vals = parseCSVLine(line);
    const obj = {};
    columns.forEach((col, i) => {
      const cell = vals[i] !== undefined ? vals[i] : "";
      if (col === "timestamp" || /time|date/i.test(col)) {
        obj[col] = cell;
      } else {
        const num = parseFloat(cell);
        obj[col] = cell === "" || Number.isNaN(num) ? cell : num;
      }
    });
    return obj;
  });
  return { columns, rows };
}

/** `2025-03-23 12:00:00,123 [INFO] django.server: message` */
function parseDjangoLogLine(line) {
  const m = line.match(/^([\d\-:,\s.]+)\s+\[(\w+)\]\s+([^:]+):\s*(.*)$/);
  if (!m) {
    return { raw: line, time: null, level: "OTHER", logger: null, message: line };
  }
  return {
    raw: line,
    time: m[1].trim(),
    level: m[2],
    logger: m[3].trim(),
    message: m[4],
  };
}

function levelStyle(level) {
  switch (level) {
    case "ERROR":
    case "CRITICAL":
      return { color: "#ffb4b4", fontWeight: 600 };
    case "WARNING":
    case "WARN":
      return { color: "#ffe08a" };
    case "INFO":
      return { color: "#b8e0ff" };
    case "DEBUG":
      return { color: "#9aa7b8" };
    default:
      return { color: "#d8dee9" };
  }
}

function formatBytes(n) {
  if (n == null || Number.isNaN(n)) return "—";
  const u = ["B", "KB", "MB", "GB", "TB"];
  let v = n;
  let i = 0;
  while (v >= 1024 && i < u.length - 1) {
    v /= 1024;
    i++;
  }
  return `${v < 10 && i > 0 ? v.toFixed(1) : Math.round(v)} ${u[i]}`;
}

const LOG_TAB_ROVER = "rover";
const LOG_TAB_DJANGO = "django";

function ServicePill({ ok, label }) {
  return (
    <span
      className="badge rounded-pill me-1 mb-1"
      style={{
        backgroundColor: ok ? "rgba(25,135,84,0.35)" : "rgba(220,53,69,0.4)",
        color: "#fff",
        border: `1px solid ${ok ? "rgba(25,135,84,0.9)" : "rgba(220,53,69,0.85)"}`,
        fontWeight: 500,
      }}
    >
      {label}
    </span>
  );
}

function LogViewer() {
  document.title = "Logs";
  const API_BASE = getApiBase();

  const [activeTab, setActiveTab] = useState(LOG_TAB_ROVER);
  const [fileList, setFileList] = useState([]);
  const [selectedFile, setSelectedFile] = useState("");
  const [columns, setColumns] = useState([]);
  const [rows, setRows] = useState([]);
  const [sortConfig, setSortConfig] = useState({ key: null, direction: "asc" });
  const [csvViewMode, setCsvViewMode] = useState("tail");

  const [djangoLines, setDjangoLines] = useState([]);
  const [djangoLoading, setDjangoLoading] = useState(false);
  const [djangoError, setDjangoError] = useState(null);
  const djangoLogEndRef = useRef(null);

  const [health, setHealth] = useState(null);
  const [healthError, setHealthError] = useState(null);

  const fetchDjangoLogs = useCallback(() => {
    setDjangoLoading(true);
    setDjangoError(null);
    axios
      .get(`${API_BASE}/django-logs/`)
      .then((res) => {
        setDjangoLines(res.data.lines || []);
      })
      .catch((err) => {
        console.error("Failed to fetch Django logs:", err);
        setDjangoError(err.message || "Failed to load Django logs");
        setDjangoLines([]);
      })
      .finally(() => setDjangoLoading(false));
  }, [API_BASE]);

  const fetchHealth = useCallback(() => {
    axios
      .get(`${API_BASE}/status/health/`)
      .then((res) => {
        setHealth(res.data);
        setHealthError(null);
      })
      .catch((err) => {
        console.error("Failed to fetch basestation health:", err);
        setHealth(null);
        setHealthError(err.message || "Health check failed");
      });
  }, [API_BASE]);

  useEffect(() => {
    axios
      .get(`${API_BASE}/list-logs/`)
      .then((res) => {
        setFileList(res.data.files || []);
      })
      .catch((err) => console.error("Failed to list logs:", err));
  }, [API_BASE]);

  useEffect(() => {
    if (!selectedFile) {
      setColumns([]);
      setRows([]);
      return;
    }
    axios
      .get(`${API_BASE}/get-log/${selectedFile}/`)
      .then((res) => {
        const text = res.data.content;
        const parsed = parseCSV(text);
        setColumns(parsed.columns);
        setRows(parsed.rows);
        setSortConfig({ key: null, direction: "asc" });
      })
      .catch((err) => {
        console.error("Failed to fetch log content:", err);
        setColumns([]);
        setRows([]);
      });
  }, [selectedFile, API_BASE]);

  useEffect(() => {
    if (activeTab !== LOG_TAB_DJANGO) return;
    fetchDjangoLogs();
    const interval = setInterval(fetchDjangoLogs, 1000);
    return () => clearInterval(interval);
  }, [activeTab, fetchDjangoLogs]);

  useEffect(() => {
    fetchHealth();
    const interval = setInterval(fetchHealth, 2500);
    return () => clearInterval(interval);
  }, [fetchHealth]);

  useEffect(() => {
    if (activeTab === LOG_TAB_DJANGO && djangoLogEndRef.current) {
      djangoLogEndRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [djangoLines, activeTab]);

  const sortedRows = useMemo(() => {
    if (!sortConfig.key) return rows;
    return [...rows].sort((a, b) => {
      const va = a[sortConfig.key];
      const vb = b[sortConfig.key];
      if (va < vb) return sortConfig.direction === "asc" ? -1 : 1;
      if (va > vb) return sortConfig.direction === "asc" ? 1 : -1;
      return 0;
    });
  }, [rows, sortConfig]);

  const displayedCsvRows = useMemo(() => {
    if (csvViewMode === "all") return sortedRows;
    const n = 500;
    return sortedRows.length <= n ? sortedRows : sortedRows.slice(-n);
  }, [sortedRows, csvViewMode]);

  const handleSort = (col) => {
    if (sortConfig.key === col) {
      setSortConfig((prev) => ({
        key: col,
        direction: prev.direction === "asc" ? "desc" : "asc",
      }));
    } else {
      setSortConfig({ key: col, direction: "asc" });
    }
  };

  const sys = health?.system;
  const mem = sys?.memory;
  const gpu = sys?.gpu;
  const nv = gpu?.nvidia_smi;
  const services = health?.services;

  return (
    <div className="container-fluid px-3 logViewerPage">
      <h3 className="text-white mb-3">Log Viewer</h3>

      <section
        className="rounded border border-secondary mb-4 p-3"
        style={{ backgroundColor: "rgba(33,37,41,0.92)" }}
      >
        <div className="d-flex flex-wrap justify-content-between align-items-center gap-2 mb-2">
          <h5 className="text-white mb-0">Basestation status</h5>
          <button type="button" className="btn btn-sm btn-outline-light" onClick={fetchHealth}>
            Refresh status
          </button>
        </div>
        {healthError && (
          <div className="alert alert-warning py-2 small mb-2" role="alert">
            {healthError}
          </div>
        )}
        {health && (
          <>
            <div className="text-white-50 small mb-2">Services (probed from the basestation host)</div>
            <div className="d-flex flex-wrap align-items-center gap-2 mb-3">
              <ServicePill ok={services?.django?.running} label="Django API" />
              <ServicePill
                ok={services?.django_ros2?.healthy}
                label={`ROS2 in Django (${services?.django_ros2?.registered_nodes ?? 0} nodes)`}
              />
              <ServicePill ok={services?.arm_fastapi?.running} label="Arm FastAPI" />
              <ServicePill ok={services?.drive_fastapi?.running} label="Drive FastAPI" />
            </div>
            {services?.arm_fastapi && !services.arm_fastapi.running && (
              <div className="text-warning small mb-1">
                Arm FastAPI: {services.arm_fastapi.error || `HTTP ${services.arm_fastapi.http_status}`}
              </div>
            )}
            {services?.drive_fastapi && !services.drive_fastapi.running && (
              <div className="text-warning small mb-2">
                Drive FastAPI: {services.drive_fastapi.error || `HTTP ${services.drive_fastapi.http_status}`}
              </div>
            )}

            <div className="row g-2 small text-white">
              <div className="col-6 col-md-4 col-lg-2">
                <div className="text-white-50">CPU</div>
                <div className="fs-6">{sys?.cpu_percent != null ? `${sys.cpu_percent}%` : "—"}</div>
                {sys?.loadavg_1m != null && (
                  <div className="text-white-50" style={{ fontSize: "0.75rem" }}>
                    load 1m: {sys.loadavg_1m}
                  </div>
                )}
              </div>
              <div className="col-6 col-md-4 col-lg-3">
                <div className="text-white-50">Memory</div>
                <div className="fs-6">
                  {mem?.used_percent != null ? `${mem.used_percent}% used` : "—"}
                </div>
                <div className="text-white-50" style={{ fontSize: "0.75rem" }}>
                  {formatBytes(mem?.used_bytes)} / {formatBytes(mem?.total_bytes)}
                </div>
              </div>
              <div className="col-6 col-md-4 col-lg-2">
                <div className="text-white-50">Temp (max zone)</div>
                <div className="fs-6">
                  {sys?.temperature_c_max != null ? `${sys.temperature_c_max} °C` : "—"}
                </div>
              </div>
              <div className="col-12 col-md-8 col-lg-5">
                <div className="text-white-50">GPU</div>
                <div className="fs-6">
                  {nv?.available ? (
                    <>
                      {nv.utilization_percent != null ? `${nv.utilization_percent}% util` : "util n/a"}
                      {nv.memory_used_mb != null && nv.memory_total_mb != null && (
                        <span className="ms-2">
                          {Math.round(nv.memory_used_mb)} / {Math.round(nv.memory_total_mb)} MiB
                        </span>
                      )}
                      {nv.temperature_c != null && (
                        <span className="ms-2">{nv.temperature_c} °C</span>
                      )}
                    </>
                  ) : gpu?.jetson_clock_mhz != null ? (
                    <span>Jetson GPU clock ~{gpu.jetson_clock_mhz} MHz (nvidia-smi n/a)</span>
                  ) : (
                    "No GPU metrics (or nvidia-smi unavailable)"
                  )}
                </div>
              </div>
            </div>

            {sys?.thermal_zones?.length > 0 && (
              <details className="mt-2 text-white-50 small">
                <summary style={{ cursor: "pointer" }}>Thermal zones</summary>
                <ul className="mb-0 mt-1 ps-3">
                  {sys.thermal_zones.map((z) => (
                    <li key={z.id}>
                      {z.type}: {z.celsius} °C
                    </li>
                  ))}
                </ul>
              </details>
            )}

            {services?.django_ros2 && (
              <details className="mt-2 text-white-50 small">
                <summary style={{ cursor: "pointer" }}>ROS2 (Django process) detail</summary>
                <ul className="mb-0 mt-1 ps-3">
                  <li>rclpy ok: {String(services.django_ros2.rclpy_ok)}</li>
                  <li>executor thread alive: {String(services.django_ros2.executor_thread_alive)}</li>
                  <li>registered nodes: {services.django_ros2.registered_nodes}</li>
                  <li>standard ROS imports: {String(services.django_ros2.ros_standard_imports_ok)}</li>
                </ul>
              </details>
            )}
          </>
        )}
        {!health && !healthError && (
          <div className="text-white-50 small">Loading status…</div>
        )}
      </section>

      <ul className="nav nav-tabs mb-3">
        <li className="nav-item">
          <button
            type="button"
            className={`nav-link ${activeTab === LOG_TAB_ROVER ? "active" : ""}`}
            onClick={() => setActiveTab(LOG_TAB_ROVER)}
            style={{
              color: activeTab === LOG_TAB_ROVER ? undefined : "rgba(255,255,255,0.75)",
              borderColor: "#dee2e6 #dee2e6 transparent",
              backgroundColor: activeTab === LOG_TAB_ROVER ? "#212529" : "transparent",
            }}
          >
            Rover / CSV logs
          </button>
        </li>
        <li className="nav-item">
          <button
            type="button"
            className={`nav-link ${activeTab === LOG_TAB_DJANGO ? "active" : ""}`}
            onClick={() => setActiveTab(LOG_TAB_DJANGO)}
            style={{
              color: activeTab === LOG_TAB_DJANGO ? undefined : "rgba(255,255,255,0.75)",
              borderColor: "#dee2e6 #dee2e6 transparent",
              backgroundColor: activeTab === LOG_TAB_DJANGO ? "#212529" : "transparent",
            }}
          >
            Django server logs
          </button>
        </li>
      </ul>

      {activeTab === LOG_TAB_ROVER && (
        <>
          <div className="mb-3">
            <label className="form-label text-white">Select a log file</label>
            <select
              className="form-select"
              value={selectedFile}
              onChange={(e) => setSelectedFile(e.target.value)}
            >
              <option value="">— Choose CSV from server —</option>
              {fileList.map((fname) => (
                <option key={fname} value={fname}>
                  {fname}
                </option>
              ))}
            </select>
          </div>

          {columns.length > 0 && (
            <>
              <div className="d-flex flex-wrap justify-content-between align-items-center gap-2 mb-2">
                <span className="text-white-50 small">
                  {columns.length} columns · {rows.length} rows
                  {csvViewMode === "tail" && rows.length > 500 && (
                    <span> (showing latest 500 of {rows.length})</span>
                  )}
                </span>
                <div className="btn-group btn-group-sm">
                  <button
                    type="button"
                    className={`btn ${csvViewMode === "tail" ? "btn-light" : "btn-outline-light"}`}
                    onClick={() => setCsvViewMode("tail")}
                  >
                    Latest 500 rows
                  </button>
                  <button
                    type="button"
                    className={`btn ${csvViewMode === "all" ? "btn-light" : "btn-outline-light"}`}
                    onClick={() => setCsvViewMode("all")}
                  >
                    Show all
                  </button>
                </div>
              </div>
              <div
                className="table-responsive log-viewer-csv-wrap rounded border border-secondary"
                style={{ maxHeight: "min(70vh, 720px)", overflow: "auto" }}
              >
                <table className="table table-sm table-dark table-striped mb-0 log-viewer-csv-table">
                  <thead
                    style={{
                      position: "sticky",
                      top: 0,
                      zIndex: 2,
                      boxShadow: "0 1px 0 rgba(255,255,255,0.12)",
                    }}
                  >
                    <tr>
                      {columns.map((col) => (
                        <th
                          key={col}
                          className="text-nowrap user-select-none"
                          style={{ cursor: "pointer", backgroundColor: "#2b3035" }}
                          onClick={() => handleSort(col)}
                        >
                          {col}
                          {sortConfig.key === col && (
                            <span> {sortConfig.direction === "asc" ? "↑" : "↓"}</span>
                          )}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {displayedCsvRows.map((row, idx) => (
                      <tr key={idx}>
                        {columns.map((col) => (
                          <td key={col} className="text-nowrap align-middle">
                            {row[col] === "" || row[col] == null ? (
                              <span className="text-secondary">—</span>
                            ) : typeof row[col] === "number" ? (
                              Number.isInteger(row[col]) ? (
                                row[col]
                              ) : (
                                row[col].toFixed(4).replace(/\.?0+$/, "")
                              )
                            ) : (
                              String(row[col])
                            )}
                          </td>
                        ))}
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </>
          )}
        </>
      )}

      {activeTab === LOG_TAB_DJANGO && (
        <div className="mb-3">
          <div className="d-flex justify-content-between align-items-center mb-2">
            <span className="text-white-50 small">
              Live server log (newest at bottom). Parsed: time, level, logger, message — not raw JSON.
            </span>
            <button
              type="button"
              className="btn btn-sm btn-outline-light"
              onClick={fetchDjangoLogs}
              disabled={djangoLoading}
            >
              {djangoLoading ? "Loading…" : "Refresh"}
            </button>
          </div>
          {djangoError && (
            <div className="alert alert-warning py-2" role="alert">
              {djangoError}
            </div>
          )}
          <div
            className="rounded border border-secondary p-2 log-viewer-terminal"
            style={{
              maxHeight: "min(65vh, 640px)",
              overflowY: "auto",
              backgroundColor: "#0d1117",
              fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Consolas, monospace',
              fontSize: "0.8125rem",
              lineHeight: 1.45,
            }}
          >
            {djangoLines.length === 0 && !djangoLoading && !djangoError && (
              <span className="text-secondary">No log lines captured yet.</span>
            )}
            {djangoLines.map((line, i) => {
              const p = parseDjangoLogLine(line);
              if (p.logger) {
                return (
                  <div key={i} className="mb-1">
                    <span className="text-secondary">{p.time}</span>{" "}
                    <span style={levelStyle(p.level)}>[{p.level}]</span>{" "}
                    <span className="text-info" style={{ opacity: 0.9 }}>
                      {p.logger}
                    </span>
                    <span className="text-secondary">: </span>
                    <span style={{ color: "#e6edf3" }}>{p.message}</span>
                  </div>
                );
              }
              return (
                <div key={i} className="mb-1" style={{ color: "#e6edf3" }}>
                  {p.message}
                </div>
              );
            })}
            <div ref={djangoLogEndRef} />
          </div>
        </div>
      )}
    </div>
  );
}

export default LogViewer;
