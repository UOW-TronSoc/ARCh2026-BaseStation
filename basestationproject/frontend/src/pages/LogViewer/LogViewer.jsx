// src/pages/LogViewer.jsx

import React, { useState, useEffect, useMemo, useCallback } from "react";
import axios from "axios";
import { getApiBase } from "../../config";

// Utility to parse any CSV text into columns + row objects
function parseCSV(text) {
  const lines = text.trim().split("\n");
  if (lines.length < 2) {
    return { columns: [], rows: [] };
  }
  const columns = lines[0].split(",").map((h) => h.trim());
  const rows = lines.slice(1).map((line) => {
    const vals = line.split(",").map((v) => v.trim());
    const obj = {};
    columns.forEach((col, i) => {
      if (col === "timestamp") {
        obj[col] = vals[i];
      } else {
        const num = parseFloat(vals[i]);
        obj[col] = isNaN(num) ? vals[i] : num;
      }
    });
    return obj;
  });
  return { columns, rows };
}

const LOG_TAB_ROVER = "rover";
const LOG_TAB_DJANGO = "django";

function LogViewer() {
  document.title = "Logs";
  const API_BASE = getApiBase();

  const [activeTab, setActiveTab] = useState(LOG_TAB_ROVER);
  const [fileList, setFileList] = useState([]);
  const [selectedFile, setSelectedFile] = useState("");
  const [columns, setColumns] = useState([]);
  const [rows, setRows] = useState([]);
  const [sortConfig, setSortConfig] = useState({ key: null, direction: "asc" });

  // Django server logs
  const [djangoLines, setDjangoLines] = useState([]);
  const [djangoLoading, setDjangoLoading] = useState(false);
  const [djangoError, setDjangoError] = useState(null);

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

  // Fetch available log filenames from the server
  useEffect(() => {
    axios
      .get(`${API_BASE}/list-logs/`)
      .then((res) => {
        setFileList(res.data.files || []);
      })
      .catch((err) => console.error("Failed to list logs:", err));
  }, [API_BASE]);

  // When a log file is selected, fetch its CSV content
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
        const { columns, rows } = parseCSV(text);
        setColumns(columns);
        setRows(rows);
        setSortConfig({ key: null, direction: "asc" });
      })
      .catch((err) => {
        console.error("Failed to fetch log content:", err);
        setColumns([]);
        setRows([]);
      });
  }, [selectedFile, API_BASE]);

  // When Django tab is active: fetch immediately and refresh at 1 Hz
  useEffect(() => {
    if (activeTab !== LOG_TAB_DJANGO) return;
    fetchDjangoLogs();
    const interval = setInterval(fetchDjangoLogs, 1000);
    return () => clearInterval(interval);
  }, [activeTab, fetchDjangoLogs]);

  // Sorting logic: memoize sortedRows based on sortConfig
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

  // Toggle sort when a column header is clicked
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

  return (
    <div className="container-fluid px-3 logViewerPage">
      <h3 className="text-white">Log Viewer</h3>

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
            Rover / CSV Logs
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
            Django Server Logs
          </button>
        </li>
      </ul>

      {activeTab === LOG_TAB_ROVER && (
        <>
          <div className="mb-3">
            <label className="form-label text-white">Select a log file:</label>
            <select
              className="form-select"
              value={selectedFile}
              onChange={(e) => setSelectedFile(e.target.value)}
            >
              <option value="">-- Choose CSV from server --</option>
              {fileList.map((fname) => (
                <option key={fname} value={fname}>
                  {fname}
                </option>
              ))}
            </select>
          </div>

          {columns.length > 0 && (
            <div
              className="table-responsive"
              style={{ maxHeight: "500px", overflowY: "auto" }}
            >
              <table className="table table-sm table-bordered">
                <thead className="table-light">
                  <tr>
                    {columns.map((col) => (
                      <th
                        key={col}
                        style={{ cursor: "pointer", whiteSpace: "nowrap" }}
                        onClick={() => handleSort(col)}
                      >
                        {col}
                        {sortConfig.key === col && (
                          <span>
                            {" "}
                            {sortConfig.direction === "asc" ? "↑" : "↓"}
                          </span>
                        )}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {sortedRows.map((row, idx) => (
                    <tr key={idx}>
                      {columns.map((col) => (
                        <td key={col} style={{ whiteSpace: "nowrap" }}>
                          {row[col]}
                        </td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </>
      )}

      {activeTab === LOG_TAB_DJANGO && (
        <div className="mb-3">
          <div className="d-flex justify-content-between align-items-center mb-2">
            <span className="text-white-50 small">
              Live logs from the basestation Django server (in-memory buffer).
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
          <pre
            className="bg-dark text-light border rounded p-3 small mb-0"
            style={{
              maxHeight: "60vh",
              overflowY: "auto",
              whiteSpace: "pre-wrap",
              wordBreak: "break-all",
            }}
          >
            {djangoLines.length === 0 && !djangoLoading && !djangoError
              ? "No log lines captured yet."
              : djangoLines.join("\n")}
          </pre>
        </div>
      )}
    </div>
  );
}

export default LogViewer;
