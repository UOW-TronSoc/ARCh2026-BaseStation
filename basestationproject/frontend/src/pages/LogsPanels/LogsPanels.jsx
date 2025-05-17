import React, { useEffect, useState } from "react";
import axios from "axios";

const API_BASE = "http://127.0.0.1:8000/api";

export default function LogsPanels() {
  const [logs, setLogs] = useState([]);
  const [filter, setFilter] = useState("");
  const [autoRefresh, setAutoRefresh] = useState(true);

  // Fetch logs from backend
  const fetchLogs = async () => {
    try {
      const { data } = await axios.get(`${API_BASE}/rover-logs/`);
      setLogs(data.logs || []);
    } catch (err) {
      setLogs([]);
    }
  };

  // Auto-refresh
  useEffect(() => {
    fetchLogs();
    if (!autoRefresh) return;
    const timer = setInterval(fetchLogs, 2000);
    return () => clearInterval(timer);
  }, [autoRefresh]);

  // Download logs as text file
  const downloadLogs = () => {
    const blob = new Blob([logs.join("\n")], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "rover_logs.txt";
    a.click();
    URL.revokeObjectURL(url);
  };

  // Filter logs by keyword/type/source
  const filteredLogs = logs.filter(
    (log) => !filter || log.toLowerCase().includes(filter.toLowerCase())
  );

  return (
    <div className="container my-4">
      <h2>System Logs & Panels</h2>
      <div className="mb-3 d-flex align-items-center gap-3">
        <input
          type="text"
          placeholder="Filter by type/source/keyword"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          className="form-control w-auto"
        />
        <button className="btn btn-primary" onClick={downloadLogs}>
          Download Logs
        </button>
        <div className="form-check form-switch ms-3">
          <input
            className="form-check-input"
            type="checkbox"
            checked={autoRefresh}
            onChange={() => setAutoRefresh((v) => !v)}
            id="autoRefreshSwitch"
          />
          <label className="form-check-label" htmlFor="autoRefreshSwitch">
            Auto-refresh
          </label>
        </div>
      </div>

      {/* Panels: Add more status/health panels as needed */}
      <div className="row mb-4">
        <div className="col-md-4">
          <div className="card p-3">
            <h5>Status Panel</h5>
            <p>System: <span className="text-success">Online</span></p>
            {/* Add more status/health indicators here */}
          </div>
        </div>
        {/* Add more panels here */}
      </div>

      <div className="card">
        <div className="card-header">Logs</div>
        <div
          className="card-body"
          style={{ maxHeight: 400, overflowY: "auto", fontFamily: "monospace", fontSize: 14 }}
        >
          {filteredLogs.length === 0 ? (
            <div className="text-muted">No logs available.</div>
          ) : (
            filteredLogs.map((log, idx) => <div key={idx}>{log}</div>)
          )}
        </div>
      </div>
    </div>
  );
}