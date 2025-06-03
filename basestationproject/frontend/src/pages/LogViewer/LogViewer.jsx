// src/pages/LogViewer.jsx

import React, { useState, useEffect, useMemo } from "react";
import axios from "axios";

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

export default function LogViewer() {
  document.title = "Logs"
  const API_BASE = "http://127.0.0.1:8000/api";

  const [fileList, setFileList] = useState([]);
  const [selectedFile, setSelectedFile] = useState("");
  const [columns, setColumns] = useState([]);
  const [rows, setRows] = useState([]);
  const [sortConfig, setSortConfig] = useState({ key: null, direction: "asc" });

  // Fetch available log filenames from the server
  useEffect(() => {
    axios
      .get(`${API_BASE}/list-logs/`)
      .then((res) => {
        setFileList(res.data.files || []);
      })
      .catch((err) => console.error("Failed to list logs:", err));
  }, []);

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
  }, [selectedFile]);

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
    <div className="container my-4">
      <h3 className="text-white">Log Viewer</h3>

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
    </div>
  );
}
