import React, { useEffect, useState } from "react";
import axios from "axios";
import "bootstrap/dist/css/bootstrap.min.css";

const ScriptManager = () => {
  const [status, setStatus] = useState({});
  const [camerasOpen, setCamerasOpen] = useState(false);

  // fetch running/stopped status
  const fetchStatus = async () => {
    try {
      const { data } = await axios.get("http://localhost:8081/status");
      setStatus(data);
    } catch (err) {
      console.error(err);
    }
  };

  useEffect(() => {
    document.title = "Script Manager";
    fetchStatus();
    const iv = setInterval(fetchStatus, 2000);
    return () => clearInterval(iv);
  }, []);

  const startScript = (name) =>
    axios
      .post(`http://localhost:8081/start-script/${encodeURIComponent(name)}`)
      .then(fetchStatus);

  const stopScript = (name) =>
    axios
      .post(`http://localhost:8081/stop-script/${encodeURIComponent(name)}`)
      .then(fetchStatus);

  const startAll = () =>
    axios.post("http://localhost:8081/start-all").then(fetchStatus);
  const stopAll = () =>
    axios.post("http://localhost:8081/stop-all").then(fetchStatus);

  // split camera vs other scripts
  const allNames = Object.keys(status);
  const cameraNames = allNames.filter((n) => n.startsWith("Camera "));
  const otherNames = allNames.filter((n) => !n.startsWith("Camera "));

  return (
    <div className="container mt-5 text-white">
      <h1 className="text-center mb-4">Script Manager</h1>

      <div className="d-flex justify-content-center mb-4">
        <button className="btn btn-success mx-2" onClick={startAll}>
          Start All
        </button>
        <button className="btn btn-danger mx-2" onClick={stopAll}>
          Stop All
        </button>
      </div>

      <div className="table-responsive">
        <table className="table table-dark table-bordered text-center">
          <thead>
            <tr>
              <th>Script Name</th>
              <th>Status</th>
              <th>Action</th>
            </tr>
          </thead>
          <tbody>
            {/* camera dropdown header */}
            {cameraNames.length > 0 && (
              <tr
                onClick={() => setCamerasOpen((o) => !o)}
                style={{ cursor: "pointer" }}
              >
                <td colSpan="3" className="bg-secondary">
                  <span className="me-2">
                    {camerasOpen ? "▼" : "▶"}
                  </span>
                  Camera Streams ({cameraNames.length})
                </td>
              </tr>
            )}

            {/* camera rows */}
            {camerasOpen &&
              cameraNames.map((name) => {
                const state = status[name];
                return (
                  <tr key={name}>
                    <td className="ps-5">{name}</td>
                    <td>
                      {state === "running" ? (
                        <span className="badge bg-success">Running</span>
                      ) : (
                        <span className="badge bg-danger">Stopped</span>
                      )}
                    </td>
                    <td>
                      {state === "running" ? (
                        <button
                          className="btn btn-outline-danger btn-sm"
                          onClick={() => stopScript(name)}
                        >
                          Stop
                        </button>
                      ) : (
                        <button
                          className="btn btn-outline-success btn-sm"
                          onClick={() => startScript(name)}
                        >
                          Start
                        </button>
                      )}
                    </td>
                  </tr>
                );
              })}

            {/* other scripts */}
            {otherNames.map((name) => {
              const state = status[name];
              return (
                <tr key={name}>
                  <td>{name}</td>
                  <td>
                    {state === "running" ? (
                      <span className="badge bg-success">Running</span>
                    ) : (
                      <span className="badge bg-danger">Stopped</span>
                    )}
                  </td>
                  <td>
                    {state === "running" ? (
                      <button
                        className="btn btn-outline-danger btn-sm"
                        onClick={() => stopScript(name)}
                      >
                        Stop
                      </button>
                    ) : (
                      <button
                        className="btn btn-outline-success btn-sm"
                        onClick={() => startScript(name)}
                      >
                        Start
                      </button>
                    )}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default ScriptManager;
