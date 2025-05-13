import React, { useState, useEffect } from "react";
import "bootstrap/dist/css/bootstrap.min.css";
import "./CameraFeed.css";

const CameraFeed = () => {
  const CAMERA_COUNT = 10;
  const [activeCameras, setActiveCameras] = useState(Array(CAMERA_COUNT).fill(false));
  const [imageSrcs, setImageSrcs] = useState(Array(CAMERA_COUNT).fill(""));

  const [focusedCameras, setFocusedCameras] = useState([]);


  useEffect(() => {
    document.title = "Camera Feed";

    const intervals = activeCameras.map((isActive, index) => {
      if (isActive) {
        return setInterval(() => {
          setImageSrcs((prevSrcs) => {
            const newSrcs = [...prevSrcs];
            newSrcs[index] = `http://localhost:8000/api/video_feed/${index}/?time=${new Date().getTime()}`;
            return newSrcs;
          });
        }, 33);
      }
      return null;
    });

    return () => {
      intervals.forEach((interval) => interval && clearInterval(interval));
    };
  }, [activeCameras]);

  const toggleCamera = (index) => {
    setActiveCameras((prev) => {
      const newStates = [...prev];
      newStates[index] = !newStates[index];
      return newStates;
    });
  };

  const toggleFocus = (index) => {
    setFocusedCameras((prev) =>
      prev.includes(index)
        ? prev.filter((i) => i !== index)
        : [...prev, index]
    );
  };

  const getSidebarCameras = () => {
    return Array.from({ length: CAMERA_COUNT }, (_, i) => i).filter((i) => !focusedCameras.includes(i));
  };  

  return (
    <>

      <div className="container-fluid mt-4 main-container text-white">
        <h2 className="text-center mb-4">Live Camera Feeds</h2>
        <div className="row">
          {/* Focused Cameras */}
          <div className="col-md-9">
            <div className="row">
              {focusedCameras.length > 0 ? (
                focusedCameras.map((id) => {
                  const colSize = focusedCameras.length === 1 ? 12 : (focusedCameras.length === 2 ? 6 : 6);
                  return (
                    <div key={id} className={`col-md-${colSize} mb-4 focused-camera`}>
                      <button
                        className={`btn ${activeCameras[id] ? "btn-danger btn-accent-secondary" : "btn-success btn-accent-primary"} mb-2 w-100`}
                        onClick={() => toggleCamera(id)}
                      >
                        {activeCameras[id] ? "Turn Off" : "Turn On"} Camera {id}
                      </button>
                      {activeCameras[id] && (
                        <img
                          src={imageSrcs[id]}
                          alt={`Camera ${id}`}
                          className="img-fluid border rounded w-100"
                          style={{ maxHeight: "70vh", objectFit: "contain", cursor: "pointer" }}
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


          {/* Sidebar */}
          <div className="col-md-3">
            {getSidebarCameras().map((id) => (
              <div key={id} className="mb-4 small-camera">
                <button
                  className={`btn btn-sm ${activeCameras[id] ? "btn-danger" : "btn-success"} mb-1 w-100`}
                  onClick={() => toggleCamera(id)}
                >
                  {activeCameras[id] ? "Turn Off" : "Turn On"} Camera {id}
                </button>
                {activeCameras[id] && (
                  <img
                    src={imageSrcs[id]}
                    alt={`Camera ${id}`}
                    className="img-fluid border rounded"
                    style={{ cursor: "pointer", maxHeight: "150px", objectFit: "cover" }}
                    onClick={() => toggleFocus(id)}
                  />
                )}
              </div>
            ))}
          </div>
        </div>
      </div>
    </>
  );
};

export default CameraFeed;
