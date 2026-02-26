import React, { useEffect } from "react";
import { FoxgloveViewer } from "@foxglove/embed-react";
import "./AutoMap.css";

export default function AutoMap() {
  useEffect(() => {
    document.title = "AutoMap";
  }, []);

  return (
    <div className="autoMapPage">
      <div className="autoMapContainer">
        <FoxgloveViewer
          className="foxgloveViewer"
          style={{ flex: 1, minHeight: 0 }}
          colorScheme="auto"
          onReady={() => console.log("Foxglove AutoMap ready")}
          onError={(msg) => console.error("Foxglove error:", msg)}
        />
      </div>
    </div>
  );
}
