import React from "react";
import enableSwitchStyles from "components/EnableSwitch/EnableSwitch.module.css";
import styles from "./SpeedControlCard.module.css";

export default function SpeedControlCard({ enabled, setEnabled, controllerInfo, children }) {
  return (
    <div className="card w-100 speedCard">
      <div className={`card-header ${styles.cardHeader}`}>
        <h5 className="mb-0 header">Drive</h5>
        <div
          className={`form-check form-switch mt-2 ${enabled ? enableSwitchStyles.panelActive : enableSwitchStyles.panelInactive}`}
        >
          <input
            type="checkbox"
            className={`form-check-input ${styles.switch} ${enabled ? enableSwitchStyles.switchOn : enableSwitchStyles.switchOff}`}
            id="driveEnableSwitch"
            role="switch"
            checked={enabled}
            onChange={() => setEnabled(!enabled)}
            aria-checked={enabled}
          />
          <label className="form-check-label" htmlFor="driveEnableSwitch">
            <span className={enabled ? enableSwitchStyles.titleOn : enableSwitchStyles.titleOff}>
              {enabled ? "Drive active" : "Drive off — keyboard / gamepad idle"}
            </span>
            <span className="d-block small text-secondary mt-1">
              When off, no drive commands are sent to the backend.
            </span>
          </label>
        </div>
      </div>

      <div className={`card-body ${styles.cardBody}`}>
        <div className={styles.deviceInfo}>
          <div>
            <strong>Active device:</strong> {controllerInfo?.name ?? "None"}
          </div>
          {controllerInfo?.type === "logitech-extreme-3d" &&
            typeof controllerInfo?.throttle === "number" && (
              <div className="small text-muted">
                Throttle: {Math.round(controllerInfo.throttle * 100)}%
              </div>
            )}
        </div>

        {children ? (
          <>
            <hr className={styles.sectionRule} />
            {children}
          </>
        ) : null}
      </div>
    </div>
  );
}
