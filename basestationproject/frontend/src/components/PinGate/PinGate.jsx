import React, { useState, useRef, useEffect } from "react";
import { useAuth } from "../../context/AuthContext";
import "./PinGate.css";

const PIN_LENGTH = 6;

export default function PinGate() {
  const { loading, authenticated, pinConfigured, error, verifyPin, checkAuth } = useAuth();
  const [pin, setPin] = useState("");
  const [submitError, setSubmitError] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const inputRef = useRef(null);

  const showGate = !loading && !authenticated && (pinConfigured || error);

  useEffect(() => {
    if (showGate) inputRef.current?.focus();
  }, [showGate]);

  const handleChange = (e) => {
    const v = e.target.value.replace(/\D/g, "").slice(0, PIN_LENGTH);
    setPin(v);
    setSubmitError("");
    if (v.length === PIN_LENGTH) {
      submitPin(v);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === "Backspace" && pin.length > 0) {
      setPin((p) => p.slice(0, -1));
      setSubmitError("");
    }
  };

  const handlePaste = (e) => {
    e.preventDefault();
    const pasted = (e.clipboardData?.getData("text") || "").replace(/\D/g, "").slice(0, PIN_LENGTH);
    if (pasted.length === PIN_LENGTH) {
      setPin(pasted);
      setSubmitError("");
      submitPin(pasted);
    } else if (pasted) {
      setPin(pasted);
      setSubmitError("");
    }
  };

  const submitPin = async (value) => {
    setSubmitting(true);
    setSubmitError("");
    try {
      const ok = await verifyPin(value);
      if (!ok) {
        setSubmitError("Invalid PIN");
        setPin("");
      }
    } catch (err) {
      const msg = err.response?.data?.error || err.message || "Verification failed";
      setSubmitError(msg);
      setPin("");
    } finally {
      setSubmitting(false);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (pin.length === PIN_LENGTH && !submitting) submitPin(pin);
  };

  if (!showGate) return null;

  return (
    <div className="pinGateOverlay" role="dialog" aria-modal="true" aria-label="Enter PIN">
      <div className="pinGateCard">
        {error ? (
          <>
            <h2 className="pinGateTitle">Connection Error</h2>
            <p className="pinGateHint">{error}. Make sure the backend is running.</p>
            <button type="button" className="btn btn-primary mt-3" onClick={() => { setSubmitError(""); setPin(""); checkAuth(); }}>
              Retry
            </button>
          </>
        ) : (
          <>
            <h2 className="pinGateTitle">Enter PIN</h2>
            <p className="pinGateHint">6-digit PIN required</p>
            <form onSubmit={handleSubmit} className="pinGateForm">
              <input
            ref={inputRef}
            type="password"
            inputMode="numeric"
            pattern="[0-9]*"
            autoComplete="one-time-code"
            maxLength={PIN_LENGTH}
            value={pin}
            onChange={handleChange}
            onKeyDown={handleKeyDown}
            className="pinGateInput"
            placeholder="000000"
            disabled={submitting}
            aria-label="6-digit PIN"
          />
          {submitError && <p className="pinGateError">{submitError}</p>}
            </form>
          </>
        )}
      </div>
    </div>
  );
}
