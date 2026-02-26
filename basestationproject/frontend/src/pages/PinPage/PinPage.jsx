import React, { useState, useRef, useEffect } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import { useAuth } from "../../context/AuthContext";
import "./PinPage.css";

const PIN_LENGTH = 6;

export default function PinPage() {
  const navigate = useNavigate();
  const location = useLocation();
  const { loading, authenticated, pinConfigured, error, verifyPin, checkAuth } = useAuth();
  const [pin, setPin] = useState("");
  const [submitError, setSubmitError] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const inputRef = useRef(null);

  const from = location.state?.from?.pathname || "/";

  useEffect(() => {
    if (!loading && authenticated) {
      navigate(from, { replace: true });
    }
  }, [loading, authenticated, navigate, from]);

  useEffect(() => {
    if (!loading && !authenticated && (pinConfigured || error)) {
      inputRef.current?.focus();
    }
  }, [loading, authenticated, pinConfigured, error]);

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

  const submitPin = async (value) => {
    setSubmitting(true);
    setSubmitError("");
    try {
      const ok = await verifyPin(value);
      if (ok) {
        navigate(from, { replace: true });
      } else {
        setSubmitError("Invalid PIN");
        setPin("");
      }
    } catch (err) {
      setSubmitError(err.response?.data?.error || err.message || "Verification failed");
      setPin("");
    } finally {
      setSubmitting(false);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (pin.length === PIN_LENGTH && !submitting) submitPin(pin);
  };

  if (loading) {
    return (
      <div className="pinPage">
        <div className="pinPageCard">
          <p className="pinPageHint">Checking authentication…</p>
        </div>
      </div>
    );
  }

  if (!pinConfigured && !error) {
    return (
      <div className="pinPage">
        <div className="pinPageCard">
          <h2 className="pinPageTitle">No PIN Configured</h2>
          <p className="pinPageHint">Run: python manage.py set_pin 123456</p>
        </div>
      </div>
    );
  }

  return (
    <div className="pinPage">
      <div className="pinPageCard">
        {error ? (
          <>
            <h2 className="pinPageTitle">Connection Error</h2>
            <p className="pinPageHint">{error}. Make sure the backend is running.</p>
            <button type="button" className="btn btn-primary mt-3" onClick={() => { setSubmitError(""); setPin(""); checkAuth(); }}>
              Retry
            </button>
          </>
        ) : (
          <>
            <h2 className="pinPageTitle">Enter PIN</h2>
            <p className="pinPageHint">6-digit PIN required</p>
            <form onSubmit={handleSubmit} className="pinPageForm">
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
                className="pinPageInput"
                placeholder="000000"
                disabled={submitting}
                aria-label="6-digit PIN"
              />
              {submitError && <p className="pinPageError">{submitError}</p>}
            </form>
          </>
        )}
      </div>
    </div>
  );
}
