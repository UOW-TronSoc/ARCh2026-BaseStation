import React from "react";
import { Navigate, useLocation } from "react-router-dom";
import { useAuth } from "../../context/AuthContext";

/**
 * Redirects to /pin when not authenticated and PIN is configured (or auth check failed).
 */
export default function ProtectedRoute({ children }) {
  const location = useLocation();
  const { loading, authenticated, pinConfigured, error } = useAuth();

  if (loading) {
    return (
      <div className="d-flex align-items-center justify-content-center min-vh-100">
        <p className="text-muted">Loading…</p>
      </div>
    );
  }

  if (!authenticated && (pinConfigured || error)) {
    return <Navigate to="/pin" state={{ from: location }} replace />;
  }

  return children;
}
