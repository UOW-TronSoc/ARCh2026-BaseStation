import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from "react";
import axios from "axios";
import { getBackendBase } from "../config";

// Ensure all API requests send cookies (for session auth)
axios.defaults.withCredentials = true;

const AuthContext = createContext(null);

// Track latest checkAuth for interceptor (avoids stale closure)
let globalCheckAuth = () => {};
axios.interceptors.response.use(
  (r) => r,
  (err) => {
    if (err.response?.status === 401 && err.response?.data?.code === "pin_required") {
      globalCheckAuth(); // Refreshes auth state; ProtectedRoute will redirect to /pin
    }
    return Promise.reject(err);
  }
);

export function AuthProvider({ children }) {
  const [auth, setAuth] = useState({
    loading: true,
    authenticated: false,
    pinConfigured: false,
    error: null,
  });

  const checkAuth = useCallback(async () => {
    try {
      const { data } = await axios.get(`${getBackendBase()}/api/auth-status/`);
      setAuth({
        loading: false,
        authenticated: !!data.authenticated,
        pinConfigured: !!data.pin_configured,
        error: null,
      });
    } catch {
      setAuth({ loading: false, authenticated: false, pinConfigured: true, error: "Could not reach server" });
    }
  }, []);

  useEffect(() => {
    globalCheckAuth = checkAuth;
    checkAuth();
  }, [checkAuth]);

  const verifyPin = useCallback(async (pin) => {
    const { data } = await axios.post(`${getBackendBase()}/api/pin-verify/`, { pin });
    if (data?.ok) {
      setAuth((prev) => ({ ...prev, authenticated: true, error: null }));
      return true;
    }
    return false;
  }, []);

  const value = {
    ...auth,
    checkAuth,
    verifyPin,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const ctx = useContext(AuthContext);
  if (!ctx) throw new Error("useAuth must be used within AuthProvider");
  return ctx;
}
