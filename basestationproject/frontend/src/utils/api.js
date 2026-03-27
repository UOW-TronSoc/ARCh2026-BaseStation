import axios from "axios";

const COMMAND_TIMEOUT_MS = 3000;

/**
 * POST wrapper with a 3-second timeout for control commands.
 * If the request is still pending after 3 s it is aborted so stale
 * commands never flood the backend after a network hiccup.
 * Servo-demo and other intentionally long operations should use
 * plain axios.post instead.
 */
export const postCmd = (url, data, config = {}) =>
  axios.post(url, data, { timeout: COMMAND_TIMEOUT_MS, ...config });

/** Returns true when an axios error is a timeout/abort — expected for fast commands. */
export const isTimeoutError = (err) =>
  err?.code === "ECONNABORTED" || err?.name === "CanceledError" || err?.name === "AbortError";
