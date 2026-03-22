/**
 * API base URLs derived from the current page origin.
 * When you open http://10.0.0.1:5173 from another device (e.g. 10.0.0.2),
 * requests go to 10.0.0.1, not localhost.
 */
const getOrigin = () => {
  if (typeof window !== "undefined" && window.location) {
    return `${window.location.protocol}//${window.location.hostname}`;
  }
  return "http://127.0.0.1";
};

/** Django API base (port 8000), e.g. http://10.0.0.1:8000/api */
export const getApiBase = () => `${getOrigin()}:8000/api`;

/** Django backend root (port 8000), e.g. http://10.0.0.1:8000 */
export const getBackendBase = () => `${getOrigin()}:8000`;

/** FastAPI command endpoint (port 8080), e.g. http://10.0.0.1:8080/command */
export const getCommandUrl = () => `${getOrigin()}:8080/command`;

/** FastAPI arm control base (port 8001), e.g. http://10.0.0.1:8001/arm */
export const getArmApiBase = () => `${getOrigin()}:8001/arm`;

/** Script manager API (port 8081), e.g. http://10.0.0.1:8081 */
export const getScriptManagerBase = () => `${getOrigin()}:8081`;
