#!/usr/bin/env bash
# Loads nvm and runs npm run dev so systemd can find npm (e.g. when installed via nvm).
set -e
export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
if [[ -f "$NVM_DIR/nvm.sh" ]]; then
  source "$NVM_DIR/nvm.sh"
  nvm use default 2>/dev/null || true
fi
exec npm run dev
