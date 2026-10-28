#!/bin/zsh
# Double-click to run Mechanism Sandbox: starts the dev server and opens it
# in your browser. Close this Terminal window (or press Ctrl+C) to stop it.

cd "${0:A:h}" || exit 1

# Finder-launched scripts don't read your shell profile, so add the usual Node locations.
export PATH="/opt/homebrew/bin:/usr/local/bin:$HOME/.volta/bin:$PATH"
[ -s "$HOME/.nvm/nvm.sh" ] && source "$HOME/.nvm/nvm.sh"

if ! command -v npm >/dev/null 2>&1; then
  echo "Node.js isn't installed. Get it from https://nodejs.org and double-click this file again."
  read -k1 "?Press any key to close."
  exit 1
fi

if [ ! -d node_modules ]; then
  echo "First run: installing dependencies…"
  npm install || { read -k1 "?Install failed. Press any key to close."; exit 1; }
fi

echo "Starting Mechanism Sandbox… (close this window to stop it)"
npm run dev -- --open
