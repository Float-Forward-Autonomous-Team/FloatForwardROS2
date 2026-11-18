#!/usr/bin/env bash
# Expose the read-only public foxglove_bridge (port 8766) through a Cloudflare quick tunnel
# and print the link that opens the public live page on it. Runs on the Mac, not in the container.
# Never point this at port 8765: that bridge accepts commands. The address is random and
# changes every time; stop with Ctrl+C.
set -euo pipefail

# the Vercel address from `make publish`; override with PAGE_URL=https://… make tunnel
PAGE_URL="${PAGE_URL:-https://floatforward-live.vercel.app/}"
PORT=8766

if ! command -v cloudflared >/dev/null; then
  echo "cloudflared is not installed: brew install cloudflared" >&2
  exit 1
fi
if ! nc -z localhost "$PORT" 2>/dev/null; then
  echo "Nothing on port $PORT yet. Start \`make vrx\` and \`make race\` first (the public bridge starts with the race)." >&2
  exit 1
fi
# refuse to open the tunnel unless the bridge behind it is really read-only
node "$(dirname "$0")/check_public_bridge.mjs" "ws://localhost:$PORT" || exit 1

# cloudflared prints the tunnel address in its log; pick it out and show the shareable link
cloudflared tunnel --no-autoupdate --url "http://localhost:$PORT" 2>&1 | while IFS= read -r line; do
  if [[ "$line" =~ https://([a-z0-9-]+\.trycloudflare\.com) ]]; then
    host="${BASH_REMATCH[1]}"
    echo
    echo "  Public live page:  ${PAGE_URL}?ws=wss://${host}"
    echo "  (stays up while this runs; Ctrl+C to stop)"
    echo
  elif [[ "$line" == *ERR* ]]; then
    echo "$line"
  fi
done
