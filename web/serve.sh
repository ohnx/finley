#!/bin/sh
# Serve the repo root, because the page fetches maps/, scenarios/ and policies/
# from it. Open the URL printed below.
set -eu
cd "$(dirname "$0")/.."
echo "ohtsim UI:  http://localhost:${1:-8000}/web/"
exec python3 -m http.server "${1:-8000}"
