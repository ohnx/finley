#!/bin/sh
# Assemble the static site into _site/.
#
# Used by both the Pages workflow and local testing, so what CI publishes is
# exactly what you can check on your own machine. The page fetches maps/,
# scenarios/ and policies/ as siblings of web/, so the site keeps the repo's
# layout and a redirect at the root points into it.
set -eu
cd "$(dirname "$0")/.."
OUT=${1:-_site}

./web/build.sh >/dev/null

rm -rf "$OUT"
mkdir -p "$OUT/web"
cp web/index.html web/style.css web/app.js web/ohtsim.wasm "$OUT/web/"
cp -r maps scenarios policies "$OUT/"

cat > "$OUT/index.html" <<'HTML'
<!doctype html>
<meta charset="utf-8">
<title>finley</title>
<meta http-equiv="refresh" content="0; url=web/">
<link rel="canonical" href="web/">
<p>Redirecting to <a href="web/">the simulator</a>.</p>
HTML

echo "assembled $OUT ($(du -sh "$OUT" | cut -f1))"
