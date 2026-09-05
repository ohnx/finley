#!/bin/sh
# Build the wasm module the page loads.
#
# No bundler, no npm, no wasm-bindgen: the core has no dependencies and the shim
# is a raw C ABI, so this is the entire build step.
set -eu
cd "$(dirname "$0")/.."
rustup target add wasm32-unknown-unknown 2>/dev/null || true
cargo build --release --target wasm32-unknown-unknown -p ohtsim-wasm
cp target/wasm32-unknown-unknown/release/ohtsim_wasm.wasm web/ohtsim.wasm
ls -l web/ohtsim.wasm
