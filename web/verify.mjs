// Checks that the wasm build behaves identically to the native one.
//
//   ./web/build.sh && node web/verify.mjs
//
// The sim is deterministic by design, so "same seed, same numbers" holds across
// targets too. If wasm and native disagree, something in the port is
// target-dependent and the UI is showing a different fab from the one the
// headless runner reports on.

import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), "..");

// From: cargo run --release --bin headless -- maps/demo_loop.json \
//           scenarios/baseline.json policies/<name>.json 20000
const EXPECTED = {
  default:           { created: 163, completed: 149, p95: 2594, stuck: 0 },
  starvation_biased: { created: 173, completed: 158, p95: 2121, stuck: 0 },
};

const M = { TICK: 0, CREATED: 1, COMPLETED: 2, THROUGHPUT: 3, MEAN_CYCLE: 4,
            P95: 5, UTIL: 6, BACKLOG: 7, DEADLOCKS: 8, STUCK: 9 };

const bytes = fs.readFileSync(path.join(ROOT, "web/ohtsim.wasm"));
const { instance } = await WebAssembly.instantiate(bytes, {});
const e = instance.exports;
const enc = new TextEncoder();

function put(rel) {
  const b = enc.encode(fs.readFileSync(path.join(ROOT, rel), "utf8"));
  const ptr = e.oht_alloc(b.length);
  new Uint8Array(e.memory.buffer, ptr, b.length).set(b);
  return [ptr, b.length];
}

let failures = 0;
for (const [name, want] of Object.entries(EXPECTED)) {
  const bufs = [put("maps/demo_loop.json"), put("scenarios/baseline.json"),
                put(`policies/${name}.json`)];
  const sim = e.oht_new(...bufs.flat());
  for (const [ptr, len] of bufs) e.oht_free_buf(ptr, len);

  if (sim === 0) {
    const err = new Uint8Array(e.memory.buffer, e.oht_error_ptr(), e.oht_error_len());
    console.error(`FAIL ${name}: ${new TextDecoder().decode(err)}`);
    failures++;
    continue;
  }

  e.oht_tick(sim, 20000);
  // Re-derived after ticking on purpose: growing wasm memory detaches any view
  // taken earlier, and 20k ticks does grow it.
  const m = new Float64Array(e.memory.buffer, e.oht_metrics(sim), e.oht_metric_count());
  const got = { created: m[M.CREATED], completed: m[M.COMPLETED],
                p95: m[M.P95], stuck: m[M.STUCK] };
  e.oht_drop(sim);

  const bad = Object.keys(want).filter((k) => got[k] !== want[k]);
  if (bad.length) {
    console.error(`FAIL ${name}: ${bad.map((k) => `${k} ${got[k]} != ${want[k]}`).join(", ")}`);
    failures++;
  } else {
    console.log(`ok   ${name}: ${got.completed} lots, p95 ${got.p95}, ${got.stuck} recoveries`);
  }
}

// A duplicated class method in app.js is silently legal -- the later
// definition simply wins -- so a bad edit can shadow a working renderer method
// with a stale copy, or delete one entirely, and the only symptom is a blank
// canvas at runtime. Both happened while building the lot and OHT panels.
const KEYWORDS = new Set(["if", "for", "while", "switch", "catch", "return",
                          "function", "do", "else"]);
const app = fs.readFileSync(path.join(ROOT, "web/app.js"), "utf8");
// Check within each class, so the two constructors do not look like a clash.
for (const cls of app.split(/^class\s+(\w+)/m).slice(1).reduce(
       (acc, part, i, all) => (i % 2 ? acc : [...acc, [part, all[i + 1]]]), [])) {
  const [name, body] = cls;
  const defs = [...body.matchAll(/^ {2}([A-Za-z_$][\w$]*)\s*\([^)]*\)\s*\{/gm)]
    .map((m) => m[1])
    .filter((n) => !KEYWORDS.has(n));
  const dupes = [...new Set(defs.filter((n, i) => defs.indexOf(n) !== i))];
  if (dupes.length) {
    console.error(`FAIL class ${name} defines these more than once: ${dupes.join(", ")}`);
    failures++;
  }
}

if (failures) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nwasm matches the native build");
