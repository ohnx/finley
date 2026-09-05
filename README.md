# finley

**ohtsim** is finley's simulation core: an OHT (overhead hoist transport) fab —
the ceiling-mounted robots that move FOUPs between tools in a semiconductor
fab. It runs headless for batch policy sweeps and compiles to wasm for the
browser UI in `web/`.

## Layout

```
crates/ohtsim/           the simulation core, no dependencies
  src/geom.rs            grid, direction bitmask, directed track graph
  src/model.rs           vehicles, machines, lots, jobs, ports
  src/movement.rs        one-vehicle-per-cell movement resolution  (unit tested)
  src/routing.rs         Dijkstra over (cell, heading), congestion-weighted
  src/dispatch.rs        weighted scoring over (job, vehicle, destination)
  src/policy.rs          the configuration space
  src/world.rs           the tick loop
  src/config.rs          JSON loading
  src/json.rs            minimal JSON parser (keeps the crate dependency-free)
  src/validate.rs        map validation
  src/bin/headless.rs    runner; compares two policies on one job stream
  src/bin/trace.rs       per-tick state dump, for diffing against the reference
  tests/                 whole-tick-loop properties
crates/ohtsim-wasm/      browser shim; raw C ABI, no wasm-bindgen
web/                     the UI: build.sh, serve.sh, verify.mjs, and the page
maps/                    fab layouts
scenarios/               what work arrives
policies/                how it is dispatched
reference/               Python prototypes; the behavioural ground truth
                         (trace_sim.py is the Python side of src/bin/trace.rs)
```

## Running the UI

```
./web/build.sh     # compiles the wasm; the entire build step
./web/serve.sh     # serves the repo root, prints the URL
```

Then open <http://localhost:8000/web/>. It must be served rather than opened as
a file: the page fetches the map, scenario and policy JSON from the repo root,
and `WebAssembly.instantiateStreaming` needs a real MIME type.

The page ticks the actual simulation — it is not a replay. Play/pause/step, a
speed slider up to 200 ticks per frame (about 12,000 ticks/second in Chromium),
and a policy selector that rebuilds the world so the two shipped policies can be
compared by eye.

On the map: track is drawn as one-way arrows, congestion as a heat wash that
shows traffic waves propagating backward from a hoisting vehicle, and spur cells
are tinted so parking reads as distinct from the main line.

Each tool is drawn as its body plus a neck out to each of its load ports, so a
port reads as part of the machine it serves — the map places ports wherever the
track runs, which is not always against the tool. A port itself is a bay,
coloured green for in and rust for out, with the lot drawn sitting on it when
one is there. Watching out-bays fill is watching backpressure arrive.

Four tabs alongside:

- **Lots** — every lot in the fab with its recipe progress, what tool kind it
  needs next, and where it is. Click one to follow it: it stays ringed on the
  map as it moves, on whichever tab you are on. A lot inside a tool is either
  *processing* or *done, no free out-port* — the second is a finished lot that
  cannot leave, which stalls the tool behind it, so it is called out rather than
  lumped in with work in progress.
- **OHTs** — all eight vehicles, what each is doing, what it carries and where
  it is headed. Click one to draw its planned route, which is how a congestion
  detour becomes visible. Hovering a vehicle on the map shows the same. A lot
  names the vehicle carrying it and a vehicle names its lot, and both are links,
  so following one to the other does not mean hunting through the other tab.
- **Tools** — utilisation (share of ticks with a lot in process) and queue
  depth per machine. Source and sink show "—": nothing is ever *in process* at
  either, so a percentage there would read 100% and mean nothing.
- **Stats** — the cycle-time distribution as a histogram with p50 and p95
  marked, because two policies can agree on the mean and disagree completely on
  the tail, plus the throughput counters.

Vehicle states are named for what you can see rather than for the leg of the
job: a vehicle is **fetching** while it drives empty toward a pickup and
**delivering** while it drives loaded toward a dropoff, with **hoisting** for
the 20-tick cycle at either end. (The core calls these `ToPickup` and
`ToDropoff`. Because `carrying` is true exactly during the dropoff leg, the
colour already says whether a vehicle is loaded, so the UI does not draw a
separate cargo marker.)

Machine footprints sit *under* the track. That is not an overlap bug: rails are
ceiling-mounted, so they legitimately run over tools, and the sim ignores
machine `w`/`h` entirely — they are presentational only.

`node web/verify.mjs` checks the wasm build reproduces the native numbers, and
that no class in `app.js` defines a method twice — a duplicate silently shadows
the earlier definition, and the only symptom is a blank canvas.

### Publishing it

`.github/workflows/pages.yml` builds the wasm and deploys to GitHub Pages on
every push to `main`, or on demand from the Actions tab. It is inert until Pages
is switched on: **Settings → Pages → Source: "GitHub Actions"**.

Two things to know before switching it on. A Pages site is publicly reachable
even when the repo is private, unless you are on Enterprise Cloud with access
control — so this publishes the simulator to the internet. And Pages on a
private repo needs a paid plan; on Free it is public repos only.

`./web/dist.sh` assembles exactly what the workflow publishes, so you can check
it locally first:

```
./web/dist.sh && (cd _site && python3 -m http.server)
```

`DESIGN.md` explains why any of this is shaped the way it is.

## Running headless

```
cargo test
cargo run --release --bin headless -- \
    maps/demo_loop.json scenarios/baseline.json policies/default.json 20000
```

The runner validates the map first and exits non-zero with a problem list if
it fails.

Pass a second policy file to compare two policies on an identical job stream:

```
cargo run --release --bin headless -- \
    maps/demo_loop.json scenarios/baseline.json \
    policies/default.json 20000 policies/starvation_biased.json
```

## The track format

A track cell holds the OR of its **allowed exit directions**:

```
N=1   E=2   S=4   W=8      0 = no track
```

A cell you may leave heading north or east is `1|2 = 3`. Values are 0..15. The
directed graph is implicit; no edges are stored anywhere. Rails are one-way, so
head-on deadlock is impossible by construction.

**One cell = one vehicle length.** This is load-bearing. If you raise the grid
resolution later, either vehicles must start claiming k cells or they silently
get shorter and headway stops meaning anything physical.

## Configuration

Three documents on purpose — one map runs against many scenarios, one scenario
against many policies. That separation is what makes experiments possible.

Policies are weighted scoring functions over named criteria, the way a fab IE
tunes an MCS. No policy is a preset: "nearest vehicle first" is just what you
get when `travel_to_pickup` dominates and everything else is zero. Conditional
profiles swap the active weights when a trigger fires (`backlog_above`,
`queue_depth_above`, `starvation_above`), which gives policy structure without a
scripting language. `always` is the fallback and is sorted last so it cannot
shadow the conditional profiles.

## Findings

**Parking must be on spurs.** The first demo map put parking cells directly on
the main loop. One idle vehicle parks, the loop is severed, and the fab
gridlocks — zero lots completed in 6000 ticks. Spurs are short branches that
leave the loop and rejoin it. Two rules fall out: routing must never path
*through* a spur (`Router::set_avoid`), and a vehicle must only ever target
parking that is currently free — which has to include spurs another vehicle is
already driving to, not just spurs that are occupied right now.

`src/validate.rs` checks any new map for this and runs before every simulation:
strong connectivity, no dangling exit bits, no dead ends, ports on track and off
spurs, and the main line still strongly connected with every spur cell removed.
`reference/gen_map2.py` has the original version of the same checks.

**Arrival rate has to be set against the bottleneck.** litho at 2 tools x 2
chambers / 120 ticks, visited twice per lot, caps the fab near 16.7 lots per
1000 ticks. The first scenario released 45, so every policy looked identically
terrible. The baseline now runs at 12.

**Prepositioning must not commit to a single spur.** Straight-line distance to
a tool is a crude proxy on a directed track graph and ties are common on a
symmetric map, so collapsing the target set to one cell let a coin flip decide
where the whole fleet waited. It also sent every idle vehicle to the *same*
empty spur; the losers arrived to find it taken and re-decided from where they
stood, which is the main line. Keeping the line clear outranks the starvation
preference — see `DESIGN.md` for the full account.

On 20 000 ticks:

| | default | starvation_biased |
|---|---|---|
| lots created | 100 | 172 |
| completed | 77 | 157 |
| p95 cycle | 3191 | 2571 |
| utilisation | 32% | 90% |
| mean backlog | 6.52 | 3.78 |
| deadlocks | 0 | 0 |
| stuck recoveries | 45 | 0 |

`starvation_biased` currently dominates `default` on every axis, so the two
shipped policies do not yet demonstrate a tradeoff. Both are also still
source-limited rather than fleet-limited: the scenario offers ~240 lots over
20 000 ticks and neither creates that many. Retuning them into a real tradeoff
is the open question — see `DESIGN.md`.

## Known rough edges

- Stuck-vehicle recovery fires 45 times per 20k ticks under `default`, and not
  at all under `starvation_biased`. That is 45 *events*, not 45 vehicles — the
  demo map has 8, and one in a bad spot can trigger recovery repeatedly. Not
  fatal — the vehicle reroutes and carries on — but the underlying stalls are
  worth investigating rather than papering over with a bigger
  `stuck_threshold`.
- An idle vehicle can still end up on the main line when *every* spur is taken:
  it has nowhere to go and stops where it stands. The demo map has exactly as
  many spurs as vehicles, so there is no slack.
- Resource deadlock (vehicles waiting on ports that will never free) has no
  detector yet. Movement deadlock does. The distinction matters: a packed ring
  all wanting to advance is a legal *rotation*, and a chain behind a hoisting
  vehicle is a transient stall — neither is a deadlock, and `movement.rs` has
  tests asserting they are not reported as one.
- The delivery-cost term in dispatch seeds its distance field with an arbitrary
  legal heading, so it can be off by one curve. Immaterial for ranking.
- No buffers or stockers yet. Backpressure currently manifests as lots stuck
  inside machines when output ports are full, which works but is coarser than
  real under-track buffers.

## The browser boundary

`crates/ohtsim` stays free of rendering dependencies; `crates/ohtsim-wasm` is
the only thing that knows a browser exists, and it is a raw C ABI rather than
wasm-bindgen so the core stays dependency-free.

Two rules hold it together, both documented at their call sites:

- **Nothing is serialised per frame.** `Snapshot` is struct-of-arrays and
  `World::snapshot_into` refills it without reallocating; the shim hands JS
  pointers into those flat `Vec`s. Serialising every tick would eat most of the
  reason for choosing Rust.
- **Static map geometry never crosses the boundary.** JS already fetches the map
  JSON to build the world, so it reads track bits, machine rectangles and port
  cells from that. One source of truth.

On the JS side: re-derive every typed-array view from `memory.buffer` each
frame, because growing wasm memory detaches the old ones, and treat every
pointer as valid only until the next `oht_tick`.
