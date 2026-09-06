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
  src/vehicles.rs        the vehicle half of the tick: jobs, parking, movement
  src/validate.rs        map validation
  src/bin/headless.rs    runner; compares two policies on one job stream
  src/bin/trace.rs       per-tick state dump, for diffing against the reference
  tests/                 whole-tick-loop properties
crates/ohtsim-wasm/      browser shim; raw C ABI, no wasm-bindgen
web/                     the UI: build.sh, serve.sh, verify.mjs, and the page
maps/                    fab layouts: fab.json (20 tools), demo_loop.json (9)
scenarios/               what work arrives
policies/                how it is dispatched
reference/               Python prototypes; the behavioural ground truth
                         (trace_sim.py is the Python side of src/bin/trace.rs,
                         gen_fab.py generates maps/fab.json)
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
a fab picker, and a policy selector. Both pickers rebuild the world, so two
policies or two layouts can be compared by eye.

Two fabs ship. **fab** is the default: 31×17, 20 tools across seven kinds, 30
vehicles, and a 68-move recipe — the one the game is actually about. **demo
loop** is the original 9-tool map, kept because it is small enough to follow a
single vehicle around and because most of the invariants were found on it.

A tick is **five seconds** of fab time (`tick_seconds` in the scenario). The
simulation itself is unitless; the scale exists so the UI can say "17h" next to
a cycle time of 12,142 ticks, and "38 lots/day" next to 2.22 per 1000 ticks.
Five seconds is roughly one vehicle move, which keeps movement legible at 1×
while making a full lot's journey watchable in under a minute at 200×.

On the map: track is drawn as one-way arrows, congestion as a heat wash that
shows traffic waves propagating backward from a hoisting vehicle, and spur cells
are tinted so parking reads as distinct from the main line.

Each tool is drawn as its body together with the cells its load ports sit on,
which meet because every body is placed against its own ports in the map —
`reference/gen_map2.py` checks that when it generates one. A port is a bay,
coloured green for in and rust for out, with the lot drawn sitting on it when
one is there. Watching out-bays fill is watching backpressure arrive.

What keeps the fab alive is `wip_cap` in the scenario. Reentrant recipes will
otherwise fill every port in the fab and deadlock it permanently, and storage
does not prevent that — buffers were built, measured, and removed once it was
clear they only delayed it. See `DESIGN.md` for the operating curve.

Four tabs alongside:

- **Lots** — every lot in the fab with its recipe progress, what tool kind it
  needs next, and where it is. Click one to follow it: it stays ringed on the
  map as it moves, on whichever tab you are on. A lot inside a tool is either
  *processing* or *done, no free out-port* — the second is a finished lot that
  cannot leave, which stalls the tool behind it, so it is called out rather than
  lumped in with work in progress.
- **OHTs** — every vehicle, what each is doing, what it carries and where
  it is headed. Click one to draw its planned route, which is how a congestion
  detour becomes visible. Hovering a vehicle on the map shows the same. A lot
  names the vehicle carrying it and a vehicle names its lot, and both are links,
  so following one to the other does not mean hunting through the other tab.
- **Tools** — utilisation (share of ticks with a lot in process) and queue
  depth per machine. The source shows "—": nothing is ever *in process* there,
  so a percentage would read 100% and mean nothing. This is the fastest way to
  find the bottleneck — on the fab map litho sits near 80% while implant sits
  near 15%.
- **Stats** — the cycle-time distribution as a histogram with p50 and p95
  marked, because two policies can agree on the mean and disagree completely on
  the tail, plus the throughput counters. The histogram spans the observed
  range rather than zero to worst: a fab whose lots all finish between 11k and
  14k ticks would otherwise draw its whole distribution in the last fifth of
  the axis.

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

```
cargo run --release --bin sweep [ticks] [seeds]   # policy sensitivity
cargo run --release --bin fleet [ticks]           # throughput vs fleet size
```

`OHT_MAP` and `OHT_SCENARIO` override what either one runs; `sweep` also takes
`OHT_VEHICLES`. `fleet` answers the question `sweep` depends on — whether
transport is the constraint at all, since the weights cannot move a fab whose
rate is set by its WIP cap.

Prints how far each policy weight moves the fab against the seed-to-seed noise.
Short version: barely at all, on either map and at every fleet size — the whole
configuration space is worth about 5% once collapses and deadlocks are excluded.
`DESIGN.md` has the table and the reason.

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
    maps/fab.json scenarios/fab.json policies/default.json 200000
```

The runner validates the map first and exits non-zero with a problem list if
it fails. The fab needs a long horizon to say anything: mean cycle time is
around 12,000 ticks, so a 20,000-tick run is almost entirely fill transient.

Pass a second policy file to compare two policies on an identical job stream:

```
cargo run --release --bin headless -- \
    maps/fab.json scenarios/fab.json \
    policies/default.json 200000 policies/starvation_biased.json
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
chambers / 120 ticks, visited twice per lot, caps the demo fab near 16.7 lots
per 1000 ticks. The first scenario released 45, so every policy looked
identically terrible. The demo baseline now runs at 12.

**Prepositioning must not commit to a single spur.** Straight-line distance to
a tool is a crude proxy on a directed track graph and ties are common on a
symmetric map, so collapsing the target set to one cell let a coin flip decide
where the whole fleet waited. It also sent every idle vehicle to the *same*
empty spur; the losers arrived to find it taken and re-decided from where they
stood, which is the main line. Keeping the line clear outranks the starvation
preference — see `DESIGN.md` for the full account.

**The weights still barely matter, and the map was not the reason.** The fab
map fixed what it was built to fix — dispatch went from ranking 2.3 candidates a
call to 22.6 — and swept over 400,000 ticks at three fleet sizes the answer is
unchanged. Every large swing is a collapse (`idle.mode = StayPut` completes
nothing) or a setting that wedges a seed, not a tuning gain; the real effects
are `dest_congestion = 0` costing 5% and two route weights worth 2-3% each.
Transport-limiting the fab at 18 vehicles does not wake them up either. The
structural reason is that every lot needs the same 68 moves on a symmetric grid,
so dispatch cannot change how much transport work exists, only how much *empty*
travel is spent reaching it — and `travel_to_pickup` is exactly that term, which
is why it is the one that matters and why it matters as a cliff. What the other
criteria need is heterogeneity: batching, changeover, queue-time windows, rework.
Those mechanics are not flavour, they are what makes the configuration space
non-degenerate. `DESIGN.md` has the table.

**The demo map was too small for the game to work.** Dispatch scores every
(job, vehicle, destination) triple, but on the demo map the mean number of
assignable vehicles was **0.24**, and when dispatch planned at all it ranked a
mean of **2.3 candidates**. A weighted scoring function cannot express anything
with one candidate, so the entire configuration space was inert: ten of twelve
knobs moved throughput less than the seed-to-seed noise. That is what
`maps/fab.json` is for. On it the same numbers are **14.6 assignable vehicles
and 22.6 candidates per planning call**, with litho and the fleet both around
71% busy. `DESIGN.md` has the full comparison.

**The WIP cap is a cliff, not a slope.** At 30 vehicles the fab runs clean for
a million ticks on twelve seeds at caps 28 through 34, and wedges permanently at
36 — on one seed in six, at tick 13,294, during the fill transient. Nothing
degrades on the way there; throughput rises with the cap right up to the edge.
`wip_cap` ships at **32**, below the edge rather than at the throughput
maximum, and `tests/integration.rs` runs six seeds through the fill transient so
a retune cannot quietly land back on the cliff.

**Dispatch cost most of the tick, and did not have to.** On the fab map the
simulation ran at 13.8k ticks/second, 88% of it inside `dispatch::plan`, which
built one full-map distance field per idle vehicle plus one per pending pickup.
Three changes took it to **50.7k** with identical behaviour: search *backwards*
from each pickup so one pass serves every vehicle, do not expand states nothing
can arrive in (most of the state space, on one-way track), and ask for delivery
costs one target at a time instead of filling the map to read three cells of
it. Same lots completed, same p95. A fourth — memoising futile plans — was
tried and removed: worth 30x before those three and nothing after, and it never
applied to a wedged fab anyway, because a wedged fab keeps assigning deliveries
that fail rather than going quiet.

## Known rough edges

- An idle vehicle can still end up on the main line when *every* spur is taken:
  it has nowhere to go, so it keeps circulating rather than stopping, which
  costs track capacity. Both maps have exactly as many spurs as vehicles, so
  there is no slack.
- The delivery-cost term in dispatch seeds its search with an arbitrary legal
  heading out of the pickup, so it can be off by one curve. Immaterial for
  ranking.
- Resource deadlock is detected and reported but never recovered from. That is
  deliberate for now — a wedged fab is a legible failure state, and the WIP cap
  is the control that avoids it — but it means a bad cap produces a run that
  goes quiet rather than one that visibly degrades.
- Tools have separate input and output ports. Real tools mostly have two to
  four *bidirectional* load ports and the FOUP stays on the one it arrived at.
  Modelling that would change where backpressure shows up.
- The fab recipe is one 68-step flow with no batching, no changeover penalty,
  no queue-time windows and no rework. Those are the mechanics still to land;
  see `DESIGN.md`.

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

Two entry points exist for work that is not built yet but is coming:

- `oht_set_policy` swaps the policy on a running world without disturbing the
  fab, which is what a weights UI needs — the point of tuning is watching the
  *same* fab respond to a changed weight, and rebuilding would hide that.
- `oht_validate_map` checks a map document without building a world from it, so
  an editor can report problems while someone is still drawing.
