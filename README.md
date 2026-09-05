# ohtsim

Headless simulation core for an OHT (overhead hoist transport) fab, built to be
compiled both natively for batch policy sweeps and to WASM for a browser UI.

## Layout

```
src/geom.rs      grid, direction bitmask, directed track graph
src/model.rs     vehicles, machines, lots, jobs, ports
src/movement.rs  one-vehicle-per-cell movement resolution  (unit tested)
src/routing.rs   Dijkstra over (cell, heading) with congestion-weighted edges
src/dispatch.rs  weighted scoring over (job, vehicle, destination) triples
src/policy.rs    the configuration space
src/world.rs     the tick loop
src/config.rs    JSON loading
src/json.rs      minimal JSON parser (keeps the crate dependency-free)
src/validate.rs  map validation
src/bin/headless.rs  runner; compares two policies on one job stream
src/bin/trace.rs     per-tick state dump, for diffing against the reference
tests/           whole-tick-loop properties
maps/            fab layouts
scenarios/       what work arrives
policies/        how it is dispatched
reference/       Python prototypes; the behavioural ground truth
                 (trace_sim.py is the Python side of src/bin/trace.rs)
```

## Running

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
preference — see `HANDOFF.md` for the full account.

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
is the open question — see `HANDOFF.md`.

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

## Toward the browser build

Keep `src/` free of rendering dependencies. Add `wasm-bindgen` behind a feature
flag and expose `World::tick` plus `World::snapshot`.

**Do not serialise the snapshot per frame.** `Snapshot` is deliberately
struct-of-arrays; expose pointers into those flat `Vec`s and read WASM linear
memory from JS as typed arrays. Serialising to JSON every tick would eat most of
the reason for choosing Rust.
