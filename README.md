# ohtsim

Headless simulation core for an OHT (overhead hoist transport) fab, built to be
compiled both natively for batch policy sweeps and to WASM for a browser UI.

## Status — read this first

**The Rust has never been compiled.** There was no Rust toolchain available
where this was written and no network to install one, so `cargo build` will
almost certainly surface some syntax or borrow-checker errors on first run.
The *design*, however, has been validated: the movement resolver has unit tests
ported from a working Python prototype, and a full Python reference
implementation of the same model runs the demo map end to end without deadlock.

Expect to spend a short while on compile errors, not on debugging the model.

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
maps/            fab layouts
scenarios/       what work arrives
policies/        how it is dispatched
```

## Running

```
cargo test
cargo run --release --bin headless -- \
    maps/demo_loop.json scenarios/baseline.json policies/default.json 20000
```

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

Four documents on purpose — one map runs against many scenarios, one scenario
against many policies. That separation is what makes experiments possible.

Policies are weighted scoring functions over named criteria, the way a fab IE
tunes an MCS. No policy is a preset: "nearest vehicle first" is just what you
get when `travel_to_pickup` dominates and everything else is zero. Conditional
profiles swap the active weights when a trigger fires (`backlog_above`,
`queue_depth_above`, `starvation_above`), which gives policy structure without a
scripting language. `always` is the fallback and is sorted last so it cannot
shadow the conditional profiles.

## Findings from the reference run

Two things the Python reference caught that were not obvious on paper:

**Parking must be on spurs.** The first demo map put parking cells directly on
the main loop. One idle vehicle parks, the loop is severed, and the fab
gridlocks — zero lots completed in 6000 ticks. Spurs are short branches that
leave the loop and rejoin it. Two rules fall out and both are implemented:
routing must never path *through* a spur (`Router::set_avoid`), and a vehicle
must only ever target parking that is currently free.

`gen_map2.py` validates any new map for this: strong connectivity, no dangling
exit bits, no dead ends, ports on track, and the main line still strongly
connected with every spur cell removed.

**Arrival rate has to be set against the bottleneck.** litho at 2 tools x 2
chambers / 120 ticks, visited twice per lot, caps the fab near 16.7 lots per
1000 ticks. The first scenario released 45, so every policy looked identically
terrible. The baseline now runs at 12.

With those fixed, on 20 000 ticks:

| | default | starvation_biased |
|---|---|---|
| completed | 77 | 79 |
| p95 cycle | 3191 | 5509 |
| utilisation | 32% | 57% |
| mean backlog | 6.52 | 4.54 |
| deadlocks | 0 | 0 |

Nearly identical throughput, wildly different tails. That tradeoff is the game.

## Known rough edges

- Stuck-vehicle recovery fires 45–160 times per 20k ticks. Not fatal — the
  vehicle reroutes and carries on — but the underlying stalls are worth
  investigating rather than papering over with a bigger `stuck_threshold`.
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
