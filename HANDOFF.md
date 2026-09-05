# ohtsim — session handoff

Context for a fresh Claude Code session. The project was designed in a chat
session; this captures the decisions and their reasoning so they don't get
relitigated, plus the current state of the code.

---

## What this is

A game about swarm robotics, simulating an OHT (overhead hoist transport)
system — the ceiling-mounted robots that move FOUPs between tools in a
semiconductor fab. Little boxes on rails carrying little boxes.

The intended fun is **tuning dispatch parameters and watching emergent
behaviour**, not building a fab simulator. Planned UI is a cute isometric
rendering on one side, dispatch controls on the other, with speed controls to
run faster than real time. A graphical map editor is a later maybe.

**Why OHT and not self-driving cars** (this was considered and rejected):
self-driving pulls you into continuous space — steering, acceleration,
collision geometry, perception noise — and the coordination problem gets buried
under control problems. OHT gives a discrete graph and discrete decisions, so
every choice is legible and you can see *why* the system jammed. No overtaking
means congestion is genuinely emergent. Throughput is a natural score and
gridlock a natural failure mode.

## Domain background

Real fab OHT is **centralized and hierarchical**, not swarm-like: MES decides a
lot needs moving, MCS turns that into transport jobs and usually computes the
route, the vehicle handles only local reflexes (headway sensing, speed control,
hoist sequence, SEMI E84 handshake at load ports). Junctions use
permission/blocking zones like railway block signaling, not peer-to-peer
negotiation.

Two structural facts matter more than the control architecture for simulation
realism: rails are **unidirectional** (no head-on deadlock, the map is a
directed graph) and vehicles **cannot overtake** (a vehicle doing a 15–20s
hoist cycle blocks everything behind it, so congestion propagates backward like
traffic waves).

Decentralized approaches exist in the literature — contract-net auctions,
pheromone-style congestion avoidance — if you later want the game to lean
harder into emergence.

## Design decisions and why

**Rust core, compiled to WASM for the UI.** Same core runs headless at full
speed for batch policy sweeps and in the browser for the game. Zero
dependencies (hand-rolled JSON parser) to keep the WASM bundle small.

**Policies are data, not code.** This was the central design tension: the user
wanted the widest possible configuration space (sliders felt limiting) but
didn't want simulation internals leaking to the player. Resolved by copying how
fabs actually work — a **weighted scoring function over named criteria**. Fab
IEs tune weights over terms like distance to pickup, lot priority, time waiting,
slack to due date, downstream starvation risk, route congestion.

The richness lives in the *basis*, not in a menu of presets. "Nearest vehicle
first" is not something the player selects — it's what you get when distance
weight is high and everything else is zero. Every term names something a fab
person would recognize; nothing about the movement resolver or tick counters is
exposed. If arbitrary policy code is wanted later, embed Rhai.

Three separate scoring functions, because they produce different pathologies:
job→vehicle assignment, route cost, idle-vehicle repositioning. Plus
**conditional profiles** — trigger conditions that swap the active weight set
(`backlog_above`, `queue_depth_above`, `starvation_above`) — which is how fabs
get policy structure without a scripting language.

**Four config documents, not one:** map (grid + track + machine placement),
scenario (vehicles, arrival rate, recipes, seed), policy (weights). Splitting
them is what lets you run one map against many scenarios and one scenario
against many policies.

**Track as a bitmask, 0–15.** Each cell holds the OR of its *allowed exit
directions* (N=1 E=2 S=4 W=8); 0 means no track. The user's first sketch was
0/1/2/4/8 — one direction per cell — which can't express a junction. The
directed graph is implicit; no edges stored anywhere.

**One cell = one vehicle length.** Load-bearing invariant. If grid resolution
is raised later for smoother animation or finer machine footprints, either
vehicles must claim k cells or they silently get shorter and headway stops being
physical.

**Fixed integer tick, seeded RNG, deterministic.** Comparing policies is
meaningless unless two runs see an identical job stream. Also gives replay and
headless batch runs.

**Headless core with no rendering imports.** `World::snapshot()` returns
struct-of-arrays. For the WASM layer: **do not serialise the snapshot per
frame** — expose pointers into the flat `Vec`s and read linear memory from JS as
typed arrays, or you eat most of the reason for choosing Rust.

**Recipes name machine *kinds*, not specific tools**, so the dispatcher chooses
between identical tools — that's where load balancing lives. Reentrant flows
(lots revisiting litho five or six times in reality) are just repeated kinds,
which concentrates traffic and creates natural hotspots.

**One movement primitive covers driving, curving, and hoisting.** A vehicle has
`ready_in` ticks before it may act again; entering a cell charges by manoeuvre
(straight 1, curve 3 — real OHTs slow substantially through curves, which is the
cheap knob that makes layout geometry matter), and a hoist is the same mechanism
with a much bigger number. A hoisting vehicle is just an ordinary blocked cell
for 20 ticks.

## The movement resolver

Naive per-vehicle iteration breaks trains: six vehicles nose-to-tail all wanting
to advance would each see the cell ahead occupied and stall, so the train creeps
one vehicle per tick and produces phantom congestion. The whole tick resolves as
a unit instead:

1. Each ready vehicle proposes a target cell (or nothing).
2. Contested cells go to one winner by priority rule; losers stall.
3. Iteratively move anyone whose target is now free, until quiescent.
4. Whatever remains forms a wait-graph. Find cycles.

**Deadlock taxonomy — this distinction matters and has tests asserting it:**
- A fully packed cycle where every member wants the next cell is a legal
  **rotation**, moved atomically. Not a deadlock.
- A chain terminating in a hoisting vehicle is a **transient stall**. Not a
  deadlock either — it clears when the hoist finishes.
- Movement deadlock proper appears as a cycle whose members aren't all
  proposing.
- **Resource deadlock** (vehicles waiting on ports that will never free) is a
  separate problem and currently **has no detector**. This is the main gap.

Current merge priority rule: loaded vehicles and long-blocked vehicles win
contested cells. Swapping this for pure FIFO changes fab behaviour a lot and is
worth exposing as a tunable.

## Current state of the code

**The Rust has never been compiled.** No toolchain and no network in the
session where it was written. Expect syntax and borrow-checker errors on first
`cargo build`. The design, however, is validated — see below.

```
src/geom.rs      grid, direction bitmask, directed track graph
src/model.rs     vehicles, machines, lots, jobs, ports
src/movement.rs  movement resolution (has unit tests)
src/routing.rs   Dijkstra over (cell, heading) with congestion-weighted edges
src/dispatch.rs  weighted scoring over (job, vehicle, destination) triples
src/policy.rs    the configuration space
src/world.rs     the tick loop
src/config.rs    JSON loading
src/json.rs      minimal JSON parser
src/bin/headless.rs   runner; compares two policies on an identical job stream
maps/ scenarios/ policies/
reference/       Python prototypes used to validate the design
```

Routing searches over `(cell, heading)` states rather than plain cells, because
curve cost depends on arrival direction. Dispatch uses one distance field per
idle vehicle rather than one Dijkstra per candidate pair, or scoring would cost
thousands of searches per tick.

### How it was validated without a compiler

`reference/resolver_proto.py` prototypes the resolver with tests for the train,
merge, rotation, and false-deadlock cases; those tests were then ported into
`src/movement.rs`. `reference/reference_sim.py` is a full Python implementation
of the same model that runs the demo map end to end. `reference/gen_map2.py`
generates and validates the demo map.

### Two bugs the reference implementation caught

**Parking was on the main loop.** One idle vehicle parks, the loop is severed,
the fab gridlocks — zero lots completed in 6000 ticks. Fixed with parking
**spurs** (short branches that leave the loop and rejoin it). Two rules fall out,
both now implemented: routing must never path *through* a spur
(`Router::set_avoid`), and a vehicle must only target parking that is currently
free — otherwise it queues for an occupied spur while sitting on the main line,
which is the blockage spurs exist to prevent.

**Arrival rate wasn't set against the bottleneck.** litho at 2 tools × 2
chambers / 120 ticks, visited twice per lot, caps the fab near 16.7 lots per
1000 ticks. The scenario released 45, so every policy looked identically
terrible. Baseline is now 12.

After both fixes, over 20 000 ticks:

| | default | starvation_biased |
|---|---|---|
| completed | 77 | 79 |
| p95 cycle | 3191 | 5509 |
| utilisation | 32% | 57% |
| mean backlog | 6.52 | 4.54 |
| deadlocks | 0 | 0 |

Near-identical throughput, very different tails. That tradeoff is the game.

## Known rough edges

- Stuck-vehicle recovery fires 45–160 times per 20k ticks. Not fatal (the
  vehicle reroutes and continues) but the underlying stalls deserve
  investigation rather than a bigger `stuck_threshold`.
- No resource-deadlock detector.
- No buffers or stockers. Backpressure currently shows up as lots stuck inside
  machines when output ports are full — works, but coarser than real
  under-track buffers, and buffers are also a good thing for the player to
  allocate.
- Dispatch's delivery-cost term seeds its distance field with an arbitrary legal
  heading, so it can be off by one curve. Immaterial for ranking.

## Suggested next steps

1. Get it compiling. `cargo test` first — the resolver tests are the ones that
   matter.
2. Port `gen_map2.py`'s validation into Rust **before** building the map editor.
   It checks strong connectivity, dead ends, dangling exit bits, ports on track,
   and that the main line stays strongly connected with every spur removed.
   Without it, every hand-drawn map will strand vehicles in ways that are
   painful to debug from an isometric view.
3. Add buffers/stockers and a resource-deadlock detector.
4. Then the WASM shim and the isometric renderer.
