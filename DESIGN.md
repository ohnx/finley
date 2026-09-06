# finley — design notes

Why the thing is built the way it is. Most of what follows is reasoning that
isn't recoverable from the code: options that were considered and rejected,
invariants that look arbitrary until you know what broke without them, and
numbers that were measured rather than guessed.

`README.md` covers what the pieces are and how to run them. This covers why.

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

**Three config documents, not one:** map (grid + track + machine placement),
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
- **Resource deadlock** (lots waiting on ports that will never free) is a
  separate problem with its own detector, and it turned out to be real rather
  than theoretical — see "The deadlock that actually happened" below.

Current merge priority rule: loaded vehicles and long-blocked vehicles win
contested cells. Swapping this for pure FIFO changes fab behaviour a lot and is
worth exposing as a tunable.

## Implementation notes

The core has no dependencies, deliberately: the same crate compiles to wasm for
the UI and every byte of dependency shows in the bundle, which is also why the
JSON parser is hand-rolled.

Routing searches over `(cell, heading)` states rather than plain cells, because
curve cost depends on arrival direction. Dispatch builds one distance field per
idle vehicle rather than one Dijkstra per candidate pair; the other way round
costs thousands of searches per tick.

## The web UI

Top-down, not the isometric this document originally specified. Isometric is
the eventual game's look; what was needed first was a view that makes emergent
congestion legible, and a flat heat overlay does that better than a diagonal
projection. It draws from the same snapshot data, so isometric can layer on
later without touching the core.

The page ticks the real simulation rather than replaying a recording, because
the loop the game is about — change a weight, watch the fab respond — does not
exist otherwise.

The boundary is deliberately thin. `crates/ohtsim-wasm` is a raw C ABI rather
than wasm-bindgen, so the core stays dependency-free and there is no JS
toolchain to keep current. Two rules hold it together:

- **Nothing is serialised per frame.** `World::snapshot_into` refills flat
  `Vec`s in place and the shim hands JS pointers into them. Serialising every
  tick would eat most of the reason for choosing Rust.
- **Static map geometry never crosses the boundary.** JS already fetched the
  map JSON to construct the world, so it reads track bits, machine rectangles
  and port cells from that. One source of truth.

On the JS side: re-derive every typed-array view from `memory.buffer` each
frame, because growing wasm memory detaches the old ones, and treat every
pointer as valid only until the next tick.

Both targets must agree. The sim is deterministic, so if wasm and native
diverge, something in the port is target-dependent and the UI is showing a
different fab from the one the runner reports on — `web/verify.mjs` guards
that. Checked in a real browser: paused and stepped to exactly tick 2000, the page
and the runner both give 24 lots created, 8 completed, 45.2% utilisation, 3.34
mean backlog.

Machine footprints are drawn *under* the track. That is not an overlap bug:
rails are ceiling-mounted, so they legitimately run over tools, and the sim
ignores machine `w`/`h` entirely — they are presentational only.

### How the design was validated before any of it compiled

`reference/resolver_proto.py` prototypes the resolver with tests for the train,
merge, rotation, and false-deadlock cases; those tests were then ported into
`src/movement.rs`. `reference/reference_sim.py` is a full Python implementation
of the same model that runs the demo map end to end. `reference/gen_map2.py`
generates and validates the demo map.

### Two bugs the reference implementation caught

**Parking was on the main loop.** One idle vehicle parks, the loop is severed,
the fab gridlocks — zero lots completed in 6000 ticks. Fixed with parking
**spurs** (short branches that leave the loop and rejoin it). Two rules fall
out: routing must never path *through* a spur (`Router::set_avoid`), and a
vehicle must only target parking that is currently free — otherwise it queues
for an occupied spur while sitting on the main line, which is the blockage spurs
exist to prevent.

The second rule was only half-implemented: it checked which spurs were occupied
*now* but not which ones another vehicle was already driving to, so two vehicles
still committed to the same empty spur. See the porting bugs below.

**Arrival rate wasn't set against the bottleneck.** litho at 2 tools × 2
chambers / 120 ticks, visited twice per lot, caps the fab near 16.7 lots per
1000 ticks. The scenario released 45, so every policy looked identically
terrible. Baseline is now 12.

### Three bugs found porting to Rust

Once it compiled, `default` reproduced the Python reference exactly — 77 lots,
p95 3191, 31.9% utilisation, 6.52 backlog, 45 stuck recoveries.
`starvation_biased` did
not: 11 lots against the reference's 79, with arrivals themselves choked off
because the source had backed up. `src/bin/trace.rs` dumps per-tick state so
the two implementations can be diffed line by line; that located all three.

- **Vehicle placement strided** through the parking pool as
  `pool[(i * 7 + 1) % len]` instead of walking it in order. Parking is listed
  as spur pairs, so map order is meaningful.
- **The hungriest tool was picked with `max_by`**, which returns the *last*
  maximum. Early in a run every tool is equally starved, so that tie-break
  alone decided where the whole fleet went to wait.
- **Prepositioning collapsed its target set to a single spur.** This was the
  real one. Straight-line distance to a tool is a crude proxy on a directed
  track graph and ties are common on a symmetric map, so a coin flip decided
  where the fleet waited — flipping it moved throughput from 11 lots to 72.
  Worse, committing to one cell meant every idle vehicle chose the *same*
  empty spur; the losers arrived to find it taken and re-decided from where
  they stood, which is the main line. An idle vehicle sat on the loop for 43%
  of the run, worst at the sink and cmp2 port cells.

The fix: exclude spurs another vehicle is already driving to, keep every spur
tied for nearest and let the router choose among them by real route cost, and
fall back to any free spur when the starvation-biased target is unreachable.
Keeping the line clear outranks the starvation preference.

### Where that leaves the numbers

Over 20 000 ticks, after the fix:

| | default | starvation_biased |
|---|---|---|
| lots created | 168 | 177 |
| completed | 144 | 165 |
| p95 cycle | 3741 | 2448 |
| utilisation | 49% | 94% |
| mean backlog | 7.68 | 4.07 |
| deadlocks | 0 | 0 |
| resource deadlocks | 0 | 0 |
| stuck recoveries | 0 | 0 |

**This invalidates the design claim the old table supported.** The previous
numbers showed near-identical throughput with very different tails, and the
conclusion drawn was "that tradeoff is the game." That tradeoff was partly an
artifact of the prepositioning bug. `starvation_biased` now *dominates*
`default` on every axis — throughput, tail latency, utilisation, backlog — so
the two shipped policies no longer demonstrate a tradeoff at all. One is simply
better.

Both are still source-limited: the scenario offers ~240 lots over 20 000 ticks
and neither creates that many, because backpressure at the source throttles
arrivals. So `default` is not hitting a physical ceiling, it is just leaving
the fleet idle 68% of the time.

Making the tradeoff visible again is now the most interesting open question,
and it is a game-design question rather than a bug: it probably means tuning
the shipped weight sets so each wins on a different axis, and raising the
arrival rate until the fleet is the binding constraint rather than the source.

## Known rough edges

- ~~Stuck-vehicle recovery fires 45 times per 20k ticks.~~ Now zero under both
  policies. The stalls this flagged for investigation turned out to be vehicles
  halting on the main line with no spur free — see below.
- ~~An idle vehicle can end up stopped on the main line when every spur is
  taken.~~ Such a vehicle circulates instead: it loiters around the loop one hop
  at a time and parks the moment a spur frees. Rails are one-way and nothing
  overtakes, so a halted vehicle is a wall — that accounted for 9,413
  vehicle-ticks of blocking per 20k under `default`, and 136 afterwards. The
  remainder is the single tick between finishing a job and deciding where next.

  One consequence to keep in mind: utilisation counts any vehicle that is not
  parked, so circulating registers as busy. The default policy's figure rose
  from 32% to 49% without any more transport being done. If that number starts
  carrying weight, it should count only vehicles on a job.
- ~~No resource-deadlock detector.~~ There is one now, and it found a real
  deadlock — see below.
- ~~No buffers or stockers.~~ Three overhead hoist buffers on the demo map,
  six slots. Still a good thing for the player to allocate: count and placement
  are untuned.
- Dispatch's delivery-cost term seeds its distance field with an arbitrary legal
  heading, so it can be off by one curve. Immaterial for ranking.

## The deadlock that actually happened

Reentrant recipes and one output port per tool are enough to deadlock the fab,
and it is not a rare corner. Under the default policy the cycle closed at around
tick 11,900 every run:

```
litho1/litho2  hold finished lots needing etch,  out-ports full
etch1/etch2    hold finished lots needing cmp,   out-ports full
cmp1/cmp2      hold finished lots needing litho, out-ports full
```

Nothing in transit, vehicles idle, jobs pending that can never be assigned
because no destination input port can ever free. The metrics looked merely
disappointing rather than broken, which is why it went unnoticed for so long.

**Buffers are the fix, and they are what real fabs use.** Overhead hoist buffers
and under-track storage exist precisely so a finished lot has somewhere to go
that is not another tool's port. The demo map now has three, six slots total,
spread around the loop rather than pooled.

Two rules make them pay for themselves rather than cost throughput:

- **Buffering must unblock something.** A lot is only sent to a buffer when no
  tool of the kind it needs can take it *and* the tool it is sitting on has a
  finished lot that cannot reach an output port. Without that second condition
  every buffered lot costs two transport moves where one would do; on a fleet
  already near saturation that is pure loss, and it cost `starvation_biased`
  30 lots and doubled its p95 before the condition was added.
- **A buffered lot is never re-buffered.** Otherwise two buffers pass it back
  and forth while it makes no progress through its recipe.

With both: zero resource deadlocks over 50 000 ticks under either policy, and
throughput up from 85 to 144 lots for `default` and 159 to 165 for
`starvation_biased`.

### A note on load ports

Real 300mm tools do **not** have separate input and output ports. They have two
to four SEMI E15.1 load ports, and those are bidirectional: a FOUP docks at one,
the EFEM robot moves wafers into the tool and returns them to the same FOUP on
the same port, and the carrier sits there for the whole visit.

Worth knowing, and worth not confusing with the deadlock. Switching to
bidirectional load ports would *not* have fixed it — the cycle would just form
on load ports instead of output ports. What breaks a circular wait is somewhere
to put a lot that is not a tool port at all. The in/out split is kept for now
because it is legible: a lot visibly moves from an in-bay to an out-bay, and
backpressure is obvious on the map.

## Open questions

1. **Retune the shipped policies so they show a tradeoff again** (see above).
   This is the one that decides whether the game premise holds up, and the UI
   now makes it possible to watch what each policy actually does.
2. ~~Buffers and stockers, and a resource-deadlock detector.~~ Done. Buffer
   count and placement are untuned, though, and are exactly the sort of thing
   the player should be deciding.
3. Bidirectional load ports, to match real tools (see above). A model change,
   not a fix — the deadlock is already handled.
4. Policy editing in the UI. The weights *are* the game; exposing them as live
   sliders is the obvious next step, and needs the policy struct crossing the
   FFI boundary rather than only the config JSON.
5. An isometric renderer, over the same snapshot data the top-down one uses.
6. A map editor. `validate.rs` exists so that it can surface `Problem::cell` as
   a highlight rather than letting people draw fabs that strand vehicles.

## Validating changes

`reference/reference_sim.py` is the behavioural ground truth and it still runs:

```
python3 reference/reference_sim.py 20000
```

`src/bin/trace.rs` emits one line of world state per tick in a format close
enough to diff against it, which is how the three bugs above were found. It is
worth keeping that comparison working — but note the reference has quirks of
its own that are *not* design decisions. Its parking tie-break, for instance,
falls out of CPython's set iteration order. Where Rust and Python disagree,
decide which is right on the merits rather than making Rust bug-compatible.
