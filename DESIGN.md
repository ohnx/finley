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
| lots created | 177 | 170 |
| arrivals deferred | 23 | 30 |
| completed | 164 | 156 |
| p95 cycle | 2210 | 2249 |
| utilisation | 97% | 98% |
| mean backlog | 2.89 | 3.35 |
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
  are untuned, and they trade against the WIP cap rather than replacing it.
- Dispatch's delivery-cost term seeds its distance field with an arbitrary legal
  heading, so it can be off by one curve. Immaterial for ranking.

## The deadlock that actually happened

Recipes are reentrant — litho, etch, cmp, litho — and every place a lot can rest
is finite: input ports, chambers, output ports, buffer slots. Admit enough lots
to fill them all and the tools end up holding finished lots for each other in a
cycle:

```
litho1/litho2  hold finished lots needing etch,  out-ports full
etch1/etch2    hold finished lots needing cmp,   out-ports full
cmp1/cmp2      hold finished lots needing litho, out-ports full
```

Nothing in transit, vehicles idle, jobs pending that can never be assigned. It
is permanent: the fab completes 647 lots and then nothing, ever. The metrics
look merely disappointing rather than broken, which is why it went unnoticed.

**The fix is release control, not storage.** Buffers were tried first and were
the wrong answer — they moved the failure from tick 12,000 to tick 90,000 and
made it look fixed at any horizon short of that. Storage is finite too; filling
it just takes longer.

Capping work in progress is what actually prevents it, and it is what real fabs
do. The scenario now carries `wip_cap`: a lot that would arrive while the fab is
at its cap is not released, and waits outside the line.

### The operating curve

Sweeping the cap over 300 000 ticks, default policy:

| cap | completed | throughput | p95 cycle | mean WIP | deadlock |
|---|---|---|---|---|---|
| none | 647 | 2.16 | 5008 | 28.6 | **terminal** |
| 8 | 1865 | 6.22 | 1481 | 7.2 | none |
| 12 | 2290 | 7.63 | 1856 | 10.6 | none |
| **16** | **2372** | **7.91** | 2636 | 14.2 | none |
| 20 | 2329 | 7.76 | 3471 | 17.8 | none |
| 24 | 2210 | 7.37 | 4488 | 22.0 | none |
| 28 | 1096 | 3.65 | 5279 | 26.2 | **terminal** |

This is the classic characteristic curve. Throughput peaks around 16 and then
falls while cycle time keeps climbing — past the peak, extra WIP buys queueing
and nothing else. The demo scenario ships at 16, which is also roughly what
Little's law predicts from the bottleneck, and which leaves a wide margin below
the ~28 where the fab dies.

Capped at 16 the fab runs clean for **1 000 000 ticks** under both policies. The
earlier claim that buffers fixed it rested on a 50 000-tick run, which was
simply too short to see the failure — worth remembering before declaring this
one fixed either.

### What buffers are actually for

Not this. At the shipped cap the fab completes the same number of lots with them
and without (2372 against 2385 — marginally *better* without, since a buffered
lot costs two transport moves where one would do).

What they buy is tolerance. They widen the range of caps that stay safe: at a
cap of 24 the fab runs with buffers and dies without. So they turn a
badly-set cap into degraded throughput rather than a dead fab, which is worth
having, and there is a test asserting exactly that and nothing more.

### A note on load ports

Real 300mm tools do **not** have separate input and output ports. They have two
to four SEMI E15.1 load ports, and those are bidirectional: a FOUP docks at one,
the EFEM robot moves wafers into the tool and returns them to the same FOUP on
the same port, and the carrier sits there for the whole visit.

Worth knowing, and worth not confusing with the deadlock. Bidirectional load
ports would not have fixed it either — the cycle would form on load ports
instead of output ports. The in/out split is kept for now because it is legible:
a lot visibly moves from an in-bay to an out-bay, and backpressure is obvious on
the map.

## Fleet size and the spur bug

Half the fleet was dead for most of this project's life. Vehicles that parked on
the inner cell of a two-cell spur could not route out of it — routing refuses to
path *through* a spur, and taken literally that forbids leaving one too — so
they were unreachable in every distance field, never scored by dispatch, and
never moved. The `default` policy's 49% utilisation was four working vehicles
out of eight.

With all of them live the map turned out to be over-fleeted. Throughput peaks
around five to seven vehicles and collapses at eight: 65% of vehicle time
blocked, an 83-cell loop saturated by 20-tick hoists with no overtaking. Between
six and seven it is also chaotic, swinging between 165 and 1780 lots depending
on the WIP cap. The scenario ships **five**, which is stable and beats the old
eight-with-four-dead.

Two lessons worth keeping. A fab with half a fleet still completes lots, so
nothing failed loudly — there is now a test asserting every vehicle carries a
job. And fleet size and WIP cap interact: they are two knobs on the same curve,
and neither can be tuned alone.

## Do the weights actually do anything?

The design premise is that the interesting behaviour lives in the *basis* of
weighted criteria rather than in a menu of presets. `cargo run --release --bin
sweep` tests that: it moves one knob at a time from the shipped default over
200 000 ticks and four seeds, and prints the seed-to-seed spread so an effect
can be read against the noise it has to beat.

At the shipped operating point (five vehicles, cap 16) the answer is mostly no.
Throughput swing across each knob's whole range, against a seed-to-seed spread
of **±0.03**:

| knob | swing | |
|---|---|---|
| `idle.mode` | 8.33 | `StayPut` collapses the fab to 0.27/1k |
| `dispatch.travel_to_pickup` | 4.26 | all of it between 0 and 1; ≥1 is flat |
| `dispatch.dest_congestion` | 1.68 | monotonic, and lower is better than the default |
| `dispatch.lot_wait` | 0.65 | raising it hurts |
| `dispatch.dest_queue` | 0.44 | |
| `dispatch.dest_starvation` | 0.27 | |
| `dispatch.steps_remaining` | 0.09 | |
| `dispatch.lot_priority` | 0.07 | |
| `route.congestion` | 0.06 | |
| `route.curve` | 0.05 | |
| `congestion_decay` | 0.04 | at the noise floor |
| `idle.dwell_before_move` | 0.00 | bit-identical runs |

Two of the three that move anything are "do not set this stupidly" rather than
tuning: `StayPut` parks vehicles on the main line, and `travel_to_pickup = 0`
means ignoring how far away the vehicle is. Above those cliffs everything is
flat. Tuning the best settings together gains 1.4% over the shipped default, and
they do not compose — `travel_to_pickup` and `dest_congestion` are substitutes,
not additive.

### Why: dispatch has nothing to choose between

At five vehicles the fab is transport-saturated at 97.7% utilisation, and the
mean number of vehicles available to assign is **0.24**. Both more than one
idle vehicle and more than one pending job — an actual choice — happens on
**2.3% of ticks**. A weighted scoring function over criteria cannot express
anything when there is one candidate.

At six vehicles that rises to 22.7% of ticks, ten times the choice. But six is
also where the fab turns chaotic: the seed-to-seed spread goes from ±0.03 to
**±2.69**, as large as the effects themselves, so the bigger swings there are
the job stream rather than the policy.

So the configuration space is currently inert, and the game premise does not yet
hold on this map. The fix is not a policy change: the loop is too small for a
fleet with slack in it, so there is never a decision worth making. A larger map
with more track per vehicle would give dispatch real choices without tipping
into congestion collapse — that is now the most interesting thing to try, and it
is a level-editor question rather than a tuning one.

`idle.dwell_before_move` deserves a note: it is not dead code, it is dead under
`NearestPark`. A parked vehicle's own spur is always among its targets, so it
never decides to move and the dwell timer never gates anything. Under
`Preposition` it is live (8.54 down to 8.48 across the range).

## Open questions

1. **Retune the shipped policies so they show a tradeoff again** (see above).
   This is the one that decides whether the game premise holds up, and the UI
   now makes it possible to watch what each policy actually does.
2. ~~Buffers and stockers, and a resource-deadlock detector.~~ Done, along with
   the WIP cap that is the actual fix. Cap, buffer count and buffer placement
   are all untuned and all trade against each other — exactly the sort of thing
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
