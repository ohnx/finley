"""
Prototype + tests for the OHT movement resolver.
Verifies the algorithm before porting to Rust.

Model:
  - cells are ints, one vehicle per cell
  - each ready vehicle proposes a target cell; not-ready vehicles propose None
  - resolution must:
      (a) advance a nose-to-tail train fully in one tick
      (b) pick one winner when two vehicles contest a cell
      (c) rotate a fully-packed cycle atomically
      (d) report cycles that cannot rotate
"""

from typing import Optional


def resolve(occupancy: dict, proposals: dict, priority):
    """
    occupancy: cell -> vehicle_id
    proposals: vehicle_id -> Optional[cell]   (None = not moving this tick)
    priority:  vehicle_id -> sort key (lower wins a contested cell)

    returns (moves, stalled, deadlocked_cycles)
      moves: vehicle_id -> destination cell
    """
    pos = {v: c for c, v in occupancy.items()}

    # --- Phase 1: contested-cell arbitration -------------------------------
    # Group proposals by target. Only one vehicle may claim a target.
    claims: dict = {}
    for v, tgt in proposals.items():
        if tgt is None:
            continue
        claims.setdefault(tgt, []).append(v)

    active = {}          # vehicle -> target, after arbitration
    lost = set()
    for tgt, contenders in claims.items():
        contenders.sort(key=priority)
        active[contenders[0]] = tgt
        lost.update(contenders[1:])

    # --- Phase 2: iterative settle ----------------------------------------
    # Move anyone whose target is currently free; repeat until quiescent.
    moves = {}
    free = lambda c: c not in occupancy
    occupancy = dict(occupancy)  # local mutable copy

    progress = True
    while progress:
        progress = False
        for v, tgt in list(active.items()):
            if v in moves:
                continue
            if free(tgt):
                src = pos[v]
                del occupancy[src]
                occupancy[tgt] = v
                moves[v] = tgt
                progress = True

    # --- Phase 3: cycles among the remainder ------------------------------
    # Build wait-graph over vehicles that still haven't moved: v -> blocker.
    remaining = [v for v in active if v not in moves]
    waits_on = {}
    for v in remaining:
        tgt = active[v]
        blocker = occupancy.get(tgt)
        if blocker is not None:
            waits_on[v] = blocker

    cycles = find_cycles(waits_on)

    deadlocked = []
    for cyc in cycles:
        # A cycle can rotate only if every member is actively proposing a move
        # (i.e. is in `active`) and each waits on the next member of the cycle.
        if all(v in active for v in cyc):
            for v in cyc:
                moves[v] = active[v]
        else:
            deadlocked.append(cyc)

    stalled = set(lost) | {v for v in active if v not in moves}
    return moves, stalled, deadlocked


def find_cycles(waits_on: dict):
    """Cycles in a functional graph (each node has <=1 outedge)."""
    seen = set()
    cycles = []
    for start in waits_on:
        if start in seen:
            continue
        path, idx = [], {}
        node = start
        while node is not None and node not in seen:
            if node in idx:
                cycles.append(path[idx[node]:])
                break
            idx[node] = len(path)
            path.append(node)
            node = waits_on.get(node)
        seen.update(path)
    return cycles


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------

def t_train():
    """Six vehicles nose-to-tail on cells 0..5, all advancing +1.
    Vehicle 5 (the leader) has free space ahead."""
    occ = {i: f"v{i}" for i in range(6)}
    props = {f"v{i}": i + 1 for i in range(6)}
    moves, stalled, dead = resolve(occ, props, lambda v: int(v[1:]))
    assert len(moves) == 6, moves
    assert not stalled and not dead
    print("train:            all 6 advance in one tick  OK")


def t_train_blocked():
    """Same train, but leader is hoisting (proposes None). Nobody moves."""
    occ = {i: f"v{i}" for i in range(6)}
    props = {f"v{i}": i + 1 for i in range(5)}
    props["v5"] = None
    moves, stalled, dead = resolve(occ, props, lambda v: int(v[1:]))
    assert moves == {}, moves
    assert len(stalled) == 5
    print("train blocked:    0 move, 5 stall behind hoist  OK")


def t_merge():
    """Two vehicles contest cell 10. Lower priority number wins."""
    occ = {1: "a", 2: "b"}
    props = {"a": 10, "b": 10}
    pri = {"a": 1, "b": 0}
    moves, stalled, dead = resolve(occ, props, lambda v: pri[v])
    assert moves == {"b": 10}, moves
    assert stalled == {"a"}
    print("merge:            one wins, one stalls  OK")


def t_rotation():
    """Ring of 4 cells, fully packed, everyone advancing. Must rotate."""
    occ = {0: "a", 1: "b", 2: "c", 3: "d"}
    props = {"a": 1, "b": 2, "c": 3, "d": 0}
    moves, stalled, dead = resolve(occ, props, lambda v: v)
    assert len(moves) == 4, moves
    assert not dead
    print("rotation:         packed ring rotates atomically  OK")


def t_broken_cycle():
    """Packed ring where one vehicle is hoisting. Chain stalls, no rotation,
    and it is NOT reported as deadlock (the hoist will finish)."""
    occ = {0: "a", 1: "b", 2: "c", 3: "d"}
    props = {"a": 1, "b": 2, "c": 3, "d": None}
    moves, stalled, dead = resolve(occ, props, lambda v: v)
    assert moves == {}, moves
    assert dead == [], dead
    print("broken cycle:     stalls without false deadlock  OK")


def t_two_trains_merge():
    """Two trains converging on a shared cell 100."""
    occ = {0: "a0", 1: "a1", 50: "b0", 51: "b1"}
    props = {"a0": 100, "a1": 0, "b0": 100, "b1": 50}
    pri = {"a0": 0, "a1": 1, "b0": 2, "b1": 3}
    moves, stalled, dead = resolve(occ, props, lambda v: pri[v])
    # a0 wins the merge, a1 follows into the vacated cell 0.
    # b0 loses and stalls, so b1 stays put behind it.
    assert moves == {"a0": 100, "a1": 0}, moves
    assert stalled == {"b0", "b1"}, stalled
    print("two trains merge: winner's train flows, loser's holds  OK")


# Guarded so reference_sim.py can import `resolve` without the test output
# landing in the middle of its own.
if __name__ == "__main__":
    for t in (t_train, t_train_blocked, t_merge, t_rotation,
              t_broken_cycle, t_two_trains_merge):
        t()
    print("\nall resolver tests passed")
