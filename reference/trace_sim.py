"""
Python side of the divergence trace.

Emits one line of world state per tick in the same format as
`src/bin/trace.rs`, so the Rust port can be diffed against this reference
line by line:

    python3 reference/trace_sim.py policies/starvation_biased.json 2000 > py.trace
    cargo run --release --bin trace -- maps/demo_loop.json \
        scenarios/baseline.json policies/starvation_biased.json 2000 > rs.trace
    diff py.trace rs.trace | head

The first differing tick is the one to investigate. This is how the three
porting bugs in HANDOFF.md were found.

Note that the reference has quirks that are *not* design decisions -- its
parking tie-break falls out of CPython's set iteration order, for one -- so a
divergence is a question to answer on the merits, not automatically a Rust bug.
"""
import json, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reference_sim as R

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
CODE = {"idle": "i", "to_pickup": "p", "loading": "L",
        "to_dropoff": "d", "unloading": "U", "repos": "r"}


def main():
    policy = sys.argv[1] if len(sys.argv) > 1 else "policies/default.json"
    ticks = int(sys.argv[2]) if len(sys.argv) > 2 else 2000

    mapcfg = json.load(open(os.path.join(BASE, "maps/demo_loop.json")))
    scen = json.load(open(os.path.join(BASE, "scenarios/baseline.json")))
    pol = json.load(open(os.path.join(BASE, policy)))
    # Matches the Rust config loader, which sorts `always` last so it cannot
    # shadow the conditional profiles.
    pol["profiles"].sort(key=lambda p: p["trigger"]["type"] == "always")

    w = R.World(mapcfg, scen, pol)
    out = []
    for _ in range(ticks):
        w.tick()
        veh = " ".join(
            f"{CODE[v['state'][0]]}{v['cell']}:{v['ready']}:{len(v['route'])}"
            for v in w.vehicles
        )
        out.append(f"{w.tick_n} prof={w.profile} pend={len(w.pending)} "
                   f"lots={len(w.lots)} done={w.m['completed']} | {veh}")
    print("\n".join(out))


if __name__ == "__main__":
    main()
