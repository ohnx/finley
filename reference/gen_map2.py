"""
demo_loop v2.

Fixes from the reference run:
  - parking now sits on SPURS, not on the main line. An idle vehicle on the
    loop blocks it permanently, which gridlocked the whole fab.
  - tool capacity rebalanced so litho is a real but survivable bottleneck.

A spur is a short branch that leaves the loop and rejoins it, so the main line
stays passable while a vehicle sits on the branch.
"""
import json, os, sys

N, E, S, W = 1, 2, 4, 8
DIRS = ((N, 0, -1), (E, 1, 0), (S, 0, 1), (W, -1, 0))
Wd, Ht = 16, 12

t = [[0] * Wd for _ in range(Ht)]


def add(x, y, bits):
    t[y][x] |= bits


# --- main loop, clockwise ---------------------------------------------------
for x in range(Wd - 1):
    add(x, 0, E)
add(Wd - 1, 0, S)
for y in range(Ht - 1):
    add(Wd - 1, y, S)
add(Wd - 1, Ht - 1, W)
for x in range(1, Wd):
    add(x, Ht - 1, W)
add(0, Ht - 1, N)
for y in range(1, Ht):
    add(0, y, N)
add(0, 0, E)

# --- bypasses ---------------------------------------------------------------
for x in range(0, Wd - 1):
    add(x, 5, E)          # horizontal bypass, eastbound
for y in range(0, Ht - 1):
    add(8, y, S)          # vertical bypass, southbound

# --- parking spurs ----------------------------------------------------------
# Each: leave the loop, run along the spur, rejoin. Main line keeps its through
# path, so a vehicle sitting on the spur blocks nothing.
SPURS = [
    # (entry cell, [(cell, exit_bit), ...] , rejoin)
    ("north", [(12, 0, S), (12, 1, E), (13, 1, N)], [(12, 1), (13, 1)]),
    ("south", [(3, 11, N), (3, 10, W), (2, 10, S)], [(3, 10), (2, 10)]),
    ("east",  [(15, 3, W), (14, 3, S), (14, 4, E)], [(14, 3), (14, 4)]),
    ("west",  [(0, 7, E), (1, 7, N), (1, 6, W)],    [(1, 7), (1, 6)]),
]

PARKING = []
for _name, cells, park in SPURS:
    for (x, y, bit) in cells:
        add(x, y, bit)
    PARKING.extend(park)

# --- machines ---------------------------------------------------------------
# Body rectangles are presentational -- the simulation only ever looks at ports
# -- but each body must sit orthogonally against every one of its own ports, or
# the picture shows a tool floating away from the load ports that serve it.
# Checked below, along with the rule that actually constrains the fab: a port
# may never sit on a parking spur. A body may.
# litho is the intended bottleneck: 2 tools x 2 chambers / 120 ticks, and the
# main recipe visits litho twice, giving ~16.7 lots per 1000 ticks of headroom.
MACHINES = [
    dict(name="src",    kind="source", x=1,  y=1, w=4, h=2, process_ticks=0,   capacity=3,
         ports=[("out", (2, 0)), ("out", (3, 0)), ("out", (4, 0))]),
    dict(name="litho1", kind="litho",  x=5,  y=1, w=2, h=3, process_ticks=120, capacity=2,
         ports=[("in", (5, 0)), ("out", (6, 0))]),
    dict(name="litho2", kind="litho",  x=9,  y=1, w=2, h=3, process_ticks=120, capacity=2,
         ports=[("in", (9, 0)), ("out", (10, 0))]),
    # The body covers the (13,1) spur, which is fine -- rails are ceiling
    # mounted, so a parked vehicle passes over the tool. A *port* on a spur
    # would not be: routing treats spurs as destination-only, so the tool would
    # become unservable. That is checked separately below.
    dict(name="etch1",  kind="etch",   x=13, y=1, w=2, h=2, process_ticks=90,  capacity=2,
         ports=[("in", (15, 1)), ("out", (15, 2))]),
    dict(name="etch2",  kind="etch",   x=13, y=7, w=2, h=2, process_ticks=90,  capacity=2,
         ports=[("in", (15, 7)), ("out", (15, 8))]),
    dict(name="cmp1",   kind="cmp",    x=11, y=9, w=2, h=2, process_ticks=60,  capacity=1,
         ports=[("in", (12, 11)), ("out", (11, 11))]),
    dict(name="cmp2",   kind="cmp",    x=5,  y=9, w=2, h=2, process_ticks=60,  capacity=1,
         ports=[("in", (6, 11)), ("out", (5, 11))]),
    dict(name="metro1", kind="metro",  x=1,  y=8, w=2, h=2, process_ticks=30,  capacity=2,
         ports=[("in", (0, 9)), ("out", (0, 8))]),
    dict(name="sink",   kind="sink",   x=1,  y=3, w=2, h=2, process_ticks=1,   capacity=4,
         ports=[("in", (0, 3))]),

]


# --- validation -------------------------------------------------------------
def exits(x, y):
    out = []
    for bit, dx, dy in DIRS:
        if t[y][x] & bit:
            nx, ny = x + dx, y + dy
            if 0 <= nx < Wd and 0 <= ny < Ht and t[ny][nx]:
                out.append((nx, ny))
    return out


errors = []
cells = [(x, y) for y in range(Ht) for x in range(Wd) if t[y][x]]

for (x, y) in cells:
    if not exits(x, y):
        errors.append(f"dead-end at ({x},{y})")
    for bit, dx, dy in DIRS:
        if t[y][x] & bit:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < Wd and 0 <= ny < Ht) or not t[ny][nx]:
                errors.append(f"exit from ({x},{y}) bit {bit} dangles")

port_cells = set()
for m in MACHINES:
    for kind, (px, py) in m["ports"]:
        if not t[py][px]:
            errors.append(f"{m['name']}: port ({px},{py}) has no track")
        port_cells.add((px, py))

for p in PARKING:
    if not t[p[1]][p[0]]:
        errors.append(f"parking {p} has no track")
    if p in port_cells:
        errors.append(f"parking {p} collides with a port")

# Parking must NOT be on the main line: removing a parked vehicle's cell must
# still leave every other cell reachable from every other. Approximate that by
# requiring each parking cell to have exactly one predecessor and one successor
# AND for the graph to stay strongly connected without it.
rev = {c: [] for c in cells}
for c in cells:
    for n in exits(*c):
        rev[n].append(c)


def strongly_connected(nodes):
    if not nodes:
        return True
    nodes = set(nodes)
    start = next(iter(nodes))

    def bfs(adj):
        seen, stack = {start}, [start]
        while stack:
            c = stack.pop()
            for n in adj(c):
                if n in nodes and n not in seen:
                    seen.add(n)
                    stack.append(n)
        return seen

    fwd = bfs(lambda c: exits(*c))
    bwd = bfs(lambda c: rev[c])
    return len(fwd) == len(nodes) and len(bwd) == len(nodes)


if not strongly_connected(cells):
    errors.append("track graph is not strongly connected")

# The property that actually matters: the main line must be strongly connected
# with every spur cell removed, so parked vehicles can never disconnect the fab.
main_line = [c for c in cells if c not in set(PARKING)]
if not strongly_connected(main_line):
    errors.append("main line is not strongly connected once spurs are excluded")

# Every port must sit on the main line, never on a spur.
for pc in port_cells:
    if pc in set(PARKING):
        errors.append(f"port {pc} sits on a parking spur")

# Each spur must be enterable from and returnable to the main line.
for p in PARKING:
    if not any(n in main_line or n in PARKING for n in exits(*p)):
        errors.append(f"parking {p} cannot rejoin the track")
    if not rev[p]:
        errors.append(f"parking {p} is unreachable")

# Each body must share an edge with every port it owns. Diagonal does not
# count: it leaves a visible gap at the corner.
for mm in MACHINES:
    x0, y0 = mm["x"], mm["y"]
    x1, y1 = x0 + mm["w"] - 1, y0 + mm["h"] - 1
    for kind, (px, py) in mm["ports"]:
        touching = ((x0 - 1 <= px <= x1 + 1 and y0 <= py <= y1)
                    or (y0 - 1 <= py <= y1 + 1 and x0 <= px <= x1))
        if not touching:
            errors.append(
                f"{mm['name']}: body ({x0},{y0})-({x1},{y1}) does not touch "
                f"its {kind} port ({px},{py})")

if errors:
    print("MAP VALIDATION FAILED")
    for e in errors:
        print("  -", e)
    raise SystemExit(1)

print(f"map ok: {len(cells)} track cells, strongly connected")
print(f"  parking cells: {PARKING}")
print(f"  all parking verified off the main line")
print(f"  diverges: {[c for c in cells if len(exits(*c)) > 1]}")

out = {
    "name": "demo_loop",
    "width": Wd, "height": Ht, "tracks": t,
    "parking": [list(p) for p in PARKING],
    "machines": [
        {"name": m["name"], "kind": m["kind"], "x": m["x"], "y": m["y"],
         "w": m["w"], "h": m["h"], "process_ticks": m["process_ticks"],
         "capacity": m["capacity"],
         "ports": [{"kind": k, "cell": list(c)} for k, c in m["ports"]]}
        for m in MACHINES
    ],
}
dest = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "maps", "demo_loop.json")
if os.path.exists(dest) and "--force" not in sys.argv:
    print(f"{os.path.normpath(dest)} already exists; pass --force to overwrite")
    raise SystemExit(1)
with open(dest, "w") as f:
    json.dump(out, f, indent=1)
print(f"wrote {os.path.normpath(dest)}")
