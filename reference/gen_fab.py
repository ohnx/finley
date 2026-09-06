"""
Generate maps/fab.json -- the seven-tool-kind fab.

Layout is a one-way Manhattan grid: horizontal aisles alternate east/west,
vertical aisles alternate north/south, and the perimeter runs clockwise so the
corners are not dead ends. Every interior intersection has two exits, so unlike
the single demo loop there is a genuine choice of route between any two tools,
which is what gives the route weights something to decide.

Tools sit in the blocks between aisles, each with four load ports on the aisle
above it: two in-bays and two out-bays, except the source (all out) and test
(all in). Two ports per tool was tried first and starved dispatch -- most ticks
had no free destination of the right kind, which is both slow to simulate and
dull to watch. Each block also carries a one-cell parking spur in its free
column, entered from the aisle above and rejoining the vertical aisle beside it
-- one cell rather than two, so a parked vehicle is never trapped behind another
parked vehicle the way it was on the demo map.

    python3 reference/gen_fab.py [--force]
"""
import json, os, sys

N, E, S, W = 1, 2, 4, 8
DIRS = [(N, 0, -1), (E, 1, 0), (S, 0, 1), (W, -1, 0)]

# Aisle positions. Blocks are the 4x3 interiors between them.
XA = [0, 6, 12, 18, 24, 30]
YA = [0, 4, 8, 12, 16]
Wd, Ht = XA[-1] + 1, YA[-1] + 1

# Perimeter runs clockwise: top east, right south, bottom west, left north.
# Interior aisles alternate, so every crossing offers two ways on.
def row_dir(y):
    if y == YA[0]:
        return E
    if y == YA[-1]:
        return W
    return W if YA.index(y) % 2 == 0 else E

def col_dir(x):
    if x == XA[0]:
        return N
    if x == XA[-1]:
        return S
    return S if XA.index(x) % 2 == 0 else N

t = [[0] * Wd for _ in range(Ht)]
for y in YA:
    for x in range(Wd):
        t[y][x] |= row_dir(y)
for x in XA:
    for y in range(Ht):
        t[y][x] |= col_dir(x)

# --- tools ------------------------------------------------------------------
# Counts come from the visit mix and the process times: litho is the bottleneck
# and gets the most, deposition next. Furnace stands in for a batch tool with
# capacity 4 until batching proper lands.
FLEET = [
    ("src",    "source",     0,   3),
    ("litho1", "litho",    108,   1), ("litho2", "litho", 108, 1),
    ("litho3", "litho",    108,   1), ("litho4", "litho", 108, 1),
    ("litho5", "litho",    108,   1),
    ("depo1",  "deposition", 48,  1), ("depo2", "deposition", 48, 1),
    ("depo3",  "deposition", 48,  1),
    ("etch1",  "etch",       36,  1), ("etch2", "etch", 36, 1),
    ("cmp1",   "cmp",        36,  1), ("cmp2",  "cmp",  36, 1),
    ("impl1",  "implant",    24,  1), ("impl2", "implant", 24, 1),
    ("clean1", "clean",      12,  1), ("clean2", "clean", 12, 1),
    ("furn1",  "furnace",   288,  4), ("furn2", "furnace", 288, 4),
    ("test1",  "test",       24,  4),
]

blocks = [(xi, yi) for yi in range(len(YA) - 1) for xi in range(len(XA) - 1)]
assert len(FLEET) <= len(blocks), f"{len(FLEET)} tools into {len(blocks)} blocks"

# Spread same-kind tools apart: walk the blocks with a stride coprime to the
# count so litho tools do not end up neighbours.
order = [blocks[(i * 7) % len(blocks)] for i in range(len(blocks))]
seen, spread = set(), []
for b in order:
    if b not in seen:
        seen.add(b)
        spread.append(b)

MACHINES = []
for (name, kind, ticks, cap), (xi, yi) in zip(FLEET, spread):
    xa, ya = XA[xi], YA[yi]
    body = dict(x=xa + 1, y=ya + 1, w=4, h=2)
    # Four load ports per tool, which is what a real 300mm tool carries. It is
    # also what keeps the fab off the deadlock cliff: a finished lot needs an
    # out-port to leave by, and with a single one of each, a 68-step reentrant
    # flow across eight tool kinds closes a circular wait at a WIP of about 20.
    if kind == "source":
        ports = [("out", (xa + i, ya)) for i in (1, 2, 3, 4)]
    elif kind == "test":
        ports = [("in", (xa + i, ya)) for i in (1, 2, 3, 4)]
    else:
        ports = [("in", (xa + 1, ya)), ("in", (xa + 2, ya)),
                 ("out", (xa + 3, ya)), ("out", (xa + 4, ya))]
    MACHINES.append(dict(name=name, kind=kind, process_ticks=ticks, capacity=cap,
                         ports=ports, **body))

# --- parking spurs ----------------------------------------------------------
# One cell in each block's free column: enter from the aisle above heading
# south, leave east onto the vertical aisle. One cell deep on purpose.
PARKING = []
for xi, yi in blocks:
    xa, ya = XA[xi], YA[yi]
    sx, sy = xa + 5, ya + 1
    t[ya][sx] |= S
    t[sy][sx] |= E
    PARKING.append((sx, sy))

# Strip exit bits that point off the grid or at a blank cell. Setting the bits
# per aisle is the readable way to lay the grid out, but it leaves the boundary
# claiming exits that lead nowhere, and a bit the neighbour cannot accept is a
# dangling bit -- routing treats it as a wall, so the map lies about itself.
for y in range(Ht):
    for x in range(Wd):
        if not t[y][x]:
            continue
        for bit, dx, dy in DIRS:
            if not t[y][x] & bit:
                continue
            nx, ny = x + dx, y + dy
            if not (0 <= nx < Wd and 0 <= ny < Ht) or not t[ny][nx]:
                t[y][x] &= ~bit

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
rev = {c: [] for c in cells}
for c in cells:
    for n in exits(*c):
        rev[n].append(c)

for (x, y) in cells:
    if not exits(x, y):
        errors.append(f"dead end at ({x},{y})")

def strongly_connected(nodes):
    nodes = set(nodes)
    if not nodes:
        return True
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
    return (len(bfs(lambda c: exits(*c))) == len(nodes)
            and len(bfs(lambda c: rev[c])) == len(nodes))

if not strongly_connected(cells):
    errors.append("track graph is not strongly connected")
main_line = [c for c in cells if c not in set(PARKING)]
if not strongly_connected(main_line):
    errors.append("main line is not strongly connected once spurs are excluded")

port_cells = set()
for m in MACHINES:
    for kind, (px, py) in m["ports"]:
        if not t[py][px]:
            errors.append(f"{m['name']}: port ({px},{py}) has no track")
        if (px, py) in port_cells:
            errors.append(f"two machines share port cell ({px},{py})")
        if (px, py) in set(PARKING):
            errors.append(f"{m['name']}: port ({px},{py}) sits on a spur")
        port_cells.add((px, py))
    x0, y0, x1, y1 = m["x"], m["y"], m["x"] + m["w"] - 1, m["y"] + m["h"] - 1
    for kind, (px, py) in m["ports"]:
        touching = ((x0 - 1 <= px <= x1 + 1 and y0 <= py <= y1)
                    or (y0 - 1 <= py <= y1 + 1 and x0 <= px <= x1))
        if not touching:
            errors.append(f"{m['name']}: body does not touch its {kind} port ({px},{py})")

if errors:
    print("MAP VALIDATION FAILED")
    for e in errors:
        print("  -", e)
    raise SystemExit(1)

dest = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps", "fab.json")
if os.path.exists(dest) and "--force" not in sys.argv:
    print(f"{os.path.normpath(dest)} exists; pass --force to overwrite")
    raise SystemExit(1)
out = dict(
    name="fab", width=Wd, height=Ht, tracks=t,
    parking=[list(p) for p in PARKING],
    machines=[dict(name=m["name"], kind=m["kind"], x=m["x"], y=m["y"], w=m["w"], h=m["h"],
                   process_ticks=m["process_ticks"], capacity=m["capacity"],
                   ports=[dict(kind=k, cell=list(c)) for k, c in m["ports"]])
              for m in MACHINES])
with open(dest, "w") as f:
    json.dump(out, f, indent=1)
print(f"wrote {os.path.normpath(dest)}: {Wd}x{Ht}, {len(cells)} track cells, "
      f"{len(MACHINES)} tools, {len(PARKING)} spurs")
print(f"  junctions (2+ exits): {sum(1 for c in cells if len(exits(*c)) > 1)}")
