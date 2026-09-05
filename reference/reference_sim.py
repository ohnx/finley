"""
Reference implementation of the OHT sim, mirroring the Rust design closely
enough to validate it end to end. Not the deliverable — a design check.
"""
import json, heapq, sys
from resolver_proto import resolve

N, E, S, W = 1, 2, 4, 8
DIRS = [(N, 0, -1), (E, 1, 0), (S, 0, 1), (W, -1, 0)]
DIR_IDX = {N: 0, E: 1, S: 2, W: 3}
OPP = {N: S, S: N, E: W, W: E}


class Grid:
    def __init__(self, w, h, track):
        self.w, self.h = w, h
        self.track = track  # flat

    def idx(self, x, y): return y * self.w + x
    def xy(self, c): return (c % self.w, c // self.w)

    def step(self, c, bit):
        if not self.track[c] or not (self.track[c] & bit):
            return None
        x, y = self.xy(c)
        for b, dx, dy in DIRS:
            if b == bit:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    nc = self.idx(nx, ny)
                    return nc if self.track[nc] else None
        return None

    def exits(self, c):
        return [(b, self.step(c, b)) for b, _, _ in DIRS if self.step(c, b) is not None]


def manoeuvre(a, b):
    if a == b: return "straight"
    if OPP[a] == b: return "reverse"
    return "curve"


def dijkstra(grid, cong, rw, start, heading, targets=None, avoid=frozenset()):
    """Returns (dist_by_cell, prev_state, start_state)."""
    ns = len(grid.track) * 4
    dist = [float("inf")] * ns
    prev = [-1] * ns
    s0 = start * 4 + DIR_IDX[heading]
    dist[s0] = 0.0
    pq = [(0.0, s0)]
    goal = None
    tset = set(targets) if targets else None
    while pq:
        d, s = heapq.heappop(pq)
        if d > dist[s]: continue
        cell, di = s // 4, s % 4
        if tset and cell in tset and s != s0:
            goal = s; break
        fd = DIRS[di][0]
        for b, _, _ in DIRS:
            nxt = grid.step(cell, b)
            if nxt is None: continue
            if nxt in avoid and (tset is None or nxt not in tset): continue
            m = manoeuvre(fd, b)
            if m == "reverse": continue
            step_cost = rw["length"] + (rw["curve"] if m == "curve" else 0.0)
            nd = d + step_cost + rw["congestion"] * cong[nxt]
            n_s = nxt * 4 + DIR_IDX[b]
            if nd < dist[n_s]:
                dist[n_s] = nd; prev[n_s] = s
                heapq.heappush(pq, (nd, n_s))
    field = [float("inf")] * len(grid.track)
    for s in range(ns):
        c = s // 4
        if dist[s] < field[c]: field[c] = dist[s]
    return field, prev, s0, goal


def path_to(grid, cong, rw, start, heading, targets, avoid=frozenset()):
    if start in targets: return []
    _, prev, s0, goal = dijkstra(grid, cong, rw, start, heading, targets, avoid)
    if goal is None: return None
    path, cur = [], goal
    while cur != s0:
        path.append(cur // 4)
        cur = prev[cur]
        if cur == -1: return None
    path.reverse()
    return path


class Rng:
    def __init__(self, seed): self.s = seed or 0x9E3779B97F4A7C15
    def u64(self):
        x = self.s
        x ^= (x >> 12) & 0xFFFFFFFFFFFFFFFF
        x ^= (x << 25) & 0xFFFFFFFFFFFFFFFF
        x ^= (x >> 27) & 0xFFFFFFFFFFFFFFFF
        self.s = x & 0xFFFFFFFFFFFFFFFF
        return (self.s * 0x2545F4914F6CDD1D) & 0xFFFFFFFFFFFFFFFF
    def f32(self): return (self.u64() >> 40) / (1 << 24)


class World:
    def __init__(self, mapcfg, scen, pol):
        g = mapcfg["tracks"]
        flat = [g[y][x] for y in range(mapcfg["height"]) for x in range(mapcfg["width"])]
        self.grid = Grid(mapcfg["width"], mapcfg["height"], flat)
        self.pol = pol
        self.scen = scen
        self.rng = Rng(scen["seed"])
        self.tick_n = 0

        self.machines = []
        for i, m in enumerate(mapcfg["machines"]):
            ports = [{"kind": p["kind"], "cell": self.grid.idx(*p["cell"]),
                      "lot": None, "res": None} for p in m["ports"]]
            self.machines.append(dict(
                id=i, name=m["name"], kind=m["kind"], ports=ports,
                process_ticks=m["process_ticks"], capacity=m["capacity"],
                in_process=[], idle_ticks=0, starvation=0.0))

        self.parking = [self.grid.idx(*p) for p in mapcfg["parking"]]
        self.spur = frozenset(self.parking)
        self.cong = [0.0] * len(self.grid.track)
        self.occ = [None] * len(self.grid.track)

        self.vehicles = []
        pool = list(self.parking) + [c for c in range(len(self.grid.track))
                                     if self.grid.track[c]]
        placed = []
        for c in pool:
            if len(placed) >= scen["vehicles"]: break
            if c in placed: continue
            placed.append(c)
        for i, c in enumerate(placed):
            ex = self.grid.exits(c)
            self.vehicles.append(dict(id=i, cell=c, heading=ex[0][0] if ex else E,
                                      ready=0, state=("idle", None), route=[],
                                      carrying=None, blocked=0, idle_n=0))
            self.occ[c] = i

        self.lots, self.jobs, self.pending, self.lot_job = [], [], [], []
        self.m = dict(completed=0, created=0, cycle=[], deadlocks=0,
                      stuck=0, rotated=0, busy=0, cap=0, backlog=[],
                      machine_idle=[0] * len(self.machines))
        self.profile = 0

    # -- helpers
    def free_port(self, m, kind):
        for i, p in enumerate(m["ports"]):
            if p["kind"] == kind and p["lot"] is None and p["res"] is None:
                return i
        return None

    def load_of(self, m):
        return sum(1 for p in m["ports"] if p["kind"] == "in" and p["lot"]) + len(m["in_process"])

    def prof(self): return self.pol["profiles"][self.profile]

    # -- phases
    def tick(self):
        self.tick_n += 1
        d = self.pol["congestion_decay"]
        for c in range(len(self.cong)):
            self.cong[c] = self.cong[c] * d + (1.0 - d if self.occ[c] is not None else 0.0)
        self.run_machines()
        self.spawn()
        self.create_jobs()
        self.select_profile()
        self.assign()
        self.advance()
        self.move()
        self.m["backlog"].append(len(self.pending))
        self.m["cap"] += len(self.vehicles)
        for v in self.vehicles:
            if v["state"][0] != "idle": self.m["busy"] += 1

    def run_machines(self):
        for mi, m in enumerate(self.machines):
            if m["kind"] == "source": continue
            while len(m["in_process"]) < m["capacity"]:
                slot = next(((i, p["lot"]) for i, p in enumerate(m["ports"])
                             if p["kind"] == "in" and p["lot"] is not None), None)
                if not slot: break
                pi, lot = slot
                m["ports"][pi]["lot"] = None
                m["in_process"].append([lot, m["process_ticks"]])
                self.lots[lot]["state"] = ("processing", mi)
            for e in m["in_process"]:
                if e[1] > 0: e[1] -= 1
            still = []
            for lot, rem in m["in_process"]:
                if rem > 0:
                    still.append([lot, rem]); continue
                L = self.lots[lot]
                L["step"] += 1
                if L["step"] >= len(L["recipe"]):
                    L["state"] = ("done", None)
                    self.m["completed"] += 1
                    self.m["cycle"].append(self.tick_n - L["created"])
                    continue
                op = self.free_port(m, "out")
                if op is None:
                    L["step"] -= 1
                    still.append([lot, 0])
                else:
                    m["ports"][op]["lot"] = lot
                    L["state"] = ("at_port", (mi, op))
                    L["wait_since"] = self.tick_n
            m["in_process"] = still
            starved = not m["in_process"] and m["kind"] != "sink"
            m["starvation"] = m["starvation"] * 0.99 + (0.01 if starved else 0.0)
            if starved:
                m["idle_ticks"] += 1
                self.m["machine_idle"][mi] += 1

    def spawn(self):
        if self.rng.f32() >= self.scen["arrival_per_1000"] / 1000.0: return
        srcs = [i for i, m in enumerate(self.machines) if m["kind"] == "source"]
        if not srcs: return
        si = srcs[self.rng.u64() % len(srcs)]
        p = self.free_port(self.machines[si], "out")
        if p is None: return
        rs = self.scen["recipes"]
        total = sum(r["weight"] for r in rs)
        pick, chosen = self.rng.f32() * total, 0
        for i, r in enumerate(rs):
            pick -= r["weight"]
            if pick <= 0: chosen = i; break
        hot = self.rng.f32() < self.scen["hot_fraction"]
        lid = len(self.lots)
        self.lots.append(dict(id=lid, recipe=list(rs[chosen]["steps"]), step=0,
                              state=("at_port", (si, p)), created=self.tick_n,
                              wait_since=self.tick_n, priority=1.0 if hot else 0.0))
        self.lot_job.append(None)
        self.machines[si]["ports"][p]["lot"] = lid
        self.m["created"] += 1

    def create_jobs(self):
        for lid, L in enumerate(self.lots):
            if self.lot_job[lid] is not None: continue
            if L["state"][0] != "at_port": continue
            mi, pi = L["state"][1]
            if self.machines[mi]["ports"][pi]["kind"] != "out": continue
            if L["step"] >= len(L["recipe"]): continue
            jid = len(self.jobs)
            self.jobs.append(dict(id=jid, lot=lid, frm=(mi, pi), to=None,
                                  assigned=None, created=self.tick_n))
            self.pending.append(jid)
            self.lot_job[lid] = jid

    def select_profile(self):
        chosen = 0
        for i, p in enumerate(self.pol["profiles"]):
            t = p["trigger"]
            if t["type"] == "always":
                chosen = i; continue
            hit = False
            if t["type"] == "backlog_above":
                hit = len(self.pending) > t["n"]
            elif t["type"] == "queue_depth_above":
                hit = any(m["kind"] == t["kind"] and self.load_of(m) >= t["n"]
                          for m in self.machines)
            elif t["type"] == "starvation_above":
                hit = any(m["kind"] == t["kind"] and m["idle_ticks"] >= t["ticks"]
                          for m in self.machines)
            if hit: chosen = i; break
        self.profile = chosen

    def assign(self):
        if not self.pending: return
        idle = [v for v in self.vehicles if v["state"][0] in ("idle", "repos")]
        if not idle: return
        dw, rw = self.prof()["dispatch"], self.prof()["route"]

        vfield = {}
        for v in idle:
            f, _, _, _ = dijkstra(self.grid, self.cong, rw, v["cell"], v["heading"], None, self.spur)
            vfield[v["id"]] = f
        pfield = {}
        cands = []
        for jid in self.pending:
            J = self.jobs[jid]; L = self.lots[J["lot"]]
            pcell = self.machines[J["frm"][0]]["ports"][J["frm"][1]]["cell"]
            if pcell not in pfield:
                ex = self.grid.exits(pcell)
                hd = ex[0][0] if ex else E
                pfield[pcell], _, _, _ = dijkstra(self.grid, self.cong, rw, pcell, hd, None, self.spur)
            df = pfield[pcell]
            if L["step"] >= len(L["recipe"]): continue
            kind = L["recipe"][L["step"]]
            wait = self.tick_n - L["wait_since"]
            steps_left = len(L["recipe"]) - L["step"]
            lot_term = (-dw["lot_wait"] * wait - dw["lot_priority"] * L["priority"]
                        + dw["steps_remaining"] * steps_left)
            for mi, m in enumerate(self.machines):
                if m["kind"] != kind: continue
                port = self.free_port(m, "in")
                if port is None: continue
                dcell = m["ports"][port]["cell"]
                dcost = df[dcell]
                if dcost == float("inf"): continue
                dest_term = (-dw["dest_starvation"] * m["starvation"]
                             + dw["dest_queue"] * self.load_of(m)
                             + dw["dest_congestion"] * dcost)
                for v in idle:
                    pc = vfield[v["id"]][pcell]
                    if pc == float("inf"): continue
                    cands.append((dw["travel_to_pickup"] * pc + lot_term + dest_term,
                                  v["id"], jid, (mi, port)))
        cands.sort(key=lambda c: (c[0], c[1], c[2], c[3][0]))
        uv, uj, up = set(), set(), set()
        for score, vid, jid, dest in cands:
            if vid in uv or jid in uj or dest in up: continue
            v = self.vehicles[vid]
            J = self.jobs[jid]
            pcell = self.machines[J["frm"][0]]["ports"][J["frm"][1]]["cell"]
            p = path_to(self.grid, self.cong, rw, v["cell"], v["heading"], {pcell}, self.spur)
            if p is None: continue
            uv.add(vid); uj.add(jid); up.add(dest)
            J["assigned"] = vid; J["to"] = dest
            self.machines[dest[0]]["ports"][dest[1]]["res"] = vid
            self.machines[J["frm"][0]]["ports"][J["frm"][1]]["res"] = vid
            v["state"] = ("to_pickup", jid); v["route"] = p; v["blocked"] = 0
            self.pending.remove(jid)

    def advance(self):
        hoist = self.pol["kinematics"]["hoist_ticks"]
        rw = self.prof()["route"]
        for v in self.vehicles:
            if v["ready"] > 0:
                v["ready"] -= 1; continue
            kind, arg = v["state"]
            if kind == "to_pickup" and not v["route"]:
                v["state"] = ("loading", arg); v["ready"] = hoist
            elif kind == "loading":
                J = self.jobs[arg]; mi, pi = J["frm"]; lot = J["lot"]
                self.machines[mi]["ports"][pi]["lot"] = None
                self.machines[mi]["ports"][pi]["res"] = None
                self.lots[lot]["state"] = ("transit", v["id"])
                v["carrying"] = lot
                dest = J["to"]
                dcell = self.machines[dest[0]]["ports"][dest[1]]["cell"]
                p = path_to(self.grid, self.cong, rw, v["cell"], v["heading"], {dcell}, self.spur)
                if p is None:
                    self.machines[mi]["ports"][pi]["lot"] = lot
                    self.lots[lot]["state"] = ("at_port", (mi, pi))
                    v["carrying"] = None; v["state"] = ("idle", None)
                else:
                    v["route"] = p; v["state"] = ("to_dropoff", arg)
            elif kind == "to_dropoff" and not v["route"]:
                v["state"] = ("unloading", arg); v["ready"] = hoist
            elif kind == "unloading":
                J = self.jobs[arg]; dest = J["to"]; lot = J["lot"]
                self.machines[dest[0]]["ports"][dest[1]]["lot"] = lot
                self.machines[dest[0]]["ports"][dest[1]]["res"] = None
                self.lots[lot]["state"] = ("at_port", dest)
                self.lots[lot]["wait_since"] = self.tick_n
                v["carrying"] = None; v["state"] = ("idle", None)
                self.lot_job[lot] = None
            elif kind in ("idle", "repos"):
                self.reposition(v)

    def reposition(self, v):
        if v["state"][0] == "repos" and v["route"]: return
        v["idle_n"] += 1
        if v["idle_n"] < self.pol["idle"]["dwell_before_move"]: return
        mode = self.pol["idle"]["mode"]
        if mode == "stay_put" or not self.parking:
            v["state"] = ("idle", None); return
        targets = {c for c in self.parking if self.occ[c] is None or self.occ[c] == v['id']}
        if not targets:
            v["state"] = ("idle", None); return
        if mode == "preposition":
            cands = [m for m in self.machines if m["kind"] not in ("source", "sink")]
            if cands:
                hungry = max(cands, key=lambda m: m["starvation"])
                tc = hungry["ports"][0]["cell"]
                tx, ty = self.grid.xy(tc)
                best = min(targets, key=lambda c: (self.grid.xy(c)[0] - tx) ** 2
                           + (self.grid.xy(c)[1] - ty) ** 2)
                targets = {best}
        if v["cell"] in targets:
            v["state"] = ("idle", None); v["idle_n"] = 0; return
        p = path_to(self.grid, self.cong, self.prof()["route"], v["cell"], v["heading"], targets, self.spur)
        if p is not None:
            v["route"] = p; v["state"] = ("repos", None)

    def move(self):
        occ = {c: self.occ[c] for c in range(len(self.occ)) if self.occ[c] is not None}
        props, pri = {}, {}
        for v in self.vehicles:
            pri[v["id"]] = -v["blocked"] * 10 - (5 if v["carrying"] is not None else 0)
            props[v["id"]] = v["route"][0] if (v["ready"] == 0 and v["route"]) else None
        moves, stalled, dead = resolve(occ, props, lambda vid: pri[vid])
        self.m["deadlocks"] += len(dead)
        kin = self.pol["kinematics"]
        for vid, dest in moves.items():
            v = self.vehicles[vid]
            src = v["cell"]
            sx, sy = self.grid.xy(src); dx_, dy_ = self.grid.xy(dest)
            nb = next(b for b, ddx, ddy in DIRS if (sx + ddx, sy + ddy) == (dx_, dy_))
            mv = manoeuvre(v["heading"], nb)
            cost = kin["straight_ticks"] if mv == "straight" else kin["curve_ticks"]
            self.occ[src] = None; self.occ[dest] = vid
            v["cell"] = dest; v["heading"] = nb
            v["ready"] = max(0, cost - 1); v["blocked"] = 0
            if v["route"]: v["route"].pop(0)
        for vid in stalled:
            self.vehicles[vid]["blocked"] += 1
        for v in self.vehicles:
            if v["blocked"] > self.pol["stuck_threshold"]:
                self.m["stuck"] += 1
                v["blocked"] = 0; v["route"] = []
                k, arg = v["state"]
                tgt = None
                if k == "to_pickup":
                    mi, pi = self.jobs[arg]["frm"]; tgt = self.machines[mi]["ports"][pi]["cell"]
                elif k == "to_dropoff":
                    d = self.jobs[arg]["to"]; tgt = self.machines[d[0]]["ports"][d[1]]["cell"]
                if tgt is not None:
                    p = path_to(self.grid, self.cong, self.prof()["route"],
                                v["cell"], v["heading"], {tgt}, self.spur)
                    if p is not None: v["route"] = p
                else:
                    v["state"] = ("idle", None)


def report(w, label):
    m = w.m
    cyc = sorted(m["cycle"])
    p95 = cyc[min(len(cyc) - 1, int((len(cyc) - 1) * 0.95))] if cyc else 0
    mean = sum(cyc) / len(cyc) if cyc else 0
    print(f"--- {label} ---")
    print(f"  created/completed   {m['created']} / {m['completed']}")
    print(f"  throughput          {m['completed']*1000/w.tick_n:.2f} lots / 1000 ticks")
    print(f"  cycle mean / p95    {mean:.0f} / {p95} ticks")
    print(f"  utilisation         {m['busy']/max(1,m['cap'])*100:.1f}%")
    print(f"  mean backlog        {sum(m['backlog'])/len(m['backlog']):.2f}")
    print(f"  deadlocks / stuck   {m['deadlocks']} / {m['stuck']}")
    worst = sorted(enumerate(m["machine_idle"]), key=lambda t: -t[1])[:4]
    idle = ", ".join(f"{w.machines[i]['name']} {t/w.tick_n*100:.0f}%" for i, t in worst)
    print(f"  most starved        {idle}")


if __name__ == "__main__":
    base = "/home/claude/ohtsim/"
    mapcfg = json.load(open(base + "maps/demo_loop.json"))
    scen = json.load(open(base + "scenarios/baseline.json"))
    ticks = int(sys.argv[1]) if len(sys.argv) > 1 else 6000

    for polname in ["default", "starvation_biased"]:
        pol = json.load(open(base + f"policies/{polname}.json"))
        pol["profiles"].sort(key=lambda p: p["trigger"]["type"] == "always")
        w = World(mapcfg, scen, pol)
        for _ in range(ticks):
            w.tick()
        report(w, polname)

    # determinism check
    pol = json.load(open(base + "policies/default.json"))
    runs = []
    for _ in range(2):
        w = World(mapcfg, scen, json.loads(json.dumps(pol)))
        for _ in range(1500): w.tick()
        runs.append((w.m["completed"], sum(w.m["cycle"])))
    print(f"\ndeterminism: {runs[0]} vs {runs[1]} -> "
          f"{'identical' if runs[0] == runs[1] else 'DIVERGED'}")
