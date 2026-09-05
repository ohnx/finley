// finley / ohtsim browser UI.
//
// The simulation is the Rust core compiled to wasm; this file only drives it
// and draws it. Two rules the FFI imposes, both load-bearing:
//
//   1. Re-derive every typed-array view from `memory.buffer` each frame.
//      Growing wasm memory detaches existing views, and a 20k-tick run does
//      grow it. A stale view throws or, worse, reads zeroes.
//   2. Treat every pointer as valid only until the next call that mutates the
//      world, i.e. only until the next `oht_tick`.
//
// Static map geometry (track bits, machine rectangles, port cells, spurs) is
// read from the map JSON we already fetched to build the world, not from wasm.
// One source of truth.

const ROOT = "..";
const MAP = "maps/demo_loop.json";
const SCENARIO = "scenarios/baseline.json";

// Exit-direction bitmask, matching src/geom.rs: a cell holds the OR of the
// directions you may *leave* it by.
const N = 1, E = 2, S = 4, W = 8;
const DIRS = [
  { bit: N, dx: 0, dy: -1 },
  { bit: E, dx: 1, dy: 0 },
  { bit: S, dx: 0, dy: 1 },
  { bit: W, dx: -1, dy: 0 },
];

// Must match Snapshot's encoding in src/world.rs. Loading and Unloading share
// a colour: from the outside they are the same event, a vehicle sitting still
// on the track running a hoist cycle and blocking everything behind it.
const VEH_COLOR = ["--veh-idle", "--veh-pickup", "--veh-hoist",
                   "--veh-drop", "--veh-hoist", "--veh-repos"];

// The core's names describe the leg of a delivery; these describe what you can
// see. A vehicle is empty on the way to a pickup and loaded on the way to a
// dropoff, so "fetching" and "delivering" say the same thing without needing
// the carried-lot marker the old UI drew on top.
const VEH_LABEL = ["parked", "fetching", "hoisting up",
                   "delivering", "hoisting down", "repositioning"];

// Lot placement, matching LOT_* in crates/ohtsim-wasm/src/lib.rs.
const AT_PORT = 0, IN_TRANSIT = 1, PROCESSING = 2;

// Must match METRIC_COUNT and the block built in crates/ohtsim-wasm/src/lib.rs.
const M = {
  TICK: 0, CREATED: 1, COMPLETED: 2, THROUGHPUT: 3, MEAN_CYCLE: 4, P95: 5,
  UTIL: 6, MEAN_BACKLOG: 7, DEADLOCKS: 8, STUCK: 9, ROTATED: 10,
  BACKLOG_NOW: 11, BUSY_NOW: 12,
};

// Slider stop -> ticks advanced per animation frame. Sub-1 values are
// accumulated so "0.25" really is a quarter-speed crawl rather than a stutter.
const SPEEDS = [0.25, 1, 2, 4, 10, 40, 200];

// Every canvas colour comes from style.css, read once here. Keeping the palette
// in one place means a retheme touches only the stylesheet -- and reading it per
// vehicle per frame, as this used to, is a getComputedStyle call in the hot
// path for no reason.
const PALETTE_KEYS = [
  "--veh-idle", "--veh-pickup", "--veh-hoist", "--veh-drop", "--veh-repos",
  "--veh-edge", "--carry", "--carry-edge",
  "--deck", "--track", "--track-fill", "--spur", "--spur-fill",
  "--port-in", "--port-out", "--machine-label", "--machine-edge",
  "--select", "--select-halo",
  "--heat-low", "--heat-high",
  "--m-source", "--m-sink", "--m-litho", "--m-etch", "--m-cmp", "--m-metro",
  "--m-other",
];

function readPalette() {
  const style = getComputedStyle(document.documentElement);
  const p = {};
  for (const k of PALETTE_KEYS) p[k] = style.getPropertyValue(k).trim();
  return p;
}

const $ = (id) => document.getElementById(id);

function fail(message) {
  const box = $("error");
  box.hidden = false;
  box.textContent = message;
  $("app").style.opacity = 0.35;
  console.error(message);
}

// --------------------------------------------------------------------------
// wasm glue
// --------------------------------------------------------------------------

class Sim {
  constructor(exports, mapText, scenText, polText) {
    this.e = exports;
    const bufs = [mapText, scenText, polText].map((t) => this.put(t));
    this.ptr = this.e.oht_new(...bufs.flat());
    for (const [p, len] of bufs) this.e.oht_free_buf(p, len);
    if (this.ptr === 0) throw new Error(this.lastError());
  }

  put(text) {
    const bytes = new TextEncoder().encode(text);
    const ptr = this.e.oht_alloc(bytes.length);
    if (ptr === 0) throw new Error("wasm allocation failed");
    new Uint8Array(this.e.memory.buffer, ptr, bytes.length).set(bytes);
    return [ptr, bytes.length];
  }

  lastError() {
    const view = new Uint8Array(this.e.memory.buffer,
                                this.e.oht_error_ptr(), this.e.oht_error_len());
    return new TextDecoder().decode(view) || "unknown error";
  }

  tick(n) { this.e.oht_tick(this.ptr, n); }
  free() { this.e.oht_drop(this.ptr); this.ptr = 0; }

  // Views are built fresh on every call, never cached. See the note at the top.
  view(Type, ptrFn, len) {
    return new Type(this.e.memory.buffer, ptrFn.call(this.e, this.ptr), len);
  }

  vehicles() {
    const n = this.e.oht_veh_count(this.ptr);
    return {
      n,
      x: this.view(Uint16Array, this.e.oht_veh_x, n),
      y: this.view(Uint16Array, this.e.oht_veh_y, n),
      heading: this.view(Uint8Array, this.e.oht_veh_heading, n),
      carrying: this.view(Uint8Array, this.e.oht_veh_carrying, n),
      state: this.view(Uint8Array, this.e.oht_veh_state, n),
    };
  }

  congestion() {
    return this.view(Float32Array, this.e.oht_congestion,
                     this.e.oht_cell_count(this.ptr));
  }

  machines() {
    const n = this.e.oht_machine_count(this.ptr);
    return {
      n,
      load: this.view(Uint16Array, this.e.oht_machine_load, n),
      starvation: this.view(Float32Array, this.e.oht_machine_starvation, n),
    };
  }

  metrics() {
    return this.view(Float64Array, this.e.oht_metrics,
                     this.e.oht_metric_count());
  }

  // Only lots still in the fab; the world keeps every lot it ever made.
  lots() {
    const n = this.e.oht_lot_count(this.ptr);
    return {
      n,
      id: this.view(Uint32Array, this.e.oht_lot_id, n),
      recipe: this.view(Uint8Array, this.e.oht_lot_recipe, n),
      step: this.view(Uint8Array, this.e.oht_lot_step, n),
      total: this.view(Uint8Array, this.e.oht_lot_steps_total, n),
      place: this.view(Uint8Array, this.e.oht_lot_place, n),
      a: this.view(Uint16Array, this.e.oht_lot_a, n),
      b: this.view(Uint16Array, this.e.oht_lot_b, n),
      wait: this.view(Uint32Array, this.e.oht_lot_wait, n),
      priority: this.view(Float32Array, this.e.oht_lot_priority, n),
    };
  }

  targets() {
    const n = this.e.oht_veh_count(this.ptr);
    return {
      machine: this.view(Int16Array, this.e.oht_veh_target_machine, n),
      port: this.view(Int16Array, this.e.oht_veh_target_port, n),
      lot: this.view(Int32Array, this.e.oht_veh_lot, n),
    };
  }

  // Cells the vehicle still has to visit, next hop first.
  route(v) {
    const n = this.e.oht_veh_route_len(this.ptr, v);
    if (!n) return new Uint32Array(0);
    return new Uint32Array(this.e.memory.buffer, this.e.oht_veh_route(this.ptr, v), n);
  }

  ports() {
    return this.view(Int32Array, this.e.oht_port_lot,
                     this.e.oht_port_count(this.ptr));
  }

  machineIdle() {
    return this.view(Uint32Array, this.e.oht_machine_idle_ticks,
                     this.e.oht_machine_count(this.ptr));
  }

  cycles() {
    return this.view(Float64Array, this.e.oht_cycle_times,
                     this.e.oht_cycle_count(this.ptr));
  }
}

// --------------------------------------------------------------------------
// Renderer
// --------------------------------------------------------------------------

class Renderer {
  constructor(canvas, map) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.map = map;
    this.p = readPalette();
    this.cell = 44;
    this.resize();
    addEventListener("resize", () => this.resize());
  }

  // Backing store is sized in device pixels so the fab stays crisp on HiDPI,
  // while CSS keeps the element responsive.
  resize() {
    const dpr = devicePixelRatio || 1;
    const w = this.map.width * this.cell;
    const h = this.map.height * this.cell;
    this.canvas.width = w * dpr;
    this.canvas.height = h * dpr;
    this.canvas.style.aspectRatio = `${w} / ${h}`;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = w;
    this.h = h;
  }

  draw(sim, sel) {
    const { ctx } = this;
    const v = sim.vehicles();
    this.lastVehicles = v;
    ctx.fillStyle = this.p["--deck"];
    ctx.fillRect(0, 0, this.w, this.h);
    this.drawMachines();
    this.drawCongestion(sim.congestion());
    this.drawTrack();
    if (sel.oht >= 0) this.drawRoute(sim, sel.oht, v);
    this.drawPorts(sim.ports());
    this.drawVehicles(v, sel);
    this.drawSelection(sim, sel, v);
  }

  centre(cell) {
    const { cell: c, map } = this;
    return [(cell % map.width) * c + c / 2, Math.floor(cell / map.width) * c + c / 2];
  }

  /// The route a selected vehicle still has to drive. Drawn under everything
  /// else so vehicles stay on top of it, and it is the answer to "why is that
  /// one going the long way round" -- a congestion detour is visible as a
  /// route that ignores the short path.
  drawRoute(sim, v, veh) {
    const route = sim.route(v);
    if (!route.length) return;
    const { ctx, cell, p } = this;
    ctx.save();
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(veh.x[v] * cell + cell / 2, veh.y[v] * cell + cell / 2);
    for (const c of route) ctx.lineTo(...this.centre(c));
    // Cream underlay so the line survives crossing a dark congested cell.
    ctx.strokeStyle = p["--select-halo"];
    ctx.lineWidth = 5;
    ctx.globalAlpha = 0.85;
    ctx.stroke();
    ctx.strokeStyle = p["--select"];
    ctx.lineWidth = 2.2;
    ctx.globalAlpha = 0.9;
    ctx.setLineDash([5, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    // A dot on the far end, so the destination is unambiguous when the route
    // doubles back on itself.
    const [ex, ey] = this.centre(route[route.length - 1]);
    ctx.fillStyle = p["--select"];
    ctx.beginPath();
    ctx.arc(ex, ey, 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  /// A ring around whatever is selected, plus a ring on the lot being followed.
  drawSelection(sim, sel, veh) {
    const { ctx, cell, p } = this;
    const ring = (cx, cy, r) => {
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.strokeStyle = p["--select-halo"];
      ctx.lineWidth = 4;
      ctx.stroke();
      ctx.strokeStyle = p["--select"];
      ctx.lineWidth = 1.8;
      ctx.stroke();
    };
    if (sel.oht >= 0 && sel.oht < veh.n) {
      ring(veh.x[sel.oht] * cell + cell / 2, veh.y[sel.oht] * cell + cell / 2, cell * 0.44);
    }
    const at = sel.lotCell;
    if (at != null) {
      const [cx, cy] = this.centre(at);
      ctx.save();
      ctx.setLineDash([4, 3]);
      ring(cx, cy, cell * 0.46);
      ctx.restore();
    }
  }

  drawMachines() {
    const { ctx, cell, p } = this;
    ctx.font = "500 11px ui-sans-serif, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    for (const m of this.map.machines) {
      const x = m.x * cell, y = m.y * cell;
      const w = m.w * cell, h = m.h * cell;
      ctx.fillStyle = p[`--m-${m.kind}`] || p["--m-other"];
      this.roundRect(x + 2, y + 2, w - 4, h - 4, 3);
      ctx.fill();
      ctx.strokeStyle = p["--machine-edge"];
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = p["--machine-label"];
      ctx.fillText(m.name, x + w / 2, y + h / 2);
    }
  }

  // Congestion is the whole reason for a top-down view: traffic waves show up
  // as a red smear propagating backward from wherever a vehicle is hoisting.
  drawCongestion(cong) {
    const { ctx, cell, map, p } = this;
    const low = p["--heat-low"].split(",").map(Number);
    const high = p["--heat-high"].split(",").map(Number);
    for (let y = 0; y < map.height; y++) {
      for (let x = 0; x < map.width; x++) {
        if (!map.tracks[y][x]) continue;
        const v = Math.min(cong[y * map.width + x], 1);
        if (v <= 0.02) continue;
        // Straw to rust, gaining opacity as it goes. On paper a wash that only
        // gets denser reads as one flat tint; the hue shift is what separates a
        // busy cell from a quiet one at a glance.
        const c = low.map((lo, i) => Math.round(lo + (high[i] - lo) * v));
        ctx.fillStyle = `rgba(${c[0]}, ${c[1]}, ${c[2]}, ${0.07 + v * 0.42})`;
        ctx.fillRect(x * cell, y * cell, cell, cell);
      }
    }
  }

  // Rails are one-way, so the arrows are not decoration: they are why a
  // vehicle cannot simply back out of a jam.
  drawTrack() {
    const { ctx, cell, map } = this;
    const spurs = this.spurSet();
    const p = this.p;
    ctx.lineWidth = 1.4;
    ctx.lineCap = "round";
    for (let y = 0; y < map.height; y++) {
      for (let x = 0; x < map.width; x++) {
        const bits = map.tracks[y][x];
        if (!bits) continue;
        const cx = x * cell + cell / 2;
        const cy = y * cell + cell / 2;
        const spur = spurs.has(`${x},${y}`);

        ctx.fillStyle = spur ? p["--spur-fill"] : p["--track-fill"];
        ctx.fillRect(x * cell, y * cell, cell, cell);

        ctx.strokeStyle = spur ? p["--spur"] : p["--track"];
        for (const d of DIRS) {
          if (!(bits & d.bit)) continue;
          const ex = cx + d.dx * cell * 0.42;
          const ey = cy + d.dy * cell * 0.42;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(ex, ey);
          ctx.stroke();
          // Arrowhead at the exit edge.
          const a = 4;
          ctx.beginPath();
          ctx.moveTo(ex, ey);
          ctx.lineTo(ex - d.dx * a + d.dy * a * 0.7, ey - d.dy * a + d.dx * a * 0.7);
          ctx.lineTo(ex - d.dx * a - d.dy * a * 0.7, ey - d.dy * a - d.dx * a * 0.7);
          ctx.closePath();
          ctx.fillStyle = ctx.strokeStyle;
          ctx.fill();
        }
      }
    }
  }

  spurSet() {
    if (!this._spurs) {
      this._spurs = new Set(this.map.parking.map(([x, y]) => `${x},${y}`));
    }
    return this._spurs;
  }

  /// Ports as arrowheads pointing the way a lot travels: an in-port aims at
  /// the tool, an out-port away from it. A filled head means a lot is sitting
  /// there right now, which is where backpressure shows up first -- a machine
  /// whose out-ports are all full has stopped being able to finish work.
  drawPorts(portLots) {
    const { ctx, cell, p } = this;
    let i = 0;
    for (const m of this.map.machines) {
      const mcx = (m.x + m.w / 2) * cell;
      const mcy = (m.y + m.h / 2) * cell;
      for (const port of m.ports) {
        const occupied = portLots[i++] >= 0;
        const [px, py] = port.cell;
        const cx = px * cell + cell / 2;
        const cy = py * cell + cell / 2;
        // Unit vector from the port toward the tool it serves.
        let dx = mcx - cx, dy = mcy - cy;
        const len = Math.hypot(dx, dy) || 1;
        dx /= len; dy /= len;
        // Out-ports point away: the lot is leaving the tool.
        const sign = port.kind === "in" ? 1 : -1;
        dx *= sign; dy *= sign;

        const colour = p[port.kind === "in" ? "--port-in" : "--port-out"];
        const r = 6.5;
        const tipx = cx + dx * r, tipy = cy + dy * r;
        ctx.beginPath();
        ctx.moveTo(tipx, tipy);
        ctx.lineTo(cx - dx * r * 0.5 + dy * r * 0.72, cy - dy * r * 0.5 - dx * r * 0.72);
        ctx.lineTo(cx - dx * r * 0.5 - dy * r * 0.72, cy - dy * r * 0.5 + dx * r * 0.72);
        ctx.closePath();
        if (occupied) {
          ctx.fillStyle = colour;
          ctx.fill();
        } else {
          ctx.fillStyle = p["--deck"];
          ctx.fill();
          ctx.strokeStyle = colour;
          ctx.lineWidth = 1.4;
          ctx.stroke();
        }
      }
    }
  }

  drawVehicles(v, sel) {
    const { ctx, cell, p } = this;
    for (let i = 0; i < v.n; i++) {
      const dimmed = sel.oht >= 0 && sel.oht !== i;
      const cx = v.x[i] * cell + cell / 2;
      const cy = v.y[i] * cell + cell / 2;
      const ang = [0, Math.PI / 2, Math.PI, -Math.PI / 2][v.heading[i]] || 0;

      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(ang);
      if (dimmed) ctx.globalAlpha = 0.4;
      const w = cell * 0.5, h = cell * 0.66;
      ctx.fillStyle = p[VEH_COLOR[v.state[i]]] || p["--veh-idle"];
      this.roundRect(-w / 2, -h / 2, w, h, 3);
      ctx.fill();
      // An outline, because several of the body colours are light enough to
      // disappear into a pale deck without one.
      ctx.strokeStyle = p["--veh-edge"];
      ctx.lineWidth = 1.2;
      ctx.stroke();
      // Nose bar, so heading is readable at a glance.
      ctx.fillStyle = p["--carry"];
      ctx.globalAlpha = 0.75;
      ctx.fillRect(-w / 2 + 3, -h / 2 + 3, w - 6, 2.5);
      ctx.globalAlpha = 1;
      // The vehicle id, so a row in the OHT list can be found on the map.
      ctx.rotate(-ang);
      ctx.fillStyle = p["--carry"];
      ctx.font = "600 9px ui-sans-serif, system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(String(i), 0, 0.5);
      ctx.restore();
    }
  }

  /// Which vehicle, if any, is under a point in CSS pixels.
  vehicleAt(px, py) {
    const v = this.lastVehicles;
    if (!v) return -1;
    const cx = Math.floor(px / this.cell), cy = Math.floor(py / this.cell);
    for (let i = 0; i < v.n; i++) {
      if (v.x[i] === cx && v.y[i] === cy) return i;
    }
    return -1;
  }

  roundRect(x, y, w, h, r) {
    const ctx = this.ctx;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(x, y, w, h, r);
    else ctx.rect(x, y, w, h);
  }
}

// --------------------------------------------------------------------------
// Panels
// --------------------------------------------------------------------------

const fmt = (n, d = 0) => n.toLocaleString(undefined, {
  minimumFractionDigits: d, maximumFractionDigits: d,
});

const esc = (t) => String(t).replace(/[&<>]/g, (c) =>
  ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));

/// Where a lot physically is, as a phrase and as a cell to ring on the map.
function lotPlace(map, lots, i, veh) {
  const place = lots.place[i], a = lots.a[i], b = lots.b[i];
  if (place === PROCESSING) {
    return { text: `in ${map.machines[a].name}`, cell: null };
  }
  if (place === IN_TRANSIT) {
    const cell = a < veh.n ? veh.y[a] * map.width + veh.x[a] : null;
    return { text: `riding OHT ${a}`, cell };
  }
  const port = map.machines[a].ports[b];
  return {
    text: `waiting at ${map.machines[a].name} ${port.kind}`,
    cell: port.cell[1] * map.width + port.cell[0],
  };
}

function pips(step, total) {
  let out = '<span class="pips">';
  for (let i = 0; i < total; i++) {
    const cls = i < step ? "done" : i === step ? "now" : "";
    out += `<i class="${cls}"></i>`;
  }
  return out + "</span>";
}

/// The cell to ring for the followed lot, or null if it has no place on the
/// map (inside a tool) or has completed. Cheap enough to run every frame, which
/// the canvas needs even while the Lots tab is hidden.
function followedLotCell(map, lots, veh, sel) {
  if (sel.lot < 0) return null;
  for (let i = 0; i < lots.n; i++) {
    if (lots.id[i] === sel.lot) return lotPlace(map, lots, i, veh).cell;
  }
  sel.lot = -1;                        // it completed; stop following it
  return null;
}

function renderLots(map, recipes, lots, veh, sel) {
  $("lotsummary").textContent =
    `${lots.n} lot${lots.n === 1 ? "" : "s"} in the fab. Click one to follow it.`;

  // Stable order by id, so a row does not jump under the cursor as lots
  // change state.
  const order = [...Array(lots.n).keys()].sort((x, y) => lots.id[x] - lots.id[y]);

  $("lots").innerHTML = order.map((i) => {
    const total = lots.total[i];
    const step = Math.min(lots.step[i], total);
    const kind = recipes[lots.recipe[i]]?.steps[step] ?? "done";
    const where = lotPlace(map, lots, i, veh);
    const wait = lots.place[i] === AT_PORT && lots.wait[i] > 0
      ? `${fmt(lots.wait[i])}t` : "";
    const hot = lots.priority[i] > 0 ? ' <span class="kind">hot</span>' : "";
    return `<li data-lot="${lots.id[i]}" aria-selected="${sel.lot === lots.id[i]}">
      <div class="line">
        <span class="who">#${lots.id[i]}${hot}</span>
        <span class="num">${wait}</span>
      </div>
      <div class="sub">${pips(step, total)}${step}/${total} ·
        next <span class="kind">${esc(kind)}</span> · ${esc(where.text)}</div>
    </li>`;
  }).join("");

  const detail = $("lotdetail");
  const at = order.find((i) => lots.id[i] === sel.lot);
  if (at === undefined) {
    detail.hidden = true;
    return;
  }
  const total = lots.total[at];
  const step = Math.min(lots.step[at], total);
  const steps = recipes[lots.recipe[at]]?.steps ?? [];
  const where = lotPlace(map, lots, at, veh);
  detail.hidden = false;
  detail.innerHTML = `<b>Lot #${lots.id[at]}</b> — ${esc(where.text)}
    <div class="steps">${steps.map((k, j) =>
      `<span class="${j < step ? "done" : j === step ? "now" : ""}">${esc(k)}</span>`
    ).join("")}</div>`;
}

function renderOhts(map, veh, targets, sim, sel) {
  const rows = [];
  for (let i = 0; i < veh.n; i++) {
    const st = veh.state[i];
    const tm = targets.machine[i];
    const lot = targets.lot[i];
    const hops = sim.route(i).length;
    const target = tm >= 0
      ? `${map.machines[tm].name} ${map.machines[tm].ports[targets.port[i]].kind}`
      : "—";
    const cargo = lot >= 0 ? `lot #${lot}` : "empty";
    rows.push(`<li data-oht="${i}" aria-selected="${sel.oht === i}">
      <div class="line">
        <span class="who"><i class="sw" data-state="${st}"></i> OHT ${i}</span>
        <span class="num">${hops ? `${hops} hop${hops === 1 ? "" : "s"}` : ""}</span>
      </div>
      <div class="sub">${VEH_LABEL[st]} · ${cargo}${
        tm >= 0 ? ` · → <span class="kind">${esc(target)}</span>` : ""}</div>
    </li>`);
  }
  $("ohts").innerHTML = rows.join("");
}

function renderMachines(map, mach, idle, tick) {
  $("machines").innerHTML = map.machines.map((m, i) => {
    // Source and sink never register as idle -- nothing is "in process" at
    // either -- so a utilisation figure for them would read 100% and mean
    // nothing.
    const virtual = m.kind === "source" || m.kind === "sink";
    const util = virtual || !tick ? null : 1 - idle[i] / tick;
    return `<li class="static">
      <div class="line">
        <span class="who">${esc(m.name)}</span>
        <span class="num">${util === null ? "—" : `${fmt(util * 100, 1)}%`} ·
          queue ${mach.load[i]}</span>
      </div>
      ${util === null ? "" :
        `<div class="bar util"><i style="width:${Math.max(0, util) * 100}%"></i></div>`}
    </li>`;
  }).join("");
}

/// Cycle-time distribution. A histogram rather than a mean because the whole
/// point of comparing policies here is the shape of the tail: two policies can
/// agree on the average and disagree completely on the worst lot.
function renderHistogram(canvas, cycles) {
  const ctx = canvas.getContext("2d");
  const dpr = devicePixelRatio || 1;
  const w = canvas.clientWidth || 280, h = 116;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  canvas.style.height = `${h}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const style = getComputedStyle(document.documentElement);
  const ink = style.getPropertyValue("--ink-soft").trim();
  const dim = style.getPropertyValue("--dim").trim();
  const accent = style.getPropertyValue("--accent").trim();
  const sel = style.getPropertyValue("--select").trim();

  if (!cycles.length) {
    ctx.fillStyle = dim;
    ctx.font = "12px ui-sans-serif, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("no lots completed yet", w / 2, h / 2);
    $("histnote").textContent = "";
    return;
  }

  const sorted = Float64Array.from(cycles).sort();
  const pct = (q) => sorted[Math.min(sorted.length - 1,
    Math.floor((sorted.length - 1) * q))];
  const p50 = pct(0.5), p95 = pct(0.95);
  const max = sorted[sorted.length - 1];
  const BINS = 24;
  const hi = Math.max(max, 1);
  const bins = new Array(BINS).fill(0);
  for (const c of sorted) bins[Math.min(BINS - 1, Math.floor((c / hi) * BINS))]++;
  const peak = Math.max(...bins);

  const padB = 16;
  const bw = w / BINS;
  ctx.fillStyle = accent;
  bins.forEach((n, i) => {
    if (!n) return;
    const bh = ((h - padB - 12) * n) / peak;
    ctx.fillRect(i * bw + 0.5, h - padB - bh, bw - 1, bh);
  });

  // Percentile labels ride along the top. Along the bottom they collided with
  // the axis end labels whenever p95 sat near the maximum, which is exactly
  // the case a long tail produces.
  ctx.font = "10px ui-sans-serif, system-ui, sans-serif";
  const padT = 12;
  for (const [v, label] of [[p50, "p50"], [p95, "p95"]]) {
    const x = (v / hi) * w;
    ctx.strokeStyle = sel;
    ctx.setLineDash([3, 2]);
    ctx.beginPath();
    ctx.moveTo(x, padT);
    ctx.lineTo(x, h - padB);
    ctx.stroke();
    ctx.setLineDash([]);
    const tx = Math.min(w - 12, Math.max(12, x));
    ctx.textAlign = "center";
    // A chip behind the text: the line often lands on top of a tall bar.
    ctx.fillStyle = style.getPropertyValue("--paper").trim();
    ctx.fillRect(tx - 12, 0, 24, padT - 1);
    ctx.fillStyle = sel;
    ctx.fillText(label, tx, 7);
  }
  ctx.fillStyle = dim;
  ctx.textAlign = "left";
  ctx.fillText("0", 1, h - 5);
  ctx.textAlign = "right";
  ctx.fillText(`${fmt(hi)}t`, w - 1, h - 5);
  void ink;

  $("histnote").textContent =
    `${cycles.length} completed · median ${fmt(p50)} · p95 ${fmt(p95)} · worst ${fmt(max)} ticks`;
}

function renderMetrics(m) {
  const rows = [
    ["lots created", fmt(m[M.CREATED])],
    ["lots completed", fmt(m[M.COMPLETED])],
    ["throughput", `${fmt(m[M.THROUGHPUT], 2)} /1k ticks`],
    ["cycle mean", `${fmt(m[M.MEAN_CYCLE])} ticks`],
    ["utilisation", `${fmt(m[M.UTIL] * 100, 1)}%`],
    ["vehicles busy", fmt(m[M.BUSY_NOW])],
    ["backlog now", fmt(m[M.BACKLOG_NOW])],
    ["backlog mean", fmt(m[M.MEAN_BACKLOG], 2)],
    ["stuck recoveries", fmt(m[M.STUCK]), m[M.STUCK] > 0],
    ["deadlocks", fmt(m[M.DEADLOCKS]), m[M.DEADLOCKS] > 0],
  ];
  $("metrics").innerHTML = rows.map(([k, v, warn]) =>
    `<dt>${k}</dt><dd${warn ? ' class="warn"' : ""}>${v}</dd>`).join("");
}

// --------------------------------------------------------------------------
// Main
// --------------------------------------------------------------------------

async function text(rel) {
  const res = await fetch(`${ROOT}/${rel}`);
  if (!res.ok) throw new Error(`cannot load ${rel} (HTTP ${res.status})`);
  return res.text();
}

async function main() {
  let wasmExports, mapText, scenText;
  try {
    const [wasm, mt, st] = await Promise.all([
      WebAssembly.instantiateStreaming(fetch("ohtsim.wasm"), {}),
      text(MAP),
      text(SCENARIO),
    ]);
    wasmExports = wasm.instance.exports;
    mapText = mt;
    scenText = st;
  } catch (err) {
    fail(`Failed to load.\n\n${err.message}\n\n` +
         `Build the module and serve from the repo root:\n` +
         `  ./web/build.sh\n  ./web/serve.sh`);
    return;
  }

  const map = JSON.parse(mapText);
  const recipes = JSON.parse(scenText).recipes || [];
  $("mapname").textContent =
    `${map.name} · ${map.width}×${map.height} · ${map.machines.length} tools`;

  const renderer = new Renderer($("fab"), map);
  const sel = { lot: -1, oht: -1, lotCell: null };
  let sim = null;
  let running = true;
  let carry = 0;
  let tab = "lots";

  async function build() {
    if (sim) sim.free();
    const polText = await text(`policies/${$("policy").value}.json`);
    try {
      sim = new Sim(wasmExports, mapText, scenText, polText);
    } catch (err) {
      fail(`Could not start the simulation.\n\n${err.message}`);
      sim = null;
      return;
    }
    carry = 0;
    // Selections name lots and vehicles in the old world; keep the OHT (ids
    // are stable, there are always eight) and drop the lot.
    sel.lot = -1;
    sel.lotCell = null;
    paint();
  }

  function paint() {
    if (!sim) return;
    const veh = sim.vehicles();
    const metrics = sim.metrics();
    $("tickval").textContent = fmt(metrics[M.TICK]);

    // The canvas rings the followed lot whichever tab is showing, so its
    // position is resolved every frame; the list itself only when visible.
    const lots = sim.lots();
    sel.lotCell = followedLotCell(map, lots, veh, sel);
    if (tab === "lots") renderLots(map, recipes, lots, veh, sel);

    if (tab === "ohts") renderOhts(map, veh, sim.targets(), sim, sel);
    if (tab === "machines") {
      renderMachines(map, sim.machines(), sim.machineIdle(), metrics[M.TICK]);
    }
    if (tab === "stats") {
      renderHistogram($("hist"), sim.cycles());
      renderMetrics(metrics);
    }
    renderer.draw(sim, sel);
  }

  function frame() {
    if (sim && running) {
      carry += SPEEDS[+$("speed").value];
      const n = Math.floor(carry);
      if (n > 0) {
        carry -= n;
        sim.tick(n);
        paint();
      }
    }
    requestAnimationFrame(frame);
  }

  // -- transport -----------------------------------------------------------
  $("play").addEventListener("click", () => {
    running = !running;
    $("play").textContent = running ? "Pause" : "Play";
  });
  $("step").addEventListener("click", () => {
    if (!sim) return;
    running = false;
    $("play").textContent = "Play";
    sim.tick(1);
    paint();
  });
  $("reset").addEventListener("click", build);
  $("policy").addEventListener("change", build);
  $("speed").addEventListener("input", () => {
    $("speedlabel").textContent = `${SPEEDS[+$("speed").value]}×`;
  });
  $("speed").dispatchEvent(new Event("input"));

  // -- tabs ----------------------------------------------------------------
  $("tabs").addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-tab]");
    if (!btn) return;
    tab = btn.dataset.tab;
    for (const b of $("tabs").children) {
      b.setAttribute("aria-selected", String(b === btn));
    }
    for (const name of ["lots", "ohts", "machines", "stats"]) {
      $(`panel-${name}`).hidden = name !== tab;
    }
    paint();
  });

  // -- selection -----------------------------------------------------------
  const toggle = (key, value) => {
    sel[key] = sel[key] === value ? -1 : value;
    paint();
  };
  $("lots").addEventListener("click", (ev) => {
    const li = ev.target.closest("li[data-lot]");
    if (li) toggle("lot", +li.dataset.lot);
  });
  $("ohts").addEventListener("click", (ev) => {
    const li = ev.target.closest("li[data-oht]");
    if (li) toggle("oht", +li.dataset.oht);
  });

  // -- canvas hover and click ----------------------------------------------
  const canvas = $("fab");
  const tip = $("tip");
  // The canvas is laid out responsively, so CSS pixels and its internal
  // coordinate system are not the same scale.
  const localPoint = (ev) => {
    const r = canvas.getBoundingClientRect();
    return [(ev.clientX - r.left) * (renderer.w / r.width),
            (ev.clientY - r.top) * (renderer.h / r.height)];
  };

  canvas.addEventListener("mousemove", (ev) => {
    if (!sim) return;
    const i = renderer.vehicleAt(...localPoint(ev));
    if (i < 0) { tip.hidden = true; return; }
    const veh = sim.vehicles();
    const t = sim.targets();
    const tm = t.machine[i];
    const target = tm >= 0
      ? `→ ${map.machines[tm].name} ${map.machines[tm].ports[t.port[i]].kind}`
      : "no job";
    const hops = sim.route(i).length;
    tip.textContent = `OHT ${i} · ${VEH_LABEL[veh.state[i]]}\n` +
      `${t.lot[i] >= 0 ? `carrying lot #${t.lot[i]}` : "empty"}\n` +
      `${target}${hops ? ` (${hops} hops)` : ""}`;
    tip.hidden = false;
    const s = $("stage").getBoundingClientRect();
    tip.style.left = `${ev.clientX - s.left + 14}px`;
    tip.style.top = `${ev.clientY - s.top + 14}px`;
  });
  canvas.addEventListener("mouseleave", () => { tip.hidden = true; });
  canvas.addEventListener("click", (ev) => {
    if (!sim) return;
    const i = renderer.vehicleAt(...localPoint(ev));
    if (i >= 0) toggle("oht", i);
  });

  await build();
  requestAnimationFrame(frame);
}

main();
