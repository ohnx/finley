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

  draw(sim) {
    const { ctx } = this;
    ctx.fillStyle = this.p["--deck"];
    ctx.fillRect(0, 0, this.w, this.h);
    this.drawMachines();
    this.drawCongestion(sim.congestion());
    this.drawTrack();
    this.drawPorts();
    this.drawVehicles(sim.vehicles());
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

  drawPorts() {
    const { ctx, cell } = this;
    for (const m of this.map.machines) {
      for (const port of m.ports) {
        const [x, y] = port.cell;
        ctx.fillStyle = port.kind === "in"
          ? this.p["--port-in"] : this.p["--port-out"];
        const s = 5;
        ctx.fillRect(x * cell + cell / 2 - s / 2, y * cell + cell / 2 - s / 2, s, s);
      }
    }
  }

  drawVehicles(v) {
    const { ctx, cell, p } = this;
    for (let i = 0; i < v.n; i++) {
      const cx = v.x[i] * cell + cell / 2;
      const cy = v.y[i] * cell + cell / 2;
      const ang = [0, Math.PI / 2, Math.PI, -Math.PI / 2][v.heading[i]] || 0;

      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(ang);
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
      if (v.carrying[i]) {
        const s = cell * 0.2;
        ctx.fillStyle = p["--carry"];
        ctx.fillRect(-s / 2, -s / 2, s, s);
        ctx.strokeStyle = p["--carry-edge"];
        ctx.strokeRect(-s / 2, -s / 2, s, s);
      }
      ctx.restore();
    }
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

function renderMetrics(m) {
  const rows = [
    ["tick", fmt(m[M.TICK])],
    ["lots created", fmt(m[M.CREATED])],
    ["lots completed", fmt(m[M.COMPLETED])],
    ["throughput", `${fmt(m[M.THROUGHPUT], 2)} /1k ticks`],
    ["cycle mean", `${fmt(m[M.MEAN_CYCLE])} ticks`],
    ["cycle p95", `${fmt(m[M.P95])} ticks`],
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

function renderMachines(map, mach) {
  $("machines").innerHTML = map.machines.map((m, i) => {
    const starv = Math.round((mach.starvation[i] || 0) * 100);
    return `<li>
      <div class="mrow"><span>${m.name}</span><span class="q">${mach.load[i]}</span></div>
      <div class="bar"><i style="width:${starv}%"></i></div>
    </li>`;
  }).join("");
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
  $("mapname").textContent =
    `${map.name} · ${map.width}×${map.height} · ${map.machines.length} tools`;

  const renderer = new Renderer($("fab"), map);
  let sim = null;
  let running = true;
  let carry = 0;

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
    paint();
  }

  function paint() {
    renderer.draw(sim);
    renderMetrics(sim.metrics());
    renderMachines(map, sim.machines());
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
    const s = SPEEDS[+$("speed").value];
    $("speedlabel").textContent = `${s} ticks/frame`;
  });
  $("speed").dispatchEvent(new Event("input"));

  await build();
  requestAnimationFrame(frame);
}

main();
