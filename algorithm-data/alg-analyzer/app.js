// app.js

// ---- DOM ----
const el = (id) => document.getElementById(id);

const fileInput = el("file");
const dropzone = el("dropzone");
const fileBadge = el("fileBadge");
const rowsLoaded = el("rowsLoaded");
const timeSpan = el("timeSpan");
const loadErr = el("loadErr");

const btnReplay = el("btnReplay");
const btnReset = el("btnReset");

const plotOverlay = el("plotOverlay");
const plotBadge = el("plotBadge");

const cb_fl = el("cb_fl");
const cb_fr = el("cb_fr");
const cb_rl = el("cb_rl");
const cb_rr = el("cb_rr");

const cb_overlay = el("cb_overlay");
const strideEl = el("stride");
const timeWindowEl = el("timeWindow");

const runInfo = el("runInfo");
const runErr = el("runErr");

const m_fl = el("m_fl");
const m_fr = el("m_fr");
const m_rl = el("m_rl");
const m_rr = el("m_rr");

const canvas = el("chart");
const ctx = canvas.getContext("2d");

const paramsEditor = el("paramsEditor");
const algoEditor = el("algoEditor");
const btnApplyAlgo = el("btnApplyAlgo");
const cbAutoApply = el("cbAutoApply");
const algoErr = el("algoErr");
const liveBadge = el("liveBadge");

let activeTV = null; // the function we actually call
let lastGoodSource = ""; // last compiled editor text
let autoApplyTimer = 0;

// ---- State ----
let dataset = null; // {header, rows}
let sim = null; // computed arrays

// ---- Helpers ----
function safeInt(v, fallback = 0) {
  const n = Number(v);
  return Number.isFinite(n) ? n | 0 : fallback;
}

function mapCsvGearToEnum(g) {
  // CSV: 1 forward, -1 reverse, 0 neutral
  // Algo enum: N=0, D=1, R=2
  if (g === 1) return 1; // D
  if (g === -1) return 2; // R
  return 0; // N
}

function parseCsv(text) {
  const lines = text.replace(/\r/g, "").trim().split("\n");
  if (lines.length < 2) throw new Error("CSV has no data rows.");
  const header = lines[0].split(",").map((s) => s.trim());
  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue;
    const parts = line.split(",");
    while (parts.length < header.length) parts.push("");
    if (parts.length > header.length)
      throw new Error(`Row ${i + 1}: column count mismatch.`);
    const obj = {};
    for (let c = 0; c < header.length; c++) obj[header[c]] = parts[c];
    rows.push(obj);
  }
  return { header, rows };
}

function computeMetrics(a, b) {
  let n = 0,
    sse = 0,
    sae = 0,
    max = 0;
  const len = Math.min(a.length, b.length);
  for (let i = 0; i < len; i++) {
    const x = Number(a[i]),
      y = Number(b[i]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const e = y - x;
    const ae = Math.abs(e);
    sse += e * e;
    sae += ae;
    if (ae > max) max = ae;
    n++;
  }
  if (!n) return { rmse: NaN, mae: NaN, max: NaN };
  return { rmse: Math.sqrt(sse / n), mae: sae / n, max };
}

function fmtMet(m) {
  if (!Number.isFinite(m.rmse)) return "—";
  return `${m.rmse.toFixed(1)} / ${m.mae.toFixed(1)} / ${m.max.toFixed(0)}`;
}

function clearCanvas() {
  resizeCanvasToDisplaySize();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "rgba(255,255,255,0.04)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function minMax(arr) {
  let min = Infinity,
    max = -Infinity;
  for (const v of arr) {
    const n = Number(v);
    if (!Number.isFinite(n)) continue;
    if (n < min) min = n;
    if (n > max) max = n;
  }
  if (min === Infinity) ((min = 0), (max = 1));
  return { min, max };
}

function drawLine(xs, ys, xMap, yMap, color, width, dash = null) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash || []);
  ctx.beginPath();
  let started = false;

  for (let i = 0; i < xs.length; i++) {
    const x = xs[i],
      y = ys[i];
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const px = xMap(x),
      py = yMap(y);
    if (!started) {
      ctx.moveTo(px, py);
      started = true;
    } else ctx.lineTo(px, py);
  }

  if (started) ctx.stroke();
  ctx.restore();
}

// ---- Loading ----
async function loadFile(file) {
  loadErr.textContent = "";
  runErr.textContent = "";
  try {
    const text = await file.text();
    const parsed = parseCsv(text);

    const req = [
      "ts_ms",
      "t",
      "s",
      "tq_fl",
      "tq_fr",
      "tq_rl",
      "tq_rr",
      "rpm_fl",
      "rpm_fr",
      "rpm_rl",
      "rpm_rr",
      "gear",
    ];
    for (const k of req) {
      if (!parsed.header.includes(k))
        throw new Error(`Missing required column: ${k}`);
    }

    dataset = parsed;

    fileBadge.textContent = file.name;
    rowsLoaded.textContent = String(dataset.rows.length);

    const t0 = Number(dataset.rows[0].ts_ms);
    const t1 = Number(dataset.rows[dataset.rows.length - 1].ts_ms);
    timeSpan.textContent =
      Number.isFinite(t0) && Number.isFinite(t1)
        ? `${(t1 - t0).toLocaleString()} ms`
        : "—";

    btnReplay.disabled = false;
    btnReset.disabled = false;

    runInfo.textContent = "Ready. Click “Replay & Compare”.";
    clearCanvas();
    resetMetrics();
    runReplay();
  } catch (e) {
    dataset = null;
    loadErr.textContent = String(e?.message || e);
    btnReplay.disabled = true;
  }
}

function resetMetrics() {
  m_fl.textContent =
    m_fr.textContent =
    m_rl.textContent =
    m_rr.textContent =
      "—";
}

function lerpInt(a, b, u) {
  return Math.round(lerp(a, b, u)) | 0;
}

function lerpLastRPM(prev, curr, u) {
  return {
    receivedTorqueL: lerpInt(
      prev.lastFront.receivedTorqueL,
      curr.lastFront.receivedTorqueL,
      u,
    ),
    receivedTorqueR: lerpInt(
      prev.lastFront.receivedTorqueR,
      curr.lastFront.receivedTorqueR,
      u,
    ),
    rpmL: lerpInt(prev.lastFront.rpmL, curr.lastFront.rpmL, u),
    rpmR: lerpInt(prev.lastFront.rpmR, curr.lastFront.rpmR, u),
  };
}
// ---- Replay ----
function runReplay() {
  runErr.textContent = "";
  if (!dataset) return;

  if (typeof activeTV !== "function") {
    runErr.textContent =
      "No active TorqueVectoring function. Use the Live Algorithm editor (Apply).";
    return;
  }

  const rows = dataset.rows;
  const n = rows.length;

  sim = {
    ts: new Array(n),
    rec: {
      tq_fl: new Array(n),
      tq_fr: new Array(n),
      tq_rl: new Array(n),
      tq_rr: new Array(n),
    },
    sim: {
      tq_fl: new Array(n),
      tq_fr: new Array(n),
      tq_rl: new Array(n),
      tq_rr: new Array(n),
    },
  };

  const tStart = performance.now();
  const timeScaling =
    typeof window.timeScaling === "number" &&
    Number.isFinite(window.timeScaling)
      ? Math.max(1, Math.floor(window.timeScaling))
      : 1;

  let lastInput = null;

  for (let i = 0; i < n; i++) {
    const r = rows[i];

    const ts = Number(r.ts_ms);
    sim.ts[i] = Number.isFinite(ts) ? ts : i;

    // recorded torques
    sim.rec.tq_fl[i] = safeInt(r.tq_fl);
    sim.rec.tq_fr[i] = safeInt(r.tq_fr);
    sim.rec.tq_rl[i] = safeInt(r.tq_rl);
    sim.rec.tq_rr[i] = safeInt(r.tq_rr);

    let out = { fl: 0, fr: 0, rl: 0, rr: 0 };

    try {
      const currInput = {
        gear: mapCsvGearToEnum(safeInt(r.gear)),
        t: safeInt(r.t),
        s: safeInt(r.s),
        lastFront: {
          receivedTorqueL: safeInt(r.tq_fl),
          receivedTorqueR: safeInt(r.tq_fr),
          rpmL: safeInt(r.rpm_fl),
          rpmR: safeInt(r.rpm_fr),
        },
        lastRear: {
          receivedTorqueL: safeInt(r.tq_rl),
          receivedTorqueR: safeInt(r.tq_rr),
          rpmL: safeInt(r.rpm_rl),
          rpmR: safeInt(r.rpm_rr),
        },
      };
      // Run internal ticks at 20ms resolution within this 100ms sample interval.
      // u goes 0..1 across the ticks (includes endpoints).

      if (lastInput === null || timeScaling <= 1) {
        // first iteration runs only once, all others go into the loop
        out =
          activeTV(
            currInput.gear,
            currInput.t,
            currInput.s,
            currInput.lastFront,
            currInput.lastRear,
          ) || out;
      } else {
        for (let k = 1; k <= timeScaling; k++) {
          const u = k / timeScaling; // 0.2..1.0 for 5 ticks
          const t = lerpInt(lastInput.t, currInput.t, u);
          const s = lerpInt(lastInput.s, currInput.s, u);
          const lastFront = lerpLastRPM(
            lastInput.lastFront,
            currInput.lastFront,
            u,
          );
          const lastRear = lerpLastRPM(
            lastInput.lastFront,
            currInput.lastFront,
            u,
          );

          out = activeTV(currInput.gear, t, s, lastFront, lastRear) || out;
        }
        lastInput = currInput;
      }
    } catch (e) {
      runErr.textContent = `Algorithm error at row ${i + 1}: ${String(e?.message || e)}`;
      break;
    }

    // clamp to your range (store only the LAST tick result)
    sim.sim.tq_fl[i] = Math.max(-1000, Math.min(1000, safeInt(out.fl)));
    sim.sim.tq_fr[i] = Math.max(-1000, Math.min(1000, safeInt(out.fr)));
    sim.sim.tq_rl[i] = Math.max(-1000, Math.min(1000, safeInt(out.rl)));
    sim.sim.tq_rr[i] = Math.max(-1000, Math.min(1000, safeInt(out.rr)));
  }
  const tEnd = performance.now();

  // metrics
  m_fl.textContent = fmtMet(computeMetrics(sim.rec.tq_fl, sim.sim.tq_fl));
  m_fr.textContent = fmtMet(computeMetrics(sim.rec.tq_fr, sim.sim.tq_fr));
  m_rl.textContent = fmtMet(computeMetrics(sim.rec.tq_rl, sim.sim.tq_rl));
  m_rr.textContent = fmtMet(computeMetrics(sim.rec.tq_rr, sim.sim.tq_rr));

  runInfo.textContent = `Replayed ${n.toLocaleString()} rows in ${(tEnd - tStart).toFixed(1)} ms.`;
  draw();
}

// ---- Plotting ----
function getRecordedSignal(name, i) {
  if (!dataset) return NaN;
  const r = dataset.rows[i];
  if (!r || !(name in r)) return NaN;
  return safeInt(r[name], NaN);
}

function getSimSignal(name, i) {
  // simulated exists only for tq_*
  if (!sim) return NaN;
  if (name === "tq_fl") return sim.sim.tq_fl[i];
  if (name === "tq_fr") return sim.sim.tq_fr[i];
  if (name === "tq_rl") return sim.sim.tq_rl[i];
  if (name === "tq_rr") return sim.sim.tq_rr[i];
  return NaN;
}

function draw() {
  if (!dataset || !sim) return;

  const { cssW, cssH } = resizeCanvasToDisplaySize();
  // Use CSS dimensions for layout math now:
  const W = cssW,
    H = cssH;

  const stride = Math.max(1, safeInt(strideEl.value, 1));
  const timeWindow = Math.max(0, safeInt(timeWindowEl.value, 0));

  const show = {
    fl: !!cb_fl.checked,
    fr: !!cb_fr.checked,
    rl: !!cb_rl.checked,
    rr: !!cb_rr.checked,
  };

  const wheels = Object.entries(show)
    .filter(([_, v]) => v)
    .map(([k]) => k);
  plotBadge.textContent = wheels.length
    ? wheels.map((w) => w.toUpperCase()).join("+")
    : "None";

  const overlayName = plotOverlay.value;
  const overlayOn = !!cb_overlay.checked && !!overlayName;

  // colors per wheel (recorded solid, simulated dashed)
  const C = {
    fl: "#60a5fa",
    fr: "#a78bfa",
    rl: "#34d399",
    rr: "#fb7185",
    ov: "#fbbf24",
  };

  const tsEnd = sim.ts[sim.ts.length - 1];
  const tsStart = sim.ts[0];
  const windowStartTs = timeWindow > 0 ? tsEnd - timeWindow : tsStart;

  const xs = [];
  const gears = [];
  const series = {
    fl_rec: [],
    fl_sim: [],
    fr_rec: [],
    fr_sim: [],
    rl_rec: [],
    rl_sim: [],
    rr_rec: [],
    rr_sim: [],
    ov: [],
  };

  // Build arrays
  for (let i = 0; i < sim.ts.length; i += stride) {
    const ts = sim.ts[i];
    if (ts < windowStartTs) continue;
    xs.push(ts);

    // gear for this plotted sample
    const g = mapCsvGearToEnum(safeInt(dataset.rows[i].gear));
    gears.push(g);

    series.fl_rec.push(sim.rec.tq_fl[i]);
    series.fl_sim.push(sim.sim.tq_fl[i]);
    series.fr_rec.push(sim.rec.tq_fr[i]);
    series.fr_sim.push(sim.sim.tq_fr[i]);
    series.rl_rec.push(sim.rec.tq_rl[i]);
    series.rl_sim.push(sim.sim.tq_rl[i]);
    series.rr_rec.push(sim.rec.tq_rr[i]);
    series.rr_sim.push(sim.sim.tq_rr[i]);

    if (overlayOn) {
      series.ov.push(safeInt(dataset.rows[i][overlayName], NaN));
    } else {
      series.ov.push(NaN);
    }
  }

  // Determine Y range from selected torque series only (so overlay doesn’t wreck scaling)
  // Always scale using ALL torques (stable axis)
  let yAll = []
    .concat(series.fl_rec, series.fl_sim)
    .concat(series.fr_rec, series.fr_sim)
    .concat(series.rl_rec, series.rl_sim)
    .concat(series.rr_rec, series.rr_sim)
    .filter(Number.isFinite);

  const { min: minX, max: maxX } = minMax(xs);
  const { min: minY0, max: maxY0 } = minMax(yAll);
  const padY = (maxY0 - minY0) * 0.08 || 1;
  const minY = minY0 - padY;
  const maxY = maxY0 + padY;

  // Overlay handling:
  // For t/s (and tq_*), plot RAW so values match the torque axis (e.g. 300 == 300).
  // For other signals (rpm/cu/etc), keep the old "fit into axis" behavior so it stays visible.
  let ovPlot = series.ov;

  // Layout

  const pad = { l: 70, r: 18, t: 22, b: 36 };
  const pw = W - pad.l - pad.r;
  const ph = H - pad.t - pad.b;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "rgba(0,0,0,0.18)";
  ctx.fillRect(0, 0, W, H);

  // Grid
  ctx.strokeStyle = "rgba(255,255,255,0.06)";
  ctx.lineWidth = 1;
  for (let k = 0; k <= 5; k++) {
    const y = pad.t + (ph * k) / 5;
    ctx.beginPath();
    ctx.moveTo(pad.l, y);
    ctx.lineTo(pad.l + pw, y);
    ctx.stroke();
  }
  for (let k = 0; k <= 6; k++) {
    const x = pad.l + (pw * k) / 6;
    ctx.beginPath();
    ctx.moveTo(x, pad.t);
    ctx.lineTo(x, pad.t + ph);
    ctx.stroke();
  }

  // Labels
  ctx.fillStyle = "rgba(157,176,198,0.95)";
  ctx.font = `12px ui-sans-serif, system-ui`;
  ctx.fillText(`Torques (recorded solid, simulated dashed)`, pad.l, pad.t - 8);

  // Y ticks
  ctx.fillStyle = "rgba(157,176,198,0.85)";
  ctx.font = `11px ui-sans-serif, system-ui`;
  for (let k = 0; k <= 5; k++) {
    const y = pad.t + (ph * k) / 5;
    const v = maxY - (maxY - minY) * (k / 5);
    ctx.fillText(v.toFixed(0), 10, y + 4);
  }

  // X ticks (relative)
  const x0 = minX;
  for (let k = 0; k <= 6; k++) {
    const x = pad.l + (pw * k) / 6;
    const v = minX + (maxX - minX) * (k / 6) - x0;
    ctx.fillText(`${Math.round(v)}ms`, x - 18, pad.t + ph + 24);
  }

  const xMap = (x) => pad.l + ((x - minX) / (maxX - minX || 1)) * pw;
  const yMap = (y) => pad.t + ph - ((y - minY) / (maxY - minY || 1)) * ph;

  // --- Background bands for Reverse gear (Gear.R == 2) ---
  ctx.save();
  ctx.fillStyle = "rgba(255, 80, 80, 0.08)"; // subtle red

  let segStart = null;
  for (let j = 0; j < xs.length; j++) {
    const isR = gears[j] === Gear.R;

    if (isR && segStart === null) segStart = j;
    if ((!isR || j === xs.length - 1) && segStart !== null) {
      // end segment at j (exclusive), or include last point
      const endIdx = isR && j === xs.length - 1 ? j : j - 1;

      const x0 = xMap(xs[segStart]);
      const x1 = xMap(xs[endIdx]);

      // Make sure very short segments still visible
      const w = Math.max(1, x1 - x0);

      ctx.fillRect(x0, pad.t, w, ph);
      segStart = null;
    }
  }
  ctx.restore();

  // ---- zero line (y = 0) ----
  if (minY < 0 && maxY > 0) {
    // only if zero is in visible range
    const y0 = yMap(0);

    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,0.25)";
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 6]); // dashed center line
    ctx.beginPath();
    ctx.moveTo(pad.l, y0);
    ctx.lineTo(pad.l + pw, y0);
    ctx.stroke();

    // optional label
    ctx.fillStyle = "rgba(255,255,255,0.45)";
    ctx.font = "11px ui-sans-serif, system-ui";
    ctx.fillText("0", pad.l - 18, y0 + 4);

    ctx.restore();
  }

  const dashSim = [3, 2]; // dashed simulated

  // Draw selected wheels
  if (show.fl) {
    drawLine(xs, series.fl_rec, xMap, yMap, C.fl, 2);
    drawLine(xs, series.fl_sim, xMap, yMap, C.fl, 2, dashSim);
  }
  if (show.fr) {
    drawLine(xs, series.fr_rec, xMap, yMap, C.fr, 2);
    drawLine(xs, series.fr_sim, xMap, yMap, C.fr, 2, dashSim);
  }
  if (show.rl) {
    drawLine(xs, series.rl_rec, xMap, yMap, C.rl, 2);
    drawLine(xs, series.rl_sim, xMap, yMap, C.rl, 2, dashSim);
  }
  if (show.rr) {
    drawLine(xs, series.rr_rec, xMap, yMap, C.rr, 2);
    drawLine(xs, series.rr_sim, xMap, yMap, C.rr, 2, dashSim);
  }

  if (overlayOn) {
    drawLine(xs, ovPlot, xMap, yMap, C.ov, 1.5);
  }
}

function resizeCanvasToDisplaySize() {
  const dpr = Math.max(1, window.devicePixelRatio || 1);

  // Use CSS pixel size (reliable)
  const cssW = Math.max(300, canvas.clientWidth | 0);
  const cssH = Math.max(240, canvas.clientHeight | 0);

  const w = Math.floor(cssW * dpr);
  const h = Math.floor(cssH * dpr);

  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }

  // Critical: make drawing coordinates use CSS pixels
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  // Return CSS size so draw() can use it if needed
  return { cssW, cssH, dpr };
}

function extractBodyFromFunctionSource(fnSrc) {
  // fnSrc is "function TorqueVectoring(...) { ... }"
  const start = fnSrc.indexOf("{");
  const end = fnSrc.lastIndexOf("}");
  if (start < 0 || end <= start) return "";
  return fnSrc.slice(start + 1, end).trim() + "\n";
}

function initAlgorithm() {
  if (typeof window.TorqueVectoring === "function")
    activeTV = window.TorqueVectoring;

  // fill params editor
  if (typeof window.getParamsSource === "function") {
    paramsEditor.value = window.getParamsSource();
  } else {
    paramsEditor.value = "const params = { TV: {} };\n";
  }

  // fill algorithm body editor from file
  if (typeof window.getTorqueVectoringSource === "function") {
    algoEditor.value = extractBodyFromFunctionSource(
      window.getTorqueVectoringSource(),
    );
  } else {
    algoEditor.value = "";
  }

  btnApplyAlgo.disabled = false;
}

function compileEditorToFunction(paramsSrc, bodySrc) {
  // Create a params object from paramsSrc
  // Expect paramsSrc to define `const params = ...;` OR `params = ...;` and then `return params;`
  const makeParams = new Function(`
    "use strict";
    ${paramsSrc}
    if (typeof params === "undefined") throw new Error("params is not defined in params block");
    return params;
  `);

  const paramsObj = makeParams();

  const fn = new Function(
    "currGear",
    "t",
    "s",
    "lastFront",
    "lastRear",
    "clamp",
    "Torques",
    "Gear",
    "params",
    `"use strict";\n${bodySrc}`,
  );

  return (currGear, t, s, lastFront, lastRear) => {
    const out = fn(
      currGear,
      t,
      s,
      lastFront,
      lastRear,
      clamp,
      Torques,
      Gear,
      paramsObj,
    );
    if (!out || typeof out !== "object")
      throw new Error("Must return {fl,fr,rl,rr}.");
    return {
      fl: Math.max(-1000, Math.min(1000, safeInt(out.fl))),
      fr: Math.max(-1000, Math.min(1000, safeInt(out.fr))),
      rl: Math.max(-1000, Math.min(1000, safeInt(out.rl))),
      rr: Math.max(-1000, Math.min(1000, safeInt(out.rr))),
    };
  };
}

function applyEditorAlgorithm() {
  algoErr.textContent = "";
  try {
    const compiled = compileEditorToFunction(
      paramsEditor.value,
      algoEditor.value,
    );

    // quick smoke test (doesn't prove correctness, just catches obvious issues)
    compiled(
      1,
      0,
      0,
      { receivedTorqueL: 0, receivedTorqueR: 0, rpmL: 0, rpmR: 0 },
      { receivedTorqueL: 0, receivedTorqueR: 0, rpmL: 0, rpmR: 0 },
    );

    activeTV = compiled;
    liveBadge.textContent = "Using live editor ✅";

    // Re-run instantly if we already have data
    if (dataset) runReplay();
  } catch (e) {
    algoErr.textContent = "Compile/apply error: " + (e?.message || e);
  }
}

function scheduleAutoApply() {
  clearTimeout(autoApplyTimer);
  autoApplyTimer = setTimeout(() => {
    if (cbAutoApply.checked) applyEditorAlgorithm();
  }, 400);
}

// ---- Events ----

btnApplyAlgo.addEventListener("click", applyEditorAlgorithm);

paramsEditor.addEventListener("input", () => {
  if (cbAutoApply.checked) scheduleAutoApply();
});

algoEditor.addEventListener("input", () => {
  if (cbAutoApply.checked) scheduleAutoApply();
});

cbAutoApply.addEventListener("change", () => {
  if (cbAutoApply.checked) scheduleAutoApply();
});

fileInput.addEventListener("change", async () => {
  const f = fileInput.files?.[0];
  if (f) await loadFile(f);
});

dropzone.addEventListener("dragover", (e) => {
  e.preventDefault();
  dropzone.style.borderColor = "rgba(96,165,250,0.55)";
});
dropzone.addEventListener("dragleave", () => {
  dropzone.style.borderColor = "rgba(255,255,255,0.18)";
});
dropzone.addEventListener("drop", async (e) => {
  e.preventDefault();
  dropzone.style.borderColor = "rgba(255,255,255,0.18)";
  const f = e.dataTransfer.files?.[0];
  if (f) await loadFile(f);
});

btnReplay.addEventListener("click", runReplay);

btnReset.addEventListener("click", () => {
  dataset = null;
  sim = null;
  fileInput.value = "";
  fileBadge.textContent = "No file";
  rowsLoaded.textContent = "0";
  timeSpan.textContent = "—";
  loadErr.textContent = "";
  runErr.textContent = "";
  runInfo.textContent = "Load a file, then click “Replay & Compare”.";
  btnReplay.disabled = true;
  btnReset.disabled = true;
  resetMetrics();
  clearCanvas();
});

[cb_fl, cb_fr, cb_rl, cb_rr, cb_overlay].forEach((e) => {
  e.addEventListener("change", () => {
    if (sim) draw();
  });
});
plotOverlay.addEventListener("change", () => {
  if (sim) draw();
});
strideEl.addEventListener("change", () => {
  if (sim) draw();
});
timeWindowEl.addEventListener("change", () => {
  if (sim) draw();
});

let resizeRAF = 0;
window.addEventListener("resize", () => {
  cancelAnimationFrame(resizeRAF);
  resizeRAF = requestAnimationFrame(() => {
    if (sim) draw();
    else clearCanvas();
  });
});

// init
clearCanvas();
initAlgorithm();
