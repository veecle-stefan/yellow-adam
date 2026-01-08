/* Wheel-CSV Drive Replay
   - Parses CSV with headers including: ts_ms, rpm_fl, rpm_fr, rpm_rl, rpm_rr
   - Interprets rpm_* as pulses-per-second (default), then converts via PPR + wheel circumference.
   - Updates pose using differential-drive approximation (left vs right distance).
*/

const el = (id) => document.getElementById(id);

const canvas = el("c");
const ctx = canvas.getContext("2d");

const fileInput = el("file");
const btnPlay = el("btnPlay");
const btnReset = el("btnReset");
const speedInput = el("speed");
const seek = el("seek");
const seekLabel = el("seekLabel");

const trackInput = el("track");
const wheelbaseInput = el("wheelbase");
const circInput = el("circ");
const pprInput = el("ppr");

const statusEl = el("status");
const poseEl = el("pose");
const stepEl = el("step");
const distEl = el("dist");

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS pixels
}
window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// ----------------------- Data + Simulation State -----------------------

let samples = []; // {ts_ms, s, tq_fl,tq_fr,tq_rl,tq_rr, rpm_fl,rpm_fr,rpm_rl,rpm_rr}
let poses = []; // pose per index: {x_cm, y_cm, theta_rad}
let wheelPaths = {
  fl: [],
  fr: [],
  rl: [],
  rr: [],
};
let cumDistCm = []; // cumulative center distance per pose index
let maxAbsTq = 1000; // auto-detected from file (abs max over tq_*); fallback 1000
const CURRENT_UNITS_PER_AMP = 50; // tune: 1A = 50 units
const VOLT_SCALE = 0.01; // tune: raw vf/vr to volts (e.g. 3801 -> 38.01V)
const TEMP_SCALE = 0.1; // tune: raw tf/tr to °C (e.g. 257 -> 25.7°C)
let idx = 0;

let playing = false;
let lastRAF = 0;
let accumMs = 0;

// camera/view
let cam = {
  cx: 0,
  cy: 0, // world center to look at (cm)
  scale: 2.0, // px per cm
  targetScale: 2.0,
  targetCx: 0,
  targetCy: 0,
};

// bounding box of path (cm)
let bounds = { minX: 0, minY: 0, maxX: 0, maxY: 0, init: false };

// ----------------------- CSV Parsing -----------------------

function parseCSV(text) {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0);
  if (lines.length < 2) return [];

  const headers = lines[0].split(",").map((h) => h.trim());
  const h = Object.fromEntries(headers.map((name, i) => [name, i]));

  const required = ["ts_ms", "rpm_fl", "rpm_fr", "rpm_rl", "rpm_rr"];
  for (const r of required) {
    if (!(r in h)) throw new Error(`Missing column: ${r}`);
  }

  const out = [];
  for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(",");
    if (cols.length < headers.length) continue;

    const s = {
      ts_ms: Number(cols[h.ts_ms]),
      // optional inputs (missing cols default to 0)
      s: h.s != null ? Number(cols[h.s]) : 0,
      tq_fl: h.tq_fl != null ? Number(cols[h.tq_fl]) : 0,
      tq_fr: h.tq_fr != null ? Number(cols[h.tq_fr]) : 0,
      tq_rl: h.tq_rl != null ? Number(cols[h.tq_rl]) : 0,
      tq_rr: h.tq_rr != null ? Number(cols[h.tq_rr]) : 0,
      cu_fl: h.cu_fl != null ? Number(cols[h.cu_fl]) : 0,
      cu_fr: h.cu_fr != null ? Number(cols[h.cu_fr]) : 0,
      cu_rl: h.cu_rl != null ? Number(cols[h.cu_rl]) : 0,
      cu_rr: h.cu_rr != null ? Number(cols[h.cu_rr]) : 0,
      vf: h.vf != null ? Number(cols[h.vf]) : 0,
      vr: h.vr != null ? Number(cols[h.vr]) : 0,
      tf: h.tf != null ? Number(cols[h.tf]) : 0,
      tr: h.tr != null ? Number(cols[h.tr]) : 0,
      rpm_fl: Number(cols[h.rpm_fl]),
      rpm_fr: Number(cols[h.rpm_fr]),
      rpm_rl: Number(cols[h.rpm_rl]),
      rpm_rr: Number(cols[h.rpm_rr]),
    };
    if (!Number.isFinite(s.ts_ms)) continue;
    out.push(s);
  }
  return out;
}

// ----------------------- Kinematics -----------------------

function getConstants() {
  return {
    track_cm: Number(trackInput.value) || 30,
    wheelbase_cm: Number(wheelbaseInput.value) || 50,
    wheel_circ_cm: Number(circInput.value) || 20,
    pulses_per_rev: Math.max(1, Number(pprInput.value) || 15),
  };
}

/**
 * Convert "rpm_*" value to linear distance traveled during dt.
 * Assumption: rpm_* is pulses-per-second (hall clicks/s).
 * Then pulses in dt: pulses = value * dt_s
 * revolutions = pulses / pulses_per_rev
 * distance_cm = revolutions * wheel_circ_cm
 *
 */
function pulsesPerSecondToDistanceCm(
  pulsesPerSec,
  dt_s,
  wheel_circ_cm,
  pulses_per_rev,
) {
  const pulses = pulsesPerSec * dt_s;
  const rev = pulses / pulses_per_rev;
  return rev * wheel_circ_cm;
}

function stepPose(prev, sample, nextSample) {
  const { track_cm, wheel_circ_cm, pulses_per_rev } = getConstants();

  // dt from timestamps; fallback to 0.1s
  let dt_s = 0.1;
  if (nextSample) {
    const dt_ms = nextSample.ts_ms - sample.ts_ms;
    if (Number.isFinite(dt_ms) && dt_ms > 0 && dt_ms < 2000)
      dt_s = dt_ms / 1000;
  }

  const d_fl = pulsesPerSecondToDistanceCm(
    sample.rpm_fl,
    dt_s,
    wheel_circ_cm,
    pulses_per_rev,
  );
  const d_fr = pulsesPerSecondToDistanceCm(
    sample.rpm_fr,
    dt_s,
    wheel_circ_cm,
    pulses_per_rev,
  );
  const d_rl = pulsesPerSecondToDistanceCm(
    sample.rpm_rl,
    dt_s,
    wheel_circ_cm,
    pulses_per_rev,
  );
  const d_rr = pulsesPerSecondToDistanceCm(
    sample.rpm_rr,
    dt_s,
    wheel_circ_cm,
    pulses_per_rev,
  );

  const dL = 0.5 * (d_fl + d_rl);
  const dR = 0.5 * (d_fr + d_rr);

  const dTheta = (dL - dR) / Math.max(1e-6, track_cm);
  const ds = 0.5 * (dR + dL);

  // integrate in body frame (midpoint)
  const thetaMid = prev.theta_rad + dTheta * 0.5;
  const x = prev.x_cm + ds * Math.cos(thetaMid);
  const y = prev.y_cm + ds * Math.sin(thetaMid);
  const theta = prev.theta_rad + dTheta;

  return {
    x_cm: x,
    y_cm: y,
    theta_rad: theta,
    dt_s,
    d_fl,
    d_fr,
    d_rl,
    d_rr,
    dL,
    dR,
    ds,
  };
}

function wheelWorld(pose, dx_cm, dy_cm) {
  const c = Math.cos(pose.theta_rad);
  const s = Math.sin(pose.theta_rad);
  return {
    x_cm: pose.x_cm + dx_cm * c - dy_cm * s,
    y_cm: pose.y_cm + dx_cm * s + dy_cm * c,
  };
}

function recomputeAll() {
  poses = [];
  wheelPaths = { fl: [], fr: [], rl: [], rr: [] };
  cumDistCm = [];
  bounds = { minX: 0, minY: 0, maxX: 0, maxY: 0, init: false };

  const p0 = { x_cm: 0, y_cm: 0, theta_rad: 0 };
  poses.push(p0);
  cumDistCm.push(0);

  // seed wheel paths (torque for the very first point uses first sample if available)
  const { track_cm, wheelbase_cm } = getConstants();
  const ax = wheelbase_cm / 2;
  const wy = track_cm / 2;
  const s0 = samples[0] || { tq_fl: 0, tq_fr: 0, tq_rl: 0, tq_rr: 0 };
  const fl0 = wheelWorld(p0, +ax, +wy);
  const fr0 = wheelWorld(p0, +ax, -wy);
  const rl0 = wheelWorld(p0, -ax, +wy);
  const rr0 = wheelWorld(p0, -ax, -wy);
  wheelPaths.fl.push({ ...fl0, tq: Number.isFinite(s0.tq_fl) ? s0.tq_fl : 0 });
  wheelPaths.fr.push({ ...fr0, tq: Number.isFinite(s0.tq_fr) ? s0.tq_fr : 0 });
  wheelPaths.rl.push({ ...rl0, tq: Number.isFinite(s0.tq_rl) ? s0.tq_rl : 0 });
  wheelPaths.rr.push({ ...rr0, tq: Number.isFinite(s0.tq_rr) ? s0.tq_rr : 0 });

  // bounds should include wheels too (better zoom)
  for (const p of [fl0, fr0, rl0, rr0, { x_cm: p0.x_cm, y_cm: p0.y_cm }]) {
    updateBounds(p.x_cm, p.y_cm);
  }

  for (let i = 0; i < samples.length - 1; i++) {
    const prev = poses[i];
    const cur = samples[i];
    const nxt = samples[i + 1];
    const stepped = stepPose(prev, cur, nxt);
    const p = {
      x_cm: stepped.x_cm,
      y_cm: stepped.y_cm,
      theta_rad: stepped.theta_rad,
    };
    poses.push(p);

    // cumulative distance from center ds
    const prevDist = cumDistCm[i] || 0;
    cumDistCm.push(prevDist + (Number.isFinite(stepped.ds) ? stepped.ds : 0));

    // wheel traces for this pose, using torque from cur sample
    const { track_cm: tcm, wheelbase_cm: wcm } = getConstants();
    const ax2 = wcm / 2;
    const wy2 = tcm / 2;
    const fl = wheelWorld(p, +ax2, +wy2);
    const fr = wheelWorld(p, +ax2, -wy2);
    const rl = wheelWorld(p, -ax2, +wy2);
    const rr = wheelWorld(p, -ax2, -wy2);

    wheelPaths.fl.push({ ...fl, tq: Number.isFinite(cur.tq_fl) ? cur.tq_fl : 0 });
    wheelPaths.fr.push({ ...fr, tq: Number.isFinite(cur.tq_fr) ? cur.tq_fr : 0 });
    wheelPaths.rl.push({ ...rl, tq: Number.isFinite(cur.tq_rl) ? cur.tq_rl : 0 });
    wheelPaths.rr.push({ ...rr, tq: Number.isFinite(cur.tq_rr) ? cur.tq_rr : 0 });

    for (const wp of [fl, fr, rl, rr]) updateBounds(wp.x_cm, wp.y_cm);
  }

  idx = 0;
  seek.max = Math.max(0, poses.length - 1);
  seek.value = "0";
  seek.disabled = poses.length <= 1;
  seekLabel.textContent = `0 / ${Math.max(0, poses.length - 1)}`;
}

function updateBounds(x, y) {
  if (!bounds.init) {
    bounds = { minX: x, maxX: x, minY: y, maxY: y, init: true };
  } else {
    bounds.minX = Math.min(bounds.minX, x);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.maxY = Math.max(bounds.maxY, y);
  }
}

// ----------------------- Camera / Zoom-to-fit -----------------------

function updateCameraTargets(uptoIndex) {
  // bounds of path up to current index (incremental zoom-out feel)
  // recompute partial bounds cheaply by tracking global bounds is fine,
  // but for "reveal while running" we compute partial by scanning limited range occasionally.
  // We'll do a lightweight approach: compute partial bounds from 0..uptoIndex every N frames.
  const padCm = 40; // extra margin around path
  let minX = 0,
    maxX = 0,
    minY = 0,
    maxY = 0;

  if (uptoIndex <= 1) {
    const p = poses[uptoIndex] || { x_cm: 0, y_cm: 0 };
    minX = maxX = p.x_cm;
    minY = maxY = p.y_cm;
  } else {
    // partial bounds scan (fast enough for typical logs)
    minX = Infinity;
    minY = Infinity;
    maxX = -Infinity;
    maxY = -Infinity;
    for (let i = 0; i <= uptoIndex; i++) {
      const p = poses[i];
      if (!p) continue;
      minX = Math.min(minX, p.x_cm);
      maxX = Math.max(maxX, p.x_cm);
      minY = Math.min(minY, p.y_cm);
      maxY = Math.max(maxY, p.y_cm);
    }
  }

  minX -= padCm;
  maxX += padCm;
  minY -= padCm;
  maxY += padCm;

  const rect = canvas.getBoundingClientRect();
  const w = rect.width,
    h = rect.height;

  const spanX = Math.max(1, maxX - minX);
  const spanY = Math.max(1, maxY - minY);

  // px per cm to fit
  const s = Math.min(w / spanX, h / spanY);

  cam.targetScale = clamp(s, 0.05, 20);
  cam.targetCx = (minX + maxX) * 0.5;
  cam.targetCy = (minY + maxY) * 0.5;
}

function clamp(v, a, b) {
  return Math.max(a, Math.min(b, v));
}

// ----------------------- Drawing -----------------------

function worldToScreen(x_cm, y_cm) {
  const rect = canvas.getBoundingClientRect();
  const w = rect.width,
    h = rect.height;

  const sx = (x_cm - cam.cx) * cam.scale + w * 0.5;
  const sy = (y_cm - cam.cy) * cam.scale + h * 0.5;
  return { x: sx, y: sy };
}

function roundRect(ctx, x, y, w, h, r) {
  const rr = Math.max(0, Math.min(r, Math.min(w, h) / 2));
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
}

function drawGrid() {
  const rect = canvas.getBoundingClientRect();
  const w = rect.width,
    h = rect.height;

  // grid step in cm, choose a "nice" step depending on zoom
  const cmPerPx = 1 / cam.scale;
  const targetPx = 60;
  let stepCm = targetPx * cmPerPx;
  // snap to 1-2-5 * 10^n
  const pow = Math.pow(10, Math.floor(Math.log10(stepCm)));
  const m = stepCm / pow;
  if (m < 1.5) stepCm = 1 * pow;
  else if (m < 3.5) stepCm = 2 * pow;
  else if (m < 7.5) stepCm = 5 * pow;
  else stepCm = 10 * pow;

  const leftWorld = cam.cx - (w * 0.5) / cam.scale;
  const rightWorld = cam.cx + (w * 0.5) / cam.scale;
  const topWorld = cam.cy - (h * 0.5) / cam.scale;
  const botWorld = cam.cy + (h * 0.5) / cam.scale;

  const startX = Math.floor(leftWorld / stepCm) * stepCm;
  const startY = Math.floor(topWorld / stepCm) * stepCm;

  ctx.save();
  ctx.lineWidth = 1;

  // minor grid
  ctx.strokeStyle = "rgba(255,255,255,0.06)";
  for (let x = startX; x <= rightWorld; x += stepCm) {
    const p1 = worldToScreen(x, topWorld);
    const p2 = worldToScreen(x, botWorld);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }
  for (let y = startY; y <= botWorld; y += stepCm) {
    const p1 = worldToScreen(leftWorld, y);
    const p2 = worldToScreen(rightWorld, y);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }

  // axes
  ctx.strokeStyle = "rgba(255,255,255,0.18)";
  const ax1 = worldToScreen(0, topWorld);
  const ax2 = worldToScreen(0, botWorld);
  ctx.beginPath();
  ctx.moveTo(ax1.x, ax1.y);
  ctx.lineTo(ax2.x, ax2.y);
  ctx.stroke();

  const ay1 = worldToScreen(leftWorld, 0);
  const ay2 = worldToScreen(rightWorld, 0);
  ctx.beginPath();
  ctx.moveTo(ay1.x, ay1.y);
  ctx.lineTo(ay2.x, ay2.y);
  ctx.stroke();

  ctx.restore();
}

function tqToLineWidthPx(tq) {
  // thickness scales automatically with maxAbsTq in the file
  const a = Math.abs(Number.isFinite(tq) ? tq : 0);
  const denom = Math.max(1e-6, Number.isFinite(maxAbsTq) ? maxAbsTq : 1000);
  const n = Math.min(1, a / denom);
  // tune these if you want thicker/thinner traces
  return 1.0 + n * 7.0;
}

function drawWheelTraces(uptoIndex) {
  if (uptoIndex < 1) return;

  const wheels = [
    { key: "fl", style: "rgba(255, 92, 92, 0.85)" },
    { key: "fr", style: "rgba(92, 190, 255, 0.85)" },
    { key: "rl", style: "rgba(120, 255, 160, 0.85)" },
    { key: "rr", style: "rgba(255, 210, 92, 0.85)" },
  ];

  for (const w of wheels) {
    const arr = wheelPaths[w.key];
    if (!arr || arr.length < 2) continue;
    const maxI = Math.min(uptoIndex, arr.length - 1);

    ctx.save();
    ctx.strokeStyle = w.style;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    // per-segment drawing to vary thickness
    for (let i = 0; i < maxI; i++) {
      const a = arr[i];
      const b = arr[i + 1];
      const p1 = worldToScreen(a.x_cm, a.y_cm);
      const p2 = worldToScreen(b.x_cm, b.y_cm);
      ctx.lineWidth = tqToLineWidthPx(a.tq);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }

    ctx.restore();
  }
}

function drawSteeringIndicator(pose, steerVal) {
  const s = Number.isFinite(steerVal) ? steerVal : 0;
  if (Math.abs(s) < 1) return;

  // Draw a short line on the *imaginary middle trace* (vehicle center)
  // whose ORIENTATION indicates steering input.
  // Tune these:
  const LINE_LEN_CM = 10; // fixed length
  const MAX_STEER_ANGLE_DEG = 25; // max rotation at |s|=1000

  const n = Math.max(-1, Math.min(1, s / 1000));
  const steerAngle = (n * MAX_STEER_ANGLE_DEG * Math.PI) / 180;

  // base: perpendicular to heading; rotate by steerAngle
  const ang = pose.theta_rad + Math.PI / 2 + steerAngle;
  const dx = Math.cos(ang) * LINE_LEN_CM;
  const dy = Math.sin(ang) * LINE_LEN_CM;

  const p1 = worldToScreen(pose.x_cm, pose.y_cm);
  const p2 = worldToScreen(pose.x_cm + dx, pose.y_cm + dy);

  ctx.save();
  ctx.strokeStyle = "rgba(255,180,90,0.95)";
  ctx.lineWidth = 3;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(p1.x, p1.y);
  ctx.lineTo(p2.x, p2.y);
  ctx.stroke();
  ctx.restore();
}

function drawCar(pose, sample) {
  const { track_cm, wheelbase_cm } = getConstants();

  // car body dimensions (top-down box) in cm
  const bodyL = wheelbase_cm * 1.15;
  const bodyW = track_cm * 1.35;

  // wheel box
  const wheelL = bodyL * 0.22;
  const wheelW = bodyW * 0.18;

  const x = pose.x_cm,
    y = pose.y_cm,
    th = pose.theta_rad;

  // corners in body frame
  const halfL = bodyL / 2,
    halfW = bodyW / 2;

  function rot(dx, dy) {
    const c = Math.cos(th),
      s = Math.sin(th);
    return { x: x + dx * c - dy * s, y: y + dx * s + dy * c };
  }

  const corners = [
    rot(+halfL, +halfW),
    rot(+halfL, -halfW),
    rot(-halfL, -halfW),
    rot(-halfL, +halfW),
  ];

  // body
  ctx.save();
  ctx.fillStyle = "rgba(255,255,255,0.10)";
  ctx.strokeStyle = "rgba(255,255,255,0.65)";
  ctx.lineWidth = 2;

  ctx.beginPath();
  corners.forEach((c, i) => {
    const p = worldToScreen(c.x, c.y);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  // heading indicator (front center)
  const front = rot(+halfL, 0);
  const center = worldToScreen(x, y);
  const frontS = worldToScreen(front.x, front.y);
  ctx.strokeStyle = "rgba(255,180,90,0.9)";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(center.x, center.y);
  ctx.lineTo(frontS.x, frontS.y);
  ctx.stroke();

  // wheels: positions in body frame (approx)
  const ax = wheelbase_cm / 2;
  const wy = track_cm / 2;

  const wheelCenters = [
    { name: "FL", dx: +ax, dy: +wy },
    { name: "FR", dx: +ax, dy: -wy },
    { name: "RL", dx: -ax, dy: +wy },
    { name: "RR", dx: -ax, dy: -wy },
  ];

  // --- Power / telemetry overlay ---
  const smp = sample || {};
  const vFront = (Number.isFinite(smp.vf) ? smp.vf : 0) * VOLT_SCALE;
  const vRear  = (Number.isFinite(smp.vr) ? smp.vr : 0) * VOLT_SCALE;
  const tFront = (Number.isFinite(smp.tf) ? smp.tf : 0) * TEMP_SCALE;
  const tRear  = (Number.isFinite(smp.tr) ? smp.tr : 0) * TEMP_SCALE;
  const a_fl = (Number.isFinite(smp.cu_fl) ? smp.cu_fl : 0) / CURRENT_UNITS_PER_AMP;
  const a_fr = (Number.isFinite(smp.cu_fr) ? smp.cu_fr : 0) / CURRENT_UNITS_PER_AMP;
  const a_rl = (Number.isFinite(smp.cu_rl) ? smp.cu_rl : 0) / CURRENT_UNITS_PER_AMP;
  const a_rr = (Number.isFinite(smp.cu_rr) ? smp.cu_rr : 0) / CURRENT_UNITS_PER_AMP;
  const w_fl = a_fl * vFront;
  const w_fr = a_fr * vFront;
  const w_rl = a_rl * vRear;
  const w_rr = a_rr * vRear;
  const w_sum = w_fl + w_fr + w_rl + w_rr;
  const wattsByName = { FL: w_fl, FR: w_fr, RL: w_rl, RR: w_rr };

  ctx.fillStyle = "rgba(0,0,0,0.55)";
  ctx.strokeStyle = "rgba(255,255,255,0.35)";
  ctx.lineWidth = 1;

  for (const w of wheelCenters) {
    // wheel rectangle corners in world
    const wl = wheelL / 2,
      ww = wheelW / 2;
    const c0 = rot(w.dx + wl, w.dy + ww);
    const c1 = rot(w.dx + wl, w.dy - ww);
    const c2 = rot(w.dx - wl, w.dy - ww);
    const c3 = rot(w.dx - wl, w.dy + ww);

    ctx.beginPath();
    [c0, c1, c2, c3].forEach((c, i) => {
      const p = worldToScreen(c.x, c.y);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // wattage label
    const wc = rot(w.dx, w.dy);
    const ws = worldToScreen(wc.x, wc.y);
    const wval = wattsByName[w.name] || 0;
    ctx.save();
    ctx.fillStyle = "rgba(255,255,255,0.92)";
    ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(`${Math.round(wval)}W`, ws.x, ws.y);
    ctx.restore();
  }

  // center telemetry box
  {
    const c = worldToScreen(x, y);
    const lines = [
      `Σ ${Math.round(w_sum)}W`,
      `Vf ${vFront.toFixed(2)}V  Vr ${vRear.toFixed(2)}V`,
      `Tf ${tFront.toFixed(1)}°C Tr ${tRear.toFixed(1)}°C`,
    ];
    ctx.save();
    ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const padX = 10, padY = 8, lineH = 14;
    const wMax = Math.max(...lines.map(l => ctx.measureText(l).width));
    const boxW = wMax + padX * 2;
    const boxH = lines.length * lineH + padY * 2;
    ctx.fillStyle = "rgba(10,14,18,0.70)";
    ctx.strokeStyle = "rgba(255,255,255,0.20)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    roundRect(ctx, c.x - boxW/2, c.y - boxH/2, boxW, boxH, 10);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.92)";
    for (let i=0;i<lines.length;i++) {
      ctx.fillText(lines[i], c.x, c.y - boxH/2 + padY + lineH/2 + i*lineH);
    }
    ctx.restore();
  }

  // car center marker
  ctx.fillStyle = "rgba(255,255,255,0.9)";
  ctx.beginPath();
  ctx.arc(center.x, center.y, 3, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

function draw() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);

  drawGrid();

  // show wheel traces up to idx
  drawWheelTraces(idx);

  // draw steering indicator (perpendicular line)
  const p = poses[idx] || { x_cm: 0, y_cm: 0, theta_rad: 0 };
  const steerVal = samples[idx] ? samples[idx].s : 0;
  drawSteeringIndicator(p, steerVal);

  // draw car at idx
  drawCar(p, samples[idx] || null);

  // total distance bottom-left
  {
    const rect = canvas.getBoundingClientRect();
    const dcm = cumDistCm[idx] || 0;
    const dm = dcm / 100.0;
    ctx.save();
    ctx.font = "13px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";
    ctx.fillStyle = "rgba(255,255,255,0.92)";
    ctx.textAlign = "left";
    ctx.textBaseline = "alphabetic";
    ctx.fillText(`Distance: ${dm.toFixed(2)} m`, 12, rect.height - 14);
    ctx.restore();
  }

  // HUD
  const deg = (p.theta_rad * 180) / Math.PI;
  poseEl.textContent = `x=${p.x_cm.toFixed(1)}cm y=${p.y_cm.toFixed(1)}cm θ=${deg.toFixed(1)}°`;

  // distance
  if (distEl) {
    const cm = cumDistCm[idx] || 0;
    const m = cm / 100;
    distEl.textContent = `${m.toFixed(2)} m`;
  }

  if (samples.length >= 2 && idx < samples.length - 1) {
    const dt = samples[idx + 1].ts_ms - samples[idx].ts_ms;
    stepEl.textContent = `dt=${dt}ms`;
  } else {
    stepEl.textContent = `dt=—`;
  }
}

// ----------------------- Playback -----------------------

function setUIEnabled(ready) {
  btnPlay.disabled = !ready;
  btnReset.disabled = !ready;
  seek.disabled = !ready;
}

function setPlaying(on) {
  playing = on;
  btnPlay.textContent = playing ? "Pause" : "Play";
}

function reset() {
  setPlaying(false);
  idx = 0;
  seek.value = "0";
  seekLabel.textContent = `0 / ${Math.max(0, poses.length - 1)}`;
  accumMs = 0;
  lastRAF = 0;
  updateCameraTargets(idx);
  draw();
}

function stepForward() {
  if (idx >= poses.length - 1) {
    setPlaying(false);
    return;
  }
  idx++;
  seek.value = String(idx);
  seekLabel.textContent = `${idx} / ${Math.max(0, poses.length - 1)}`;
}

function tickRAF(ts) {
  if (!poses.length) return;

  if (!lastRAF) lastRAF = ts;
  const dt = ts - lastRAF;
  lastRAF = ts;

  // camera easing (smooth zoom/pan)
  const ease = 0.08;
  cam.scale += (cam.targetScale - cam.scale) * ease;
  cam.cx += (cam.targetCx - cam.cx) * ease;
  cam.cy += (cam.targetCy - cam.cy) * ease;

  if (playing) {
    const speed = Math.max(0.01, Number(speedInput.value) || 1.0);

    accumMs += dt * speed;

    // advance based on actual CSV timestamps, not assumed 100ms
    while (idx < samples.length - 1) {
      const stepMs = samples[idx + 1].ts_ms - samples[idx].ts_ms;
      const ms = Number.isFinite(stepMs) && stepMs > 0 ? stepMs : 100;
      if (accumMs >= ms) {
        accumMs -= ms;
        stepForward();
      } else break;
    }

    updateCameraTargets(idx);
  } else {
    // still keep camera targets updated when scrubbing
    updateCameraTargets(idx);
  }

  draw();
  requestAnimationFrame(tickRAF);
}

// ----------------------- Events -----------------------

fileInput.addEventListener("change", async () => {
  const f = fileInput.files && fileInput.files[0];
  if (!f) return;

  statusEl.textContent = "Reading file…";

  try {
    const text = await f.text();
    samples = parseCSV(text);

    if (samples.length < 2) throw new Error("Not enough samples.");

    // Ensure chronological
    samples.sort((a, b) => a.ts_ms - b.ts_ms);

    // auto-detect torque range for trace thickness
    maxAbsTq = 0;
    for (const row of samples) {
      const vals = [row.tq_fl, row.tq_fr, row.tq_rl, row.tq_rr];
      for (const v of vals) {
        const a = Math.abs(Number.isFinite(v) ? v : 0);
        if (a > maxAbsTq) maxAbsTq = a;
      }
    }
    if (!Number.isFinite(maxAbsTq) || maxAbsTq <= 0) maxAbsTq = 1000;

    recomputeAll();
    setUIEnabled(true);
    setPlaying(false);

    // initialize camera
    cam.cx = 0;
    cam.cy = 0;
    cam.scale = 2.0;
    updateCameraTargets(0);

    statusEl.textContent = `Loaded ${samples.length} samples.`;
    draw();

    // start loop once
    if (!lastRAF) requestAnimationFrame(tickRAF);
  } catch (e) {
    console.error(e);
    statusEl.textContent = `Error: ${e.message || e}`;
    samples = [];
    poses = [];
    path = [];
    setUIEnabled(false);
    draw();
  }
});

btnPlay.addEventListener("click", () => {
  if (!poses.length) return;
  if (idx >= poses.length - 1) reset();
  setPlaying(!playing);
});

btnReset.addEventListener("click", reset);

seek.addEventListener("input", () => {
  idx = Number(seek.value) || 0;
  seekLabel.textContent = `${idx} / ${Math.max(0, poses.length - 1)}`;
  setPlaying(false);
  updateCameraTargets(idx);
  draw();
});

// Recompute when constants change (tuning)
for (const inp of [trackInput, wheelbaseInput, circInput, pprInput]) {
  inp.addEventListener("change", () => {
    if (!samples.length) return;
    const savedIdx = idx;
    recomputeAll();
    idx = clamp(savedIdx, 0, poses.length - 1);
    seek.value = String(idx);
    seekLabel.textContent = `${idx} / ${Math.max(0, poses.length - 1)}`;
    updateCameraTargets(idx);
    draw();
  });
}

// kick initial draw
draw();
