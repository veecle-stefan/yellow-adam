// app_v2.js
// WS UI for ESP32 vehicle controller.
// - cleaner axle layout
// - per-wheel spark chart (torque / current / rpm)
// - local 2D history storage (timestamp + raw fields)

document.body.style.userSelect = "none";

const wsStateEl   = document.getElementById("ws_state");
const modeStateEl = document.getElementById("mode_state");
const padEl       = document.getElementById("drivePad");

let padActive = false;
let padOriginX = 0;
let padOriginY = 0;
let originSet = false;
let lastT = 0;
let lastS = 0;

// ---------- Joystick on drive pad ----------
let manualActive = false;
let manualT = 0, manualS = 0;

// When user releases the pad we set UI to 0,0 immediately.
// Prevent a stale status frame from overwriting the 0,0 for a brief moment.
let manualReleaseUntilMs = 0;

// While external control is active, we periodically send steer (heartbeat).
let extCtrlSteerTicker = 0;

const allGears = ['N', 'D', 'R'];
let currGearIdx = 0;
// ---------- External control (runtime-only; no persistence) ----------
let extCtrlActive = false;

const btnExtCtrlTake = document.getElementById("btn_take_control");
const btnExtCtrlStop = document.getElementById("btn_stop_control");

function setExtCtrlUI(active){
  extCtrlActive = !!active;

  // Toggle button visibility
  if (btnExtCtrlTake) btnExtCtrlTake.style.display = extCtrlActive ? "none" : "inline-block";
  if (btnExtCtrlStop) btnExtCtrlStop.style.display = extCtrlActive ? "inline-block" : "none";

  // Optional: CSS hook
  document.body.classList.toggle("extctrl", extCtrlActive);
}

function startExtCtrlSteerTicker(){
  clearInterval(extCtrlSteerTicker);
  extCtrlSteerTicker = setInterval(() => {
    if(!extCtrlActive) return;
    // Send periodically even if values didn't change
    const t = manualActive ? manualT : 0;
    const s = manualActive ? manualS : 0;
    wsSend({ type:"cmd", name:"steer", t, s });
  }, 100);
}

function stopExtCtrlSteerTicker(){
  clearInterval(extCtrlSteerTicker);
  extCtrlSteerTicker = 0;
}

function setExtCtrl(active, { send = true } = {}){
  setExtCtrlUI(active);

  if(extCtrlActive) startExtCtrlSteerTicker();
  else stopExtCtrlSteerTicker();
  if(send){
    wsSend({ type:"cmd", name:"extctrl", on: extCtrlActive });
  }
  // Safety: when stopping control, also stop joystick + send steer=0 immediately.
  if(!extCtrlActive){
    if(manualActive) stopManual();
    else wsSend({ type:"cmd", name:"steer", t:0, s:0 });
  }
}

btnExtCtrlTake?.addEventListener("click", (e) => {
  e.preventDefault();
  e.stopPropagation();
  console.log('Enabling extrnal control');
  setExtCtrl(true);
});

btnExtCtrlStop?.addEventListener("click", (e) => {
  e.preventDefault();
  e.stopPropagation();
  console.log('Disabling extrnal control');
  setExtCtrl(false);
});

// Default: control OFF on page load
setExtCtrlUI(false);
stopExtCtrlSteerTicker();

// ---------- Limit profiles ----------
const profiles = [
  { name:"Kids",     maxThrottle:150,  maxSpeedF:150, maxSpeedR: 60 },
  { name:"Advanced", maxThrottle:400,  maxSpeedF:300, maxSpeedR: 80 },
  { name:"Mad Max",  maxThrottle:1000, maxSpeedF:500, maxSpeedR: 100 },
];

const limitProfileEl  = document.getElementById("limit_profile");
const limitThrottleEl = document.getElementById("limit_throttle");
const limitThrottleOut= document.getElementById("limit_throttle_out");
const limitSpeedFEl    = document.getElementById("limit_speed_f");
const limitSpeedFOut   = document.getElementById("limit_speed_f_out");
const limitSpeedREl    = document.getElementById("limit_speed_r");
const limitSpeedROut   = document.getElementById("limit_speed_r_out");
const btnSendLimits   = document.getElementById("btn_send_limits");

function updateLimitOutputs(){
  if(limitThrottleOut && limitThrottleEl) limitThrottleOut.textContent = String(Number(limitThrottleEl.value) | 0);
  if(limitSpeedFOut && limitSpeedFEl) limitSpeedFOut.textContent = String(Number(limitSpeedFEl.value) | 0);
  if(limitSpeedROut && limitSpeedREl) limitSpeedROut.textContent = String(Number(limitSpeedREl.value) | 0);
}

function applyProfile(p){
  if(!p) return;
  if(limitThrottleEl) limitThrottleEl.value = String(p.maxThrottle);
  if(limitSpeedFEl) limitSpeedFEl.value = String(p.maxSpeedF);
  if(limitSpeedREl) limitSpeedREl.value = String(p.maxSpeedR);
  updateLimitOutputs();
}

function initProfilesUI(){
  if(!limitProfileEl) return;
  // Add "Custom" first
  limitProfileEl.innerHTML = "";
  const opt0 = document.createElement("option");
  opt0.value = "__custom__";
  opt0.textContent = "Custom";
  limitProfileEl.appendChild(opt0);

  for(const p of profiles){
    const opt = document.createElement("option");
    opt.value = p.name;
    opt.textContent = p.name;
    limitProfileEl.appendChild(opt);
  }

  // Default profile: Kids
  limitProfileEl.value = profiles[0]?.name ?? "__custom__";
  applyProfile(profiles[0]);

  limitProfileEl.addEventListener("change", () => {
    const sel = limitProfileEl.value;
    const p = profiles.find(x => x.name === sel);
    if(p) applyProfile(p);
  });

  const onManualChange = () => {
    // When user drags sliders, treat as "Custom"
    if(limitProfileEl.value !== "__custom__"){
      limitProfileEl.value = "__custom__";
    }
    updateLimitOutputs();
  };

  limitThrottleEl?.addEventListener("input", onManualChange);
  limitSpeedFEl?.addEventListener("input", onManualChange);
  limitSpeedREl?.addEventListener("input", onManualChange);
  updateLimitOutputs();

  btnSendLimits?.addEventListener("click", () => {
    const mt = Math.max(50, Math.min(1000, Number(limitThrottleEl?.value ?? 0) || 0));
    const msf = Math.max(50, Math.min(500,  Number(limitSpeedFEl?.value ?? 0) || 0));
    const msr = Math.max(50, Math.min(200,  Number(limitSpeedREl?.value ?? 0) || 0));
    wsSend({ type:"cmd", name:"limit", maxThrottle: mt|0, maxSpeedFwd: msf|0, maxSpeedRev: msr|0 });
  });
}

initProfilesUI();

// ---------- TV Tuning (id-based, no key strings sent) ----------
// IMPORTANT: ID must match the table index in driveParams.cpp (kTVParams order).
// You said it's OK to hard-code this in JS to save memory on ESP32.
const tuningKeys = [
  { id: 0,  name: "MaxTorquePerTick",      min: 0.0, max: 1.0 },
  { id: 1,  name: "SteerTorqueLowFactor",  min: 0.0, max: 1.0 },
  { id: 2,  name: "SteerTorqueHighFactor", min: 0.0, max: 1.0 },
  { id: 3,  name: "SteerTorqueHighSpeed",  min: 0.0, max: 500.0 },

  { id: 4,  name: "RearFadeSpeed0",        min: 0.0, max: 500.0 },
  { id: 5,  name: "RearFadeSpeed1",        min: 0.0, max: 500.0 },
  { id: 6,  name: "RearFadeThrottle0",     min: 0.0, max: 1.0 },
  { id: 7,  name: "RearFadeThrottle1",     min: 0.0, max: 1.0 },

  { id: 8,  name: "SteerTorqueFront",      min: 0.0, max: 1000.0 },
  { id: 9,  name: "SteerTorqueRear",       min: 0.0, max: 1000.0 },

  { id: 10, name: "SlipRatio",             min: 0.0, max: 1.0 },
  { id: 11, name: "SlipDownFactor",        min: 0.0, max: 1.0 },
  { id: 12, name: "SlipMinScale",          min: 0.0, max: 1.0 },
  { id: 13, name: "SlipRecoverPerTick",    min: 0.0, max: 1.0 },
  { id: 14, name: "SlipSpeedEps",          min: 0.0, max: 500.0 },
  { id: 15, name: "SlipTorqueEps",         min: 0.0, max: 1.0 },

  { id: 16, name: "DriveFrontShareLow",    min: 0.0, max: 1.0 },
  { id: 17, name: "DriveFrontShareHigh",   min: 0.0, max: 1.0 },
  { id: 18, name: "BrakeFrontShareLow",    min: 0.0, max: 1.0 },
  { id: 19, name: "BrakeFrontShareHigh",   min: 0.0, max: 1.0 },
  { id: 20, name: "BiasHighThrottle",      min: 0.0, max: 5.0 },
];

const tuneKeyEl    = document.getElementById("tune_key");
const tuneValueEl  = document.getElementById("tune_value");
const tuneValueOut = document.getElementById("tune_value_out");
const tuneSliderEl = document.getElementById("tune_slider");
const tuneMinMaxEl = document.getElementById("tune_minmax");
const btnSendTune  = document.getElementById("btn_send_tune");

// Persist last values locally so the input is populated when switching params.
// Last (and initial) values shown/sent per param id.
// Initialized from the C++ TVParams defaults so the UI matches firmware at first load.
const tuningLast = {
  0:  0.5,   // MaxTorquePerTick
  1:  1.0,   // SteerTorqueLowFactor
  2:  0.7,   // SteerTorqueHighFactor
  3:  30.0,  // SteerTorqueHighSpeed

  4:  15.0,  // RearFadeSpeed0
  5:  30.0,  // RearFadeSpeed1
  6:  0.05,  // RearFadeThrottle0
  7:  0.3,   // RearFadeThrottle1

  8:  220.0, // SteerTorqueFront
  9:  260.0, // SteerTorqueRear

  10: 0.20,  // SlipRatio
  11: 0.70,  // SlipDownFactor
  12: 0.25,  // SlipMinScale
  13: 0.20,  // SlipRecoverPerTick
  14: 20.0,  // SlipSpeedEps
  15: 0.10,  // SlipTorqueEps

  16: 0.55,  // DriveFrontShareLow
  17: 0.30,  // DriveFrontShareHigh
  18: 0.60,  // BrakeFrontShareLow
  19: 0.80,  // BrakeFrontShareHigh
  20: 0.60,  // BiasHighThrottle
};

function fmt3(x){
  const n = Number(x);
  if(!Number.isFinite(n)) return "0";
  // Keep short but readable
  return (Math.round(n * 1000) / 1000).toString();
}

function clamp01(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

function setTuneUIForParam(p){
  if(!p) return;

  // Slider setup
  tuneSliderEl.min  = String(p.min);
  tuneSliderEl.max  = String(p.max);
  // Slider step: make it usable for both [0..1] params and large ranges.
  const range = Math.abs(p.max - p.min);
  let step = 0.05;                 // good default for 0..1 params
  if (range > 1.0) step = range / 100;  // ~1000 steps across range
  if (step < 0.001) step = 0.001;

  tuneSliderEl.step = String(step);
  tuneValueEl.step  = String(step);

  // Value: use last sent or mid of range
  const last = (tuningLast[p.id] != null) ? tuningLast[p.id] : p.min;
  const v = clamp01(last, p.min, p.max);

  tuneValueEl.value = fmt3(v);
  tuneSliderEl.value = String(v);

  tuneMinMaxEl.textContent = `${p.min} .. ${p.max}`;
  tuneValueOut.textContent = fmt3(v);
}

function currentTuningParam(){
  const id = Number(tuneKeyEl?.value ?? 0);
  return tuningKeys.find(x => x.id === id) || tuningKeys[0];
}

function initTuningUI(){
  if(!tuneKeyEl || !tuneValueEl || !tuneSliderEl || !btnSendTune) return;

  // Populate dropdown
  tuneKeyEl.innerHTML = "";
  for(const p of tuningKeys){
    const opt = document.createElement("option");
    opt.value = String(p.id);
    opt.textContent = `${p.name}`;
    tuneKeyEl.appendChild(opt);
  }

  // Default: first entry
  tuneKeyEl.value = String(tuningKeys[0].id);
  setTuneUIForParam(tuningKeys[0]);

  // When selecting another param, update UI and preserve last value for each id
  tuneKeyEl.addEventListener("change", () => {
    setTuneUIForParam(currentTuningParam());
  });

  // Keep number + slider in sync
  tuneSliderEl.addEventListener("input", () => {
    const p = currentTuningParam();
    const v = clamp01(Number(tuneSliderEl.value), p.min, p.max);
    tuneValueEl.value = fmt3(v);
    tuneValueOut.textContent = fmt3(v);
  });

  tuneValueEl.addEventListener("input", () => {
    const p = currentTuningParam();
    const v = clamp01(Number(tuneValueEl.value), p.min, p.max);
    tuneSliderEl.value = String(v);
    tuneValueOut.textContent = fmt3(v);
  });

  // Send tuning command
  btnSendTune.addEventListener("click", () => {
    const p = currentTuningParam();
    const v = clamp01(Number(tuneValueEl.value), p.min, p.max);

    tuningLast[p.id] = v;

    // Protocol: id + f16
    // (Your firmware consumes extCmd.p1.u16=id and extCmd.p1.f16=value)
    wsSend({ type:"cmd", name:"tune_tv", id: p.id, v });

    // keep UI consistent
    tuneValueEl.value = fmt3(v);
    tuneSliderEl.value = String(v);
    tuneValueOut.textContent = fmt3(v);
  });
}

initTuningUI();

// ---------- UI helpers ----------
function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

function setWsState(text) { wsStateEl.textContent = text; }

function setManualMode(on){
  document.body.classList.toggle("manual", !!on);
  modeStateEl.textContent = "mode: " + (on ? "manual" : "ws");
}

function setHBar(id, value){
  const el = document.querySelector(`.hbar[data-id="${id}"]`);
  if(!el) return;
  const min = Number(el.dataset.min), max = Number(el.dataset.max);
  const v = clamp(value, min, max);

  const posMax = max > 0 ? max : 1;
  const negMax = min < 0 ? Math.abs(min) : 1;
  const pos = v > 0 ? (v / posMax) : 0;
  const neg = v < 0 ? (Math.abs(v) / negMax) : 0;

  el.querySelector(".pos").style.width = (pos * 50).toFixed(1) + "%";
  el.querySelector(".neg").style.width = (neg * 50).toFixed(1) + "%";
}

function setNum(which, v){
  const el = document.getElementById("val_" + which);
  if(el) el.textContent = String(v | 0);
}

function setBoardStat(id, volts, tempC) {
  volts = Number(volts);
  tempC = Number(tempC);
  if (!Number.isFinite(volts) || !Number.isFinite(tempC)) return;

  const elV = document.getElementById("volt_" + id);
  const elT = document.getElementById("temp_" + id);
  if (elV) elV.textContent = (volts / 100).toFixed(1);
  if (elT) elT.textContent = (tempC / 10).toFixed(1);
}

function setTxt(id, text){
  const el = document.getElementById(id);
  if(el) el.textContent = text;
}

function setWheelReadouts(w, tqNm, currRaw, rpm){
  // tqNm is already Nm
  const nm = Number(tqNm);
  const cr = Number(currRaw);
  const rp = Number(rpm);

  setTxt(`tq_${w}`, (Number.isFinite(nm) ? (nm|0) : 0));
  setTxt(`cu_${w}`, (Number.isFinite(cr) ? (cr/100) : 0).toFixed(1) + " A");
  setTxt(`rpm_${w}`, (Number.isFinite(rp) ? (rp|0) : 0) + " rpm");
}

// ---------- 2D History storage ----------
// 1st dim: samples over time
// 2nd dim: fixed schema of raw fields
const HISTORY_SCHEMA = [
  "ts_ms",
  "t", "s",
  "tq_fl","tq_fr","tq_rl","tq_rr",
  "cu_fl","cu_fr","cu_rl","cu_rr",
  "rpm_fl","rpm_fr","rpm_rl","rpm_rr",
  "vf","tf","vr","tr",
  "gear","low","high","il","ir"
];

const history2d = [];
// expose for debugging
window.VEECLE_HISTORY = { schema: HISTORY_SCHEMA, data: history2d };

function recordHistorySample(ts, msg){
  const tq = msg.torque || {};
  const cu = msg.curr || {};
  const vel = msg.vel || {};
  const boards = msg.boards || {};
  const v = msg.vehicle || {};

  // Keep gear as a small int for compactness
  const gear = (v.gear === 'D') ? 1 : (v.gear === 'R' ? -1 : 0);

  const row = [
    ts,
    msg.t ?? 0, msg.s ?? 0,
    tq.fl ?? 0, tq.fr ?? 0, tq.rl ?? 0, tq.rr ?? 0,
    cu.fl ?? 0, cu.fr ?? 0, cu.rl ?? 0, cu.rr ?? 0,
    vel.fl ?? 0, vel.fr ?? 0, vel.rl ?? 0, vel.rr ?? 0,
    boards.vf ?? 0, boards.tf ?? 0, boards.vr ?? 0, boards.tr ?? 0,
    gear, v.low ? 1 : 0, v.high ? 1 : 0, v.il ? 1 : 0, v.ir ? 1 : 0
  ];

  history2d.push(row);
}

function downloadCsv(){
  // CSV of the 2D array. (Browser storage; no JSON.)
  const lines = [];
  lines.push(HISTORY_SCHEMA.join(","));
  for(const row of history2d){
    lines.push(row.join(","));
  }
  const blob = new Blob([lines.join("\n")], { type:"text/csv" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `veecle_history_${Date.now()}.csv`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

document.getElementById("btn_dump").onclick = () => downloadCsv();

// ---------- Charts ----------
class SparkChart {
  constructor(canvas, { maxPoints = 50 } = {}){
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.maxPoints = maxPoints;

    // store last N samples
    this.tq = [];
    this.cu = [];
    this.rpm = [];

    this._dirty = true;
    this._raf = 0;

    // Handle resize
    const ro = new ResizeObserver(() => this._resize());
    ro.observe(canvas);
    this._resize();
  }

  push(tq, cu, rpm){
    this.tq.push(Number(tq) || 0);
    this.cu.push(Number(cu) || 0);
    this.rpm.push(Number(rpm) || 0);
    if(this.tq.length > this.maxPoints){ this.tq.shift(); this.cu.shift(); this.rpm.shift(); }
    this._requestDraw();
  }

  _resize(){
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const rect = this.canvas.getBoundingClientRect();
    const w = Math.max(1, Math.floor(rect.width * dpr));
    const h = Math.max(1, Math.floor(rect.height * dpr));
    if(this.canvas.width !== w || this.canvas.height !== h){
      this.canvas.width = w;
      this.canvas.height = h;
      this._requestDraw();
    }
  }

  _requestDraw(){
    if(this._raf) return;
    this._raf = requestAnimationFrame(() => {
      this._raf = 0;
      this.draw();
    });
  }

  _minMax(arr){
    if(arr.length === 0) return {min:0, max:1};
    let min = arr[0], max = arr[0];
    for(let i=1;i<arr.length;i++){
      const v = arr[i];
      if(v < min) min = v;
      if(v > max) max = v;
    }
    if(min === max){
      // prevent flatline division by 0
      max = min + 1;
    }
    return {min, max};
  }

  _drawSeries(arr, color){
    const ctx = this.ctx;
    const n = arr.length;
    if(n < 2) return;
    const w = this.canvas.width;
    const h = this.canvas.height;

    const pad = Math.floor(h * 0.12);
    const top = pad;
    const bot = h - pad;
    const usableH = bot - top;

    const {min, max} = this._minMax(arr);
    const MIN_RANGE = 100;              // pick in your units (e.g. current mA/torque units)
    let lo = min, hi = max;
    if ((hi - lo) < MIN_RANGE) {
      const mid = 0.5 * (hi + lo);
      lo = mid - 0.5 * MIN_RANGE;
      hi = mid + 0.5 * MIN_RANGE;
    }

    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1, Math.floor(h * 0.010));
    ctx.beginPath();
    for(let i=0;i<n;i++){
      const x = (i / (n - 1)) * (w - 1);
      const yN = (arr[i] - lo) / (hi - lo);
      const y = bot - yN * usableH;
      if(i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  draw(){
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;

    ctx.clearRect(0, 0, w, h);

    // subtle grid
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.lineWidth = 1;
    const rows = 4;
    for(let r=1;r<rows;r++){
      const y = (r/rows) * h;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // series
    this._drawSeries(this.tq, "#60a5fa");
    this._drawSeries(this.cu, "#f59e0b");
    this._drawSeries(this.rpm, "#a78bfa");
  }
}

const charts = {
  fl: new SparkChart(document.getElementById("spark_fl")),
  fr: new SparkChart(document.getElementById("spark_fr")),
  rl: new SparkChart(document.getElementById("spark_rl")),
  rr: new SparkChart(document.getElementById("spark_rr")),
};

// ---------- Transport ----------
let transport = null;

class WebSocketTransport {
  constructor(url, { onState, onMessage }) {
    this.url = url;
    this.onState = onState;
    this.onMessage = onMessage;
    this.ws = null;
    this._reconnectTimer = 0;
  }
  connect() {
    this.onState?.("connecting");
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => this.onState?.("connected");
    this.ws.onclose = () => {
      this.onState?.("disconnected");
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = setTimeout(() => this.connect(), 800);
    };
    this.ws.onerror = () => { try { this.ws.close(); } catch {} };
    this.ws.onmessage = (ev) => this.onMessage?.(ev.data);
  }
  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }
  close() {
    clearTimeout(this._reconnectTimer);
    this._reconnectTimer = 0;
    try { this.ws?.close(); } catch {}
  }
}

class ReplayTransport {
  constructor(ndjsonText, { onState, onMessage, delayMs = 100, loop = true }) {
    this.lines = ndjsonText
      .split(/\r?\n/)
      .map(l => l.trim())
      .filter(l => l && !l.startsWith("#"));
    this.onState = onState;
    this.onMessage = onMessage;
    this.delayMs = delayMs;
    this.loop = loop;
    this._i = 0;
    this._timer = 0;
    this._running = false;
  }
  connect() {
    this.onState?.("connected");
    this._running = true;
    this._scheduleNext(0);
  }
  _scheduleNext(ms){
    clearTimeout(this._timer);
    this._timer = setTimeout(() => this._tick(), ms);
  }
  _tick(){
    if(!this._running) return;
    if(this.lines.length === 0){ this._scheduleNext(this.delayMs); return; }

    if(this._i >= this.lines.length){
      if(this.loop) this._i = 0;
      else { this.close(); return; }
    }

    const line = this.lines[this._i++];
    this.onMessage?.(line);
    this._scheduleNext(this.delayMs);
  }
  send(_obj){}
  close(){
    this._running = false;
    clearTimeout(this._timer);
    this._timer = 0;
    this.onState?.("disconnected");
  }
}

function wsSend(obj){ 
  transport?.send(obj);
  console.log(JSON.stringify(obj));
}

function isReplayMode() {
  return new URLSearchParams(location.search).has("replay");
}

async function setupTransport() {
  if (isReplayMode()) {
    setWsState("ws: replay (load file)");
    const input = document.getElementById("replay_file");
    const delayEl = document.getElementById("replay_delay");
    if (!input) { setWsState("ws: replay (missing file input)"); return; }

    input.addEventListener("change", async () => {
      const file = input.files && input.files[0];
      if (!file) return;
      const delayMs = Math.max(0, Number(delayEl?.value ?? 100) || 100);
      const text = await file.text();
      transport?.close();
      transport = new ReplayTransport(text, {
        delayMs,
        loop: true,
        onState: (s) => setWsState("ws: " + (s === "connected" ? "replay" : s)),
        onMessage: handleIncoming
      });
      transport.connect();
    });
    return;
  }

  const url = `ws://${location.host}/ws`;
  transport = new WebSocketTransport(url, {
    onState: (s) => {
      if (s === "connecting") setWsState("ws: connecting");
      else if (s === "connected") setWsState("ws: connected");
      else if (s === "disconnected") setWsState("ws: disconnected");
      else setWsState("ws: " + s);
    },
    onMessage: handleIncoming
  });
  transport.connect();
}

// ---------- Incoming WS ----------
function handleIncoming(evDataString) {
  let msg;
  try { msg = JSON.parse(evDataString); } catch { return; }
  if (!msg || msg.type !== "status") return;

  // Record raw sample
  recordHistorySample(Date.now(), msg);

  // Only update T/S from WS if not in manual override.
  // After releasing the pad we briefly keep 0,0 on screen to avoid stale frames.
  if(!manualActive && Date.now() > manualReleaseUntilMs){
    const t = msg.t ?? 0;
    const s = msg.s ?? 0;
    setNum("throttle", t); setHBar("throttle", t);
    setNum("steering", s); setHBar("steering", s);
  }

  const tq = msg.torque || {};
  const cu = msg.curr || {};
  const vel = msg.vel || {};

  // Readouts + chart per wheel
  for(const w of ["fl","fr","rl","rr"]){
    const tNm = tq[w] ?? 0;
    const cRaw = cu[w] ?? 0;
    const rpm  = vel[w] ?? 0;
    setWheelReadouts(w, tNm, cRaw, rpm);
    charts[w].push(tNm, cRaw, rpm);
  }

  const boards = msg.boards || {};
  setBoardStat("front", boards.vf, boards.tf);
  setBoardStat("rear",  boards.vr, boards.tr);

  // Lights + gear state for button highlight
  const L = msg.vehicle || {};
  setGear("btn_gear", L.gear);
  setBtn("btn_high",  !!L.high);
  setBtn("btn_ind_l", !!L.il);
  setBtn("btn_ind_r", !!L.ir);
}

// ---------- Buttons ----------
function setBtn(id, on){
  const b = document.getElementById(id);
  if(!b) return;
  b.classList.toggle("on", !!on);
}

function setGear(id, text){
  const b = document.getElementById(id);
  if(!b) return;
  b.textContent = text || "N";
}

function toggleGear(id){
  const b = document.getElementById(id);
  if(!b) return false;
  currGearIdx = (currGearIdx + 1) % allGears.length;
  const newGear = allGears[currGearIdx];
  b.textContent = newGear;
  return newGear;
}

function toggleBtn(id){
  const b = document.getElementById(id);
  if(!b) return false;
  b.classList.toggle("on");
  return b.classList.contains("on");
}

document.getElementById("btn_gear").onclick  = () => wsSend({type:"cmd", name:"gear", gear: toggleGear("btn_gear")});
document.getElementById("btn_high").onclick  = () => wsSend({type:"cmd", name:"high",  on: toggleBtn("btn_high")});
document.getElementById("btn_ind_l").onclick = () => wsSend({type:"cmd", name:"ind_l", on: toggleBtn("btn_ind_l")});
document.getElementById("btn_ind_r").onclick = () => wsSend({type:"cmd", name:"ind_r", on: toggleBtn("btn_ind_r")});


function computeTSFromPointer(clientX, clientY){
  const r = padEl.getBoundingClientRect();
  const cx = r.left + r.width / 2;
  const cy = r.top  + r.height / 2;
  const dx = clientX - cx;
  const dy = clientY - cy;
  const nx = clamp(dx / (r.width / 2),  -1, 1);
  const ny = clamp(dy / (r.height / 2), -1, 1);
  const s = Math.round(nx * 1000);
  const t = Math.round((-ny) * 1000);
  return { t, s };
}

function applyManual(t, s){
  manualT = clamp(t, -1000, 1000);
  manualS = clamp(s, -1000, 1000);
  setManualMode(true);
  setNum("throttle", manualT); setHBar("throttle", manualT);
  setNum("steering", manualS); setHBar("steering", manualS);
}

function startManual(){
  manualActive = true;
  setManualMode(true);
}
function stopManual(){
  manualActive = false;

  // Immediately show 0,0 on the UI (and keep it briefly to avoid stale WS overwrite)
  manualT = 0; manualS = 0;
  manualReleaseUntilMs = Date.now() + 200;
  setNum("throttle", 0); setHBar("throttle", 0);
  setNum("steering", 0); setHBar("steering", 0);

  setManualMode(false);
  wsSend({ type:"cmd", name:"steer", t:0, s:0 });
}
function computeTSFromDelta(dx, dy){
  const rect = padEl.getBoundingClientRect();
  const r = Math.max(40, Math.min(rect.width, rect.height) * 0.45);

  // dx: right = +steer, dy: down = + so invert for throttle
  let nx = dx / r;
  let ny = -dy / r;

  // clamp [-1..1]
  nx = Math.max(-1, Math.min(1, nx));
  ny = Math.max(-1, Math.min(1, ny));

  // deadband makes "near zero" easy
  const db = 0.06;
  if (Math.abs(nx) < db) nx = 0;
  if (Math.abs(ny) < db) ny = 0;

  // map to your integer domain [-1000..1000]
  const s = Math.round(nx * 1000);
  const t = Math.round(ny * 1000);

  return { t, s };
}

padEl.addEventListener("pointerdown", (e) => {
  if(!extCtrlActive) return;
  e.preventDefault();

  // IMPORTANT: capture the pointer so moves keep coming even if leaving the pad
  try { padEl.setPointerCapture(e.pointerId); } catch {}

  // Set origin, do NOT apply torque/steer yet (prevents dangerous jump)
  padOriginX = e.clientX;
  padOriginY = e.clientY;
  originSet = true;

  applyManual(0, 0);
  startManual();
});

window.addEventListener("pointermove", (e) => {
  if(!manualActive || !extCtrlActive) return;
  e.preventDefault();
  if(!originSet) return;

  const {t, s} = computeTSFromDelta(e.clientX - padOriginX, e.clientY - padOriginY);
  applyManual(t, s);
});

window.addEventListener("pointerup", (e) => {
  if(!manualActive) return;
  e.preventDefault();
  originSet = false;
  stopManual();
});

window.addEventListener("pointercancel", (e) => {
  if(!manualActive) return;
  e.preventDefault();
  originSet = false;
  stopManual();
});


// Poweroff (always visible)
document.getElementById("btn_poweroff")?.addEventListener("click", (e) => {
  e.preventDefault();
  e.stopPropagation();
  wsSend({ type:"cmd", name:"poweroff" });
});

setupTransport();
