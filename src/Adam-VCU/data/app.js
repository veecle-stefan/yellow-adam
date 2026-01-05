// data/app.js
// Minimal WS + UI + joystick (touch + mouse) for the toy car portal.
//
// Status from ESP -> browser (example):
// {
//   "type":"status",
//   "t":123, "s":-45,
//   "torque":{"fl":100,"fr":90,"rl":110,"rr":95},
//   "vehicle":{"gear":"R", "low":0,"high":0,"il":0,"ir":1}
// }
//
// Control from browser -> ESP:
// Use cmd messages:
// { "type":"cmd", "name":"extctrl", "on":true }
// { "type":"cmd", "name":"extctrl", "on":false }
// { "type":"cmd", "name":"steer", "t":123, "s":-45 }
// { "type":"cmd", "name":"steer", "t":0, "s":0 }
//
// Button commands from browser -> ESP:
// { "type":"cmd", "name":"drl",   "on":true }
// { "type":"cmd", "name":"ind_l", "on":false }

document.body.style.userSelect = "none"; // optional polish: no text selection anywhere

const wsStateEl   = document.getElementById("ws_state");
const modeStateEl = document.getElementById("mode_state");
const carEl       = document.getElementById("car");
const allGears = ['N', 'D', 'R'];
let currGearIdx = 0;

// ---------- UI helpers ----------
function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

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

function setVBar(id, value){
  const el = document.querySelector(`.vbar[data-id="${id}"]`);
  if(!el) return;

  const min = Number(el.dataset.min), max = Number(el.dataset.max);
  const v = clamp(value, min, max);

  const posMax = max > 0 ? max : 1;
  const negMax = min < 0 ? Math.abs(min) : 1;

  const pos = v > 0 ? (v / posMax) : 0;
  const neg = v < 0 ? (Math.abs(v) / negMax) : 0;

  el.querySelector(".pos").style.height = (pos * 50).toFixed(1) + "%";
  el.querySelector(".neg").style.height = (neg * 50).toFixed(1) + "%";
}

function setNum(which, v){
  const el = document.getElementById("val_" + which);
  if(el) el.textContent = String(v | 0);
}

function setManualMode(on){
  document.body.classList.toggle("manual", !!on);
  modeStateEl.textContent = "mode: " + (on ? "manual" : "ws");
}

// ---------- Transport (WS or Replay) ----------
let transport = null;

function setWsState(text) {
  wsStateEl.textContent = text;
}

function handleIncoming(evDataString) {
  let msg;
  try { msg = JSON.parse(evDataString); } catch { return; }
  if (!msg || msg.type !== "status") return;

  // Only update T/S from WS if not in manual override
  if(!manualActive){
    const t = msg.t ?? 0;
    const s = msg.s ?? 0;
    setNum("throttle", t); setHBar("throttle", t);
    setNum("steering", s); setHBar("steering", s);
  }

  // Wheel torques always from WS
  const tq = msg.torque || {};
  setVBar("torque_fl", tq.fl ?? 0);
  setVBar("torque_fr", tq.fr ?? 0);
  setVBar("torque_rl", tq.rl ?? 0);
  setVBar("torque_rr", tq.rr ?? 0);

  // Wheel currents (numbers next to wheels)
  const cu = msg.curr || {};
  setVBar("curr_fl", cu.fl ?? 0);
  setVBar("curr_fr", cu.fr ?? 0);
  setVBar("curr_rl", cu.rl ?? 0);
  setVBar("curr_rr", cu.rl ?? 0);

  // Wheel velocities
  const vel = msg.vel || {};
  setWheelSpeed("fl", vel.fl ?? 0);
  setWheelSpeed("fr", vel.fr ?? 0);
  setWheelSpeed("rl", vel.rl ?? 0);
  setWheelSpeed("rr", vel.rr ?? 0);

  const boards = msg.boards || {};
  setBoardStat("front", boards.vf, boards.tf);
  setBoardStat("rear",  boards.vr, boards.tr);

  // Lights (optional)
  const L = msg.vehicle || {};
  setGear("btn_gear",   L.gear);
  setBtn("btn_high",  !!L.high);
  setBtn("btn_ind_l", !!L.il);
  setBtn("btn_ind_r", !!L.ir);
}

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

    this.ws.onerror = () => {
      try { this.ws.close(); } catch {}
    };

    this.ws.onmessage = (ev) => {
      this.onMessage?.(ev.data);
    };
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

/**
 * ReplayTransport plays newline-delimited JSON (NDJSON).
 * - Each line must be one JSON object (your normal WS payload).
 * - Delay can be fixed (e.g., 100ms) or derived from a timestamp field.
 */
class ReplayTransport {
  constructor(ndjsonText, { onState, onMessage, delayMs = 100, loop = true }) {
    this.lines = ndjsonText
      .split(/\r?\n/)
      .map(l => l.trim())
      .filter(l => l && !l.startsWith("#")); // allow comments with '#'

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

  send(_obj) {
    // Optional: in replay mode you can ignore outgoing commands,
    // or log them for debugging:
    // console.log("Replay send:", _obj);
  }

  close() {
    this._running = false;
    clearTimeout(this._timer);
    this._timer = 0;
    this.onState?.("disconnected");
  }

  _scheduleNext(ms) {
    clearTimeout(this._timer);
    this._timer = setTimeout(() => this._tick(), ms);
  }

  _tick() {
    if (!this._running) return;

    if (this.lines.length === 0) {
      // nothing to play
      this._scheduleNext(500);
      return;
    }

    if (this._i >= this.lines.length) {
      if (!this.loop) {
        this.close();
        return;
      }
      this._i = 0;
    }

    const line = this.lines[this._i++];
    // onMessage expects a string like WebSocket's ev.data
    this.onMessage?.(line);

    this._scheduleNext(this.delayMs);
  }
}

// Global send helper (your existing code uses wsSend)
function wsSend(obj){
  transport?.send(obj);
}

// Decide mode: ?replay=1 uses replay mode (plus a file input to load the log)
function isReplayMode() {
  return new URLSearchParams(location.search).has("replay");
}

// Hook up replay file input from HTML (see index.html change below)
async function setupTransport() {
  if (isReplayMode()) {
    setWsState("ws: replay (load file)");
    // Wait for user to choose a file.
    const input = document.getElementById("replay_file");
    const delayEl = document.getElementById("replay_delay");

    if (!input) {
      setWsState("ws: replay (missing file input)");
      return;
    }

    input.addEventListener("change", async () => {
      const file = input.files && input.files[0];
      if (!file) return;

      const delayMs = Math.max(0, Number(delayEl?.value ?? 100) || 100);

      const text = await file.text();
      transport?.close();

      console.log(`Loading replay file and replaying at ${delayMs}ms`);

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

  // Normal WS mode (ESP32)
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

setupTransport();

// ---------- Buttons ----------
function setBtn(id, on){
  const b = document.getElementById(id);
  if(!b) return;
  b.classList.toggle("on", !!on);
}

function setGear(id, text){
  const b = document.getElementById(id);
  if(!b) return;
  b.textContent = text;
}

function toggleGear(id){
  const b = document.getElementById(id);
  if(!b) return false;
  currGearIdx = (currGearIdx + 1) % allGears.length;
  const newGear = allGears[currGearIdx];
  b.textContent = newGear;

  console.log(`Setting gear to ${newGear}`);

  return newGear;
}


function toggleBtn(id){
  const b = document.getElementById(id);
  if(!b) return false;
  b.classList.toggle("on");
  return b.classList.contains("on");
}

// Send minimal commands (you can rename later)
document.getElementById("btn_gear").onclick  = () => wsSend({type:"cmd", gear: toggleGear("btn_gear")});
document.getElementById("btn_high").onclick  = () => wsSend({type:"cmd", name:"high",  on: toggleBtn("btn_high")});
document.getElementById("btn_ind_l").onclick = () => wsSend({type:"cmd", name:"ind_l", on: toggleBtn("btn_ind_l")});
document.getElementById("btn_ind_r").onclick = () => wsSend({type:"cmd", name:"ind_r", on: toggleBtn("btn_ind_r")});

// ---------- Joystick on car (touch + mouse) ----------
let manualActive = false;
let manualT = 0, manualS = 0;

let sendTimer = 0;
let lastSentT = 99999, lastSentS = 99999;

function computeTSFromPointer(clientX, clientY){
  const r = carEl.getBoundingClientRect();
  const cx = r.left + r.width / 2;
  const cy = r.top  + r.height / 2;

  const dx = clientX - cx;
  const dy = clientY - cy;

  // Normalize to [-1..+1]
  const nx = clamp(dx / (r.width / 2),  -1, 1);
  const ny = clamp(dy / (r.height / 2), -1, 1);

  // Convention:
  // up = +throttle, right = +steering
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

  clearInterval(sendTimer);
  sendTimer = setInterval(() => {
    if(!manualActive) return;

    // send only on change (still at a fixed tick to keep it simple)
    if(manualT === lastSentT && manualS === lastSentS) return;

    lastSentT = manualT;
    lastSentS = manualS;

    wsSend({ type:"cmd", name:"steer", t:manualT, s:manualS });
  }, 40); // ~25 Hz
}

function stopManual(){
  manualActive = false;

  clearInterval(sendTimer);
  sendTimer = 0;

  setManualMode(false);

  lastSentT = 99999;
  lastSentS = 99999;

  // stop command
  wsSend({ type:"cmd", name:"steer", t:0, s:0 });
}

function setWheelCurr(id, v){
  const el = document.getElementById("curr_" + id);
  if(el) el.textContent = (v / 100 | 0).toFixed(2) + ' A';
}

function setWheelSpeed(id, v){
  const el = document.getElementById("speed_" + id);
  if(el) el.textContent = String(v | 0) + ' rpm';
}


function setBoardStat(id, volts, tempC) {
  volts = Number(volts);
  tempC = Number(tempC);

  if (!Number.isFinite(volts) || !Number.isFinite(tempC)) return;

  const elV = document.getElementById("volt_" + id);
  const elT = document.getElementById("temp_" + id);

  if (elV) elV.textContent = (volts / 100).toFixed(1) + " V";
  if (elT) elT.textContent = (tempC / 10).toFixed(1) + " ºC";
}

// Start manual mode when pressing on the car
carEl.addEventListener("pointerdown", (e) => {
  e.preventDefault();

  const {t, s} = computeTSFromPointer(e.clientX, e.clientY);
  applyManual(t, s);
  startManual();
});

// Robust: track movement globally while active (works across macOS browsers)
window.addEventListener("pointermove", (e) => {
  if(!manualActive) return;
  e.preventDefault();

  const {t, s} = computeTSFromPointer(e.clientX, e.clientY);
  applyManual(t, s);
});

window.addEventListener("pointerup", (e) => {
  if(!manualActive) return;
  e.preventDefault();
  stopManual();
});

window.addEventListener("pointercancel", (e) => {
  if(!manualActive) return;
  e.preventDefault();
  stopManual();
});