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
// { "type":"control", "enable":true,  "t":123, "s":-45 }
// { "type":"control", "enable":false, "t":0,   "s":0 }
//
// Button commands from browser -> ESP:
// { "type":"cmd", "name":"drl",   "on":true }
// { "type":"cmd", "name":"ind_l", "on":false }

document.body.style.userSelect = "none"; // optional polish: no text selection anywhere

const wsStateEl   = document.getElementById("ws_state");
const modeStateEl = document.getElementById("mode_state");
const carEl       = document.getElementById("car");

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

// ---------- WebSocket ----------
let ws = null;

function wsSend(obj){
  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify(obj));
  }
}

function connectWs(){
  const url = `ws://${location.host}/ws`;
  wsStateEl.textContent = "ws: connecting";
  ws = new WebSocket(url);

  ws.onopen = () => {
    wsStateEl.textContent = "ws: connected";
  };

  ws.onclose = () => {
    wsStateEl.textContent = "ws: disconnected";
    setTimeout(connectWs, 800);
  };

  ws.onerror = () => {
    try { ws.close(); } catch {}
  };

  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if(!msg || msg.type !== "status") return;

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
    setBoardStat("rear", boards.vr, boards.tr);

    // Lights (optional)
    const L = msg.vehicle || {};
    setGear("btn_gear",   L.gear);
    setBtn("btn_low",   !!L.low);
    setBtn("btn_high",  !!L.high);
    setBtn("btn_ind_l", !!L.il);
    setBtn("btn_ind_r", !!L.ir);
  };
}

connectWs();

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

function toggleBtn(id){
  const b = document.getElementById(id);
  if(!b) return false;
  b.classList.toggle("on");
  return b.classList.contains("on");
}

// Send minimal commands (you can rename later)
document.getElementById("btn_low").onclick   = () => wsSend({type:"cmd", name:"low",   on: toggleBtn("btn_low")});
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

    wsSend({ type:"control", enable:true, t:manualT, s:manualS });
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
  wsSend({ type:"control", enable:false, t:0, s:0 });
}

function setWheelCurr(id, v){
  const el = document.getElementById("curr_" + id);
  if(el) el.textContent = (v / 100 | 0).toFixed(2) + ' A';
}

function setWheelSpeed(id, v){
  const el = document.getElementById("speed_" + id);
  if(el) el.textContent = String(v | 0) + ' rpm';
}


function setBoardStat(id, volts, tempC){
  const elV = document.getElementById("volt_" + id);
  const elT = document.getElementById("temp_" + id);
  if(elV) elV.textContent = String((volts / 100).toFixed(2) | 0) + ' V';
  if(elT) elT.textContent = String((tempC / 10).toFixed(1) | 0) + ' ºC';
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