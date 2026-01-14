// algorithm.js
const Gear = Object.freeze({ N: 0, D: 1, R: 2 });

function clamp(x, lo, hi) {
  x = Number(x);
  if (!Number.isFinite(x)) return lo;
  return Math.max(lo, Math.min(hi, x));
}
function Torques(fl = 0, fr = 0, rl = 0, rr = 0) {
  return { fl: fl | 0, fr: fr | 0, rl: rl | 0, rr: rr | 0 };
}

// ---------- helpers ----------
const clampF = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const lerp = (a, b, u) => a + (b - a) * u;
const absf = (x) => (x >= 0 ? x : -x);
const signOf = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);

window.timeScaling = 5;

const params = {
  TV: {
    MaxTorquePerTick: 0.5,
    SteerTorqueLowFactor: 1.0,
    SteerTorqueHighFactor: 0.7,
    SteerTorqueHighSpeed: 30.0,
    RearFadeSpeed0: 15.0,
    RearFadeSpeed1: 30.0,
    RearFadeThrottle0: 0.05,
    RearFadeThrottle1: 0.3,
    SteerTorqueFront: 220.0,
    SteerTorqueRear: 260.0,
    SlipRatio: 0.2,
    SlipDownFactor: 0.5,
    SlipMinScale: 0.25,
    SlipRecoverPerTick: 0.2,
    SlipSpeedEps: 20.0,
    SlipTorqueEps: 0.1,
    DriveFrontShareLow: 0.55,
    DriveFrontShareHigh: 0.3,
    BrakeFrontShareLow: 0.6,
    BrakeFrontShareHigh: 0.8,
    BiasHighThrottle: 0.6,
    maxDrivePower: 400,
    maxBrakePower: 600,
    maxSpeedForward: 400,
    maxSpeedReverse: 60,
    SpeedLimiterFadeBand: 30.0,
    AntiReversingSpeed: 100.0,
  },
};

// currGear: 0=N,1=D,2=R
// lastFront/lastRear: { receivedTorqueL, receivedTorqueR, rpmL, rpmR }
// Helpers: clamp(x,lo,hi), Torques(fl,fr,rl,rr), Gear

// JS port of your C++ TorqueVectoring() with the requested signature:
// TorqueVectoring(currGear, t, s, lastFront, lastRear)
// - currGear: Gear.N / Gear.D / Gear.R (0/1/2 per your comment)
// - t: throttle in [-1000..1000]
// - s: steering in [-1000..1000]
// - lastFront/lastRear: { receivedTorqueL, receivedTorqueR, rpmL, rpmR }  (we'll treat rpm* as wheel speed meas)
// Helpers assumed to exist: clamp(x,lo,hi), Torques(fl,fr,rl,rr), Gear

function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  const state =
    TorqueVectoring._state ||
    (TorqueVectoring._state = {
      vehicleSpeed: 0,

      // persistent statics from C++
      lastTdRear: 0,
      lastTdFront: 0,
      slipScale_fl: 1,
      slipScale_fr: 1,
      slipScale_rl: 1,
      slipScale_rr: 1,
    });

  // ---------- early outs ----------
  if (currGear === Gear.N) return Torques(0, 0, 0, 0);
  const allowRearYawAssist = currGear === Gear.D;

  // C++ used maxT = drivePower if throttle>0 else brakePower
  const maxT = t > 0 ? params.TV.maxDrivePower : params.TV.maxBrakePower;

  const throttleInput = (t * maxT) / 1000.0; // [-maxT..+maxT]

  let sn = s / 1000.0; // [-1..1]
  if (sn > 1) sn = 1;
  if (sn < -1) sn = -1;
  const absS = absf(sn);

  // D: forward => + command, R: forward => - command
  const gearSign = currGear === Gear.D ? +1.0 : -1.0;

  // "CURRENT speed only (do not use last*)": in your JS signature we only have lastFront/lastRear.
  // We'll interpret rpmL/rpmR as speed meas. If you later add current frames, swap this mapping.
  const getSpeed = (axleObj, left) => {
    if (!axleObj) return null;
    const v = left ? axleObj.rpmL : axleObj.rpmR; // replace if you have real speed fields
    if (v === undefined || v === null) return null;
    return v | 0; // int-ish
  };

  const sp_fl = getSpeed(lastFront, true);
  const sp_fr = getSpeed(lastFront, false);
  const sp_rl = getSpeed(lastRear, true);
  const sp_rr = getSpeed(lastRear, false);

  const ok = [sp_fl != null, sp_fr != null, sp_rl != null, sp_rr != null];
  const v = [
    ok[0] ? absf(sp_fl) : 0,
    ok[1] ? absf(sp_fr) : 0,
    ok[2] ? absf(sp_rl) : 0,
    ok[3] ? absf(sp_rr) : 0,
  ];

  const median_of = (arr) => {
    // arr length 1..4
    const a = arr.slice().sort((x, y) => x - y);
    const n = a.length;
    if (n === 1) return a[0];
    if (n === 2) return 0.5 * (a[0] + a[1]);
    if (n === 3) return a[1];
    // n===4 -> middle average (not used in C++ but fine)
    return 0.5 * (a[1] + a[2]);
  };

  const median_wheelspeeds = () => {
    const tmp = [];
    for (let i = 0; i < 4; i++) if (ok[i]) tmp.push(v[i]);
    return tmp.length ? median_of(tmp) : 0.0;
  };

  // ---------- vehicle speed (median) + speed limiter fade ----------
  const vehicleSpeed = median_wheelspeeds();

  const allowedMaxSpeed =
    currGear === Gear.D ? params.TV.maxSpeedForward : params.TV.maxSpeedReverse;
  const start = allowedMaxSpeed - params.TV.SpeedLimiterFadeBand;

  let spdFade = (vehicleSpeed - start) / params.TV.SpeedLimiterFadeBand; // [0..1]
  spdFade = clampF(spdFade, 0.0, 1.0);

  const throttle =
    throttleInput > 0 && vehicleSpeed >= start
      ? throttleInput * (1.0 - spdFade)
      : throttleInput;

  // ---------- braking anti-reversing fade ----------
  const vFadeDen = params.TV.AntiReversingSpeed;

  const brakeScale = (sp) => {
    if (sp == null || vFadeDen <= 1.0) return 1.0; // if speed missing: keep braking
    let a = absf(sp) / vFadeDen;
    a = clampF(a, 0.0, 1.0);
    return a;
  };

  const baseWheelCmd = (sp) => {
    if (throttle >= 0) {
      return gearSign * throttle;
    }
    const bmag = -throttle; // positive magnitude
    const motionSign = sp != null ? signOf(sp) : gearSign; // fallback: expected motion
    return -motionSign * bmag * brakeScale(sp);
  };

  const clampToI16 = (x) => {
    const y = clampF(x, -maxT, +maxT);
    // pack to int16 range if you want strict int16; otherwise plain int is fine in JS
    return y < 0 ? Math.ceil(y) : Math.floor(y);
  };

  const allocate_pair = (Tc, Td) => {
    // 1) differential cannot exceed max
    Td = clampF(Td, -maxT, +maxT);

    // 2) common must leave headroom for differential
    const head = absf(Td);
    const TcMax = +maxT - head;
    const TcMin = -maxT + head;
    Tc = clampF(Tc, TcMin, TcMax);

    return { l: Tc + Td, r: Tc - Td };
  };

  const rateLimit = (desired, lastKey, maxDeltaPerTick) => {
    const last = state[lastKey];
    const delta = desired - last;
    let limited = desired;
    if (delta > maxDeltaPerTick) limited = last + maxDeltaPerTick;
    if (delta < -maxDeltaPerTick) limited = last - maxDeltaPerTick;
    state[lastKey] = limited;
    return limited;
  };

  // =========================
  // (2) Front/Rear bias for longitudinal torque
  // =========================
  const Tc_front_raw = 0.5 * (baseWheelCmd(sp_fl) + baseWheelCmd(sp_fr));
  const Tc_rear_raw = 0.5 * (baseWheelCmd(sp_rl) + baseWheelCmd(sp_rr));
  const Tc_total = Tc_front_raw + Tc_rear_raw;

  const TcAbs = absf(Tc_total);
  let a = TcAbs / (params.TV.BiasHighThrottle * maxT);
  a = clampF(a, 0.0, 1.0);

  let frontShare = 0.5;
  if (Tc_total >= 0) {
    frontShare = lerp(
      params.TV.DriveFrontShareLow,
      params.TV.DriveFrontShareHigh,
      a,
    );
  } else {
    frontShare = lerp(
      params.TV.BrakeFrontShareLow,
      params.TV.BrakeFrontShareHigh,
      a,
    );
  }
  frontShare = clampF(frontShare, 0.05, 0.95);

  let Tc_front = Tc_total * frontShare;
  let Tc_rear = Tc_total * (1.0 - frontShare);

  // =========================
  // Rear axle: yaw assist
  // =========================
  let vRear = 0.0;
  if (sp_rl != null && sp_rr != null) vRear = 0.5 * (absf(sp_rl) + absf(sp_rr));
  else if (sp_rl != null) vRear = absf(sp_rl);
  else if (sp_rr != null) vRear = absf(sp_rr);
  else vRear = 0.0;

  const v0 = params.TV.RearFadeSpeed0;
  const v1 = params.TV.RearFadeSpeed1;
  let us = v1 > v0 ? (vRear - v0) / (v1 - v0) : 1.0;
  us = clampF(us, 0.0, 1.0);

  const rearEffAbs = absf(Tc_rear);
  const t0 = params.TV.RearFadeThrottle0 * maxT;
  const t1 = params.TV.RearFadeThrottle1 * maxT;
  let uLong = t1 > t0 ? (rearEffAbs - t0) / (t1 - t0) : 1.0;
  uLong = clampF(uLong, 0.0, 1.0);

  const s0 = 0.05; // hardcoded like C++
  const s1 = 0.25;
  let uSteer = (absS - s0) / (s1 - s0);
  uSteer = clampF(uSteer, 0.0, 1.0);

  const ut = uLong > uSteer ? uLong : uSteer;
  const fadeRear = us * ut;

  let Td_rear = 0.0;

  if (allowRearYawAssist && absS > 0.001) {
    const oppMag = absS * params.TV.SteerTorqueRear;

    const motionSign = (sp) => (sp != null ? signOf(sp) : gearSign);

    // Convention: rl = Tc + Td, rr = Tc - Td
    if (sn > 0) {
      // turning right => right is inner => make RR oppose motion
      Td_rear = motionSign(sp_rr) * oppMag;
    } else {
      // turning left => left is inner => make RL oppose motion
      Td_rear = -motionSign(sp_rl) * oppMag;
    }

    Td_rear *= fadeRear;

    const maxTdRearDeltaPerTick = params.TV.MaxTorquePerTick * maxT;
    Td_rear = rateLimit(Td_rear, "lastTdRear", maxTdRearDeltaPerTick);
  }

  const rp = allocate_pair(Tc_rear, Td_rear);
  let rl = rp.l;
  let rr = rp.r;

  // =========================
  // Front axle: steering actuator
  // =========================
  let vFront = 0.0;
  if (sp_fl != null && sp_fr != null)
    vFront = 0.5 * (absf(sp_fl) + absf(sp_fr));
  else if (sp_fl != null) vFront = absf(sp_fl);
  else if (sp_fr != null) vFront = absf(sp_fr);
  else vFront = 0.0;

  let u = vFront / params.TV.SteerTorqueHighSpeed;
  u = clampF(u, 0.0, 1.0);
  const k = lerp(
    params.TV.SteerTorqueLowFactor,
    params.TV.SteerTorqueHighFactor,
    u,
  );

  let Td_front = sn * params.TV.SteerTorqueFront * k;

  const maxTdFrontDeltaPerTick = params.TV.MaxTorquePerTick * maxT;
  Td_front = rateLimit(Td_front, "lastTdFront", maxTdFrontDeltaPerTick);

  const fp = allocate_pair(Tc_front, Td_front);
  let fl = fp.l;
  let fr = fp.r;

  // =========================
  // (1) ABS/ASR slip scaling
  // =========================
  const recover = (key) => {
    state[key] += params.TV.SlipRecoverPerTick;
    if (state[key] > 1.0) state[key] = 1.0;
  };

  recover("slipScale_fl");
  recover("slipScale_fr");
  recover("slipScale_rl");
  recover("slipScale_rr");

  const vRefExcluding = (excludeIdx) => {
    const tmp = [];
    for (let i = 0; i < 4; i++) {
      if (i === excludeIdx) continue;
      if (!ok[i]) continue;
      tmp.push(v[i]);
      if (tmp.length === 3) break;
    }
    return tmp.length ? median_of(tmp) : 0.0;
  };

  const applySlipLogic = (idx, wheelTorqueCmd, scaleKey) => {
    const vRefLocal = vRefExcluding(idx);
    if (vRefLocal < params.TV.SlipSpeedEps) return;

    const hi = vRefLocal * (1.0 + params.TV.SlipRatio);
    const lo = vRefLocal * (1.0 - params.TV.SlipRatio);

    const te = params.TV.SlipTorqueEps * maxT;

    let slip = false;
    if (wheelTorqueCmd > te) {
      // ASR
      if (v[idx] > hi) slip = true;
    } else if (wheelTorqueCmd < -te) {
      // ABS
      if (v[idx] < lo) slip = true;
    }

    if (slip) {
      state[scaleKey] *= params.TV.SlipDownFactor;
      if (state[scaleKey] < params.TV.SlipMinScale)
        state[scaleKey] = params.TV.SlipMinScale;
    }
  };

  applySlipLogic(0, fl, "slipScale_fl");
  applySlipLogic(1, fr, "slipScale_fr");
  applySlipLogic(2, rl, "slipScale_rl");
  applySlipLogic(3, rr, "slipScale_rr");

  fl *= state.slipScale_fl;
  fr *= state.slipScale_fr;
  rl *= state.slipScale_rl;
  rr *= state.slipScale_rr;

  // =========================
  // Final clamp + pack
  // =========================
  return Torques(
    clampToI16(fl),
    clampToI16(fr),
    clampToI16(rl),
    clampToI16(rr),
  );
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
