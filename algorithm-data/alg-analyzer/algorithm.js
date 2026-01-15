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

    // --- NEW (needed by the new C++ port) ---
    SlipRecoverTorquePerTick: 30, // VERIFY DEFAULT: C++ TV.SlipRecoverTorquePerTick (torque units per 20ms)
    SlipMinTorque: 50.0, // VERIFY DEFAULT: C++ TV.SlipMinTorque (minimum allowed |torque| envelope)
    maxRealisticAccel: 50.0, // VERIFY DEFAULT: C++ TV.maxRealisticAccel (max +ΔvRefAbs per tick)
    maxRealisticDecel: 80.0, // VERIFY DEFAULT: C++ TV.maxRealisticDecel (max -ΔvRefAbs per tick)

    SlipSpeedEps: 20.0,

    DriveFrontShareLow: 0.55,
    DriveFrontShareHigh: 0.3,
    BrakeFrontShareLow: 0.6,
    BrakeFrontShareHigh: 0.8,
    BiasHighThrottle: 0.6,

    // --- naming used by new C++ port (aliases to your existing fields) ---
    maxPowerDrive: 400, // VERIFY DEFAULT: C++ TV.maxPowerDrive (positive limit)
    maxPowerBrake: 600, // VERIFY DEFAULT: C++ TV.maxPowerBrake (brake magnitude)

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

// Drop-in replacement: TorqueVectoring(currGear, t, s, lastFront, lastRear)
// Assumptions (match your visualizer naming):
// - lastFront/lastRear: { receivedTorqueL, receivedTorqueR, rpmL, rpmR }
//   rpmL/rpmR  == signed speed*_meas
//   receivedTorqueL/R == last issued cmdL/cmdR (same sample as speed)
// - You already have: const params = {...}  (with params.TV.* fields from the C++ code)
// - You already have helpers: clamp(x,lo,hi), Torques(fl,fr,rl,rr), Gear

function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  // ---------------------------
  // Persistent "class" state
  // ---------------------------
  const st =
    TorqueVectoring._st ||
    (TorqueVectoring._st = {
      vehicleSpeedAbs: 0,

      capsInit: false,
      capPos: [0, 0, 0, 0],
      capNeg: [0, 0, 0, 0],

      lastWheelCmd: [0, 0, 0, 0], // optional final rate-limit memory
    });

  // ---------------------------
  // Constants / enums
  // ---------------------------
  const FL = 0,
    FR = 1,
    RL = 2,
    RR = 3;

  // ---------------------------
  // Small helpers
  // ---------------------------
  const absf = (x) => (x >= 0 ? x : -x);
  const signf = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);
  const clampF = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const lerp = (a, b, u) => a + (b - a) * u;

  const truncTowardZero = (x) => (x < 0 ? Math.ceil(x) : Math.floor(x));

  // sortSmallN + median as in C++
  const sortSmallN = (arr, n) => {
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (arr[j] < arr[i]) {
          const tmp = arr[i];
          arr[i] = arr[j];
          arr[j] = tmp;
        }
      }
    }
  };

  const medianSorted = (sorted, n) => {
    if (n <= 0) return 0;
    if (n === 1) return sorted[0];
    if (n === 2) return 0.5 * (sorted[0] + sorted[1]);
    if (n === 3) return sorted[1];
    return 0.5 * (sorted[1] + sorted[2]); // n=4
  };

  const axleSpeedAbs = (sense, L, R) => {
    const okL = sense.ok[L],
      okR = sense.ok[R];
    if (okL && okR) return 0.5 * (sense.wAbs[L] + sense.wAbs[R]);
    if (okL) return sense.wAbs[L];
    if (okR) return sense.wAbs[R];
    return 0;
  };

  const bestMotionWheel = (sense, vRefAbs) => {
    let best = -1;
    let bestErr = 1e9;
    for (let i = 0; i < 4; i++) {
      if (!sense.ok[i]) continue;
      const err = Math.abs(sense.wAbs[i] - vRefAbs);
      if (err < bestErr) {
        bestErr = err;
        best = i;
      }
    }
    return best;
  };

  // SolvePairCaps from C++ (priority: keep Td)
  const solvePairCaps = (TcReq, TdReq, capNegL, capPosL, capNegR, capPosR) => {
    const TdMin = 0.5 * (capNegL - capPosR);
    const TdMax = 0.5 * (capPosL - capNegR);
    const Td = clampF(TdReq, TdMin, TdMax);

    const TcMin = Math.max(capNegL - Td, capNegR + Td);
    const TcMax = Math.min(capPosL - Td, capPosR + Td);
    const Tc = clampF(TcReq, TcMin, TcMax);

    return {
      Tc,
      Td,
      TcMin,
      TcMax,
      L: Tc + Td,
      R: Tc - Td,
    };
  };

  // ---------------------------
  // Stage 0: Sense() (speeds + robust vRefAbs + per-wheel torque caps)
  // ---------------------------
  const Sense = () => {
    const TV = params.TV;

    const sense = {
      ok: [false, false, false, false],
      w: [0, 0, 0, 0], // signed wheel speeds
      wAbs: [0, 0, 0, 0], // abs wheel speeds
      vehicleSpeedAbs: 0,
      capPos: [0, 0, 0, 0],
      capNeg: [0, 0, 0, 0],
    };

    // In your JS signature, lastFront/lastRear are the "current" samples.
    const frontOk = !!lastFront;
    const rearOk = !!lastRear;

    sense.ok[FL] = sense.ok[FR] = frontOk;
    sense.ok[RL] = sense.ok[RR] = rearOk;

    const getSpeed = (axleObj, left) => {
      if (!axleObj) return 0;
      return left ? +axleObj.rpmL : +axleObj.rpmR;
    };

    sense.w[FL] = getSpeed(lastFront, true);
    sense.w[FR] = getSpeed(lastFront, false);
    sense.w[RL] = getSpeed(lastRear, true);
    sense.w[RR] = getSpeed(lastRear, false);

    for (let i = 0; i < 4; i++) sense.wAbs[i] = absf(sense.w[i]);

    // collect valid abs speeds
    const sorted = [0, 0, 0, 0];
    let n = 0;
    for (let i = 0; i < 4; i++) {
      if (sense.ok[i]) {
        sorted[n] = sense.wAbs[i];
        n++;
      }
    }
    sortSmallN(sorted, n);

    const accel = t > 0;
    const brake = t < 0;
    const coast = !accel && !brake;

    // robust measured reference v_ref_meas (abs)
    let v_ref_meas = 0;
    if (n === 0) {
      v_ref_meas = 0;
    } else if (accel) {
      v_ref_meas = n >= 2 ? sorted[1] : sorted[0]; // 2nd smallest
    } else if (brake) {
      v_ref_meas = n >= 2 ? sorted[n - 2] : sorted[n - 1]; // 2nd largest
    } else {
      v_ref_meas = medianSorted(sorted, n); // median
    }

    // standstill handling
    const eps = TV.SlipSpeedEps;
    if (st.vehicleSpeedAbs < eps) {
      if (accel) {
        if (n > 0) v_ref_meas = sorted[0];
      } else if (brake) {
        v_ref_meas = 0;
      } else {
        v_ref_meas = 0;
      }
    }

    const dv = v_ref_meas - st.vehicleSpeedAbs;
    const plausible = dv <= TV.maxRealisticAccel && dv >= -TV.maxRealisticDecel;

    if (plausible) {
      st.vehicleSpeedAbs = v_ref_meas;
    } // else: reject

    sense.vehicleSpeedAbs = st.vehicleSpeedAbs;

    // --------- caps update (ASR/ABS envelope) ----------
    const maxDrive = +TV.maxPowerDrive;
    const maxBrake = +TV.maxPowerBrake; // magnitude

    if (!st.capsInit) {
      for (let i = 0; i < 4; i++) {
        st.capPos[i] = maxDrive;
        st.capNeg[i] = -maxBrake;
      }
      st.capsInit = true;
    }

    // Pull last issued torque commands from feedback (same sample)
    const getLastTorque = (axleObj, left) => {
      if (!axleObj) return 0;
      return left
        ? +(axleObj.receivedTorqueL ?? 0)
        : +(axleObj.receivedTorqueR ?? 0);
    };

    const cmd = [
      getLastTorque(lastFront, true),
      getLastTorque(lastFront, false),
      getLastTorque(lastRear, true),
      getLastTorque(lastRear, false),
    ];

    const vRef = st.vehicleSpeedAbs;
    const vEps = TV.SlipSpeedEps;
    const slipRatio = TV.SlipRatio;
    const down = TV.SlipDownFactor;
    const recover = TV.SlipRecoverTorquePerTick;
    const minT = TV.SlipMinTorque;

    // recovery every tick toward current live limits
    for (let i = 0; i < 4; i++) {
      st.capPos[i] += recover;
      if (st.capPos[i] > maxDrive) st.capPos[i] = maxDrive;
      if (st.capPos[i] < minT) st.capPos[i] = minT;

      st.capNeg[i] -= recover;
      if (st.capNeg[i] < -maxBrake) st.capNeg[i] = -maxBrake;
      if (st.capNeg[i] > -minT) st.capNeg[i] = -minT;
    }

    const hasAnchor = n > 0 && sorted[0] < vEps;

    for (let i = 0; i < 4; i++) {
      const wi = sense.wAbs[i];
      const ci = cmd[i];

      let slipDrive = false;
      let slipBrake = false;

      if (vRef < vEps) {
        if (hasAnchor) {
          if (ci > +minT && wi > vEps) slipDrive = true;
          slipBrake = false;
        } else {
          slipDrive = false;
          slipBrake = false;
        }
      } else {
        const hi = vRef * (1 + slipRatio);
        const lo = vRef * (1 - slipRatio);

        if (ci > +minT && wi > hi) slipDrive = true;
        if (ci < -minT && wi < lo) slipBrake = true;
      }

      if (slipDrive) {
        st.capPos[i] *= down;
        if (st.capPos[i] < minT) st.capPos[i] = minT;
      }
      if (slipBrake) {
        st.capNeg[i] *= down; // negative -> closer to 0
        if (st.capNeg[i] > -minT) st.capNeg[i] = -minT;
      }
    }

    // output caps for THIS tick (clamped to live physical limits)
    for (let i = 0; i < 4; i++) {
      sense.capPos[i] = clampF(st.capPos[i], minT, maxDrive);
      sense.capNeg[i] = clampF(st.capNeg[i], -maxBrake, -minT);
    }

    return sense;
  };

  // ---------------------------
  // Compute()
  // ---------------------------
  if (currGear === Gear.N || !params || !params.TV) return Torques(0, 0, 0, 0);

  const TV = params.TV;

  // Stage 0
  const sense = Sense();
  const vRefAbs = sense.vehicleSpeedAbs;

  // Stage 1: normalize user inputs
  const sNorm = clampF(s / 1000.0, -1.0, +1.0); // convention: -left, +right
  const absS = absf(sNorm);

  const maxDrive = +TV.maxPowerDrive;
  const maxBrake = +TV.maxPowerBrake;

  // requested magnitude in torque units
  const maxT = t >= 0 ? maxDrive : maxBrake;
  const throttleInput = (t * maxT) / 1000.0;

  const gearSign = currGear === Gear.D ? +1.0 : -1.0;

  // Stage 2: trajectory intent (Tc_total_req + TdF_req + TdR_req)
  let Tc_total_req = 0;

  if (throttleInput >= 0) {
    Tc_total_req = gearSign * throttleInput;
  } else {
    const bmag = -throttleInput;

    const best = bestMotionWheel(sense, vRefAbs);
    let motionSign = gearSign;
    if (best >= 0) {
      const wbest = sense.w[best];
      motionSign = wbest > 0 ? +1 : wbest < 0 ? -1 : gearSign;
    }

    // anti-reversing fade near standstill (uses vRefAbs)
    let brakeScale = 1.0;
    if (TV.AntiReversingSpeed > 1.0) {
      brakeScale = clampF(vRefAbs / TV.AntiReversingSpeed, 0.0, 1.0);
    }

    Tc_total_req = -motionSign * bmag * brakeScale;
  }

  // front/rear bias based on |Tc_total|
  const TcAbs = absf(Tc_total_req);
  const normMax = Tc_total_req >= 0 ? maxDrive : maxBrake;

  let uBias =
    TV.BiasHighThrottle * normMax > 1e-6
      ? TcAbs / (TV.BiasHighThrottle * normMax)
      : 1.0;
  uBias = clampF(uBias, 0.0, 1.0);

  let frontShare = 0.5;
  if (Tc_total_req >= 0) {
    frontShare = lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
  } else {
    frontShare = lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);
  }
  frontShare = clampF(frontShare, 0.05, 0.95);

  let TcF_req = Tc_total_req * frontShare;
  let TcR_req = Tc_total_req * (1.0 - frontShare);

  // front steering differential (primary actuator)
  let TdF_req = 0;
  {
    const vFrontAbs = axleSpeedAbs(sense, FL, FR);
    let uf =
      TV.SteerTorqueHighSpeed > 1.0 ? vFrontAbs / TV.SteerTorqueHighSpeed : 1.0;
    uf = clampF(uf, 0.0, 1.0);
    const kSteer = lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);
    // Pair convention: FL = Tc + Td, FR = Tc - Td
    TdF_req = sNorm * TV.SteerTorqueFront * kSteer;
  }

  // rear yaw assist (only in D)
  let TdR_req = 0;
  if (currGear === Gear.D && absS > 0.001) {
    const vRearAbs = axleSpeedAbs(sense, RL, RR);

    let us = 1.0;
    if (TV.RearFadeSpeed1 > TV.RearFadeSpeed0) {
      us =
        (vRearAbs - TV.RearFadeSpeed0) /
        (TV.RearFadeSpeed1 - TV.RearFadeSpeed0);
      us = clampF(us, 0.0, 1.0);
    }

    const rearEffAbs = absf(TcR_req);
    const t0 = TV.RearFadeThrottle0 * normMax;
    const t1 = TV.RearFadeThrottle1 * normMax;

    let uLong = 1.0;
    if (t1 > t0) {
      uLong = (rearEffAbs - t0) / (t1 - t0);
      uLong = clampF(uLong, 0.0, 1.0);
    }

    const s0 = 0.05;
    const s1 = 0.25;
    let uSteer = (absS - s0) / (s1 - s0);
    uSteer = clampF(uSteer, 0.0, 1.0);

    const fadeRear = us * Math.max(uLong, uSteer);

    const oppMag = absS * TV.SteerTorqueRear;

    // SIGN: s>0 => TdR>0 => RR reduced / can go negative
    //       s<0 => TdR<0 => RL reduced / can go negative
    TdR_req = sNorm > 0 ? +oppMag : -oppMag;
    TdR_req *= fadeRear;
  }

  // Stage 3: solve axle pairs under caps (keep Td)
  let front = solvePairCaps(
    TcF_req,
    TdF_req,
    sense.capNeg[FL],
    sense.capPos[FL],
    sense.capNeg[FR],
    sense.capPos[FR],
  );

  let rear = solvePairCaps(
    TcR_req,
    TdR_req,
    sense.capNeg[RL],
    sense.capPos[RL],
    sense.capNeg[RR],
    sense.capPos[RR],
  );

  // Stage 4: reallocate missing Tc (keep Td fixed)
  const Tc_done = front.Tc + rear.Tc;
  let need = Tc_total_req - Tc_done;

  if (absf(need) > 1e-3) {
    const headroom = (Tc, TcMin, TcMax, needSign) => {
      if (needSign > 0) return TcMax - Tc;
      if (needSign < 0) return Tc - TcMin;
      return 0;
    };

    const availF = headroom(front.Tc, front.TcMin, front.TcMax, need);
    const availR = headroom(rear.Tc, rear.TcMin, rear.TcMax, need);
    const sum = availF + availR;

    if (sum > 1e-6) {
      const dF = need * (availF / sum);
      const dR = need * (availR / sum);

      front.Tc = clampF(front.Tc + dF, front.TcMin, front.TcMax);
      rear.Tc = clampF(rear.Tc + dR, rear.TcMin, rear.TcMax);

      front.L = front.Tc + front.Td;
      front.R = front.Tc - front.Td;
      rear.L = rear.Tc + rear.Td;
      rear.R = rear.Tc - rear.Td;
    }
  }

  let wheel = [front.L, front.R, rear.L, rear.R];

  // Stage 5: optional final rate limit (uses TV.MaxTorquePerTick)
  {
    const maxDelta = TV.MaxTorquePerTick * Math.max(maxDrive, maxBrake);
    for (let i = 0; i < 4; i++) {
      const d = wheel[i] - st.lastWheelCmd[i];
      if (d > maxDelta) wheel[i] = st.lastWheelCmd[i] + maxDelta;
      if (d < -maxDelta) wheel[i] = st.lastWheelCmd[i] - maxDelta;
      st.lastWheelCmd[i] = wheel[i];
    }
  }

  // Stage 6: final clamp to caps + physical limits
  for (let i = 0; i < 4; i++) {
    wheel[i] = clampF(wheel[i], sense.capNeg[i], sense.capPos[i]);
    wheel[i] = clampF(wheel[i], -maxBrake, +maxDrive);
  }

  return Torques(
    truncTowardZero(wheel[FL]),
    truncTowardZero(wheel[FR]),
    truncTowardZero(wheel[RL]),
    truncTowardZero(wheel[RR]),
  );
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
