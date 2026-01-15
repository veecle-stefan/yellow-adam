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
    // Output / safety limits
    maxTorqueDrive: 250.0,
    maxTorqueBrake: 600.0,

    maxSpeedFwd: 100.0,
    maxSpeedRev: 60.0,
    SpeedLimiterFadeBand: 30.0,

    // Steering torques
    MaxTorquePerTick: 50.0,

    SteerTorqueFront: 220.0,
    SteerTorqueRear: 260.0,
    SteerTorqueLowFactor: 1.0,
    SteerTorqueHighFactor: 0.7,
    SteerTorqueHighSpeed: 30.0,

    RearFadeSpeed0: 15.0,
    RearFadeSpeed1: 30.0,
    RearFadeTorque0: 50.0,
    RearFadeTorque1: 150.0,

    // Traction / ABS envelopes
    SlipRatio: 0.2,
    SlipDownFactor: 0.7,

    SlipMinTorque: 50.0,
    SlipRecoverTorquePerTick: 50.0,

    SlipSpeedEps: 20.0,
    maxRealisticAccel: 30.0,
    maxRealisticDecel: 40.0,

    // Front/rear torque bias
    DriveFrontShareLow: 0.55,
    DriveFrontShareHigh: 0.30,
    BrakeFrontShareLow: 0.60,
    BrakeFrontShareHigh: 0.80,

    FrontRearBiasFullTorqueDrive: 150.0,
    FrontRearBiasFullTorqueBrake: 300.0,

    // Braking near standstill
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
  // Persistent state (mirrors the C++ members)
  const st =
    TorqueVectoring._st ||
    (TorqueVectoring._st = {
      vehicleSpeedAbs: 0.0,
      capsInit: false,
      capPos: [0, 0, 0, 0],
      capNeg: [0, 0, 0, 0],
      lastWheelCmd: [0, 0, 0, 0],
    });

  const TV = params.TV;
  const FL = 0, FR = 1, RL = 2, RR = 3;

  // ---------- helpers ----------
  const absf = (x) => (x >= 0 ? x : -x);
  const clampF = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const lerp = (a, b, u) => a + (b - a) * u;
  const truncTowardZero = (x) => (x < 0 ? Math.ceil(x) : Math.floor(x));

  const sortSmallN = (arr, n) => {
    for (let i = 0; i < n; i++)
      for (let j = i + 1; j < n; j++)
        if (arr[j] < arr[i]) {
          const tmp = arr[i];
          arr[i] = arr[j];
          arr[j] = tmp;
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
    const okL = sense.ok[L], okR = sense.ok[R];
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

  const solvePairCaps = (TcReq, TdReq, capNegL, capPosL, capNegR, capPosR) => {
    const TdMin = 0.5 * (capNegL - capPosR);
    const TdMax = 0.5 * (capPosL - capNegR);
    const Td = clampF(TdReq, TdMin, TdMax);

    const TcMin = Math.max(capNegL - Td, capNegR + Td);
    const TcMax = Math.min(capPosL - Td, capPosR + Td);
    const Tc = clampF(TcReq, TcMin, TcMax);

    return {
      Tc, Td,
      TcMin, TcMax,
      L: Tc + Td,
      R: Tc - Td,
    };
  };

  // ---------- Stage 0: Sense() ----------
  const Sense = () => {
    const sense = {
      ok: [false, false, false, false],
      w: [0, 0, 0, 0],    // signed
      wAbs: [0, 0, 0, 0], // abs
      vehicleSpeedAbs: 0,
      capPos: [0, 0, 0, 0],
      capNeg: [0, 0, 0, 0],
    };

    // In your JS signature, lastFront/lastRear are the current samples for this tick.
    const frontOk = !!lastFront;
    const rearOk = !!lastRear;

    sense.ok[FL] = sense.ok[FR] = frontOk;
    sense.ok[RL] = sense.ok[RR] = rearOk;

    const getSpeed = (axle, left) => {
      if (!axle) return 0;
      return left ? +axle.rpmL : +axle.rpmR;
    };

    sense.w[FL] = getSpeed(lastFront, true);
    sense.w[FR] = getSpeed(lastFront, false);
    sense.w[RL] = getSpeed(lastRear, true);
    sense.w[RR] = getSpeed(lastRear, false);

    // abs speeds
    for (let i = 0; i < 4; i++) sense.wAbs[i] = absf(sense.w[i]);

    // gather valid abs speeds
    const sorted = [0, 0, 0, 0];
    let n = 0;
    for (let i = 0; i < 4; i++) {
      if (sense.ok[i]) sorted[n++] = sense.wAbs[i];
    }
    sortSmallN(sorted, n);

    const accel = (t > 0);
    const brake = (t < 0);

    // robust v_ref_meas (abs)
    let v_ref_meas = 0;
    if (n === 0) {
      v_ref_meas = 0;
    } else if (accel) {
      v_ref_meas = (n >= 2) ? sorted[1] : sorted[0];          // 2nd smallest
    } else if (brake) {
      v_ref_meas = (n >= 2) ? sorted[n - 2] : sorted[n - 1];  // 2nd largest
    } else {
      v_ref_meas = medianSorted(sorted, n);                   // median
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

    // plausibility gate (hard reject)
    const dv = v_ref_meas - st.vehicleSpeedAbs;
    if (dv > TV.maxRealisticAccel) st.vehicleSpeedAbs += TV.maxRealisticAccel;
    else if (dv < -TV.maxRealisticDecel) st.vehicleSpeedAbs -= TV.maxRealisticDecel;
      else st.vehicleSpeedAbs = v_ref_meas;

    sense.vehicleSpeedAbs = st.vehicleSpeedAbs;

    // ----- envelopes (ASR/ABS) -----
    const maxDrive = +TV.maxTorqueDrive;
    const maxBrake = +TV.maxTorqueBrake; // magnitude

    if (!st.capsInit) {
      for (let i = 0; i < 4; i++) {
        st.capPos[i] = maxDrive;
        st.capNeg[i] = -maxBrake;
      }
      st.capsInit = true;
    }

    const getLastTorque = (axle, left) => {
      if (!axle) return 0;
      return left ? +(axle.receivedTorqueL ?? 0) : +(axle.receivedTorqueR ?? 0);
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

    // Recovery every tick
    for (let i = 0; i < 4; i++) {
      st.capPos[i] += recover;
      if (st.capPos[i] > maxDrive) st.capPos[i] = maxDrive;
      if (st.capPos[i] < minT) st.capPos[i] = minT;

      st.capNeg[i] -= recover;
      if (st.capNeg[i] < -maxBrake) st.capNeg[i] = -maxBrake;
      if (st.capNeg[i] > -minT) st.capNeg[i] = -minT;
    }

    const hasAnchor = (n > 0) && (sorted[0] < vEps);

    // Slip detection + tighten
    for (let i = 0; i < 4; i++) {
      const wi = sense.wAbs[i];
      const ci = cmd[i];

      let slipDrive = false;
      let slipBrake = false;

      if (vRef < vEps) {
        if (hasAnchor) {
          if (ci > +minT && wi > vEps) slipDrive = true;
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
        st.capNeg[i] *= down;
        if (st.capNeg[i] > -minT) st.capNeg[i] = -minT;
      }
    }

    // publish caps for this tick (clamped)
    for (let i = 0; i < 4; i++) {
      sense.capPos[i] = clampF(st.capPos[i], minT, maxDrive);
      sense.capNeg[i] = clampF(st.capNeg[i], -maxBrake, -minT);
    }

    // ----- Speed limiter as envelope modifier (NEW) -----
    const band = TV.SpeedLimiterFadeBand;
    if (band > 1e-3) {
      const vRefAbs = sense.vehicleSpeedAbs;

      const vMax = (currGear === Gear.D) ? TV.maxSpeedFwd : TV.maxSpeedRev;
      const vStart = vMax - band;

      const hiOutlier = vRefAbs * (1 + TV.SlipRatio);
      const vEps2 = TV.SlipSpeedEps;
      const minT2 = TV.SlipMinTorque;

      const capFromSpeed = (vForCap) => {
        if (vForCap <= vStart) return maxDrive;
        if (vForCap >= vMax) return 0.0;
        const u = (vForCap - vStart) / band; // 0..1
        return maxDrive * (1.0 - clampF(u, 0.0, 1.0));
      };

      for (let i = 0; i < 4; i++) {
        let vForCap = vRefAbs;
        const wi = sense.wAbs[i];

        if (vRefAbs > vEps2) {
          if (wi > hiOutlier && wi > vStart) vForCap = wi;
        }

        const capTarget = capFromSpeed(vForCap);

        if (sense.capPos[i] > capTarget) sense.capPos[i] = capTarget;

        // keep invariant
        if (sense.capPos[i] < minT2) sense.capPos[i] = minT2;
      }
    }

    return sense;
  };

  // ---------- Compute() ----------
  if (currGear === Gear.N) return Torques(0, 0, 0, 0);

  const sense = Sense();
  const vRefAbs = sense.vehicleSpeedAbs;

  // normalize steering
  const sNorm = clampF(s / 1000.0, -1.0, +1.0);
  const absS = absf(sNorm);

  const maxDrive = +TV.maxTorqueDrive;
  const maxBrake = +TV.maxTorqueBrake;

  // requested magnitude in torque units
  const maxT = (t >= 0) ? maxDrive : maxBrake;
  const throttleInput = (t * maxT) / 1000.0;

  const gearSign = (currGear === Gear.D) ? +1.0 : -1.0;

  // Stage 2: Tc_total_req
  let Tc_total_req = 0.0;

  if (throttleInput >= 0) {
    Tc_total_req = gearSign * throttleInput;
  } else {
    const bmag = -throttleInput;

    const best = bestMotionWheel(sense, vRefAbs);
    let motionSign = gearSign;
    if (best >= 0) {
      const wb = sense.w[best];
      motionSign = (wb > 0) ? +1.0 : (wb < 0) ? -1.0 : gearSign;
    }

    let brakeScale = 1.0;
    if (TV.AntiReversingSpeed > 1.0) {
      brakeScale = clampF(vRefAbs / TV.AntiReversingSpeed, 0.0, 1.0);
    }

    Tc_total_req = (-motionSign) * bmag * brakeScale;
  }

  // front/rear bias: use absolute torque ramp with new full-torque params
  const TcAbs = absf(Tc_total_req);
  const full = (Tc_total_req >= 0) ? TV.FrontRearBiasFullTorqueDrive : TV.FrontRearBiasFullTorqueBrake;
  let uBias = (full > 1e-6) ? (TcAbs / full) : 1.0;
  uBias = clampF(uBias, 0.0, 1.0);

  let frontShare = 0.5;
  if (Tc_total_req >= 0) frontShare = lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
  else                   frontShare = lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);
  frontShare = clampF(frontShare, 0.05, 0.95);

  let TcF_req = Tc_total_req * frontShare;
  let TcR_req = Tc_total_req * (1.0 - frontShare);

  // TdF_req (front steering actuator)
  let TdF_req = 0.0;
  {
    const vFrontAbs = axleSpeedAbs(sense, FL, FR);
    let uf = (TV.SteerTorqueHighSpeed > 1.0) ? (vFrontAbs / TV.SteerTorqueHighSpeed) : 1.0;
    uf = clampF(uf, 0.0, 1.0);
    const kSteer = lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);
    TdF_req = sNorm * TV.SteerTorqueFront * kSteer;
  }

  // TdR_req (rear yaw assist; only D)
  let TdR_req = 0.0;
  if (currGear === Gear.D && absS > 0.001) {
    const vRearAbs = axleSpeedAbs(sense, RL, RR);

    let us = 1.0;
    if (TV.RearFadeSpeed1 > TV.RearFadeSpeed0) {
      us = (vRearAbs - TV.RearFadeSpeed0) / (TV.RearFadeSpeed1 - TV.RearFadeSpeed0);
      us = clampF(us, 0.0, 1.0);
    }

    const rearEffAbs = absf(TcR_req);
    const t0 = TV.RearFadeTorque0;
    const t1 = TV.RearFadeTorque1;

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

    TdR_req = (sNorm > 0) ? (+oppMag) : (-oppMag);
    TdR_req *= fadeRear;
  }

  // Stage 3: solve under caps, keep Td
  let front = solvePairCaps(
    TcF_req, TdF_req,
    sense.capNeg[FL], sense.capPos[FL],
    sense.capNeg[FR], sense.capPos[FR]
  );

  let rear = solvePairCaps(
    TcR_req, TdR_req,
    sense.capNeg[RL], sense.capPos[RL],
    sense.capNeg[RR], sense.capPos[RR]
  );

  // Stage 4: reallocate missing Tc (keep Td)
  const Tc_done = front.Tc + rear.Tc;
  let need = Tc_total_req - Tc_done;

  if (absf(need) > 1e-3) {
    const headroom = (Tc, TcMin, TcMax, needSign) => {
      if (needSign > 0) return (TcMax - Tc);
      if (needSign < 0) return (Tc - TcMin);
      return 0;
    };

    const availF = headroom(front.Tc, front.TcMin, front.TcMax, need);
    const availR = headroom(rear.Tc,  rear.TcMin,  rear.TcMax,  need);
    const sum = availF + availR;

    if (sum > 1e-6) {
      const dF = need * (availF / sum);
      const dR = need * (availR / sum);

      front.Tc = clampF(front.Tc + dF, front.TcMin, front.TcMax);
      rear.Tc  = clampF(rear.Tc  + dR, rear.TcMin,  rear.TcMax);

      front.L = front.Tc + front.Td;
      front.R = front.Tc - front.Td;
      rear.L  = rear.Tc  + rear.Td;
      rear.R  = rear.Tc  - rear.Td;
    }
  }

  let wheel = [front.L, front.R, rear.L, rear.R];

  // Stage 5: final rate limit (NOTE: MaxTorquePerTick is absolute torque units/tick now)
  for (let i = 0; i < 4; i++) {
    const d = wheel[i] - st.lastWheelCmd[i];
    if (d >  TV.MaxTorquePerTick) wheel[i] = st.lastWheelCmd[i] + TV.MaxTorquePerTick;
    if (d < -TV.MaxTorquePerTick) wheel[i] = st.lastWheelCmd[i] - TV.MaxTorquePerTick;
    wheel[i] = clampF(wheel[i], sense.capNeg[i], sense.capPos[i]);
    wheel[i] = clampF(wheel[i], -maxBrake, +maxDrive);
    st.lastWheelCmd[i] = wheel[i];
  }

  if ((wheel[FL] > 0) && (st.vehicleSpeedAbs < 10) && (t < 0))
  {
    console.log(`Producing FL=${wheel[FL]}`);
  }

  return Torques(
    truncTowardZero(wheel[FL]),
    truncTowardZero(wheel[FR]),
    truncTowardZero(wheel[RL]),
    truncTowardZero(wheel[RR])
  );
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
