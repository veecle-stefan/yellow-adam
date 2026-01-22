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
    // Output / safety limits (profile)
    maxTorqueDrive: 250.0, // VERIFY DEFAULT: C++ TV.maxTorqueDrive
    maxTorqueBrake: 600.0, // VERIFY DEFAULT: C++ TV.maxTorqueBrake
    maxSpeedFwd: 100.0,    // VERIFY DEFAULT: C++ TV.maxSpeedFwd
    maxSpeedRev: 60.0,     // VERIFY DEFAULT: C++ TV.maxSpeedRev
    SpeedLimiterFadeBand: 30.0, // VERIFY DEFAULT: C++ TV.SpeedLimiterFadeBand

    // Steering torques
    MaxTorquePerTick: 50.0, // VERIFY DEFAULT: C++ TV.MaxTorquePerTick (absolute torque units / tick)

    SteerTorqueFront: 220.0,     // VERIFY DEFAULT
    SteerTorqueRear: 260.0,      // VERIFY DEFAULT
    SteerTorqueLowFactor: 1.0,   // VERIFY DEFAULT
    SteerTorqueHighFactor: 0.7,  // VERIFY DEFAULT
    SteerTorqueHighSpeed: 30.0,  // VERIFY DEFAULT

    RearFadeSpeed0: 15.0,   // VERIFY DEFAULT
    RearFadeSpeed1: 30.0,   // VERIFY DEFAULT
    RearFadeTorque0: 50.0,  // VERIFY DEFAULT
    RearFadeTorque1: 150.0, // VERIFY DEFAULT

    // Traction / ABS (per-wheel envelopes)
    SlipRatio: 0.20,          // VERIFY DEFAULT
    SlipDownFactor: 0.70,     // VERIFY DEFAULT
    SlipMinTorque: 50.0,      // VERIFY DEFAULT
    SlipRecoverTorquePerTick: 50.0, // VERIFY DEFAULT
    SlipSpeedEps: 20.0,       // VERIFY DEFAULT
    maxRealisticAccel: 30.0,  // VERIFY DEFAULT
    maxRealisticDecel: 40.0,  // VERIFY DEFAULT

    // Front/rear torque bias (load transfer compensation)
    DriveFrontShareLow: 0.55,  // VERIFY DEFAULT
    DriveFrontShareHigh: 0.30, // VERIFY DEFAULT
    BrakeFrontShareLow: 0.60,  // VERIFY DEFAULT
    BrakeFrontShareHigh: 0.80, // VERIFY DEFAULT
    FrontRearBiasFullTorqueDrive: 150.0, // VERIFY DEFAULT
    FrontRearBiasFullTorqueBrake: 300.0, // VERIFY DEFAULT

    // Braking near standstill (anti-reversing)
    AntiReversingSpeed: 100.0, // VERIFY DEFAULT
  },
};

// ============================================================================
// Drop-in replacement: TorqueVectoring(currGear, t, s, lastFront, lastRear)
// - Matches the latest C++ structure (Sense -> Intent -> Solve -> Output).
// - Keeps persistent state inside TorqueVectoring._st (like C++ members).
//
// Expected inputs (same conventions as before):
//   currGear: Gear.N=0, Gear.D=1, Gear.R=2  (use your existing Gear enum)
//   t: throttle command [-1000..1000]
//   s: steering command [-1000..1000] (convention: -left, +right)
//   lastFront/lastRear: current feedback sample objects, with:
//     rpmL, rpmR                (signed wheel speed, same scale as C++)
//     receivedTorqueL/R         (cmdL/cmdR at reading, same sample)
// If your field names differ, map them in getSpeed/getCmd.
//
// Return:
//   - If your Torques(...) helper supports 5 args, it returns (fl,fr,rl,rr,vehicleSpeedAbs).
//   - Else it returns the usual Torques(fl,fr,rl,rr) and attaches `.vehicleSpeedAbs`.
// ============================================================================
function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  // ---------------- persistent state (C++ members) ----------------
  const st =
    TorqueVectoring._st ||
    (TorqueVectoring._st = {
      vehicleSpeedAbs: 0.0,     // m_vehicleSpeedAbs
      capsInit: false,          // m_capsInit
      capFwd: [0, 0, 0, 0],     // m_capFwd (propulsion/assist side, motor coords)
      capRev: [0, 0, 0, 0],     // m_capRev (opposing side, motor coords)
      lastTdF: 0.0,             // m_lastTdF
      lastTdR: 0.0,             // m_lastTdR
    });

  const TV = params.TV;
  const FL = 0, FR = 1, RL = 2, RR = 3;

  // ---------------- helpers ----------------
  const absf = (x) => (x >= 0 ? x : -x);
  const clampF = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const lerp = (a, b, u) => a + (b - a) * u;
  const truncTowardZero = (x) => (x < 0 ? Math.ceil(x) : Math.floor(x));

  const rateLimit = (target, last, maxDelta) => {
    const d = target - last;
    if (d > maxDelta) return last + maxDelta;
    if (d < -maxDelta) return last - maxDelta;
    return target;
  };

  const wheelSign = (w, vEps) => {
    if (w > +vEps) return +1;
    if (w < -vEps) return -1;
    return 0;
  };

  const capFromSpeed = (vForCap, vStart, vMax, maxTorque) => {
    if (vForCap <= vStart) return maxTorque;
    if (vForCap >= vMax) return 0.0;
    const u = (vForCap - vStart) / (vMax - vStart); // 0..1
    return maxTorque * (1.0 - clampF(u, 0.0, 1.0));
  };

  const tightenAssistCap = (cmdAtReading, assistSign, minT, downFactor, caps) => {
    const tSlip = absf(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capFwd = Math.min(caps.capFwd, target);
    else                caps.capRev = Math.max(caps.capRev, -target);
  };

  const tightenOpposeCap = (cmdAtReading, assistSign, minT, downFactor, caps) => {
    const tSlip = absf(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capRev = Math.max(caps.capRev, -target);
    else                caps.capFwd = Math.min(caps.capFwd, target);
  };

  const solvePairCaps = (TcReq, TdReq, capNegL, capPosL, capNegR, capPosR) => {
    const TdMin = 0.5 * (capNegL - capPosR);
    const TdMax = 0.5 * (capPosL - capNegR);
    const Td = clampF(TdReq, TdMin, TdMax);

    const TcMin = Math.max(capNegL - Td, capNegR + Td);
    const TcMax = Math.min(capPosL - Td, capPosR + Td);
    const Tc = clampF(TcReq, TcMin, TcMax);

    return { Tc, Td, TcMin, TcMax, L: Tc + Td, R: Tc - Td };
  };

  const axleSpeedAbs = (sense, L, R) => {
    const okL = sense.ok[L], okR = sense.ok[R];
    if (okL && okR) return 0.5 * (sense.wAbs[L] + sense.wAbs[R]);
    if (okL) return sense.wAbs[L];
    if (okR) return sense.wAbs[R];
    return 0.0;
  };

  const computeFrontSteeringDiff = (steering, vFrontAbs) => {
    let uf = (TV.SteerTorqueHighSpeed > 1.0) ? (vFrontAbs / TV.SteerTorqueHighSpeed) : 1.0;
    uf = clampF(uf, 0.0, 1.0);
    const kSteer = lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);
    return steering * TV.SteerTorqueFront * kSteer;
  };

  const computeRearYawAssist = (steering, absS, vRearAbs, rearEffAbs) => {
    let us = 1.0;
    if (TV.RearFadeSpeed1 > TV.RearFadeSpeed0) {
      us = (vRearAbs - TV.RearFadeSpeed0) / (TV.RearFadeSpeed1 - TV.RearFadeSpeed0);
      us = clampF(us, 0.0, 1.0);
    }

    let uLong = 1.0;
    if (TV.RearFadeTorque1 > TV.RearFadeTorque0) {
      uLong = (rearEffAbs - TV.RearFadeTorque0) / (TV.RearFadeTorque1 - TV.RearFadeTorque0);
      uLong = clampF(uLong, 0.0, 1.0);
    }

    // TODO in C++: could be params
    const s0 = 0.05;
    const s1 = 0.25;
    let uSteer = (absS - s0) / (s1 - s0);
    uSteer = clampF(uSteer, 0.0, 1.0);

    const fadeRear = us * Math.max(uLong, uSteer);
    const oppMag = absS * TV.SteerTorqueRear;

    return ((steering > 0) ? (+oppMag) : (-oppMag)) * fadeRear;
  };

  // ---- input mapping (adjust here if your object keys differ) ----
  const getSpeed = (axle, left) => {
    if (!axle) return 0.0;
    return left ? +axle.rpmL : +axle.rpmR;
  };
  const getCmd = (axle, left) => {
    if (!axle) return 0.0;
    // cmdAtReading := fb->sample.cmdL/cmdR
    return left ? +(axle.receivedTorqueL ?? 0) : +(axle.receivedTorqueR ?? 0);
  };

  // ========================================================================
  // Sense(): ReadWheelSpeeds -> EstimateVehicleMotion -> UpdateSlipEnvelopes -> ApplySpeedLimiter
  // ========================================================================
  const Sense = () => {
    const sense = {
      ok: [false, false, false, false],
      w: [0, 0, 0, 0],         // signed
      wAbs: [0, 0, 0, 0],      // abs
      cmdAtReading: [0, 0, 0, 0],
      vehicleSpeedAbs: 0.0,

      gearDir: (currGear === Gear.D) ? +1.0 : -1.0, // motor-coord "forward along gear"
      motionSign: 0.0,
      motionSignAlongGear: 0.0,

      capFwd: [0, 0, 0, 0], // propulsion/assist side
      capRev: [0, 0, 0, 0], // opposing side
    };

    const sc = {
      accel: (t > 0),
      brake: (t < 0),
      sorted: [], // [{absSpeed,index}]
      numValid: 0,
    };

    // --- ReadWheelSpeeds ---
    if (lastFront) {
      sense.ok[FL] = sense.ok[FR] = true;
      sense.w[FL] = getSpeed(lastFront, true);
      sense.w[FR] = getSpeed(lastFront, false);
      sense.wAbs[FL] = absf(sense.w[FL]);
      sense.wAbs[FR] = absf(sense.w[FR]);
      sense.cmdAtReading[FL] = getCmd(lastFront, true);
      sense.cmdAtReading[FR] = getCmd(lastFront, false);
    }
    if (lastRear) {
      sense.ok[RL] = sense.ok[RR] = true;
      sense.w[RL] = getSpeed(lastRear, true);
      sense.w[RR] = getSpeed(lastRear, false);
      sense.wAbs[RL] = absf(sense.w[RL]);
      sense.wAbs[RR] = absf(sense.w[RR]);
      sense.cmdAtReading[RL] = getCmd(lastRear, true);
      sense.cmdAtReading[RR] = getCmd(lastRear, false);
    }

    // --- EstimateVehicleMotion ---
    sc.sorted.length = 0;
    for (let i = 0; i < 4; i++) {
      if (sense.ok[i]) sc.sorted.push({ absSpeed: sense.wAbs[i], index: i });
    }
    sc.sorted.sort((a, b) => a.absSpeed - b.absSpeed);
    sc.numValid = sc.sorted.length;

    const n = sc.numValid;
    const vEps = TV.SlipSpeedEps;

    let v_ref_meas = 0.0;
    let motionSign = 0.0; // -1,0,+1
    let refWheelIdx = -1;

    if (n === 0) {
      v_ref_meas = 0.0;
      refWheelIdx = -1;
    } else {
      if (sc.accel) {
        const pick = (n >= 2) ? 1 : 0;
        v_ref_meas = sc.sorted[pick].absSpeed;
        refWheelIdx = sc.sorted[pick].index;
      } else if (sc.brake) {
        const pick = (n >= 2) ? (n - 2) : (n - 1);
        v_ref_meas = sc.sorted[pick].absSpeed;
        refWheelIdx = sc.sorted[pick].index;
      } else {
        // coasting: median + sign agreement checks
        if (n === 1) {
          v_ref_meas = sc.sorted[0].absSpeed;
          refWheelIdx = sc.sorted[0].index;
        } else if (n === 2) {
          v_ref_meas = 0.5 * (sc.sorted[0].absSpeed + sc.sorted[1].absSpeed);
          const s0 = wheelSign(sense.w[sc.sorted[0].index], vEps);
          const s1 = wheelSign(sense.w[sc.sorted[1].index], vEps);
          refWheelIdx = (s0 !== 0 && s0 === s1) ? sc.sorted[0].index : -1;
        } else if (n === 3) {
          v_ref_meas = sc.sorted[1].absSpeed;
          refWheelIdx = sc.sorted[1].index;
        } else {
          v_ref_meas = 0.5 * (sc.sorted[1].absSpeed + sc.sorted[2].absSpeed);
          const s1 = wheelSign(sense.w[sc.sorted[1].index], vEps);
          const s2 = wheelSign(sense.w[sc.sorted[2].index], vEps);
          refWheelIdx = (s1 !== 0 && s1 === s2) ? sc.sorted[1].index : -1;
        }
      }
    }

    if (refWheelIdx >= 0 && v_ref_meas > vEps) {
      const wref = sense.w[refWheelIdx];
      motionSign = (wref > 0) ? +1.0 : -1.0;
    } else {
      motionSign = 0.0;
    }

    // standstill handling
    if (st.vehicleSpeedAbs < vEps) {
      if (sc.accel) {
        if (n > 0) v_ref_meas = sc.sorted[0].absSpeed;
      } else {
        v_ref_meas = 0.0;
        motionSign = 0.0;
      }
    }

    // NEW (latest C++): rate-limit speed changes instead of hard reject
    const dv = v_ref_meas - st.vehicleSpeedAbs;
    if (dv > TV.maxRealisticAccel) st.vehicleSpeedAbs += TV.maxRealisticAccel;
    else if (dv < -TV.maxRealisticDecel) st.vehicleSpeedAbs -= TV.maxRealisticDecel;
    else st.vehicleSpeedAbs = v_ref_meas;

    sense.vehicleSpeedAbs = st.vehicleSpeedAbs;
    sense.motionSign = motionSign;
    sense.motionSignAlongGear = (absf(motionSign) > 0.5) ? (motionSign * sense.gearDir) : 0.0;

    // --- UpdateSlipEnvelopes ---
    const vRef = st.vehicleSpeedAbs;
    const minT = TV.SlipMinTorque;
    const hard = Math.max(TV.maxTorqueDrive, TV.maxTorqueBrake);

    if (!st.capsInit) {
      for (let i = 0; i < 4; i++) {
        st.capFwd[i] = +hard;
        st.capRev[i] = -hard;
      }
      st.capsInit = true;
    }

    // recover towards +/- hard
    for (let i = 0; i < 4; i++) {
      st.capFwd[i] = Math.min(st.capFwd[i] + TV.SlipRecoverTorquePerTick, +hard);
      st.capFwd[i] = Math.max(st.capFwd[i], minT);

      st.capRev[i] = Math.max(st.capRev[i] - TV.SlipRecoverTorquePerTick, -hard);
      st.capRev[i] = Math.min(st.capRev[i], -minT);
    }

    const hasAnchor = (sc.numValid > 0) && (sc.sorted[0].absSpeed < vEps);
    const moving = (absf(sense.motionSign) > 0.5);
    const assistSign = moving ? sense.motionSign : sense.gearDir;

    for (let i = 0; i < 4; i++) {
      const wi = sense.wAbs[i];
      const ci = sense.cmdAtReading[i];

      const cmdAssistStandstill = (ci * sense.gearDir) > +minT;
      const effMotionSign = moving ? sense.motionSign : sense.gearDir;
      const cmdAssistMoving = (ci * effMotionSign) > +minT;
      const cmdOpposeMoving = (ci * effMotionSign) < -minT;

      let slipAccelerate = false;
      let slipOppose = false;

      if (vRef < vEps) {
        if (hasAnchor) {
          if (cmdAssistStandstill && wi > vEps) slipAccelerate = true;
          slipOppose = false;
        }
      } else {
        const hi = vRef * (1.0 + TV.SlipRatio);
        const lo = vRef * (1.0 - TV.SlipRatio);

        if (cmdAssistMoving && wi > hi) slipAccelerate = true;
        if (cmdOpposeMoving && wi < lo) slipOppose = true;
      }

      const caps = { capFwd: st.capFwd[i], capRev: st.capRev[i] };
      if (slipAccelerate) tightenAssistCap(ci, assistSign, minT, TV.SlipDownFactor, caps);
      if (slipOppose)     tightenOpposeCap(ci, assistSign, minT, TV.SlipDownFactor, caps);
      st.capFwd[i] = caps.capFwd;
      st.capRev[i] = caps.capRev;
    }

    for (let i = 0; i < 4; i++) {
      sense.capFwd[i] = clampF(st.capFwd[i], minT, +hard);
      sense.capRev[i] = clampF(st.capRev[i], -hard, -minT);
    }

    // --- ApplySpeedLimiter ---
    const band = TV.SpeedLimiterFadeBand;
    if (band > 1e-3) {
      const vMax = (currGear === Gear.D) ? TV.maxSpeedFwd : TV.maxSpeedRev;
      const vStart = vMax - band;
      const hiOutlier = vRef * (1.0 + TV.SlipRatio);

      for (let i = 0; i < 4; i++) {
        let vForCap = vRef;
        const wi = sense.wAbs[i];

        if (vRef > TV.SlipSpeedEps) {
          if (wi > hiOutlier && wi > vStart) vForCap = wi;
        }

        const capTarget = capFromSpeed(vForCap, vStart, vMax, TV.maxTorqueDrive);

        // tighten propulsion-side envelope (sign = gearDir)
        if (sense.gearDir > 0) {
          sense.capFwd[i] = Math.min(sense.capFwd[i], capTarget);
          sense.capFwd[i] = Math.max(sense.capFwd[i], TV.SlipMinTorque);
        } else {
          sense.capRev[i] = Math.max(sense.capRev[i], -capTarget);
          sense.capRev[i] = Math.min(sense.capRev[i], -TV.SlipMinTorque);
        }
      }
    }

    return sense;
  };

  // ========================================================================
  // Compute(): Sense -> TrajectoryIntent -> SolveWheelTorques -> Output
  // ========================================================================
  if (currGear === Gear.N) {
    const out0 = Torques(0, 0, 0, 0);
    out0.vehicleSpeedAbs = 0;
    return out0;
  }

  // ---- SENSE ----
  const sense = Sense();

  // ---- TRAJECTORY INTENT ----
  const vRefAbs = sense.vehicleSpeedAbs;

  const steering = clampF(s / 1000.0, -1.0, +1.0);
  const absS = absf(steering);

  const maxDrive = TV.maxTorqueDrive;
  const maxBrake = TV.maxTorqueBrake;

  const maxT = (t >= 0) ? maxDrive : maxBrake;
  const throttleInput = (t * maxT) / 1000.0;

  let Tc_total_req = 0.0;

  if (throttleInput >= 0) {
    // NEW: drive in "gear space" (NOT multiplied by gearDir here)
    Tc_total_req = throttleInput;
  } else {
    const bmag = -throttleInput;

    let brakeScale = 1.0;
    if (TV.AntiReversingSpeed > 1.0) {
      brakeScale = clampF(vRefAbs / TV.AntiReversingSpeed, 0.0, 1.0);
    }

    let msAlongGear = sense.motionSignAlongGear;
    if (absf(msAlongGear) < 0.5) msAlongGear = +1.0; // fallback: assume moving along gear

    Tc_total_req = (-msAlongGear) * bmag * brakeScale;
  }

  // front/rear bias
  const TcAbs = absf(Tc_total_req);
  const full = (Tc_total_req >= 0) ? TV.FrontRearBiasFullTorqueDrive : TV.FrontRearBiasFullTorqueBrake;
  let uBias = (full > 1e-6) ? (TcAbs / full) : 1.0;
  uBias = clampF(uBias, 0.0, 1.0);

  let frontShare = 0.5;
  if (Tc_total_req >= 0) frontShare = lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
  else                   frontShare = lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);
  frontShare = clampF(frontShare, 0.05, 0.95);

  const TcF_req = Tc_total_req * frontShare;
  const TcR_req = Tc_total_req * (1.0 - frontShare);

  // steering diffs (rate limited, per latest C++)
  const vFrontAbs = axleSpeedAbs(sense, FL, FR);
  let TdF_req = computeFrontSteeringDiff(steering, vFrontAbs);
  TdF_req = rateLimit(TdF_req, st.lastTdF, TV.MaxTorquePerTick);
  st.lastTdF = TdF_req;

  let TdR_req = 0.0;
  if (currGear === Gear.D && absS > 0.001) {
    const vRearAbs = axleSpeedAbs(sense, RL, RR);
    TdR_req = computeRearYawAssist(steering, absS, vRearAbs, absf(TcR_req));
    TdR_req = rateLimit(TdR_req, st.lastTdR, TV.MaxTorquePerTick);
    st.lastTdR = TdR_req;
  }

  const intent = { Tc_total: Tc_total_req, TcF: TcF_req, TcR: TcR_req, TdF: TdF_req, TdR: TdR_req };

  // ---- SOLVE CONSTRAINTS (motor coords) ----
  const TcF_req_m = sense.gearDir * intent.TcF;
  const TcR_req_m = sense.gearDir * intent.TcR;

  // Td MUST NOT be flipped
  const TdF_req_m = intent.TdF;
  const TdR_req_m = intent.TdR;

  let front = solvePairCaps(
    TcF_req_m, TdF_req_m,
    sense.capRev[FL], sense.capFwd[FL],
    sense.capRev[FR], sense.capFwd[FR]
  );
  let rear = solvePairCaps(
    TcR_req_m, TdR_req_m,
    sense.capRev[RL], sense.capFwd[RL],
    sense.capRev[RR], sense.capFwd[RR]
  );

  const Tc_total_req_m = sense.gearDir * intent.Tc_total;
  const Tc_done_m = front.Tc + rear.Tc;
  let need = Tc_total_req_m - Tc_done_m;

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

  const hard = Math.max(TV.maxTorqueDrive, TV.maxTorqueBrake);
  const fl = clampF(front.L, -hard, +hard);
  const fr = clampF(front.R, -hard, +hard);
  const rl = clampF(rear.L,  -hard, +hard);
  const rr = clampF(rear.R,  -hard, +hard);

  // ---- OUTPUT ----
  // If your Torques helper supports 5 args, use it; else attach speed as a property.
  let out = Torques(truncTowardZero(fl), truncTowardZero(fr), truncTowardZero(rl), truncTowardZero(rr));
  out.vehicleSpeedAbs = sense.vehicleSpeedAbs;
  return out;
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
