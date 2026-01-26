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

// =====================
// Params (mirror of TVParams)
// =====================
const params = {
  TV: {
    // Output / safety limits (verify defaults)
    maxTorqueDrive: 250.0,          // TODO verify default
    maxTorqueBrake: 600.0,          // TODO verify default
    maxSpeedFwd: 100.0,             // TODO verify default
    maxSpeedRev: 60.0,              // TODO verify default
    SpeedLimiterFadeBand: 30.0,     // TODO verify default

    // Steering
    MaxTorquePerTick: 50.0,         // TODO verify default (units per tick)
    SteerTorqueFront: 220.0,        // TODO verify default
    SteerTorqueRear: 260.0,         // TODO verify default
    SteerTorqueLowFactor: 1.0,      // TODO verify default
    SteerTorqueHighFactor: 0.7,     // TODO verify default
    SteerTorqueHighSpeed: 30.0,     // TODO verify default
    RearFadeSpeed0: 15.0,           // TODO verify default
    RearFadeSpeed1: 30.0,           // TODO verify default
    RearFadeTorque0: 50.0,          // TODO verify default
    RearFadeTorque1: 150.0,         // TODO verify default

    // Traction / ABS
    SlipRatio: 0.20,                // TODO verify default
    SlipDownFactor: 0.70,           // TODO verify default
    SlipMinTorque: 50.0,            // TODO verify default
    SlipRecoverTorquePerTick: 50.0, // TODO verify default
    WheelMinRPM: 20.0,              // TODO verify default (noise floor / eps)

    TractionCorrectionASR: 0.75,    // TODO verify default
    TractionCorrectionABS: 0.25,    // TODO verify default

    maxRealisticAccel: 30.0,        // TODO verify default
    maxRealisticDecel: 40.0,        // TODO verify default

    // Front/rear bias
    DriveFrontShareLow: 0.55,            // TODO verify default
    DriveFrontShareHigh: 0.30,           // TODO verify default
    BrakeFrontShareLow: 0.60,            // TODO verify default
    BrakeFrontShareHigh: 0.80,           // TODO verify default
    FrontRearBiasFullTorqueDrive: 150.0, // TODO verify default
    FrontRearBiasFullTorqueBrake: 300.0, // TODO verify default

    // Anti-reversing / brake fade
    AntiReversingSpeed: 50.0,       // TODO verify default
    AntiReversingHoldSpeed: 30.0,   // TODO verify default (hold logic ignored in output)
  },
};

// =====================
// TorqueVectoring(currGear, t, s, lastFront, lastRear)
// - currGear: 0=N, 1=D, 2=R
// - t: throttle in [-1000..1000] (like your C++)
// - s: steering in [-1000..1000]
// - lastFront/lastRear: optional feedback frames { sample:{ speedL_meas,speedR_meas,cmdL,cmdR } } or null
// - returns Torques(fl,fr,rl,rr)
// =====================
function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  const TV = params.TV;

  // -----------------
  // Helpers
  // -----------------
  const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
  const lerp = (a, b, u) => a + (b - a) * u;
  const truncTowardZero = (x) => (x < 0 ? Math.ceil(x) : Math.floor(x));

  function WheelSpeed(absSpeed = 0, index = -1) {
    return { absSpeed, index };
  }

  function sortSmallN(arr, n) {
    for (let i = 0; i < n; ++i) {
      for (let j = i + 1; j < n; ++j) {
        if (arr[j].absSpeed < arr[i].absSpeed) {
          const tmp = arr[i];
          arr[i] = arr[j];
          arr[j] = tmp;
        }
      }
    }
  }

  function rateLimit(target, last, maxDelta) {
    const d = target - last;
    if (d > maxDelta) return last + maxDelta;
    if (d < -maxDelta) return last - maxDelta;
    return target;
  }

  function wheelSign(w, vEps) {
    if (w > vEps) return +1;
    if (w < -vEps) return -1;
    return 0;
  }

  function tightenAssistCap(cmdAtReading, assistSign, minT, downFactor, caps, i) {
    const tSlip = Math.abs(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capFwd[i] = Math.min(caps.capFwd[i], target);
    else                caps.capRev[i] = Math.max(caps.capRev[i], -target);
  }

  function tightenOpposeCap(cmdAtReading, assistSign, minT, downFactor, caps, i) {
    const tSlip = Math.abs(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capRev[i] = Math.max(caps.capRev[i], -target);
    else                caps.capFwd[i] = Math.min(caps.capFwd[i], target);
  }

  function capFromSpeed(vForCap, vStart, vMax, maxTorque) {
    if (vForCap <= vStart) return maxTorque;
    if (vForCap >= vMax) return 0.0;
    const u = (vForCap - vStart) / (vMax - vStart);
    return maxTorque * (1.0 - clamp(u, 0.0, 1.0));
  }

  function computeFrontSteeringDiff(steeringNorm, vFrontAbs) {
    let uf = (TV.SteerTorqueHighSpeed > 1.0) ? (vFrontAbs / TV.SteerTorqueHighSpeed) : 1.0;
    uf = clamp(uf, 0.0, 1.0);
    const kSteer = lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);
    return steeringNorm * TV.SteerTorqueFront * kSteer;
  }

  function computeRearYawAssist(steeringNorm, absS, vRearAbs, rearEffAbs) {
    let us = 1.0;
    if (TV.RearFadeSpeed1 > TV.RearFadeSpeed0) {
      us = (vRearAbs - TV.RearFadeSpeed0) / (TV.RearFadeSpeed1 - TV.RearFadeSpeed0);
      us = clamp(us, 0.0, 1.0);
    }

    let uLong = 1.0;
    if (TV.RearFadeTorque1 > TV.RearFadeTorque0) {
      uLong = (rearEffAbs - TV.RearFadeTorque0) / (TV.RearFadeTorque1 - TV.RearFadeTorque0);
      uLong = clamp(uLong, 0.0, 1.0);
    }

    // TODO: could be params.TV config (kept constant like C++)
    const s0 = 0.05;
    const s1 = 0.25;
    let uSteer = (absS - s0) / (s1 - s0);
    uSteer = clamp(uSteer, 0.0, 1.0);

    const fadeRear = us * Math.max(uLong, uSteer);
    const oppMag = absS * TV.SteerTorqueRear;

    return ((steeringNorm > 0) ? (+oppMag) : (-oppMag)) * fadeRear;
  }

  function AxleSpeedAbs(sense, L, R) {
    const okL = sense.ok[L], okR = sense.ok[R];
    if (okL && okR) return 0.5 * (sense.wAbs[L] + sense.wAbs[R]);
    if (okL) return sense.wAbs[L];
    if (okR) return sense.wAbs[R];
    return 0.0;
  }

  function SolvePairCaps(TcReq, TdReq, capNegL, capPosL, capNegR, capPosR, k) {
    const TdTraction = 0.5 * (capPosL - capPosR);
    const TdTarget = TdReq + k * TdTraction;

    const TdMin = 0.5 * (capNegL - capPosR);
    const TdMax = 0.5 * (capPosL - capNegR);
    const Td = clamp(TdTarget, TdMin, TdMax);

    const TcMin = Math.max(capNegL - Td, capNegR + Td);
    const TcMax = Math.min(capPosL - Td, capPosR + Td);
    const Tc = clamp(TcReq, TcMin, TcMax);

    return { Tc, Td, TcMin, TcMax, L: Tc + Td, R: Tc - Td };
  }

  // -----------------
  // Persistent state (function-static)
  // -----------------
  const st = TorqueVectoring._state || (TorqueVectoring._state = {
    vehicleSpeedAbs: 0.0,
    capsInit: false,
    capFwd: [0, 0, 0, 0],
    capRev: [0, 0, 0, 0],
    lastTdF: 0.0,
    lastTdR: 0.0,
  });

  // -----------------
  // Early exit
  // -----------------
  const GearN = 0, GearD = 1, GearR = 2;
  if (currGear === GearN) return Torques(0, 0, 0, 0);

  // Wheel indices
  const FL = 0, FR = 1, RL = 2, RR = 3;

  // -----------------
  // SenseData / SenseContext
  // -----------------
  const sense = {
    ok: [false, false, false, false],
    w: [0, 0, 0, 0],
    wAbs: [0, 0, 0, 0],
    cmdAtReading: [0, 0, 0, 0],

    vehicleSpeedAbs: 0.0,
    gearDir: (currGear === GearD) ? +1.0 : -1.0,
    motionSign: 0.0,
    motionSignAlongGear: 0.0,

    capFwd: [0, 0, 0, 0],
    capRev: [0, 0, 0, 0],
  };

  const sc = {
    accel: (t > 0),
    brake: (t < 0),
    sorted: [WheelSpeed(), WheelSpeed(), WheelSpeed(), WheelSpeed()],
    numValid: 0,
  };

  // -----------------
  // ReadWheelSpeeds (from lastFront/lastRear)
  // -----------------
  function ReadAxleSpeeds(fb, indexShift) {
    if (!fb) return;

    sense.ok[indexShift] = true;
    sense.ok[indexShift + 1] = true;

    const sL = +fb.sample.speedL_meas || 0;
    const sR = +fb.sample.speedR_meas || 0;

    sense.w[indexShift] = sL;
    sense.w[indexShift + 1] = sR;

    sense.wAbs[indexShift] = Math.abs(sL);
    sense.wAbs[indexShift + 1] = Math.abs(sR);

    sense.cmdAtReading[indexShift] = +fb.sample.cmdL || 0;
    sense.cmdAtReading[indexShift + 1] = +fb.sample.cmdR || 0;
  }

  ReadAxleSpeeds(lastFront, 0);
  ReadAxleSpeeds(lastRear, 2);

  // -----------------
  // EstimateVehicleMotion
  // -----------------
  (function EstimateVehicleMotion() {
    sc.numValid = 0;
    for (let i = 0; i < 4; ++i) {
      if (sense.ok[i]) {
        sc.sorted[sc.numValid].absSpeed = sense.wAbs[i];
        sc.sorted[sc.numValid].index = i;
        sc.numValid++;
      }
    }
    sortSmallN(sc.sorted, sc.numValid);

    const n = sc.numValid;
    const vEps = TV.WheelMinRPM;

    let v_ref_meas = 0.0;
    let motionSign = 0.0;
    let refWheelIdx = -1;

    if (n === 0) {
      v_ref_meas = 0.0;
      refWheelIdx = -1;
    } else if (sc.accel) {
      const pick = (n >= 2) ? 1 : 0;
      v_ref_meas = sc.sorted[pick].absSpeed;
      refWheelIdx = sc.sorted[pick].index;
    } else if (sc.brake) {
      const pick = (n >= 2) ? (n - 2) : (n - 1);
      v_ref_meas = sc.sorted[pick].absSpeed;
      refWheelIdx = sc.sorted[pick].index;
    } else {
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

    if (refWheelIdx >= 0 && v_ref_meas > vEps) {
      motionSign = (sense.w[refWheelIdx] > 0) ? +1.0 : -1.0;
    } else {
      motionSign = 0.0;
    }

    if (st.vehicleSpeedAbs < vEps) {
      if (sc.accel) {
        if (n > 0) v_ref_meas = sc.sorted[0].absSpeed;
      } else {
        v_ref_meas = 0.0;
        motionSign = 0.0;
      }
    }

    const dv = v_ref_meas - st.vehicleSpeedAbs;
    if (dv > TV.maxRealisticAccel) st.vehicleSpeedAbs += TV.maxRealisticAccel;
    else if (dv < -TV.maxRealisticDecel) st.vehicleSpeedAbs -= TV.maxRealisticDecel;
    else st.vehicleSpeedAbs = v_ref_meas;

    sense.vehicleSpeedAbs = st.vehicleSpeedAbs;
    sense.motionSign = motionSign;
    sense.motionSignAlongGear = (Math.abs(motionSign) > 0.5) ? (motionSign * sense.gearDir) : 0.0;
  })();

  // -----------------
  // UpdateSlipEnvelopes
  // -----------------
  (function UpdateSlipEnvelopes() {
    const vRef = st.vehicleSpeedAbs;
    const vEps = TV.WheelMinRPM;
    const minT = TV.SlipMinTorque;
    const hard = Math.max(TV.maxTorqueDrive, TV.maxTorqueBrake);

    if (!st.capsInit) {
      for (let i = 0; i < 4; i++) {
        st.capFwd[i] = +hard;
        st.capRev[i] = -hard;
      }
      st.capsInit = true;
    }

    for (let i = 0; i < 4; i++) {
      st.capFwd[i] = Math.min(st.capFwd[i] + TV.SlipRecoverTorquePerTick, +hard);
      st.capFwd[i] = Math.max(st.capFwd[i], minT);

      st.capRev[i] = Math.max(st.capRev[i] - TV.SlipRecoverTorquePerTick, -hard);
      st.capRev[i] = Math.min(st.capRev[i], -minT);
    }

    const hasAnchor = (sc.numValid > 0) && (sc.sorted[0].absSpeed < vEps);
    const moving = (Math.abs(sense.motionSign) > 0.5);
    const assistSign = moving ? sense.motionSign : sense.gearDir;

    for (let i = 0; i < 4; ++i) {
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
        } else {
          slipAccelerate = false;
          slipOppose = false;
        }
      } else {
        const hi = vRef * (1.0 + TV.SlipRatio);
        const lo = vRef * (1.0 - TV.SlipRatio);
        if (cmdAssistMoving && wi > hi) slipAccelerate = true;
        if (cmdOpposeMoving && wi < lo) slipOppose = true;
      }

      if (slipAccelerate) tightenAssistCap(ci, assistSign, minT, TV.SlipDownFactor, st, i);
      if (slipOppose)     tightenOpposeCap(ci, assistSign, minT, TV.SlipDownFactor, st, i);
    }

    for (let i = 0; i < 4; ++i) {
      sense.capFwd[i] = clamp(st.capFwd[i], minT, +hard);
      sense.capRev[i] = clamp(st.capRev[i], -hard, -minT);
    }
  })();

  // -----------------
  // ApplySpeedLimiter
  // -----------------
  (function ApplySpeedLimiter() {
    const band = TV.SpeedLimiterFadeBand;
    if (band <= 1e-3) return;

    const vRef = sense.vehicleSpeedAbs;
    const vMax = (currGear === GearD) ? TV.maxSpeedFwd : TV.maxSpeedRev;
    const vStart = vMax - band;
    const hiOutlier = vRef * (1.0 + TV.SlipRatio);

    for (let i = 0; i < 4; ++i) {
      let vForCap = vRef;
      const wi = sense.wAbs[i];

      if (vRef > TV.WheelMinRPM) {
        if (wi > hiOutlier && wi > vStart) vForCap = wi;
      }

      const capTarget = capFromSpeed(vForCap, vStart, vMax, TV.maxTorqueDrive);

      if (sense.gearDir > 0) {
        sense.capFwd[i] = Math.min(sense.capFwd[i], capTarget);
        sense.capFwd[i] = Math.max(sense.capFwd[i], TV.SlipMinTorque);
      } else {
        sense.capRev[i] = Math.max(sense.capRev[i], -capTarget);
        sense.capRev[i] = Math.min(sense.capRev[i], -TV.SlipMinTorque);
      }
    }
  })();

  // -----------------
  // ApplyBrakeFadeCaps (only if braking)
  // -----------------
  (function ApplyBrakeFadeCapsIfNeeded() {
    if (!(t < 0)) return;

    const vFade = TV.AntiReversingSpeed;
    if (vFade <= 1.0) return;

    const vEps = TV.WheelMinRPM;

    for (let i = 0; i < 4; ++i) {
      const wi = sense.wAbs[i];
      const ws = sense.w[i];

      if (wi >= vFade) continue;

      const fadeScale = wi / vFade;

      if (ws > vEps) {
        sense.capRev[i] = sense.capRev[i] * fadeScale;
      } else if (ws < -vEps) {
        sense.capFwd[i] = sense.capFwd[i] * fadeScale;
      } else {
        if (sense.gearDir > 0) sense.capRev[i] = 0.0;
        else                   sense.capFwd[i] = 0.0;
      }
    }
  })();

  // -----------------
  // ComputeTrajectoryIntent
  // -----------------
  const intent = (function ComputeTrajectoryIntent() {
    const steerNorm = (+s || 0) / 1000.0;
    const absS = Math.abs(steerNorm);

    const maxDrive = TV.maxTorqueDrive;
    const maxBrake = TV.maxTorqueBrake;

    const maxT = (t >= 0) ? maxDrive : maxBrake;
    const throttleInput = (+t || 0) * maxT / 1000.0;

    let Tc_total = 0.0;

    if (throttleInput >= 0) {
      Tc_total = throttleInput;
    } else {
      const bmag = -throttleInput;

      let msAlongGear = sense.motionSignAlongGear;
      if (Math.abs(msAlongGear) < 0.5) msAlongGear = +1.0;

      Tc_total = (-msAlongGear) * bmag;
    }

    const TcAbs = Math.abs(Tc_total);
    const full = (Tc_total >= 0) ? TV.FrontRearBiasFullTorqueDrive : TV.FrontRearBiasFullTorqueBrake;

    let uBias = (full > 1e-6) ? (TcAbs / full) : 1.0;
    uBias = clamp(uBias, 0.0, 1.0);

    let frontShare = 0.5;
    if (Tc_total >= 0) frontShare = lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
    else               frontShare = lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);

    frontShare = clamp(frontShare, 0.05, 0.95);

    const TcF = Tc_total * frontShare;
    const TcR = Tc_total * (1.0 - frontShare);

    const vFrontAbs = AxleSpeedAbs(sense, FL, FR);
    let TdF = computeFrontSteeringDiff(steerNorm, vFrontAbs);

    let TdR = 0.0;
    if (currGear === GearD && absS > 0.001) {
      const vRearAbs = AxleSpeedAbs(sense, RL, RR);
      TdR = computeRearYawAssist(steerNorm, absS, vRearAbs, Math.abs(TcR));
    }

    TdF = rateLimit(TdF, st.lastTdF, TV.MaxTorquePerTick);
    st.lastTdF = TdF;

    TdR = rateLimit(TdR, st.lastTdR, TV.MaxTorquePerTick);
    st.lastTdR = TdR;

    return { Tc_total, TcF, TcR, TdF, TdR };
  })();

  // -----------------
  // SolveWheelTorques
  // -----------------
  const hard = Math.max(TV.maxTorqueDrive, TV.maxTorqueBrake);

  const TcF_m = sense.gearDir * intent.TcF;
  const TcR_m = sense.gearDir * intent.TcR;

  const TdF_m = intent.TdF;
  const TdR_m = intent.TdR;

  const kFront = (TcF_m * sense.gearDir > 0) ? TV.TractionCorrectionASR : TV.TractionCorrectionABS;
  const kRear  = (TcR_m * sense.gearDir > 0) ? TV.TractionCorrectionASR : TV.TractionCorrectionABS;

  const front = SolvePairCaps(
    TcF_m, TdF_m,
    sense.capRev[FL], sense.capFwd[FL],
    sense.capRev[FR], sense.capFwd[FR],
    kFront
  );

  const rear = SolvePairCaps(
    TcR_m, TdR_m,
    sense.capRev[RL], sense.capFwd[RL],
    sense.capRev[RR], sense.capFwd[RR],
    kRear
  );

  const fl = clamp(front.L, -hard, +hard);
  const fr = clamp(front.R, -hard, +hard);
  const rl = clamp(rear.L,  -hard, +hard);
  const rr = clamp(rear.R,  -hard, +hard);

  // -----------------
  // OUTPUT (4 torques only; ignore hold flags / speed)
  // -----------------
  const out = Torques(
    truncTowardZero(fl),
    truncTowardZero(fr),
    truncTowardZero(rl),
    truncTowardZero(rr)
  );
  return out;
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
