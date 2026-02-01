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
    SteerTorqueFront: 180.0,
    SteerTorqueRear: 150.0,
    SteerTorqueLowFactor: 1.0,
    SteerTorqueHighFactor: 0.7,
    SteerTorqueHighSpeed: 30.0,
    RearFadeSpeed0: 15.0,
    RearFadeSpeed1: 30.0,
    RearFadeTorque0: 50.0,
    RearFadeTorque1: 150.0,

    // Traction / ABS (per-wheel envelopes)
    SlipRatio: 0.20,
    SlipDownFactor: 0.70,
    SlipMinTorque: 50.0,
    SlipRecoverTorquePerTick: 50.0,
    WheelMinRPM: 35.0,

    TractionCorrectionASR: 0.75,
    TractionCorrectionABS: 0.25,

    maxRealisticAccel: 30.0,
    maxRealisticDecel: 40.0,

    // Front/rear torque bias
    DriveFrontShareLow: 0.50,
    DriveFrontShareHigh: 0.30,
    BrakeFrontShareLow: 0.50,
    BrakeFrontShareHigh: 0.80,
    FrontRearBiasFullTorqueDrive: 150.0,
    FrontRearBiasFullTorqueBrake: 300.0,

    // Braking near standstill (anti-reversing)
    AntiReversingSpeed: 60.0,
    AntiReversingHoldSpeed: 10.0,
  },
};
// =====================
// Output helper (given by you)
// =====================
function Torques(fl = 0, fr = 0, rl = 0, rr = 0) {
  return { fl: fl | 0, fr: fr | 0, rl: rl | 0, rr: rr | 0 };
}

// =====================
// TorqueVectoring(currGear, t, s, lastFront, lastRear)
// - currGear: 0=N, 1=D, 2=R
// - t: throttle in [-1000..1000]
// - s: steering in [-1000..1000]
// - lastFront/lastRear: optional feedback frames { speedL_meas,speedR_meas,cmdL,cmdR } or null
// - returns Torques(fl,fr,rl,rr)
// =====================
function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  // --- persistent state (mirrors TorqueVectoring member vars) ---
  const st = (TorqueVectoring._state ||= {
    vehicleSpeedAbs: 0.0,
    capsInit: false,
    capFwd: [0, 0, 0, 0],
    capRev: [0, 0, 0, 0],
    lastTdF: 0.0,
    lastTdR: 0.0,
  });

  const TV = params.TV;


  const rateLimit = (target, last, maxDelta) => {
    const d = target - last;
    if (d > maxDelta) return last + maxDelta;
    if (d < -maxDelta) return last - maxDelta;
    return target;
  };

  const wheelSign = (w, vEps) => {
    if (w > vEps) return +1;
    if (w < -vEps) return -1;
    return 0;
  };

  const tightenAssistCap = (cmdAtReading, assistSign, minT, downFactor, caps) => {
    const tSlip = Math.abs(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capFwd = Math.min(caps.capFwd, target);
    else caps.capRev = Math.max(caps.capRev, -target);
  };

  const tightenOpposeCap = (cmdAtReading, assistSign, minT, downFactor, caps) => {
    const tSlip = Math.abs(cmdAtReading);
    const target = Math.max(minT, tSlip * downFactor);
    if (assistSign > 0) caps.capRev = Math.max(caps.capRev, -target);
    else caps.capFwd = Math.min(caps.capFwd, target);
  };

  const capFromSpeed = (vForCap, vStart, vMax, maxTorque) => {
    if (vForCap <= vStart) return maxTorque;
    if (vForCap >= vMax) return 0.0;
    const u = (vForCap - vStart) / (vMax - vStart);
    return maxTorque * (1.0 - clamp(u, 0.0, 1.0));
  };

  const computeFrontSteeringDiff = (
    steering,
    vFrontAbs,
    steerTorqueHighSpeed,
    steerTorqueLowFactor,
    steerTorqueHighFactor,
    steerTorqueFront
  ) => {
    let uf = steerTorqueHighSpeed > 1.0 ? vFrontAbs / steerTorqueHighSpeed : 1.0;
    uf = clamp(uf, 0.0, 1.0);
    const kSteer = lerp(steerTorqueLowFactor, steerTorqueHighFactor, uf);
    return steering * steerTorqueFront * kSteer;
  };

  const computeRearYawAssist = (
    steering,
    absS,
    vRearAbs,
    rearEffAbs,
    rearFadeSpeed0,
    rearFadeSpeed1,
    rearFadeTorque0,
    rearFadeTorque1,
    steerTorqueRear
  ) => {
    let us = 1.0;
    if (rearFadeSpeed1 > rearFadeSpeed0) {
      us = (vRearAbs - rearFadeSpeed0) / (rearFadeSpeed1 - rearFadeSpeed0);
      us = clamp(us, 0.0, 1.0);
    }

    let uLong = 1.0;
    if (rearFadeTorque1 > rearFadeTorque0) {
      uLong = (rearEffAbs - rearFadeTorque0) / (rearFadeTorque1 - rearFadeTorque0);
      uLong = clamp(uLong, 0.0, 1.0);
    }

    // TODO: could be params.TV config (matches your C++ hardcoded constants)
    const s0 = 0.05;
    const s1 = 0.25;
    let uSteer = (absS - s0) / (s1 - s0);
    uSteer = clamp(uSteer, 0.0, 1.0);

    const fadeRear = us * Math.max(uLong, uSteer);
    const oppMag = absS * steerTorqueRear;

    // SIGN: s>0 => TdR>0 ; s<0 => TdR<0
    return (steering > 0 ? +oppMag : -oppMag) * fadeRear;
  };

  // Wheel indices
  const FL = 0,
    FR = 1,
    RL = 2,
    RR = 3;

  const AxleSpeedAbs = (sd, L, R) => {
    const okL = sd.ok[L],
      okR = sd.ok[R];
    if (okL && okR) return 0.5 * (sd.wAbs[L] + sd.wAbs[R]);
    if (okL) return sd.wAbs[L];
    if (okR) return sd.wAbs[R];
    return 0.0;
  };

  const SolvePairCaps = (TcReq, TdReq, capNegL, capPosL, capNegR, capPosR, k) => {
    // Traction-correcting Td
    const TdTraction = 0.5 * (capPosL - capPosR);
    const TdTarget = TdReq + k * TdTraction;

    // Feasible Td range
    const TdMin = 0.5 * (capNegL - capPosR);
    const TdMax = 0.5 * (capPosL - capNegR);

    const Td = clamp(TdTarget, TdMin, TdMax);

    // Given Td, feasible Tc range
    const TcMin = Math.max(capNegL - Td, capNegR + Td);
    const TcMax = Math.min(capPosL - Td, capPosR + Td);

    const Tc = clamp(TcReq, TcMin, TcMax);

    return {
      Tc,
      Td,
      TcMin,
      TcMax,
      L: Tc + Td,
      R: Tc - Td,
    };
  };

  // --- ReadWheelSpeeds ---
  const ReadWheelSpeeds = (sd) => {
    const readAxle = (fb, indexShift) => {
      if (!fb) return;
      sd.ok[indexShift] = true;
      sd.ok[indexShift + 1] = true;

      const sl = +fb.speedL_meas || 0;
      const sr = +fb.speedR_meas || 0;
      sd.w[indexShift] = sl;
      sd.w[indexShift + 1] = sr;
      sd.wAbs[indexShift] = Math.abs(sl);
      sd.wAbs[indexShift + 1] = Math.abs(sr);

      sd.cmdAtReading[indexShift] = +fb.cmdL || 0;
      sd.cmdAtReading[indexShift + 1] = +fb.cmdR || 0;
    };

    readAxle(lastFront, 0);
    readAxle(lastRear, 2);
  };

  // --- EstimateVehicleMotion ---
  const EstimateVehicleMotion = (sd, sc) => {
    // collect valid abs speeds WITH indices
    sc.sorted.length = 0;
    for (let i = 0; i < 4; i++) {
      if (sd.ok[i]) sc.sorted.push({ absSpeed: sd.wAbs[i], index: i });
    }
    sc.sorted.sort((a, b) => a.absSpeed - b.absSpeed);
    sc.numValid = sc.sorted.length;

    const n = sc.numValid;
    const vEps = TV.WheelMinRPM;

    let v_ref_meas = 0.0;
    let motionSign = 0.0;
    let refWheelIdx = -1;

    if (n === 0) {
      v_ref_meas = 0.0;
      refWheelIdx = -1;
    } else if (sc.accel) {
      const pick = n >= 2 ? 1 : 0;
      v_ref_meas = sc.sorted[pick].absSpeed;
      refWheelIdx = sc.sorted[pick].index;
    } else if (sc.brake) {
      const pick = n >= 2 ? n - 2 : n - 1;
      v_ref_meas = sc.sorted[pick].absSpeed;
      refWheelIdx = sc.sorted[pick].index;
    } else {
      // coasting: median abs speed (+ sign only if agreement)
      if (n === 1) {
        v_ref_meas = sc.sorted[0].absSpeed;
        refWheelIdx = sc.sorted[0].index;
      } else if (n === 2) {
        v_ref_meas = 0.5 * (sc.sorted[0].absSpeed + sc.sorted[1].absSpeed);
        const s0 = wheelSign(sd.w[sc.sorted[0].index], vEps);
        const s1 = wheelSign(sd.w[sc.sorted[1].index], vEps);
        refWheelIdx = s0 !== 0 && s0 === s1 ? sc.sorted[0].index : -1;
      } else if (n === 3) {
        v_ref_meas = sc.sorted[1].absSpeed;
        refWheelIdx = sc.sorted[1].index;
      } else {
        v_ref_meas = 0.5 * (sc.sorted[1].absSpeed + sc.sorted[2].absSpeed);
        const s1 = wheelSign(sd.w[sc.sorted[1].index], vEps);
        const s2 = wheelSign(sd.w[sc.sorted[2].index], vEps);
        refWheelIdx = s1 !== 0 && s1 === s2 ? sc.sorted[1].index : -1;
      }
    }

    // motionSign from ref wheel
    if (refWheelIdx >= 0 && v_ref_meas > vEps) {
      const wref = sd.w[refWheelIdx];
      motionSign = wref > 0 ? +1.0 : -1.0;
    } else {
      motionSign = 0.0;
    }

    // Standstill handling
    if (st.vehicleSpeedAbs < vEps) {
      if (sc.accel) {
        if (n > 0) v_ref_meas = sc.sorted[0].absSpeed; // smallest
      } else {
        v_ref_meas = 0.0;
        motionSign = 0.0;
      }
    }

    // Rate-limit vehicle speed changes
    const dv = v_ref_meas - st.vehicleSpeedAbs;
    if (dv > TV.maxRealisticAccel) st.vehicleSpeedAbs += TV.maxRealisticAccel;
    else if (dv < -TV.maxRealisticDecel) st.vehicleSpeedAbs -= TV.maxRealisticDecel;
    else st.vehicleSpeedAbs = v_ref_meas;

    sd.vehicleSpeedAbs = st.vehicleSpeedAbs;
    sd.motionSign = motionSign;
    sd.motionSignAlongGear = Math.abs(sd.motionSign) > 0.5 ? sd.motionSign * sd.gearDir : 0.0;
  };

  // --- UpdateSlipEnvelopes ---
  const UpdateSlipEnvelopes = (sd, sc) => {
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

    // recover towards +/- hard
    for (let i = 0; i < 4; i++) {
      st.capFwd[i] = Math.min(st.capFwd[i] + TV.SlipRecoverTorquePerTick, +hard);
      st.capFwd[i] = Math.max(st.capFwd[i], minT);

      st.capRev[i] = Math.max(st.capRev[i] - TV.SlipRecoverTorquePerTick, -hard);
      st.capRev[i] = Math.min(st.capRev[i], -minT);
    }

    const hasAnchor = sc.numValid > 0 && sc.sorted[0].absSpeed < vEps;
    const moving = Math.abs(sd.motionSign) > 0.5;
    const assistSign = moving ? sd.motionSign : sd.gearDir;

    for (let i = 0; i < 4; i++) {
      const wi = sd.wAbs[i];
      const ci = sd.cmdAtReading[i];

      const cmdAssistStandstill = ci * sd.gearDir > +minT;
      const effMotionSign = moving ? sd.motionSign : sd.gearDir;
      const cmdAssistMoving = ci * effMotionSign > +minT;
      const cmdOpposeMoving = ci * effMotionSign < -minT;

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

      const caps = { capFwd: st.capFwd[i], capRev: st.capRev[i] };
      if (slipAccelerate)
        tightenAssistCap(ci, assistSign, minT, TV.SlipDownFactor, caps);
      if (slipOppose)
        tightenOpposeCap(ci, assistSign, minT, TV.SlipDownFactor, caps);
      st.capFwd[i] = caps.capFwd;
      st.capRev[i] = caps.capRev;
    }

    // output caps (clamped)
    for (let i = 0; i < 4; i++) {
      sd.capFwd[i] = clamp(st.capFwd[i], minT, +hard);
      sd.capRev[i] = clamp(st.capRev[i], -hard, -minT);
    }
  };

  // --- ApplySpeedLimiter ---
  const ApplySpeedLimiter = (sd, gearDir) => {
    const band = TV.SpeedLimiterFadeBand;
    if (band <= 1e-3) return;

    const vRef = sd.vehicleSpeedAbs;
    const vMax = gearDir > 0 ? TV.maxSpeedFwd : TV.maxSpeedRev;
    const vStart = vMax - band;
    const hiOutlier = vRef * (1.0 + TV.SlipRatio);

    for (let i = 0; i < 4; i++) {
      let vForCap = vRef;
      const wi = sd.wAbs[i];

      if (vRef > TV.WheelMinRPM) {
        if (wi > hiOutlier && wi > vStart) vForCap = wi;
      }

      const capTarget = capFromSpeed(vForCap, vStart, vMax, TV.maxTorqueDrive);

      if (gearDir > 0) {
        sd.capFwd[i] = Math.min(sd.capFwd[i], capTarget);
        sd.capFwd[i] = Math.max(sd.capFwd[i], TV.SlipMinTorque);
      } else {
        sd.capRev[i] = Math.max(sd.capRev[i], -capTarget);
        sd.capRev[i] = Math.min(sd.capRev[i], -TV.SlipMinTorque);
      }
    }
  };

  // --- ApplyBrakeFadeCaps (only when braking) ---
  const ApplyBrakeFadeCaps = (sd) => {
    const vFade = TV.AntiReversingSpeed;
    if (vFade <= 1.0) return;

    const vEps = TV.WheelMinRPM;

    for (let i = 0; i < 4; i++) {
      const wi = sd.wAbs[i];
      const ws = sd.w[i];

      if (wi >= vFade) continue;

      const fadeScale = wi / vFade;

      if (ws > vEps) {
        // forward wheel motion -> brake is negative torque -> capRev toward 0
        sd.capRev[i] = sd.capRev[i] * fadeScale;
      } else if (ws < -vEps) {
        // backward wheel motion -> brake is positive torque -> capFwd toward 0
        sd.capFwd[i] = sd.capFwd[i] * fadeScale;
      } else {
        // truly stopped: clamp brake-side to 0 based on gear direction
        if (sd.gearDir > 0) sd.capRev[i] = 0.0;
        else sd.capFwd[i] = 0.0;
      }
    }
  };

  // --- Sense ---
  const Sense = (gearDir, accel, brake) => {
    const sd = {
      gearDir,
      ok: [false, false, false, false],
      w: [0, 0, 0, 0],
      wAbs: [0, 0, 0, 0],
      cmdAtReading: [0, 0, 0, 0],
      vehicleSpeedAbs: 0.0,
      motionSign: 0.0,
      motionSignAlongGear: 0.0,
      capFwd: [0, 0, 0, 0],
      capRev: [0, 0, 0, 0],
    };

    const sc = { accel, brake, sorted: [], numValid: 0 };

    ReadWheelSpeeds(sd);
    EstimateVehicleMotion(sd, sc);
    UpdateSlipEnvelopes(sd, sc);
    ApplySpeedLimiter(sd, gearDir);
    if (brake) ApplyBrakeFadeCaps(sd);

    return sd;
  };

  // --- ComputeTrajectoryIntent ---
  const ComputeTrajectoryIntent = (sd, gearDir) => {
    const steering = s / 1000.0; // -left, +right
    const absS = Math.abs(steering);

    const maxDrive = TV.maxTorqueDrive;
    const maxBrake = TV.maxTorqueBrake;

    const maxT = t >= 0 ? maxDrive : maxBrake;
    const throttleInput = (t * maxT) / 1000.0;

    let Tc_total_req = 0.0;

    if (throttleInput >= 0.0) {
      Tc_total_req = throttleInput; // drive in gear direction (world coords)
    } else {
      const bmag = -throttleInput;
      let msAlongGear = sd.motionSignAlongGear;
      if (Math.abs(msAlongGear) < 0.5) msAlongGear = +1.0;
      Tc_total_req = (-msAlongGear) * bmag; // no brakeScale here (matches latest C++)
    }

    // front/rear bias
    const TcAbs = Math.abs(Tc_total_req);
    const full = Tc_total_req >= 0 ? TV.FrontRearBiasFullTorqueDrive : TV.FrontRearBiasFullTorqueBrake;

    let uBias = full > 1e-6 ? TcAbs / full : 1.0;
    uBias = clamp(uBias, 0.0, 1.0);

    let frontShare = 0.5;
    if (Tc_total_req >= 0) frontShare = lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
    else frontShare = lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);

    frontShare = clamp(frontShare, 0.05, 0.95);

    const TcF_req = Tc_total_req * frontShare;
    const TcR_req = Tc_total_req * (1.0 - frontShare);

    // front steering differential
    const vFrontAbs = AxleSpeedAbs(sd, FL, FR);
    let TdF_req = computeFrontSteeringDiff(
      steering,
      vFrontAbs,
      TV.SteerTorqueHighSpeed,
      TV.SteerTorqueLowFactor,
      TV.SteerTorqueHighFactor,
      TV.SteerTorqueFront
    );

    // rear yaw assist (only in D)
    let TdR_req = 0.0;
    const isD = gearDir > 0;
    if (isD && absS > 0.001) {
      const vRearAbs = AxleSpeedAbs(sd, RL, RR);
      TdR_req = computeRearYawAssist(
        steering,
        absS,
        vRearAbs,
        Math.abs(TcR_req),
        TV.RearFadeSpeed0,
        TV.RearFadeSpeed1,
        TV.RearFadeTorque0,
        TV.RearFadeTorque1,
        TV.SteerTorqueRear
      );
    }

    // rate limit differentials
    TdF_req = rateLimit(TdF_req, st.lastTdF, TV.MaxTorquePerTick);
    st.lastTdF = TdF_req;

    TdR_req = rateLimit(TdR_req, st.lastTdR, TV.MaxTorquePerTick);
    st.lastTdR = TdR_req;

    return { Tc_total: Tc_total_req, TcF: TcF_req, TcR: TcR_req, TdF: TdF_req, TdR: TdR_req };
  };

  // --- SolveWheelTorques ---
  const SolveWheelTorques = (sd, intent) => {
    const TcF_req_m = sd.gearDir * intent.TcF;
    const TcR_req_m = sd.gearDir * intent.TcR;

    const TdF_req_m = intent.TdF; // do NOT flip
    const TdR_req_m = intent.TdR;

    const kFront = TcF_req_m * sd.gearDir > 0 ? TV.TractionCorrectionASR : TV.TractionCorrectionABS;
    const kRear = TcR_req_m * sd.gearDir > 0 ? TV.TractionCorrectionASR : TV.TractionCorrectionABS;

    const front = SolvePairCaps(
      TcF_req_m,
      TdF_req_m,
      sd.capRev[FL],
      sd.capFwd[FL],
      sd.capRev[FR],
      sd.capFwd[FR],
      kFront
    );

    const rear = SolvePairCaps(
      TcR_req_m,
      TdR_req_m,
      sd.capRev[RL],
      sd.capFwd[RL],
      sd.capRev[RR],
      sd.capFwd[RR],
      kRear
    );

    const hard = Math.max(TV.maxTorqueDrive, TV.maxTorqueBrake);

    return {
      fl: clamp(front.L, -hard, +hard),
      fr: clamp(front.R, -hard, +hard),
      rl: clamp(rear.L, -hard, +hard),
      rr: clamp(rear.R, -hard, +hard),
    };
  };

  // =====================
  // Main (mirrors Compute())
  // =====================
  if (currGear === 0) return Torques(); // N

  const gearDir = currGear === 1 ? +1.0 : -1.0; // 1=D, 2=R
  const accel = t > 0;
  const brake = t < 0;

  // === SENSE ===
  const sense = Sense(gearDir, accel, brake);

  // === TRAJECTORY INTENT ===
  const intent = ComputeTrajectoryIntent(sense, gearDir);

  // === SOLVE CONSTRAINTS ===
  const torques = SolveWheelTorques(sense, intent);

  // === OUTPUT (API fixed: only 4 torques) ===
  let out;
  out = Torques(torques.fl, torques.fr, torques.rl, torques.rr);
  return out;
}

// ✅ Provide the function source to the UI
window.getParamsSource = () =>
  `const params = ${JSON.stringify(params, null, 2)};\n`;
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
