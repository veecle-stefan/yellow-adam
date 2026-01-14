// algorithm.js
// Provide ONE function in global scope:
//
//   TorqueVectoring(currGear, t, s, lastFront, lastRear) -> {fl,fr,rl,rr}
//
// Types (mirrors your C++ intent):
//   currGear: 0=N, 1=D, 2=R
//   t, s: int16-ish in [-1000..1000]
//   lastFront/lastRear: {
//      receivedTorqueL, receivedTorqueR, rpmL, rpmR
//   }
//
// IMPORTANT: lastFront/lastRear come from the SAME timestamp/row (as per your CSV),
// not delayed. That means you can treat them as “current feedback inputs”.

const Gear = Object.freeze({ N:0, D:1, R:2 });

function clamp(x, lo, hi){
  x = Number(x);
  if (!Number.isFinite(x)) return lo;
  return Math.max(lo, Math.min(hi, x));
}

function Torques(fl=0, fr=0, rl=0, rr=0){
  return { fl: fl|0, fr: fr|0, rl: rl|0, rr: rr|0 };
}

// ---- YOUR IMPLEMENTATION ----
function TorqueVectoring(currGear, t, s, lastFront, lastRear){
  // Example baseline (replace):
  if (currGear === Gear.N) return Torques(0,0,0,0);

  const base = clamp(t, -1000, 1000);
  const steer = clamp(s, -1000, 1000);

  // tiny steering bias toy
  const bias = (steer / 1000) * 200;

  let left  = base + bias;
  let right = base - bias;

  // optional: use rpm feedback from same timestamp
  // e.g. reduce torque if big left/right mismatch (toy)
  const frontDiff = Math.abs((lastFront.rpmL|0) - (lastFront.rpmR|0));
  const rearDiff  = Math.abs((lastRear.rpmL|0) - (lastRear.rpmR|0));
  const penaltyF = clamp(frontDiff / 5, 0, 150);
  const penaltyR = clamp(rearDiff  / 5, 0, 150);

  let fl = (left/2)  - penaltyF;
  let fr = (right/2) - penaltyF;
  let rl = (left/2)  - penaltyR;
  let rr = (right/2) - penaltyR;

  if (currGear === Gear.R){
    fl = -fl; fr = -fr; rl = -rl; rr = -rr;
  }

  return Torques(
    clamp(Math.round(fl), -1000, 1000),
    clamp(Math.round(fr), -1000, 1000),
    clamp(Math.round(rl), -1000, 1000),
    clamp(Math.round(rr), -1000, 1000),
  );
}