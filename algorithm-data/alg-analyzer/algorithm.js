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

// ✅ You edit this normally in VSCode (full syntax highlighting)
function TorqueVectoring(currGear, t, s, lastFront, lastRear) {
  if (currGear === Gear.N) return Torques(0, 0, 0, 0);

  let base = clamp(t, -1000, 1000);
  let bias = (clamp(s, -1000, 1000) / 1000) * 200;

  let left = base + bias;
  let right = base - bias;

  if (currGear === Gear.R) {
    left = -left;
    right = -right;
  }

  return Torques(
    clamp(Math.round(left / 2), -1000, 1000),
    clamp(Math.round(right / 2), -1000, 1000),
    clamp(Math.round(left / 2), -1000, 1000),
    clamp(Math.round(right / 2), -1000, 1000),
  );
}

// ✅ Provide the function source to the UI
window.getTorqueVectoringSource = () => TorqueVectoring.toString();
