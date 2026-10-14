#!/usr/bin/env node

import assert from "node:assert/strict";
import fs from "node:fs";

const html = fs.readFileSync(
  new URL("./overlay.html", import.meta.url),
  "utf8",
);
const start = html.indexOf("function trajectorySnapshot(next)");
const end = html.indexOf("\n}", start) + 2;
assert.ok(start >= 0 && end > start, "trajectorySnapshot source missing");

let now = 10_000;
const evaluator = new Function(
  "Date",
  "clamp",
  `${html.slice(start, end)}; return trajectorySnapshot;`,
)(
  {now: () => now},
  (value) => Math.max(0, Math.min(1, value)),
);

const startAxes = [0, 1000, 2048, 3000, 4095];
const targetAxes = [4095, 3000, 2048, 1000, 0];

function evaluate(tau, elapsedMs, sensorAgeMs = 0) {
  now = 10_000 + elapsedMs;
  return evaluator({
    protocol_version: 4,
    trajectory_start_norm: startAxes,
    matrix_x_norm: targetAxes[0],
    matrix_y_norm: targetAxes[1],
    matrix_z_norm: targetAxes[2],
    pitch_norm: targetAxes[3],
    roll_norm: targetAxes[4],
    slew_tau_ms: tau,
    sensor_age_ms: sensorAgeMs,
    received_unix_ms: 10_000,
  }).map((value) => Math.round(value * 4095));
}

const cases = [
  [43, 0, [0, 1000, 2048, 3000, 4095]],
  [43, 16.667, [1316, 1643, 2048, 2357, 2779]],
  [43, 33.333, [2209, 2079, 2048, 1921, 1886]],
  [43, 66.667, [3226, 2576, 2048, 1424, 869]],
  [232, 0, [0, 1000, 2048, 3000, 4095]],
  [232, 16.667, [284, 1139, 2048, 2861, 3811]],
  [232, 33.333, [548, 1268, 2048, 2732, 3547]],
  [232, 66.667, [1023, 1500, 2048, 2500, 3072]],
  [420, 0, [0, 1000, 2048, 3000, 4095]],
  [420, 16.667, [159, 1078, 2048, 2922, 3936]],
  [420, 33.333, [312, 1153, 2048, 2847, 3783]],
  [420, 66.667, [601, 1294, 2048, 2706, 3494]],
];

for (const [tau, elapsed, expected] of cases) {
  assert.deepEqual(evaluate(tau, elapsed), expected);
}
assert.deepEqual(evaluate(232, 0, 125), [1706, 1833, 2048, 2167, 2389]);
assert.equal(evaluator({protocol_version: 4, trajectory_start_norm: startAxes,
  slew_tau_ms: 42}), null);
assert.equal(evaluator({protocol_version: 3, trajectory_start_norm: startAxes,
  slew_tau_ms: 232}), null);

console.log("OBS browser trajectory evaluator fixtures passed");
