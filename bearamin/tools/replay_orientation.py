#!/usr/bin/env python3
"""Replay Bearamin matrix captures through the experimental orientation model."""

import argparse
import json
import math
import statistics
import zipfile
from pathlib import Path

RAYS = (-0.49314543, -0.33945426, -0.19891237, -0.06554346,
        0.06554346, 0.19891237, 0.33945426, 0.49314543)


def percentile(values, fraction):
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def solve3(matrix, vector):
    rows = [list(matrix[i]) + [vector[i]] for i in range(3)]
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(rows[row][pivot]))
        if abs(rows[best][pivot]) < 1e-6:
            return None
        rows[pivot], rows[best] = rows[best], rows[pivot]
        divisor = rows[pivot][pivot]
        rows[pivot] = [value / divisor for value in rows[pivot]]
        for row in range(3):
            if row == pivot:
                continue
            factor = rows[row][pivot]
            rows[row] = [
                rows[row][column] - factor * rows[pivot][column]
                for column in range(4)
            ]
    return [rows[row][3] for row in range(3)]


def fit(samples):
    if len(samples) < 12:
        return None
    points = [(depth * RAYS[xi], depth * RAYS[yi], float(depth))
              for xi, yi, depth in samples]
    if len({sample[0] for sample in samples}) < 3:
        return None
    if len({sample[1] for sample in samples}) < 3:
        return None
    weights = [1.0] * len(points)
    residuals = []
    scale = 1.0
    coefficients = None
    for robust_pass in range(5):
        normal = [[0.0] * 3 for _ in range(3)]
        rhs = [0.0] * 3
        for weight, (x, y, z) in zip(weights, points):
            row = (x, y, 1.0)
            for a in range(3):
                rhs[a] += weight * row[a] * z
                for b in range(3):
                    normal[a][b] += weight * row[a] * row[b]
        coefficients = solve3(normal, rhs)
        if coefficients is None:
            return None
        residuals = [
            z - (coefficients[0] * x + coefficients[1] * y + coefficients[2])
            for x, y, z in points
        ]
        center = statistics.median(residuals)
        scale = max(
            1.0,
            1.4826 * statistics.median(abs(value - center) for value in residuals),
        )
        if robust_pass < 4:
            weights = []
            for residual in residuals:
                unit = abs(residual - center) / (4.685 * scale)
                weights.append((1.0 - unit * unit) ** 2 if unit < 1.0 else 0.0)
    inlier_limit = max(3.0, 2.5 * scale)
    inliers = [
        (point, residual)
        for point, residual in zip(points, residuals)
        if abs(residual) <= inlier_limit
    ]
    if len(inliers) < 12 or len(inliers) / len(points) < 0.75:
        return None
    rms = math.sqrt(sum(residual * residual for _, residual in inliers) / len(inliers))
    if rms > 15.0:
        return None
    depths = [point[2] for point, _ in inliers]
    return {
        "roll": math.degrees(math.atan(coefficients[0])),
        "pitch": math.degrees(math.atan(coefficients[1])),
        "depth": statistics.median(depths),
        "rms": rms,
        "inliers": len(inliers),
        "points": len(points),
    }


class OneEuro:
    def __init__(self):
        self.initialized = False
        self.value = self.previous = self.derivative = 0.0

    def process(self, raw, delta):
        if not self.initialized or delta <= 0:
            self.initialized = True
            self.value = self.previous = raw
            self.derivative = 0.0
            return raw
        derivative_raw = (raw - self.previous) / delta
        self.derivative += self.alpha(1.0, delta) * (
            derivative_raw - self.derivative
        )
        cutoff = 0.8 + 0.03 * abs(self.derivative)
        self.value += self.alpha(cutoff, delta) * (raw - self.value)
        self.previous = raw
        return self.value

    @staticmethod
    def alpha(cutoff, delta):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / delta)


def selected_samples(frame):
    valid = []
    for index, (depth, status) in enumerate(
            zip(frame["cells_mm"], frame["statuses"])):
        row, column = divmod(index, 8)
        if row >= 2 and 1 <= depth <= 2000 and status in (5, 9):
            valid.append((row, column, depth))
    if not valid:
        return []
    music = [depth for _, _, depth in valid if 60 <= depth <= 705]
    nearest = min(music) if music else min(depth for _, _, depth in valid)
    limit = min(nearest + 250, 705)
    # Captures store logical sensor row/column. Instrument X is sensor row;
    # positive pitch runs from 6 o'clock toward noon, opposite sensor column.
    return [(row, 7 - column, depth)
            for row, column, depth in valid if depth <= limit]


def load_frames(path):
    with zipfile.ZipFile(path) as archive:
        return [json.loads(line) for line in
                archive.read("matrix.jsonl").decode().splitlines() if line]


def analyze(path):
    frames = load_frames(path)
    pitch_filter, roll_filter = OneEuro(), OneEuro()
    traces = []
    previous_us = None
    for frame in frames:
        estimate = fit(selected_samples(frame))
        if estimate is None:
            traces.append({"t": frame["core_us"] / 1e6, "confident": False})
            continue
        delta = 0.0 if previous_us is None else (frame["core_us"] - previous_us) / 1e6
        previous_us = frame["core_us"]
        traces.append({
            "t": frame["core_us"] / 1e6,
            "confident": True,
            **estimate,
            "pitch_filtered": pitch_filter.process(estimate["pitch"], delta),
            "roll_filtered": roll_filter.process(estimate["roll"], delta),
        })
    valid = [trace for trace in traces if trace["confident"]]
    summary = {"capture": Path(path).stem, "frames": len(frames),
               "confident_frames": len(valid)}
    for axis in ("pitch", "roll"):
        raw = [trace[axis] for trace in valid]
        filtered = [trace[f"{axis}_filtered"] for trace in valid]
        summary[f"{axis}_raw_p05_p95_deg"] = [
            round(percentile(raw, 0.05), 2), round(percentile(raw, 0.95), 2)]
        summary[f"{axis}_raw_range_deg"] = round(
            percentile(raw, 0.95) - percentile(raw, 0.05), 2)
        summary[f"{axis}_filtered_p05_p95_deg"] = [
            round(percentile(filtered, 0.05), 2),
            round(percentile(filtered, 0.95), 2)]
        summary[f"{axis}_filtered_range_deg"] = round(
            percentile(filtered, 0.95) - percentile(filtered, 0.05), 2)
    summary["rms_median_p95_mm"] = [
        round(statistics.median(trace["rms"] for trace in valid), 2),
        round(percentile([trace["rms"] for trace in valid], 0.95), 2),
    ]
    summary["median_depth_min_max_mm"] = [
        round(min(trace["depth"] for trace in valid), 1),
        round(max(trace["depth"] for trace in valid), 1),
    ]
    return traces, summary


def write_svg(results, output):
    width, panel_height, margin = 1000, 260, 44
    height = panel_height * len(results)
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#111"/>',
    ]
    colors = {"pitch": "#f0b84e", "roll": "#70d6d8"}
    for panel, (traces, summary) in enumerate(results):
        valid = [trace for trace in traces if trace["confident"]]
        t0, t1 = valid[0]["t"], valid[-1]["t"]
        y0 = panel * panel_height
        pieces.append(
            f'<text x="{margin}" y="{y0 + 24}" fill="white" '
            f'font-family="sans-serif">{summary["capture"]}</text>')
        for angle in (-45, 0, 45):
            y = y0 + 135 - angle * 2
            pieces.append(f'<line x1="{margin}" x2="{width-margin}" y1="{y}" y2="{y}" '
                          f'stroke="#333"/><text x="5" y="{y+4}" fill="#888" '
                          f'font-size="11">{angle}°</text>')
        for axis in ("pitch", "roll"):
            points = " ".join(
                f'{margin + (trace["t"]-t0)/(t1-t0)*(width-2*margin):.1f},'
                f'{y0 + 135 - trace[f"{axis}_filtered"]*2:.1f}'
                for trace in valid)
            pieces.append(
                f'<polyline points="{points}" fill="none" stroke="{colors[axis]}" '
                f'stroke-width="2"/>')
        pieces.append(
            f'<text x="{width-190}" y="{y0+24}" fill="{colors["pitch"]}">Pitch</text>'
            f'<text x="{width-100}" y="{y0+24}" fill="{colors["roll"]}">Roll</text>')
    pieces.append("</svg>")
    Path(output).write_text("\n".join(pieces) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("captures", nargs="+", type=Path)
    parser.add_argument("--svg", type=Path)
    args = parser.parse_args()
    results = [analyze(path) for path in args.captures]
    summaries = [summary for _, summary in results]
    by_name = {summary["capture"].lower(): summary for summary in summaries}
    if "pitch" in by_name:
        item = by_name["pitch"]
        item["filtered_cross_axis_leakage"] = round(
            item["roll_filtered_range_deg"] / item["pitch_filtered_range_deg"], 3)
    if "roll" in by_name:
        item = by_name["roll"]
        item["filtered_cross_axis_leakage"] = round(
            item["pitch_filtered_range_deg"] / item["roll_filtered_range_deg"], 3)
    print(json.dumps(summaries, indent=2))
    if args.svg:
        write_svg(results, args.svg)


if __name__ == "__main__":
    main()
