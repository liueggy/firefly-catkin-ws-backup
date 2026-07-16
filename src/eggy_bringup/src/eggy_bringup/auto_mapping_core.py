"""Pure, ROS-independent algorithms used by Eggy's automatic mapping stack."""

from __future__ import division

import math
from collections import deque


DEFAULT_SCORE_WEIGHTS = {
    "information_gain": 4.0,
    "clearance": 1.8,
    "path_length": 1.3,
    "visited": 2.0,
    "failed": 5.0,
}


def _distance(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _clearance_at(data, width, height, gx, gy, resolution, radius_m=1.5):
    radius_cells = max(1, int(math.ceil(radius_m / max(resolution, 1e-6))))
    best_sq = None
    x0 = max(0, gx - radius_cells)
    x1 = min(width - 1, gx + radius_cells)
    y0 = max(0, gy - radius_cells)
    y1 = min(height - 1, gy + radius_cells)
    for y in range(y0, y1 + 1):
        row = y * width
        for x in range(x0, x1 + 1):
            if data[row + x] < 50:
                continue
            dist_sq = (x - gx) ** 2 + (y - gy) ** 2
            if best_sq is None or dist_sq < best_sq:
                best_sq = dist_sq
    if best_sq is None:
        return radius_m
    return min(radius_m, math.sqrt(best_sq) * resolution)


def extract_frontier_clusters(data, width, height, resolution, origin_x, origin_y,
                              min_cluster_cells=8, max_clusters=80):
    """Return connected free/unknown boundaries as compact world-space candidates."""
    width = int(width)
    height = int(height)
    if width < 3 or height < 3 or len(data) != width * height:
        return []
    resolution = float(resolution)
    mask = bytearray(width * height)
    for y in range(1, height - 1):
        row = y * width
        for x in range(1, width - 1):
            idx = row + x
            if data[idx] != 0:
                continue
            if (data[idx - 1] < 0 or data[idx + 1] < 0 or
                    data[idx - width] < 0 or data[idx + width] < 0):
                mask[idx] = 1

    clusters = []
    neighbor_offsets = (-width - 1, -width, -width + 1, -1, 1,
                        width - 1, width, width + 1)
    for seed in range(width + 1, width * (height - 1) - 1):
        if not mask[seed]:
            continue
        mask[seed] = 0
        queue = deque([seed])
        cells = []
        while queue:
            idx = queue.popleft()
            cells.append(idx)
            x = idx % width
            for offset in neighbor_offsets:
                nxt = idx + offset
                nx = nxt % width
                if abs(nx - x) > 1 or nxt <= width or nxt >= width * (height - 1):
                    continue
                if mask[nxt]:
                    mask[nxt] = 0
                    queue.append(nxt)
        if len(cells) < int(min_cluster_cells):
            continue

        mean_x = sum(idx % width for idx in cells) / float(len(cells))
        mean_y = sum(idx // width for idx in cells) / float(len(cells))
        target = min(cells, key=lambda idx: (
            (idx % width - mean_x) ** 2 + (idx // width - mean_y) ** 2))
        gx = target % width
        gy = target // width
        clusters.append({
            "gx": gx,
            "gy": gy,
            "x": float(origin_x) + (gx + 0.5) * resolution,
            "y": float(origin_y) + (gy + 0.5) * resolution,
            "cell_count": len(cells),
            "information_gain": len(cells) * resolution * resolution,
            "clearance": _clearance_at(data, width, height, gx, gy, resolution),
        })
    clusters.sort(key=lambda item: item["cell_count"], reverse=True)
    return clusters[:max(1, int(max_clusters))]


def rank_frontiers(candidates, robot_xy, visited_points=None, failed_points=None,
                   weights=None, revisit_radius=0.8, failed_radius=0.9):
    visited_points = visited_points or []
    failed_points = failed_points or []
    merged_weights = dict(DEFAULT_SCORE_WEIGHTS)
    if weights:
        merged_weights.update(weights)
    ranked = []
    for source in candidates:
        item = dict(source)
        point = (item["x"], item["y"])
        path_length = float(item.get("path_length", _distance(point, robot_xy)))
        visited_penalty = sum(
            1.0 for seen in visited_points if _distance(point, seen) <= revisit_radius)
        failed_penalty = sum(
            1.0 for failed in failed_points if _distance(point, failed) <= failed_radius)
        information = math.sqrt(max(0.0, float(item.get("information_gain", 0.0))))
        clearance = min(1.5, max(0.0, float(item.get("clearance", 0.0))))
        score = (
            merged_weights["information_gain"] * information
            + merged_weights["clearance"] * clearance
            - merged_weights["path_length"] * path_length
            - merged_weights["visited"] * visited_penalty
            - merged_weights["failed"] * failed_penalty
        )
        item.update({
            "path_length": path_length,
            "visited_penalty": visited_penalty,
            "failed_penalty": failed_penalty,
            "score": score,
        })
        ranked.append(item)
    ranked.sort(key=lambda item: item["score"], reverse=True)
    return ranked


def dynamic_stop_distance(speed, base_clearance, latency_sec, deceleration, margin):
    speed = abs(float(speed))
    deceleration = max(0.05, float(deceleration))
    return (float(base_clearance) + speed * float(latency_sec)
            + speed * speed / (2.0 * deceleration) + float(margin))


def _scaled_component(value, distance, label, config):
    if abs(value) < 1e-6:
        return value, "clear"
    if distance is None or not math.isfinite(float(distance)):
        return 0.0, label + "_unknown"
    stop = dynamic_stop_distance(
        value, config["base_clearance"], config["latency_sec"],
        config["deceleration"], config["margin"])
    slow = stop + config["slow_band"]
    if distance <= stop:
        return 0.0, label + "_blocked"
    if distance < slow:
        scale = max(config["min_speed_scale"], (distance - stop) / max(0.01, slow - stop))
        return value * scale, label + "_slow"
    return value, "clear"


def safe_mapping_twist(linear_x, linear_y, angular_z, clearances, config=None):
    cfg = {
        "base_clearance": 0.18,
        "latency_sec": 0.15,
        "deceleration": 0.5,
        "margin": 0.05,
        "slow_band": 0.25,
        "min_speed_scale": 0.25,
        "rotation_clearance": 0.30,
    }
    if config:
        cfg.update(config)
    values = [float(linear_x), float(linear_y), float(angular_z)]
    checks = []
    if values[0] > 0:
        values[0], reason = _scaled_component(values[0], clearances.get("front"), "front", cfg)
        checks.append(reason)
    elif values[0] < 0:
        values[0], reason = _scaled_component(values[0], clearances.get("rear"), "rear", cfg)
        checks.append(reason)
    if values[1] > 0:
        values[1], reason = _scaled_component(values[1], clearances.get("left"), "left", cfg)
        checks.append(reason)
    elif values[1] < 0:
        values[1], reason = _scaled_component(values[1], clearances.get("right"), "right", cfg)
        checks.append(reason)
    if abs(values[2]) > 1e-6:
        rotation = clearances.get("rotation")
        if rotation is None or rotation <= cfg["rotation_clearance"]:
            return 0.0, 0.0, 0.0, "rotation_blocked"
    blocked = next((reason for reason in checks if reason.endswith("_blocked") or reason.endswith("_unknown")), None)
    if blocked:
        return 0.0, 0.0, 0.0, blocked
    slow = next((reason for reason in checks if reason.endswith("_slow")), None)
    return values[0], values[1], values[2], slow or "clear"


def completion_decision(elapsed_sec, known_cells, no_frontier_cycles, map_stable_sec,
                        min_elapsed_sec=60.0, min_known_cells=800,
                        required_no_frontier_cycles=3, required_stable_sec=30.0):
    if float(elapsed_sec) < float(min_elapsed_sec):
        return ""
    if int(known_cells) < int(min_known_cells):
        return ""
    if int(no_frontier_cycles) < int(required_no_frontier_cycles):
        return ""
    if float(map_stable_sec) < float(required_stable_sec):
        return ""
    return "map_complete"


def validate_mapping_request(payload):
    if not isinstance(payload, dict):
        raise ValueError("automatic mapping request must be a JSON object")
    if payload.get("schema_version", 1) != 1:
        raise ValueError("unsupported schema_version")
    command = str(payload.get("command", "status")).strip().lower()
    if command not in ("start", "pause", "resume", "cancel", "stop", "status"):
        raise ValueError("unsupported automatic mapping command: %s" % command)
    if command == "stop":
        command = "cancel"
    options = payload.get("options") or {}
    if not isinstance(options, dict):
        raise ValueError("options must be an object")

    def bounded(name, default, lower, upper):
        value = float(options.get(name, default))
        if not math.isfinite(value):
            raise ValueError("%s must be finite" % name)
        return max(lower, min(upper, value))

    def boolean(name, default):
        value = options.get(name, default)
        if not isinstance(value, bool):
            raise ValueError("%s must be a boolean" % name)
        return value

    normalized = {
        "schema_version": 1,
        "request_id": str(payload.get("request_id") or ""),
        "command": command,
        "options": {
            "max_duration_sec": bounded("max_duration_sec", 900.0, 30.0, 3600.0),
            "max_linear_speed": bounded("max_linear_speed", 0.22, 0.05, 0.30),
            "return_home": boolean("return_home", True),
            "save_draft_on_abort": boolean("save_draft_on_abort", True),
            "min_frontier_cells": int(bounded("min_frontier_cells", 8, 3, 200)),
        },
    }
    return normalized
