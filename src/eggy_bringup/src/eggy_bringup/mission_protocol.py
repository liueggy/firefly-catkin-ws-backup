"""Pure helpers for the Eggy mission JSON protocol (no ROS dependency)."""

import math
import uuid


def _required_bool(payload, key, default=False):
    value = payload.get(key, default)
    if not isinstance(value, bool):
        raise ValueError("%s must be a boolean" % key)
    return value


def normalize_mission_request(payload, default_frame="map", legacy_inspection=False):
    if not isinstance(payload, dict):
        raise ValueError("mission request must be a JSON object")
    schema_version = payload.get("schema_version", 1)
    if schema_version != 1:
        raise ValueError("unsupported schema_version: %s" % schema_version)
    command = str(payload.get("command", "start")).strip().lower()
    supplied_request_id = payload.get("request_id")
    request_id = str(supplied_request_id or uuid.uuid4())
    mission_type = str(payload.get("mission_type") or
                       ("inspection" if legacy_inspection else "navigation"))
    if mission_type not in ("navigation", "inspection"):
        raise ValueError("unsupported mission_type: %s" % mission_type)
    if command in ("cancel", "stop"):
        if not supplied_request_id:
            raise ValueError("cancel requires request_id")
        return {
            "schema_version": 1,
            "request_id": request_id,
            "mission_type": mission_type,
            "command": "cancel",
        }
    if command != "start":
        raise ValueError("unsupported mission command: %s" % command)

    raw_inspection = payload.get("inspection")
    if raw_inspection is None:
        raw_inspection = {"enabled": bool(legacy_inspection)}
    if not isinstance(raw_inspection, dict):
        raise ValueError("inspection must be an object")
    inspection_enabled = _required_bool(raw_inspection, "enabled", legacy_inspection)
    inspection = {
        "enabled": inspection_enabled,
        "vision_search": _required_bool(raw_inspection, "vision_search", True)
        if inspection_enabled else False,
        "ai_analysis": _required_bool(raw_inspection, "ai_analysis", True)
        if inspection_enabled else False,
    }

    raw_route = payload.get("route") or payload.get("waypoints") or []
    if not isinstance(raw_route, list) or not raw_route:
        raise ValueError("route must contain at least one waypoint")
    route = []
    point_ids = set()
    for index, raw in enumerate(raw_route):
        if not isinstance(raw, dict):
            raise ValueError("route[%d] must be an object" % index)
        try:
            x = float(raw["x"])
            y = float(raw["y"])
            yaw = float(raw.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("route[%d] has invalid pose: %s" % (index, exc))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("route[%d] pose must be finite" % index)
        item = dict(raw)
        item.update({
            "id": str(raw.get("id") or "wp_%02d" % (index + 1)),
            "frame_id": str(raw.get("frame_id") or default_frame),
            "x": x,
            "y": y,
            "yaw": yaw,
        })
        if not item["id"].strip():
            raise ValueError("route[%d] id must not be empty" % index)
        if item["id"] in point_ids:
            raise ValueError("duplicate waypoint id: %s" % item["id"])
        point_ids.add(item["id"])
        route.append(item)

    on_nav_failure = str(payload.get("on_nav_failure") or
                         ("stop" if payload.get("stop_on_nav_fail", True) else "skip"))
    if on_nav_failure not in ("stop", "skip"):
        raise ValueError("on_nav_failure must be stop or skip")
    return {
        "schema_version": 1,
        "request_id": request_id,
        "command": "start",
        "mission_type": mission_type,
        "loop": _required_bool(payload, "loop", False),
        "return_home": _required_bool(payload, "return_home", False),
        "on_nav_failure": on_nav_failure,
        "inspection": inspection,
        "route": route,
    }


def build_goal_pose_mission(frame_id, x, y, yaw, request_id=None):
    return normalize_mission_request({
        "schema_version": 1,
        "request_id": request_id or str(uuid.uuid4()),
        "command": "start",
        "mission_type": "navigation",
        "loop": False,
        "return_home": False,
        "on_nav_failure": "stop",
        "inspection": {"enabled": False},
        "route": [{
            "id": "single_goal_{:.2f}_{:.2f}".format(float(x), float(y)),
            "frame_id": frame_id or "map",
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw": round(float(yaw), 4),
        }],
    })
