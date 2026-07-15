"""Pure helpers for the authoritative Eggy profile contract."""


PROFILE_MODES = {
    "mapping": "mapping_slam",
    "navigation": "static_nav",
    "inspection": "inspection",
}


def build_profile_contract(profile, observed, map_available=False):
    if profile not in PROFILE_MODES:
        raise ValueError("profile must be mapping, navigation or inspection")
    observed = dict(observed or {})
    required = {
        "mapping": ("gmapping",),
        "navigation": ("amcl", "move_base", "mission_runner", "camera"),
        "inspection": ("amcl", "move_base", "mission_runner", "camera",
                       "meter_detection", "kimi_server", "kimi_bridge"),
    }[profile]
    ready = all(bool(observed.get(name)) for name in required)
    if profile != "mapping" and not map_available:
        ready = False
    return {
        "schema_version": 1,
        "profile": profile,
        "mode": PROFILE_MODES[profile],
        "state": "ready" if ready else "degraded",
        "localizer": "gmapping" if profile == "mapping" else "amcl",
        "capabilities": {
            "initialpose": profile in ("navigation", "inspection"),
            "mapping": profile == "mapping",
            "navigation": profile in ("navigation", "inspection"),
            "inspection": profile == "inspection",
            "camera": profile in ("navigation", "inspection"),
            "camera_raw": profile in ("navigation", "inspection"),
            "camera_overlay": profile == "inspection",
            "meter_detection": profile == "inspection",
            "profile_switch": True,
            "profiles": {
                "mapping": True,
                "navigation": bool(map_available),
                "inspection": bool(map_available),
            },
        },
        "required": list(required),
        "observed": observed,
    }
