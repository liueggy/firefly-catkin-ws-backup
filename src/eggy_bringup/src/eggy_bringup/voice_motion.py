"""Pure helpers for odometry-closed-loop voice micro motions."""

import math


def normalize_angle(angle):
    """Return *angle* wrapped to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def projected_progress(axis, start_pose, current_pose):
    """Project odometry displacement onto the robot's starting body axes."""
    start_x, start_y, start_yaw = start_pose
    current_x, current_y, current_yaw = current_pose
    dx = current_x - start_x
    dy = current_y - start_y
    if axis == "x":
        return dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
    if axis == "y":
        return -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
    if axis == "yaw":
        return normalize_angle(current_yaw - start_yaw)
    raise ValueError("unsupported motion axis: %s" % axis)


def target_reached(target, progress, tolerance):
    """Accept tolerance-band arrival and same-direction target crossing."""
    remaining = target - progress
    return abs(remaining) <= tolerance or target * remaining <= 0.0


def bounded_speed(remaining, maximum, minimum, gain):
    """Return a signed proportional command with minimum and maximum limits."""
    magnitude = min(abs(maximum), max(abs(minimum), abs(gain * remaining)))
    return math.copysign(magnitude, remaining)
