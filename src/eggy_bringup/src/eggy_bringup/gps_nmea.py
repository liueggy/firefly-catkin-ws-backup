"""Small, dependency-free NMEA helpers used by the Eggy GPS node."""

import math


KNOT_TO_MPS = 0.5144444444444445


def valid_checksum(sentence):
    """Return True when an NMEA sentence has a valid XOR checksum."""
    text = sentence.strip()
    if not text.startswith("$") or "*" not in text:
        return False
    body, checksum = text[1:].rsplit("*", 1)
    if len(checksum) < 2:
        return False
    value = 0
    for char in body:
        value ^= ord(char)
    try:
        return value == int(checksum[:2], 16)
    except ValueError:
        return False


def _coordinate(value, hemisphere):
    if not value or hemisphere not in ("N", "S", "E", "W"):
        return None
    degree_digits = 2 if hemisphere in ("N", "S") else 3
    try:
        degrees = float(value[:degree_digits])
        minutes = float(value[degree_digits:])
    except (TypeError, ValueError):
        return None
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


def parse_sentence(sentence):
    """Parse the GGA/RMC/GSV fields needed by the lightweight driver."""
    text = sentence.strip()
    if not valid_checksum(text):
        return None
    fields = text[1:text.rfind("*")].split(",")
    kind = fields[0][-3:] if fields else ""
    try:
        if kind == "GGA" and len(fields) >= 10:
            quality = int(fields[6] or 0)
            return {
                "type": "GGA",
                "utc": fields[1],
                "latitude": _coordinate(fields[2], fields[3]),
                "longitude": _coordinate(fields[4], fields[5]),
                "fix_quality": quality,
                "fix": quality > 0,
                "satellites": int(fields[7] or 0),
                "hdop": float(fields[8]) if fields[8] else None,
                "altitude": float(fields[9]) if fields[9] else None,
            }
        if kind == "RMC" and len(fields) >= 10:
            speed = float(fields[7] or 0.0) * KNOT_TO_MPS
            course = float(fields[8]) if fields[8] else None
            return {
                "type": "RMC",
                "utc": fields[1],
                "fix": fields[2] == "A",
                "latitude": _coordinate(fields[3], fields[4]),
                "longitude": _coordinate(fields[5], fields[6]),
                "speed_mps": speed,
                "course_deg": course,
            }
        if kind == "GSV" and len(fields) >= 4:
            return {
                "type": "GSV",
                "sentences": int(fields[1] or 0),
                "sentence_index": int(fields[2] or 0),
                "satellites_visible": int(fields[3] or 0),
            }
    except (TypeError, ValueError, IndexError):
        return None
    return {"type": kind} if kind else None


def enu_velocity(speed_mps, course_deg):
    """Convert NMEA course (clockwise from north) to east/north velocity."""
    if course_deg is None:
        return 0.0, 0.0
    angle = math.radians(course_deg)
    return speed_mps * math.sin(angle), speed_mps * math.cos(angle)
