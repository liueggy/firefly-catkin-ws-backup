"""Pure priority/lease selection for Eggy's cmd_vel arbiter."""


def select_source(sources, now, blocked=False):
    if blocked:
        return None
    live = []
    for name, item in sources.items():
        if now - float(item["stamp"]) <= float(item["timeout"]):
            live.append((int(item["priority"]), float(item["stamp"]), name))
    return max(live)[2] if live else None
