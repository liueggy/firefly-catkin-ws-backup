"""Pure helpers for safe, single-flight runtime profile switching."""

import os
import threading
import uuid


class SwitchGate:
    """Reject overlapping mode changes while leaving status requests responsive."""

    def __init__(self):
        self._lock = threading.Lock()
        self._owner = ""

    def try_begin(self, request_id):
        if not self._lock.acquire(False):
            return False
        self._owner = str(request_id or "")
        return True

    def finish(self):
        self._owner = ""
        self._lock.release()

    @property
    def owner(self):
        return self._owner


def activate_map_directory(active_link, destination):
    """Atomically point active_link at a validated map directory.

    Returns the previous resolved destination, or an empty string when no
    active map existed.  The caller can use it for rollback.
    """

    destination = os.path.abspath(destination)
    for filename in ("map.yaml", "map.pgm", "metadata.json"):
        if not os.path.isfile(os.path.join(destination, filename)):
            raise ValueError("map directory is incomplete: %s" % filename)

    previous = os.path.realpath(active_link) if os.path.lexists(active_link) else ""
    temporary = "%s.tmp-%s" % (active_link, uuid.uuid4().hex)
    try:
        os.symlink(destination, temporary)
        if os.name == "nt" and os.path.lexists(active_link):
            # Windows cannot atomically replace a directory symlink.  The
            # deployed Linux path below still uses atomic rename semantics.
            os.unlink(active_link)
        os.replace(temporary, active_link)
    finally:
        if os.path.lexists(temporary):
            os.unlink(temporary)
    return previous


def restore_map_directory(active_link, previous):
    """Restore the active map link after a failed runtime transition."""

    if not previous:
        if os.path.lexists(active_link):
            os.unlink(active_link)
        return
    temporary = "%s.rollback-%s" % (active_link, uuid.uuid4().hex)
    try:
        os.symlink(previous, temporary)
        if os.name == "nt" and os.path.lexists(active_link):
            os.unlink(active_link)
        os.replace(temporary, active_link)
    finally:
        if os.path.lexists(temporary):
            os.unlink(temporary)
