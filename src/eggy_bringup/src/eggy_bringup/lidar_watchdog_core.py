"""Pure state policy for recovering a live-but-stalled lidar process."""


class LidarWatchdogPolicy:
    def __init__(self, startup_grace=8.0, stale_timeout=2.0,
                 recovery_cooldown=10.0):
        self.startup_grace = float(startup_grace)
        self.stale_timeout = float(stale_timeout)
        self.recovery_cooldown = float(recovery_cooldown)
        self.process_pid = None
        self.process_started_at = None
        self.last_scan_at = None
        self.last_recovery_at = None

    def observe_process(self, pid, now):
        if pid == self.process_pid:
            return
        self.process_pid = pid
        self.process_started_at = float(now) if pid is not None else None
        self.last_scan_at = None

    def observe_scan(self, now):
        if self.process_pid is not None:
            self.last_scan_at = float(now)

    def scan_age(self, now):
        if self.last_scan_at is None:
            return None
        return max(0.0, float(now) - self.last_scan_at)

    def process_age(self, now):
        if self.process_started_at is None:
            return None
        return max(0.0, float(now) - self.process_started_at)

    def should_recover(self, now):
        now = float(now)
        if self.process_pid is None or self.process_started_at is None:
            return False
        if self.last_recovery_at is not None:
            if now - self.last_recovery_at < self.recovery_cooldown:
                return False
        if self.last_scan_at is None:
            return now - self.process_started_at > self.startup_grace
        return now - self.last_scan_at > self.stale_timeout

    def mark_recovery(self, now):
        self.last_recovery_at = float(now)
