import unittest

from eggy_bringup.lidar_watchdog_core import LidarWatchdogPolicy


class LidarWatchdogPolicyTest(unittest.TestCase):
    def test_waits_for_startup_grace_before_recovery(self):
        policy = LidarWatchdogPolicy(startup_grace=8.0, stale_timeout=2.0,
                                     recovery_cooldown=10.0)
        policy.observe_process(101, 0.0)

        self.assertFalse(policy.should_recover(7.9))
        self.assertTrue(policy.should_recover(8.1))

    def test_recovers_when_an_established_scan_stream_stalls(self):
        policy = LidarWatchdogPolicy(startup_grace=8.0, stale_timeout=2.0,
                                     recovery_cooldown=10.0)
        policy.observe_process(101, 0.0)
        policy.observe_scan(4.0)

        self.assertFalse(policy.should_recover(5.9))
        self.assertTrue(policy.should_recover(6.1))

    def test_does_not_recover_when_driver_process_is_absent(self):
        policy = LidarWatchdogPolicy(startup_grace=1.0, stale_timeout=1.0,
                                     recovery_cooldown=1.0)

        self.assertFalse(policy.should_recover(100.0))

    def test_new_driver_pid_gets_a_fresh_startup_grace(self):
        policy = LidarWatchdogPolicy(startup_grace=8.0, stale_timeout=2.0,
                                     recovery_cooldown=10.0)
        policy.observe_process(101, 0.0)
        policy.observe_scan(2.0)
        policy.observe_process(202, 20.0)

        self.assertFalse(policy.should_recover(27.9))
        self.assertTrue(policy.should_recover(28.1))

    def test_recovery_cooldown_prevents_restart_storms(self):
        policy = LidarWatchdogPolicy(startup_grace=1.0, stale_timeout=1.0,
                                     recovery_cooldown=10.0)
        policy.observe_process(101, 0.0)

        self.assertTrue(policy.should_recover(1.1))
        policy.mark_recovery(1.1)
        self.assertFalse(policy.should_recover(5.0))
        self.assertTrue(policy.should_recover(11.2))


if __name__ == "__main__":
    unittest.main()
