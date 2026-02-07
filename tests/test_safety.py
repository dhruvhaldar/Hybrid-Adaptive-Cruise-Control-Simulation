import unittest
from safety_monitor import SafetyMonitor

class TestSafetyMonitor(unittest.TestCase):

    def test_initialization(self):
        s = SafetyMonitor(safe_distance=10)
        self.assertEqual(s.safe_distance, 10)
        self.assertEqual(len(s.violations), 0)

    def test_check_safety(self):
        s = SafetyMonitor(safe_distance=10)
        self.assertTrue(s.check_safety(11, 0.0))
        self.assertFalse(s.check_safety(9, 1.0))
        self.assertEqual(len(s.violations), 1)
        self.assertEqual(s.violations[0], (1.0, 9))

if __name__ == '__main__':
    unittest.main()
