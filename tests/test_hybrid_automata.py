import unittest
from controller import ACC_Controller

class TestACCController(unittest.TestCase):

    def setUp(self):
        self.c = ACC_Controller()

    def test_initial_state(self):
        self.assertEqual(self.c.state, ACC_Controller.CRUISE)

    def test_quantization(self):
        c = ACC_Controller(sensor_resolution=0.1)
        # Avoid .5 rounding issues by picking clearer values
        self.assertAlmostEqual(c.quantize_measurement(10.06), 10.1)
        self.assertAlmostEqual(c.quantize_measurement(10.04), 10.0)

        c2 = ACC_Controller(sensor_resolution=0.5)
        self.assertAlmostEqual(c2.quantize_measurement(10.2), 10.0)
        self.assertAlmostEqual(c2.quantize_measurement(10.3), 10.5)

    def test_state_transitions(self):
        c = ACC_Controller(safe_distance=10, time_gap=1.0)

        # 1. Cruise -> Follow
        # desired_following = 10 + 1.0 * 20 = 30
        # distance = 25 (< 30) -> Follow
        c.get_control_action(ego_v=20, lead_v=20, distance=25)
        self.assertEqual(c.state, ACC_Controller.FOLLOW)

        # 2. Follow -> Cruise
        # Hysteresis: > 30 * 1.1 = 33
        # distance = 34 -> Cruise
        c.get_control_action(ego_v=20, lead_v=20, distance=34)
        self.assertEqual(c.state, ACC_Controller.CRUISE)

        # 3. Cruise -> Emergency Brake
        # distance < 10 * 0.8 = 8
        # distance = 7 -> Emergency Brake
        c.get_control_action(ego_v=20, lead_v=20, distance=7)
        self.assertEqual(c.state, ACC_Controller.EMERGENCY_BRAKE)

    def test_delay(self):
        c = ACC_Controller(delay_steps=1)

        # Step 1: Input (10, 10, 100). Buffer: [(10, 10, 100)].
        # Uses buffer[0] -> (10, 10, 100). State -> CRUISE.
        c.get_control_action(ego_v=10, lead_v=10, distance=100)
        self.assertEqual(c.state, ACC_Controller.CRUISE)

        # Step 2: Input (10, 10, 5). Buffer: [(10, 10, 100), (10, 10, 5)].
        # Uses buffer[0] (oldest) -> (10, 10, 100). State stays CRUISE.
        # Even though distance is 5 (Emergency), the controller sees 100 due to delay.
        c.get_control_action(ego_v=10, lead_v=10, distance=5)
        self.assertEqual(c.state, ACC_Controller.CRUISE)

        # Step 3: Input (10, 10, 100). Buffer: [(10, 10, 5), (10, 10, 100)].
        # Uses buffer[0] -> (10, 10, 5). State -> EMERGENCY_BRAKE.
        c.get_control_action(ego_v=10, lead_v=10, distance=100)
        self.assertEqual(c.state, ACC_Controller.EMERGENCY_BRAKE)

if __name__ == '__main__':
    unittest.main()
