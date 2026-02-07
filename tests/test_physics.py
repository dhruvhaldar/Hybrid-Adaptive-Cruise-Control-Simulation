import unittest
from vehicle import Vehicle

class TestVehicle(unittest.TestCase):

    def test_initialization(self):
        v = Vehicle(position=10, velocity=5, acceleration=1)
        self.assertEqual(v.x, 10)
        self.assertEqual(v.v, 5)
        self.assertEqual(v.a, 1)

    def test_update_zero_accel(self):
        v = Vehicle(velocity=10)
        v.update(dt=1.0, control_input=0.0)
        self.assertEqual(v.x, 10.0)  # x = x0 + v*dt = 0 + 10*1
        self.assertEqual(v.v, 10.0)  # v = v0 + a*dt = 10 + 0

    def test_update_constant_accel(self):
        v = Vehicle(velocity=0)
        # Update 1
        v.update(dt=1.0, control_input=2.0)
        # x = 0 + 0*1 = 0
        # v = 0 + 2*1 = 2
        self.assertEqual(v.x, 0.0)
        self.assertEqual(v.v, 2.0)

        # Update 2
        v.update(dt=1.0, control_input=2.0)
        # x = 0 + 2*1 = 2
        # v = 2 + 2*1 = 4
        self.assertEqual(v.x, 2.0)
        self.assertEqual(v.v, 4.0)

if __name__ == '__main__':
    unittest.main()
