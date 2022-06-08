"""Unit tests for the ahrs in scion_utils.
"""
import unittest
import utils.scion_utils as scion_ut

tests = [
    [0.0, 0.0, 0.0],
    [-1, -1, -1],
    [1, 1, 1],
    [1, -1, 1],
    [-100, 100, -100],
    [-180.00000000, -180.00000000, -359.99999],  # Edge cases
    [180.000000000, 180.00000000, 359.99999]
]


class TestInput(unittest.TestCase):
    def test_0(self, test=0):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_1(self, test=1):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_2(self, test=2):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_3(self, test=3):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_4(self, test=4):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_5(self, test=5):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])

    def test_6(self, test=6):
        AHRS = scion_ut.AHRSDataWrapper(debug=False)
        AHRS.callback(data=f'P|{tests[test][0]}R|{tests[test][1]}Y|{tests[test][2]}')
        self.assertEqual(AHRS.pitch, tests[test][0])
        self.assertEqual(AHRS.roll, tests[test][1])
        self.assertEqual(AHRS.yaw, tests[test][2])


if __name__ == '__main__':
    unittest.main()
