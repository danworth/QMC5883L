import sys
import unittest
import mock
sys.modules["smbus"] = mock.MagicMock()
import QMC5883L


class TestCompass(unittest.TestCase):

    def setUp(self):
        self.compass = QMC5883L.Compass()
        self.compass.bus = mock.MagicMock()

    def test_set_declination(self):
        self.compass.set_declination(90)
        self.assertEqual("{:0.2f}".format(self.compass.declination), str(1.57))

    def test_write_byte(self):
        self.compass._write_byte(1, 10)
        self.compass.bus.write_byte_data.assert_called_with(
            self.compass.ADDRESS, 1, 10)

    def test_read_byte(self):
        self.compass.bus.read_byte_data.return_value = 0b1
        self.assertEqual(self.compass._read_byte(1), 1)
        self.compass.bus.read_byte_data.assert_called_with(
            self.compass.ADDRESS, 1)

    def test_read_axis_positive_number(self):
        self.compass.bus.read_byte_data.side_effect = [0b00000001, 0b00000000]
        self.assertEqual(self.compass._read_axis(self.compass.X_REGS), 1)

    def test_read_axis_negative_number(self):
        self.compass.bus.read_byte_data.side_effect = [0b11111111, 0b11111111]
        self.assertEqual(self.compass._read_axis(self.compass.X_REGS), -1)

    def test_get_axes_data_ready(self):
        self.compass.scale = 1
        self.compass.bus.read_byte_data.side_effect = [0b00000001,  # Data ready.
                                                       0b00000001, 0b00000000,  # X-axis
                                                       0b00000010, 0b00000000,  # Y-axis
                                                       0b00000011, 0b00000000]  # Z-axis
        self.assertEqual(self.compass.get_axes(), (1, 2, 3))

    def test_get_axes_data_not_ready(self):
        self.compass.scale = 1
        # Data not ready.
        self.compass.bus.read_byte_data.side_effect = [0b00000000]
        self.assertEqual(self.compass.get_axes(), (None, None, None))

    def test_get_heading(self):
        self.compass.scale = 1
        self.compass.bus.read_byte_data.side_effect = [0b00000001,  # Date ready.
                                                       # X-axis.
                                                       0b00000001, 0b00000000,
                                                       # Y-axis.
                                                       0b00000010, 0b00000000,
                                                       0b00000011, 0b00000000]  # Z-axis.
        self.assertEqual(self.compass.get_heading(), (63, 26))

    def test_calibrate_xyraw(self):
        self.compass.scale = 1
        self.compass.CALIBRATION_STEPS = 1
        self.compass.bus.read_byte_data.side_effect = [0b00000001,  # Data ready.
                                                       # X-axis.
                                                       0b00000001, 0b00000000,
                                                       # Y-axis.
                                                       0b00000010, 0b00000000,
                                                       0b00000011, 0b00000000]  # Z-axis.
        self.assertEqual(self.compass.calibrate_xyraw(),
                         (1.0, 2.0, 0, [1], [2]))


if __name__ == '__main__':
    unittest.main()
