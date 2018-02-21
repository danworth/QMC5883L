import smbus
import time
import math


class Compass:
    """ Class to control the QMC5883L digital compass."""

    ADDRESS = 0x0d  # Address of compass on bus.
    X_REGS = (0, 1)  # Regs for x low and x high bytes.
    Y_REGS = (2, 3)  # Regs for y low and y high bytes.
    Z_REGS = (4, 5)  # Regs for x low and z high bytes.
    DATA_STATE_REG = 6  # - |- |- |- |- |Data Skip|Overflow|Data Ready|
    CONTROL_REG_1 = 9  # Reg to control mode of compass.
    CONTROL_REG_2 = 10
    RESET_PERIOD_REG = 11  # Reg to control reset period.
    SMBUS = 1
    CALIBRATION_STEPS = 500

    # common set-up config.
    CONTINUOUS_MODE = 0b00011101  # Continuous mode at 200Hz, 8G and OSR = 512
    scale = 4.35  # based on guass being 8G.

    def __init__(self, x_offset=0, y_offset=0, declination=0, mode=CONTINUOUS_MODE):
        self.bus = smbus.SMBus(self.SMBUS)
        self.mode = mode
        self._set_up()
        self.x_calibration_offset = x_offset
        self.y_calibration_offset = y_offset
        self.declination = declination

    def _set_up(self):
        """ Turn on compass and set to continuous mode."""
        self._write_byte(self.RESET_PERIOD_REG, 0b01110000)
        self._write_byte(self.CONTROL_REG_2, 0b00100000)
        self._write_byte(self.CONTROL_REG_1, self.mode)

    def set_declination(self, degrees):
        """ Set the off-set for True North."""
        self.declination = degrees * (math.pi / 180)  # convert to Rads

    def _write_byte(self, adr, value):
        """ Write byte to address provided."""
        return self.bus.write_byte_data(self.ADDRESS, adr, value)

    def _read_byte(self, adr):
        """ Read byte from address provided."""
        return self.bus.read_byte_data(self.ADDRESS, adr)

    def _read_axis(self, axis_regs):
        # Value for each axis is provided by two registers each
        # providing 8bits, hence each axis value is 16bits.
        # Both bytes are combined and the value computed based
        # on two's compliment.
        low = self._read_byte(axis_regs[0])
        # shift high byte 8bits left.
        high = (self._read_byte(axis_regs[1]) << 8)
        value = high + low
        # if 1st bit is 1 indicates the value is negative. N.B. 0x8000 = 10000000 00000000.
        if value >= 0x8000:
            # Calculate two's compliment, value minus 1 shifted left by number of bits (16).
            return value - (1 << 16)
        else:
            return value

    def get_axes(self):
        """Returns the value of x, y and z axes"""

        # Last bit of DATA_STATE_REG indicates data available.
        # & 1 returns 1 if lsb it 1.
        data_ready = self._read_byte(self.DATA_STATE_REG) & 1
        if data_ready:
            self.x_axis = (self._read_axis(self.X_REGS) * self.scale) - \
                self.x_calibration_offset
            self.y_axis = (self._read_axis(self.Y_REGS) * self.scale) - \
                self.y_calibration_offset
            self.z_axis = self._read_axis(self.Z_REGS) * self.scale
            return (self.x_axis, self.y_axis, self.z_axis)
        else:
            return(None, None, None)

    def get_heading(self):
        """Returns heading in degrees and minutes."""
        (scaled_x, scaled_y, scaled_z) = self.get_axes()
        headingRad = math.atan2(scaled_y, scaled_x)
        headingRad += self.declination

        # Correct for reversed heading
        if(headingRad < 0):
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        if(headingRad > 2 * math.pi):
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = math.degrees(headingRad)
        degrees = math.floor(headingDeg)
        minutes = round(((headingDeg - degrees) * 60))
        return (degrees, minutes)

    def calibrate_xyraw(self, write_to_file=False):
        """Calibrates x and y axis so mid-point between max and min is 0."""
        x_vals = []
        y_vals = []
        z_vals = []

        # take 500 measurements.
        for i in range(0, self.CALIBRATION_STEPS):
            (x, y, z) = self.get_axes()
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)
            time.sleep(0.01)

        # calculate adjustments and z-deviation (to ensure sensor flat during test)
        x_adjustment = (max(x_vals) + min(x_vals)) / 2
        y_adjustment = (max(y_vals) + min(y_vals)) / 2
        z_deviation = max(z_vals) - min(z_vals)

        if write_to_file:
            with open("calibration_raw.txt", "w") as file:
                file.write("x,y,z\n")
                for i in range(len(x_vals)):
                    file.write("{0:.2f},{1:.2f},{2:.2f}\n".format(
                        x_vals[i], y_vals[i], z_vals[i]))

                file.write("x adjustment = {:0.2f} y adjustment = {:0.2f}\n".format(
                    x_adjustment, y_adjustment))
                file.write("z deviation = {:0.2f}".format(z_deviation))

        # update calibration for x and y.
        self.x_calibration_adjustment = x_adjustment
        self.y_calibration_adjustment = y_adjustment

        return(x_adjustment, y_adjustment, z_deviation, x_vals, y_vals)
