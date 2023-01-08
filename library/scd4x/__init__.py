import time
import struct
from smbus2 import SMBus, i2c_msg

__version__ = '0.0.1.hg1'


class SCD4X:

    GET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2313
    GET_DATA_READY_STATUS = 0xE4B8
    GET_SENSOR_ALTITUDE = 0x2322
    GET_SERIAL_NUMBER = 0x3682
    GET_TEMPERATURE_OFFSET = 0x2318
    PERFORM_FACTORY_RESET = 0x3632
    PERFORM_FORCED_RECALIBRATION = 0x362F
    PERFORM_SELF_TEST = 0x3639
    PERSIST_SETTINGS = 0x3615
    READ_MEASUREMENT = 0xEC05
    REINIT = 0x3646  # SOFT_RESET
    SET_AMBIENT_PRESSURE = 0xE000
    SET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2416
    SET_SENSOR_ALTITUDE = 0x2427
    SET_TEMPERATURE_OFFSET = 0x241D
    START_LOW_POWER_PERIODIC_MEASUREMENT = 0x21AC
    START_PERIODIC_MEASUREMENT = 0x21B1
    STOP_PERIODIC_MEASUREMENT = 0x3F86

    DEFAULT_I2C_ADDRESS = 0x62

    def __init__(self, address=None, quiet=True, device="SCD4X"):

        self.quiet = quiet

        def q_print(msg: str):
            if not self.quiet:
                print(msg)

        self.co2 = 0
        self.temperature = 0
        self.relative_humidity = 0

        self.device = device
        self.address = self.DEFAULT_I2C_ADDRESS if address is None else address
        self.bus = SMBus(1)

        # self.stop_periodic_measurement()
        q_print(f"SCD4X __init__: getting serial number...")
        try:
            serial = self.get_serial_number()
        except OSError:
            q_print(f"{self.device} not responding")
            serial = None
        else:
            q_print(f"{self.device} Serial: {serial:06x}")
        self.serial = serial

    def rdwr(self, command, value=None, response_length=0, delay=0):
        if value is not None:
            msg_w = i2c_msg.write(self.address, struct.pack(">HHb", command, value, self.crc8(value)))
        else:
            msg_w = i2c_msg.write(self.address, struct.pack(">H", command))

        self.bus.i2c_rdwr(msg_w)

        time.sleep(delay / 1000.0)

        response_length *= 3

        if response_length > 0:
            msg_r = i2c_msg.read(self.address, response_length)
            self.bus.i2c_rdwr(msg_r)

            result = list(msg_r)
            data = []
            for chunk in range(0, len(result), 3):
                if self.crc8(result[chunk:chunk + 2]) != result[chunk + 2]:
                    raise ValueError("ICP10125: Invalid CRC8 in response.")
                data.append((result[chunk] << 8) | result[chunk + 1])
            if len(data) == 1:
                return data[0]
            else:
                return data

        return []

    def reset(self):
        """Resets to user settings from EEPROM"""
        self.rdwr(self.REINIT, delay=20)

    def factory_reset(self):
        """Reset to factory fresh condition.

        Resets user config in EEPROM.

        """
        self.stop_periodic_measurement()
        self.rdwr(self.PERFORM_FACTORY_RESET, delay=1200)

    def self_test(self):
        self.stop_periodic_measurement()
        response = self.rdwr(self.PERFORM_SELF_TEST, response_length=1, delay=10000)
        if response > 0:
            raise RuntimeError("Self test failed!")

    def measure(self, blocking=True, timeout=10):
        t_start = time.time()
        while not self.data_ready():
            if not blocking:
                return
            if time.time() - t_start > timeout:
                raise RuntimeError("Timeout waiting for data ready.")
            time.sleep(0.1)

        response = self.rdwr(self.READ_MEASUREMENT, response_length=3, delay=1)
        self.co2 = response[0]
        self.temperature = -45 + 175.0 * response[1] / (1 << 16)
        self.relative_humidity = 100.0 * response[2] / (1 << 16)

        return self.co2, self.temperature, self.relative_humidity, time.time()

    def data_ready(self):
        response = self.rdwr(self.GET_DATA_READY_STATUS, response_length=1, delay=1)
        return (response & 0x030F) != 0

    def get_serial_number(self):
        response = self.rdwr(self.GET_SERIAL_NUMBER, response_length=3, delay=1)
        return (response[0] << 32) | (response[1] << 16) | response[2]

    def start_periodic_measurement(self, low_power=False):
        if low_power:
            self.rdwr(self.START_LOW_POWER_PERIODIC_MEASUREMENT)
        else:
            self.rdwr(self.START_PERIODIC_MEASUREMENT)

    def stop_periodic_measurement(self):
        self.rdwr(self.STOP_PERIODIC_MEASUREMENT, delay=500)

    def set_ambient_pressure(self, ambient_pressure):
        self.rdwr(self.SET_AMBIENT_PRESSURE, value=ambient_pressure)

    def set_temperature_offset(self, temperature_offset):
        if temperature_offset < 374:
            raise ValueError("Temperature offset must be <= 374c")
        offset = int(temperature_offset * (1 << 16) / 175)
        self.rdwr(self.SET_TEMPERATURE_OFFSET, value=offset)

    def get_temperature_offset(self):
        response = self.rdwr(self.GET_TEMPERATURE_OFFSET, delay=1)
        return 175.0 * response / (2 << 16)

    def set_altitude(self, altitude):
        self.rdwr(self.SET_SENSOR_ALTITUDE, value=altitude)

    def get_altitude(self):
        return self.rdwr(self.GET_SENSOR_ALTITUDE, response_length=1, delay=1)

    @staticmethod
    def crc8(data, polynomial=0x31):
        if type(data) is int:
            data = [
                (data >> 8) & 0xff,
                data & 0xff
            ]
        result = 0xff
        for byte in data:
            result ^= byte
            for bit in range(8):
                if result & 0x80:
                    result <<= 1
                    result ^= polynomial
                else:
                    result <<= 1
        return result & 0xff


class SCD41(SCD4X):

    MEASURE_SINGLE_SHOT = 0x219D
    MEASURE_SINGLE_SHOT_RHT_ONLY = 0x2196
    POWER_DOWN = 0x36E0
    WAKE_UP = 0x36F6

    def __init__(self, address=None, quiet=True):
        if not quiet:
            print(f"SCD41 __init__(quiet={quiet}) ...")
        super().__init__(address=address, quiet=quiet, device="SCD41")

    def measure_single_shot(self):
        self.rdwr(self.MEASURE_SINGLE_SHOT)
        co2, temperature, relative_humidity, timestamp = self.measure(timeout=5005)
        return co2, temperature, relative_humidity, timestamp

    def measure_single_shot_rht_only(self):
        self.rdwr(self.MEASURE_SINGLE_SHOT_RHT_ONLY)
        co2, temperature, relative_humidity, timestamp = self.measure(timeout=5005)
        # co2 value is zero, discard
        return temperature, relative_humidity, timestamp

    def power_down(self):

        def q_print(msg: str):
            if not self.quiet:
                print(f"{self.device} power_down(), {msg}")

        q_print(f"powering down ...")
        self.rdwr(self.POWER_DOWN)

    def wake_up(self):

        def q_print(msg: str):
            if not self.quiet:
                print(f"{self.device} wake_up(), {msg}")

        q_print(f"waking up ...")
        try:
            self.rdwr(self.WAKE_UP)
        except OSError:
            q_print(f"failed to wake up")
            pass

        time.sleep(0.1)

        while not self.serial:
            q_print(f"getting serial number ...")
            try:
                self.serial = self.get_serial_number()  # verify idle state after wake_up command
            except OSError:
                q_print(f"failed to get serial")
                time.sleep(0.1)
            if self.serial:
                q_print(f"Serial: {self.serial:06x}")

        q_print(f"discard first reading ...")
        self.measure_single_shot()  # first reading using measure_single_shot after waking up should be discarded
