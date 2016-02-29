from iob.devices import ArduinoLeonardo
from iob import I2CTransaction
from time import sleep
import struct


class Ds1721(object):
    """
    This class demonstrates how to communicate with DS1721 temperature
    sensor from Maxim Dallas.
    """

    BASE_ADDRESS = 0b10010000
    CMD_ACCESS_CONFIG = 0xAC
    CMD_START_CONVERT = 0x51
    CMD_STOP_CONVERT = 0x22
    CMD_READ_TEMP = 0xAA

    def __init__(self, device, chip_address):
        """
        Create DS1721 device.

        :param device: IOB device
        :param chip_address: DS1721 hand-wired device address bits as integer
        """
        self.device = device
        self.write_address = (self.BASE_ADDRESS | (chip_address << 1))
        self.read_address = self.BASE_ADDRESS | (chip_address << 1) | 0b00000001

    def connect(self):
        """
        Connect IOB device
        """
        self.device.connect()

    def disconnect(self):
        """
        Disconnect IOB device
        """
        self.device.disconnect()

    def configure(self):
        """
        Write configuration register.
        12-bit conversion accuracy
        1-shot mode to conserve power
        """
        # Boost clock speed to 400kHz
        self.device.i2c_clock = ArduinoLeonardo.I2CClock.F400k

        # Configure DS1720
        t = I2CTransaction()
        t.s().a(self.write_address).w(self.CMD_ACCESS_CONFIG).w(0b00001101).p()
        self.device.i2c_run(t)

    def read_config(self):
        """
        Read configuration register. Configuration register contains
        also status bit DONE.
        """
        t = I2CTransaction()
        t.s().a(self.write_address).w(self.CMD_ACCESS_CONFIG)
        t.rs().a(self.read_address).rn().p()
        ret = self.device.i2c_run(t)
        return ret.get_data(5)  # result of rn() command

    def is_done(self):
        """
        This is a wrapper over read_config() that extracts status bit DONE
        :return: True if conversion is done and result is ready, False otherwise
        """
        config_register = self.read_config()
        return (config_register & 0x80) != 0

    def read_temperature(self):
        """
        Read temperature. Temperature is read as 16-bit signer integer
        and must be converted to Celsius
        :return: Temperature in Celsius degrees
        """
        t = I2CTransaction()
        t.s().a(self.write_address).w(self.CMD_READ_TEMP)
        t.rs().a(self.read_address).ra().rn().p()
        ret = self.device.i2c_run(t)
        # 2 bytes read from DS1721
        temp_bytes = bytes([ret.get_data(5), ret.get_data(6)])
        # We use struct to correctly handle signed integer
        raw_temp = struct.unpack('>h', temp_bytes)[0]
        celsius = raw_temp / 256
        return celsius

    def start_conversion(self):
        """
        Trigger conversion. Conversion takes ~750ms.
        """
        t = I2CTransaction()
        t.s().a(self.write_address).w(self.CMD_START_CONVERT).p()
        self.device.i2c_run(t)


arduino = ArduinoLeonardo('/dev/ttyACM0')

dev = Ds1721(arduino, chip_address=0b111)
dev.connect()
dev.configure()
dev.start_conversion()

while True:

    if not dev.is_done():
        print('Conversion in progress...')
        sleep(0.20)
    else:
        temp = dev.read_temperature()
        print('Chip temperature: %s' % temp)
        break

dev.disconnect()
