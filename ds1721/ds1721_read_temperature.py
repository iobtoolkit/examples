from iob.devices.arduinouno import ArduinoUnoTwi
from iob.devices.arduinoleonardo import ArduinoLeonardo
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

    def __init__(self, device, chip_address=0b000):
        """
        Create DS1721 device.

        :param device: IOB device
        :param chip_address: DS1721 hand-wired device address bits as integer
        """
        self.device = device
        self.write_address = (self.BASE_ADDRESS | (chip_address << 1))
        self.read_address = self.BASE_ADDRESS | (chip_address << 1) | 0b00000001

    def configure(self):
        """
        Write configuration register.
        """

        # Configure DS1720
        self.device.i2c_start()
        self.device.i2c_address(self.write_address)
        self.device.i2c_write(self.CMD_ACCESS_CONFIG)

        # Configuration bits:
        # 0b00000001 - single shot
        # 0b00001100 - 12bit resolution
        self.device.i2c_write(0b00001101)
        self.device.i2c_stop()

    def read_config(self):
        """
        Read configuration register. Configuration register contains
        also status bit DONE.
        """
        # You can read configuration using the following transaction
        # t = I2CTransaction()
        # t.s().a(self.write_address).w(self.CMD_ACCESS_CONFIG)
        # t.rs().a(self.read_address).rn().p()
        # ret = self.device.i2c_run(t)
        # return ret.get_data(5)  # result of rn() command
        # Simpler API:
        self.device.i2c_start()
        self.device.i2c_address(self.write_address)
        self.device.i2c_write(self.CMD_ACCESS_CONFIG)
        self.device.i2c_rep_start()
        self.device.i2c_address(self.read_address)
        byte = self.device.i2c_read()
        self.device.i2c_stop()
        return struct.unpack('B', byte)[0]

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
        self.device.i2c_start()
        self.device.i2c_address(self.write_address)
        self.device.i2c_write(self.CMD_READ_TEMP)
        self.device.i2c_rep_start()
        self.device.i2c_address(self.read_address)
        temp_bytes = self.device.i2c_read(2)
        self.device.i2c_stop()
        # We use struct to correctly handle signed integer
        raw_temp = struct.unpack('>h', temp_bytes)[0]
        celsius = raw_temp / 256
        return celsius

    def start_conversion(self):
        """
        Trigger conversion. Conversion takes ~750ms.
        """

        # The code below is equivalent to the following transaction
        # t = I2CTransaction()
        # t.s().a(self.write_address).w(self.CMD_START_CONVERT).p()
        # r = self.device.i2c_run(t)

        self.device.i2c_start()
        self.device.i2c_address(self.write_address)
        self.device.i2c_write(self.CMD_START_CONVERT)
        self.device.i2c_stop()


if __name__ == "__main__":

    arduino = ArduinoUnoTwi('/dev/ttyACM0')
    #arduino = ArduinoLeonardo('/dev/ttyACM0')

    arduino.connect()
    arduino.i2c_enable = True
    arduino.i2c_verbose = False  # True will dump every I2C transaction to console

    dev = Ds1721(arduino, chip_address=0b111)

    print('*** Configure DS1721')
    dev.configure()

    print('*** Start conversion - it should take ~750ms')
    dev.start_conversion()

    while True:
        print('*** Checking conversion status...')
        if not dev.is_done():
            sleep(0.2)
        else:
            print('*** Reading remperature.')
            temp = dev.read_temperature()
            print('*** Chip temperature: %s' % temp)
            break

    arduino.disconnect()
