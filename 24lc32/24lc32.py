# This module demonstrates use of new I2CHelper API, which is very suitable
# for interactive python sessions and any script that requires better
# readability.
#
# It requires a popular 24LC32 EEPROM. Please note, that 24LC32 comes in
# many variants and with different write buffer sizes. This example uses
# 32 byte write buffer by default, but you can change it to other value if
# needed.

from iob.devices.arduinouno import ArduinoUnoTwi
from iob import I2CTransaction
from iob.i2c import I2CHelperMixin
from time import time
import struct


class Eeprom24lc32(object):
    """
    This class demonstrates how to communicate with popular
    24LC32A EEPROM chip using I2C bus.
    """

    def __init__(self, device, user_chip_address=0b000, page_cache_size=32):
        """
        Create EEPROM object.

        :param iob.devices.Device  device: Device allowing I2C communication.
        :param int  user_chip_address: 24LC32 allows address cusomization using 3 bits.
        :param int  page_cache_size: Device page cache size, determining maximum size of single write operation.
        """
        if not isinstance(device, I2CHelperMixin):
            raise ValueError('This class works only with devices supporint I2C helper API')
        self.device = device
        self.chip_write_address = 0b10100000 | ((user_chip_address & 0b111) << 1)
        self.chip_read_address = 0b10100000 | ((user_chip_address & 0b111) << 1) | 0b00000001
        self.page_cache_size = page_cache_size

    def write(self, address, data):
        """
        Write data to device under specified memory address.

        :param int  address: target memory address
        :param bytes  data: bytes of data, no more than *page_cache_size*
        :raises I2CError: operation failed
        """
        if not isinstance(data, bytes) or len(data) > self.page_cache_size:
            msg = 'Expected data as 1-%s bytes. Got: %s, len = %s.'% (self.page_cache_size, type(data).__name__, len(data))
            raise ValueError(msg)
        address_word = struct.pack('>H', address & 0xFFF)
        self.device.i2c_start()
        self.device.i2c_address(self.chip_write_address)
        self.device.i2c_write(address_word)
        self.device.i2c_write(data)
        self.device.i2c_stop()

    def read(self, address, count=1):
        """
        Read bytes from address.

        :param int  address: Memory address
        :param count: Number of bytes to read
        :return: Bytes read from device.
        :raises I2CError: operation failed
        """
        address_word = struct.pack('>H', address & 0xFFF)
        self.device.i2c_start()
        self.device.i2c_address(self.chip_write_address)
        self.device.i2c_write(address_word)
        self.device.i2c_rep_start()
        self.device.i2c_address(self.chip_read_address)
        res = dev.i2c_read(count)
        self.device.i2c_stop()
        return res

    def wait(self, timeout=0.1):
        """
        Poll device until it's ready to accept new commands. According to 24LC32 datasheet,
        polling must be done by addressing the device with R/W bit set to 0 - this is write address.

        :param float  timeout: How long to wait for the device
        :return: True if device is ready, False otherwise
        """
        t = I2CTransaction()
        t.s().a(self.chip_write_address).p()
        deadline = time() + timeout
        while time() < deadline:
            r = self.device.i2c_run(t)
            if r.ack(1):
                return True
        return False

if __name__ == "__main__":

    # This example works with any device supporint I2CHelper API.
    dev = ArduinoUnoTwi('COM7')
    eeprom = Eeprom24lc32(dev)
    dev.connect()

    dev.i2c_enable = True

    # You can experiment with higher speeds, but it can fail ACK if you
    # use longer wires or inadequate pull-up resistors. 25kHz is a safe bet.
    dev.i2c_clock = ArduinoUnoTwi.I2CClock.F25k

    message = b'A secret message'

    print('Message size: %s' % len(message))

    print('Writing: %s...' % message, end='')
    eeprom.write(0, message)
    print('done')

    print('Waiting until write completes... ', end='')
    ready = eeprom.wait()
    print('done')

    print('Reading back... ', end='')
    r = eeprom.read(0, len(message))
    print('done')
    print('Got message: %s' % r)

    dev.i2c_enable = False
    dev.disconnect()
