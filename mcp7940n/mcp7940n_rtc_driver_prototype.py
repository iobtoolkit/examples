from iob.devices import ArduinoLeonardo
from iob import I2CTransaction
from datetime import datetime
from time import sleep


class Mcp7940n(object):

    CLOCK_BASE_ADDRESS = 0x00
    I2C_READ_ADDRESS = 0b11011111
    I2C_WRITE_ADDRESS = 0b11011110
    START_BIT = 0b10000000

    def __init__(self, device):
        self.dev = device

    def connect(self):
        self.dev.connect()
        self.dev.i2c_enable = True

    def disconnect(self):
        self.dev.i2c_enable = False
        self.dev.disconnect()

    @property
    def python_time(self):
        """
        Convert from RTC timestamp to Python datetime object.
        """
        date = self.get_rtc_time()
        dt = datetime(year=2000+date[6],
                      month=date[5],
                      day=date[4],
                      hour=date[2],
                      minute=date[1],
                      second=date[0])
        return dt

    @python_time.setter
    def python_time(self, dt):
        """
        Convert Python datetime object to raw numbers acceptable by RTC.

        :param dt tiem to set as datetime.datetime object
        """
        self.set_rtc_time(
            hour=dt.hour,
            minute=dt.minute,
            second=dt.second,
            weekday=dt.weekday()+1,  # convert day from 0-6 to 1-7
            day=dt.day,
            month=dt.month,
            year=dt.year - 2000      # rtc stores last 2 digits of the year
        )

    def set_rtc_time(self, hour, minute, second, weekday, day, month, year):
        """
        Low-level I2C driver prototype for setting time.
        """
        time = [
            self.START_BIT | (self._to_bcd(second) & 0b01111111),
            self._to_bcd(minute),                # no masking needed
            self._to_bcd(hour) & 0b00111111,     # mask 12/24 bit
            self._to_bcd(weekday) & 0b00000111,  # mask control bits,
            self._to_bcd(day),                   # no masking needed
            self._to_bcd(month) & 0b00011111,    # don't write LP bit
            self._to_bcd(year)                   # don't mask anything
        ]
        write = I2CTransaction()
        write.s()\
             .a(self.I2C_WRITE_ADDRESS)\
             .w(self.CLOCK_BASE_ADDRESS)
        for byte in time:
            write.w(byte)
        write.p()
        result = self.dev.i2c_run(write)
        # uncomment to debug write operations
        # print('Date write:\n%s' % result.description)

    def get_rtc_time(self):
        """
        Low-level I2C driver prototype for reading time from RTC.
        """
        read = I2CTransaction()
        # opeartions 0-4 are just low-level boilerplate
        read.s()\
            .a(self.I2C_WRITE_ADDRESS)\
            .w(self.CLOCK_BASE_ADDRESS)\
            .rs()\
            .a(self.I2C_READ_ADDRESS)
        # operations 5-11 will clock data out from RTC module
        read.ra(6).rn()
        read.p()
        result = dev.i2c_run(read)
        # uncomment to debug read operations
        # print('Date read:\n%s' % result.description)
        bcd_date = [
            result.get_data(5) & 0b01111111,  # seconds
            result.get_data(6) & 0b01111111,  # minutes
            result.get_data(7) & 0b00111111,  # hours, 24hrs clock
            result.get_data(8) & 0b00000111,  # day of week
            result.get_data(9) & 0b00111111,  # day of month
            result.get_data(10) & 0b00011111,  # month without LP bit
            result.get_data(11),               # year is encoded as 8-bit
        ]
        date = [self._from_bcd(bcd) for bcd in bcd_date]
        return date

    @staticmethod
    def _to_bcd(number):
        if number < 0 or number > 99:
            raise ValueError('Cannot encode to BCD')
        return int(str(number), 16)

    @staticmethod
    def _from_bcd(bcd):
        tenths = ((bcd & 0xF0) >> 4)
        if tenths > 0x09:
            raise ValueError('Invalid tenths digit. Must be 0-9.')
        ones = bcd & 0x0F
        if ones > 0x09:
            raise ValueError('Invalid ones digit. Must be 0-9.')
        return tenths*10 + ones


dev = ArduinoLeonardo('/dev/ttyACM0')

rtc = Mcp7940n(dev)
rtc.connect()

system_time = datetime.now()
print('Setting current system time: %s' % system_time)
rtc.python_time = system_time
print('Time read back from rtc:     %s' % rtc.python_time)

print('\nCheck running time:')
for i in range(0, 5):
    sleep(1)
    system_time = datetime.now()
    rtc_time = rtc.python_time
    print('sys: %s vs rtc: %s' % (system_time, rtc_time))

rtc.disconnect()
