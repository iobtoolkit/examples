from iob.devices import ArduinoLeonardo
from iob import I2CTransaction
from datetime import datetime, timedelta
from time import sleep
from unittest import TestCase


class Mcp7940n(object):

    CLOCK_BASE_ADDRESS = 0x00
    CONTROL_ADDRESS = 0x07
    ALARM_BASE_ADDRESS = 0x0A
    ALARM_MATCH = 0b01110000
    ALARM_POL = 0b10000000
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

    @property
    def python_alarm(self):
        """
        Convert from RTC timestamp to Python datetime object.
        """
        # alarm has no year, so we substitute it with current year
        current_year = datetime.now().now().year
        date = self.get_rtc_time()
        dt = datetime(year=current_year,
                      month=date[5],
                      day=date[4],
                      hour=date[2],
                      minute=date[1],
                      second=date[0])
        return dt

    @python_alarm.setter
    def python_alarm(self, dt):
        """
        Convert Python datetime object to raw numbers acceptable by RTC alarm API.

        :param dt time to set as datetime.datetime object
        """
        self.set_rtc_alarm(
            hour=dt.hour,
            minute=dt.minute,
            second=dt.second,
            weekday=dt.weekday()+1,  # convert day from 0-6 to 1-7
            day=dt.day,
            month=dt.month,
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
        result = self.dev.i2c_run(read)
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


    @property
    def control(self):
        t = I2CTransaction()
        t.s()\
         .a(self.I2C_WRITE_ADDRESS)\
         .w(self.CONTROL_ADDRESS)\
         .rs()\
         .a(self.I2C_READ_ADDRESS)\
         .rn()\
         .p()
        result = self.dev.i2c_run(t)
        return result.get_data(5)

    @control.setter
    def control(self, byte):
        t = I2CTransaction()
        t.s()\
         .a(self.I2C_WRITE_ADDRESS)\
         .w(self.CONTROL_ADDRESS)\
         .w(byte)\
         .p()
        result = self.dev.i2c_run(t)
        return result.get_data(3)

    def set_rtc_alarm(self, hour, minute, second, weekday, day, month):
        """
        Low-level I2C driver prototype for setting alarm time.
        """
        time = [
            self._to_bcd(second) & 0b01111111,
            self._to_bcd(minute) & 0b01111111,   # no masking needed
            self._to_bcd(hour) & 0b00111111,     # mask 12/24 bit
            self.ALARM_POL | self.ALARM_MATCH | self._to_bcd(weekday),  # combine polarization and match mode with day
            self._to_bcd(day),
            self._to_bcd(month),
        ]
        write = I2CTransaction()
        write.s()\
             .a(self.I2C_WRITE_ADDRESS)\
             .w(self.ALARM_BASE_ADDRESS)
        for byte in time:
            write.w(byte)
        write.p()
        result = self.dev.i2c_run(write)
        # uncomment to debug write operations
        # print('Date write:\n%s' % result.description)

    def get_rtc_alarm(self):
        """
        Low-level I2C driver prototype for reading alarm time from RTC.
        """
        read = I2CTransaction()
        # opeartions 0-4 are just low-level boilerplate
        read.s()\
            .a(self.I2C_WRITE_ADDRESS)\
            .w(self.ALARM_BASE_ADDRESS)\
            .rs()\
            .a(self.I2C_READ_ADDRESS)
        # operations 5-10 will clock data out from RTC module
        read.ra(5).rn()
        read.p()
        result = self.dev.i2c_run(read)
        # uncomment to debug read operations
        # print('Date read:\n%s' % result.description)
        bcd_date = [
            result.get_data(5) & 0b01111111,  # seconds
            result.get_data(6) & 0b01111111,  # minutes
            result.get_data(7) & 0b00111111,  # hours, 24hrs clock
            result.get_data(8) & 0b00000111,  # day of week
            result.get_data(9) & 0b00111111,  # day of month
            result.get_data(10) & 0b00011111,  # month without LP bit
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


class TestRtc(TestCase):
    """
    This test ensures that our RTC module is configured properly and our Python "driver"
    works as expected in terms of I2C I/O.
    """
    def setUp(self):
        self.dev = ArduinoLeonardo('/dev/ttyACM0')
        self.rtc = Mcp7940n(self.dev)
        self.rtc.connect()

    def tearDown(self):
        self.rtc.disconnect()

    def test_set_time(self):
        curr = datetime.now()
        curr_timestamp = int(curr.timestamp())
        self.rtc.python_time = curr
        rtc_timestamp = self.rtc.python_time.timestamp()
        self.assertAlmostEquals(curr_timestamp, rtc_timestamp)

    def test_clock_is_ticking(self):
        initial_time = datetime.now()
        self.rtc.python_time = initial_time

        sleep(6)

        # after 6 seconds our RTC should tick few times
        # read time back from RTC and calculate time difference between RTC and system time
        rtc_timestamp = self.rtc.python_time.timestamp()
        curr_timestamp = datetime.now().timestamp()
        rtc_sys_diff = abs(rtc_timestamp - curr_timestamp)

        # Time difference can be up to 2000ms:
        # 1. RTC is not counting milliseconds (rounding error up to 1000ms)
        # 2. We may be unfortunate to start I2C write on second-boundary (like 0.999)
        # I'm not going to do a proper synchronization here, instead I'll accept this error.
        self.assertTrue(rtc_sys_diff <= 2, "RTC time differs too much: %s. The clock is not ticking?" % rtc_sys_diff)


class TestRtcAlarm(TestCase):
    """
    This test uses Arduino Micro I2C bus to enable MCP7940N RTC Alarm
    and digital I/O port to test RTC behavior. We expect RTC to properly
    assert MFP pin depending on alarm state.

    You can imagine any other test scenario, such as proper voltage level on
    DAC output, etc.
    """
    def setUp(self):
        self.dev = ArduinoLeonardo('/dev/ttyACM0')
        self.rtc = Mcp7940n(self.dev)
        self.rtc.connect()

        # To ensure proper test isolation we should physically reset RTC module.
        # We could use DIO pin to power-cycle whole device, but for this case
        # it is sufficient to set control register to default value.
        self.rtc.control = 0x80

    def tearDown(self):
        self.rtc.disconnect()

    def test_set_alarm(self):
        """Check if alarm register is set properly"""
        curr = datetime.now()
        curr_timestamp = int(curr.timestamp())  # round to full seconds
        self.rtc.python_alarm = curr
        rtc_timestamp = self.rtc.python_alarm.timestamp()
        self.assertAlmostEquals(curr_timestamp, rtc_timestamp)

    def test_alarm_triggers_mfp(self):
        """Verify that MFP voltage is set properly when alarm is triggered"""
        current_time = datetime.now()
        # set current time and alarm in near future
        self.rtc.python_time = current_time
        alarm_time = current_time + timedelta(seconds=3)
        self.rtc.python_alarm = alarm_time

        # by default, when Alarm is not enabled, MFP state is HIGH
        self.assertTrue(self.dev.dio_pin_value[13], 'Alarm not set, but MFP is not in default state HIGH')

        control = self.rtc.control
        control |= 0b00010000  # BIT4 - enable alarm 0
        self.rtc.control = control
        self.assertEquals(self.rtc.control, control, 'Alarm 0 is not enabled')

        # When alarm 0 is set, MFP should be set to LOW
        self.assertFalse(self.dev.dio_pin_value[13], 'Alarm is set, but MFP is not LOW (not triggered)')

        # Wait for alarm
        sleep(5)

        # MFP should go high
        self.assertTrue(self.dev.dio_pin_value[13], 'Alarm should set MFP HIGH')
