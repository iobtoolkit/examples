from iob import arduino_run
from iob.devices import ArduinoUnoLite
from time import sleep


dev = ArduinoUnoLite('/dev/ttyACM0')


def setup():
    dev.connect()
    dev.dio_pin_dir[13] = 1


def loop():
    dev.dio_pin_value[13] = True
    sleep(0.5)
    dev.dio_pin_value[13] = False
    sleep(0.5)


arduino_run()
