from iob.devices import ArduinoLeonardo
from iob import I2CTransaction

dev = ArduinoLeonardo('/dev/ttyACM0')
dev.connect()

dev.i2c_enable = True

print("I2C reset on bus error = %s" % dev.i2c_rst_on_error)
print("I2C initial status = %s" % dev.i2c_status)

print('\nWriting 7 bytes:')
write = I2CTransaction()
write.s().a(0b11011110).w(0x20)
for byte in [1, 2, 3, 4, 5, 6, 7]:
    write.w(byte)
write.p()

write_result = dev.i2c_run(write)
print(write_result.description)

print('\nReading 7 bytes:')
read = I2CTransaction()
read.s()\
    .a(0b11011110)\
    .w(0x20)\
    .rs()\
    .a(0b11011111)  # example of chained API
read.ra(6).rn()     # read 6 bytes with ACK and 7th byte with NACK
read.p()            # stop bus
read_result = dev.i2c_run(read)

# returned data is stored in transaction elements 5-12 inserted
# with commands ra(6) (inserted 6 read w/ ACK operations)
# and rn() (insert single read w/ NACK operation)
data = [read_result.get_data(i) for i in range(5, 12)]
print(read_result.description)
print("Read back: %s" % data)
dev.disconnect()
