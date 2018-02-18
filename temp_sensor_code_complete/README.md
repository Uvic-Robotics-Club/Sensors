A header, source, and tester file for running the Silicon Labs Si7055 temperature sensor as a slave to an Arduino via I2C. Contains everything needed to send Celsius temperature measurements over serial.

Can be used to send measurements in single-byte degree precision or in four-byte float precision. Note that the implementation of float transmission sends the four bytes in reverse order.
