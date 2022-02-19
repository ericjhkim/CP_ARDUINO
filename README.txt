This code is for a cart pole balancing robot that uses a 6DOF IMU (XL and GYRO), a motor controller, a wheel encoder, and an Arduino Nano.

As per Arduino convention, make sure the external libraries are in the Arduino libraries folder (default location should be in your Documents folder)

sensor_test is an environment to test the IMU sensors as well as the motors.
cp_ann uses an artificial neural network controller.
cp_pid uses a PID controller.

External libraries (install as required):
Motor controller: https://github.com/pololu/qik-arduino
Averaging filter: https://github.com/kbowerma/arduino/blob/master/libraries/Average/Average.h
6DOF IMU: https://github.com/arduino-libraries/Arduino_LSM6DS3
