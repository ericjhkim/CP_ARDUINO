This code is for a cart pole balancing robot that uses a 6DOF IMU (XL and GYRO), a motor controller, a wheel encoder, and an Arduino Nano.

As per Arduino convention, make sure the external libraries are in the Arduino libraries folder (default location should be in your Documents folder)

sensor_test is an environment to test the IMU sensors as well as the motors.
cp_ann uses an artificial neural network controller.
cp_pid uses a PID controller.

External libraries (install as required):
Motor controller: https://github.com/pololu/qik-arduino
Averaging filter: https://github.com/kbowerma/arduino/blob/master/libraries/Average/Average.h
6DOF IMU: https://github.com/arduino-libraries/Arduino_LSM6DS3

Version 2 for new Nano boards:
For the new Arduino Nano 33 IOT, SoftwareSerial.h is incompatible (and unnecessary). The board has a hardware serial on pins 0 and 1 (TX and RX).
The new Nano also has a built-in 6DOF IMU.
To interface the motor controller with the Nano, connect TX and RX (from motor controller) to pins 1 and 0 (on the Arduino) respectively.
The code in cp_pid_iot already contains the commands necessary to send motor controls through this hardware serial.
cp_pid_iot also uses the built-in IMU. The PID gains must be tuned for each unique robot.