/*******************************************************************************
* Files: hw1.c, hw1_config.h, Makefile
* Author: Parker Brown
* Date: 11/15/2017
* Course: MAE 144, Fall 2017
* Description: Program runs closed feedback loop, minimizing angle error between
* left and right wheel of MIP. User spins right wheel, while left wheel tracks
* right wheel angle.
*******************************************************************************/

/*******************************************************************************
Part 1:
What are the units of the gyroscope and accelerometer readings displayed
with the rc_test_imu program?
* The gyroscope is in units of radians per second and the accelerometer is in units
of meters per second squared.

Referencing the API documentation, what are the
available full scale ranges of the gyroscope and accelerometer?
* The gyroscope has full scale ranges of 250 deg/s, 500 deg/s, 1000 deg/s, and 2000 deg/s.
The accelerometer has full scale ranges of 2G, 4G, 8G, and 16G.

For the default full
scale ranges used by test_imu, calculate the conversion rates from raw ADC to m/s2
and degrees/s for the accelerometer and gyroscope respectively.
* Since the imu has a 16 bit ADC built in, the conversion factor for the accel
is accel_FSR * 9.80665 / 32768.0 to get acceleration in meters per second squared.
2G FSR:  0.00059855041
4G FSR:  0.00119710083
8G FSR:  0.00239420166
16G FSR: 0.00478840332
* Also, the gyro conversion factor is gyro_FSR /32768.0 to get angular rate in
degrees per second.
250DPS FSR:  0.00762939453
500DPS FSR:  0.01525878906
1000DPS FSR: 0.03051757812
2000DPS FSR: 0.06103515625
