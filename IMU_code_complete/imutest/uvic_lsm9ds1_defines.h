/*
 * uvic_lsm9ds1_types.h
 * Defines all types and enumerations for LSM9DS1 inertial measurement unit
 * This version modified by Andrew Rose of UVic Robotics
 * Original "beerware": LSM9DS1_Types.h by Jim Lindblom of SparkFun Electronics
 *    https://github.com/sparkfun/LSM9DS1_Breakout
 * 
 */

#ifndef _uvic_lsm9ds1_defines_h
#define _uvic_lsm9ds1_defines_h

//Addresses of accelerometer/gyroscope and magnetometer
#define AG_ADDR 0x6B
#define M_ADDR 0x1E

//Enabler constants. Loaded into registers to make things work
#define ACCEL_REG5_ENABLE 0b00111000 //No decimation; enables ZXY output of accelerometer
#define ACCEL_REG6_BASE 0b11100000 //to be ORed onto to determine accelerometer scale, bandwidth selection, and antialiasing filter bandwidth selection. Prevents accelerometer-only mode
  #define ACCEL_REG6_SCALE_2 0b00000000
  #define ACCEL_REG6_SCALE_4 0b00010000
  #define ACCEL_REG6_SCALE_8 0b00011000
  #define ACCEL_REG6_SCALE_16 0b00001000
  #define ACCEL_REG6_BW_SEL_ODR 0b00000000
  #define ACCEL_REG6_BW_SEL_BW_XL 0b00000100

//these three SCALE values correspond to 500, 2000, and 245 degrees per second
#define GYRO_REG1_SCALE_500 0b00001000
#define GYRO_REG1_SCALE_2000 0b00011000
#define GYRO_REG1_SCALE_245 0
#define GYRO_REG2_CONFIG 0b00000000 //see datasheet page 47 for filter configuration guide

#define GYRO_REG4_BASE 0b00111010 //enable all 3 dimensions, latch interrupts, and don't use 4-direction mode

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

// Queue Lengths
#define Q_LENGTH_ACCEL 360 //slightly more than 3 seconds' worth of values
#define Q_LENGTH_GYRO 360 //just over 3 seconds, again
#define Q_LENGTH_MAG 30 //3 seconds
#define Q_LENGTH_TEMP 60 //1 second

#endif
