#pragma once //ensures that the file is included only once in a single compilation

#ifndef _uvic_lsm9ds1_h
#define _uvic_lsm9ds1_h

#include <Wire.h>
#include <queue>

#include "uvic_lsm9ds1_registers.h"
#include "uvic_lsm9ds1_types.h"
#include "uvic_lsm9ds1_defines.h"
#include "imu_queue.h"

class UVicLSM9DS1{

  public:
    
    masterSettings settings;

    //Bias arrays for the three dimensions for the gyro, accel and magnetometer
    //Will I really need the float versions? Maybe I can delete those later
    float gBias[3], aBias[3], mBias[3];
    int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
  
    float getVal(axis_spec axsp, stat_spec stsp, time_spec tisp, mode_spec mdsp);
    float[3] getTrio(stat_spec stsp, time_spec tisp, mode_spec mdsp); //return {getValx, getValy, getValz}

    //Constructor. No parameters required, since we are not touching SPI
    LSM9DS1();


    //////////////////////////////////
    //INITIALIZATION/SETUP FUNCTIONS//
    //////////////////////////////////

    // begin() -- Initialize the gyro, accelerometer, and magnetometer.
    //calls initGyro, initAccel, initMag
    // This will set up the scale and output rate of each sensor. The values set
    // in the masterSettings struct will take effect after calling this function.
    uint16_t begin();

    //Prevents any erroneous scale values from being assigned
    void ConstrainScales();

    // Calculate DPS / ADC tick, stored in gRes variable
    void calcgRes();
    // Calculate g / ADC tick, stored in aRes variable
    void calcaRes();
    // Calculate Gs / ADC tick, stored in mRes variable
    void calcmRes();

    
    //Interrupt configuration function. This is where the magic happens.
    void configInt(interrupt_select interrupt, uint8_t generator, h_lactive activeLow, pp_od pushPull);


    //These three setScale functions are only necessary if you must change the
    //scale values AFTER the initial setup!
    //Currently, the begin() function handles all this
    //but if you need to tweak it later for some reason, here you go
    void setGyroScale(uint16_t gScl);
    void setAccelScale(uint16_t aScl);
    void setMagScale(uint16_t mScl);


    //These setODR functions also are made redundant by the begin() method.
    //But if you need to tweak the ODRs of any parts of the IMU later, here you go...
    void setGyroODR(uint8_t gRate);
    void setAccelODR(uint8_t aRate);
    void setMagODR(uint8_t mRate);

    //Initialization functions. Self-explanatory
    void initAccel();
    void initGyro();
    void initMag();


    // enableFIFO() - Enable or disable the FIFO
    // Input:
    //  - enable: true = enable, false = disable.
    void enableFIFO(bool enable = true);
  
    // setFIFO() - Configure FIFO mode and Threshold
    // Input:
    //  - fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
    //    Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
    //  - fifoThs: FIFO threshold level setting
    //    Any value from 0-0x1F is acceptable.
    void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
  
    // getFIFOSamples() - Get number of FIFO samples
    uint8_t getFIFOSamples();

    void calibrateAG(bool autoCalc = true);
    //Add the below function if you gain access to the figure-8 kit
    //which you need to calibrate the magnetometer
    //It has not yet been written in the .cpp
    //void calibrateMag(bool loadIn = true);


    ///////////////////////////////////////
    //UNITS PER TICK CALCULATOR FUNCTIONS//
    ///////////////////////////////////////

    float calcGyro(int16_t gyro);
    float calcAccel(int16_t accel);
    float calcMag(int16_t mag);

    //////////////////////////
    //READ & WRITE FUNCTIONS//
    //////////////////////////

    //Note: this first function is called "ag" after the accel and gyro,
    //but it is used for a multitude of other write operations
    //such as writing to control registers
    void agWriteByte(uint8_t subAddr, uint8_t data);

    void WriteByte(uint8_t subAddr, uint8_t data);

    uint8_t agReadByte(uint8_t subAddr);

    uint8_t agReadBytes(uint8_t subAddr, uint8_t * dest, uint8_t count);

    uint8_t mReadByte(uint8_t subAddr);

    uint8_t mReadBytes(uint8_t subAddr, uint8_t * dest, uint8_t count);

    void readAccel();
    void readGyro();
    void readMag();
    void readTemp();

    ///////////////////////////////////
    //WIRE.H READ AND WRITE PROTOCOLS//
    ///////////////////////////////////
    //These interact with I2C in an entirely conventional, uninteresting manner
    //they are called by the accel/gyro, magnetometer, and temperature sensor-specific functions,
    //which provide the base addresses so these functions can do the actual work

    void UVicLSM9DS1::writeByte(uint8_t addr, uint8_t subAddr, uint8_t data);

    uint8_t UVicLSM9DS1::readByte(uint8_t addr, uint8_t subAddr);
  
    uint8_t UVicLSM9DS1::readBytes(uint8_t addr, uint8_t subAddr, uint8_t * dest, uint8_t count);

    //include a method that resets the speed when the robot stops (to avoid drifting values)

  protected:
  
   //ADD THE ARRAYS/QUEUES. AND ACCOUNT FOR THE MAGNETOMETER AND TEMPERATURE.
   //these are going to be RAW values that must be converted to actual integers via whatever formula in the get methods!
   //but is it possible to keep averages of the raw data?
   //would it be best to not store the averages, but instead live-calculate the average from the queues?
   //also: I definitely do not need the "now" variables. I can just use the tail of the queue
   //in which case the get method would also live-convert the value to its cooked format
  
   //I NEED TO WRITE ALL THE calcAccel, calcGyro, calcMag functions that turn raw data into usable stuff
   //no you don't! SparkFun has calcAccel/calcGyro/calcMag ready for you
   //all you need to do is run those functions on the results of IMUQueue.getTail or IMUQueue.getAvg
   IMUQueue q_accel_x, q_accel_y, q_accel_z;
   //int16_t accel_magnitude_now, accel_magnitude_avg; //these would be better off calculated when requested
   IMUQueue q_gyro_x, q_gyro_y, q_gyro_z;
   //int16_t gyro_magnitude_now, gyro_magnitude_avg; //these would be better off calculated when requested
   IMUQueue q_mag_x, q_mag_y, q_mag_z;
   //and finally the temperature queue
   IMUQueue q_temp;

   bool _autoCalc; //determines whether to automatically subtract accelerometer and gyroscope bias
  
   //Just a boring old parameter-less constructor
   //Since we will not be working with SPI as the original code provided for, there is no need for any parameters here
   UVic_LSM9DS1();
   
   //But the queues can't do everything. Cumulative items include a) POSITION and SPEED and b) ANGLE angle can be calculated from accel IF the robot is still
   //pos and speed need to be trios of variables for x/y/z
   //(angular VELOCITY is what's tracked by the gyroscope)
   //initial angle can be calculated using the measurements of the accelerometer when not in motion (see these calculations: http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
   //or, done as seen in LSM9DS1_Basic_I2C.ino

   //even when we're talking about the final, ultra-slick ABSOLUTE accel/gyro get functions, the principle holds: convert from raw in the get method

   //worrying about speed can be done in a future update! that's a more advanced feature



  /*
   * From os.mbed.com:
   * gyro readings are integrated to obtain orientation and accelerometer readings are integrated to obtain position!
   * 
   */

    //HEY! LSM9DS1_Basic_I2C.ino has ways to calculate roll, pitch & heading!
    //This is all done with the linear-acceleration and the magnetometer readings
    //This is a job for later
    //heading takes care of the initial zeroing effort...
    //heading also takes care of the horizontal orientation of the rover. Probably more easily than tracking it all fancily.
    //if we track heading using the magnetometer, then we only need to track gamma (angle from z-axis) using fancier methods.

}

#endif
