/*
 * Header file for the IMUQueue class
 * for use with LSM9DS1
 * Author: Andrew Rose
 */

#pragma once

#ifndef _imu_queue_h
#define _imu_queue_h

#include "Arduino.h"
#include "stdint.h"

class IMUQueue{

  public:
    void enqueue(int16_t element);
    int16_t getTail();
    int16_t getAvg();
    IMUQueue(int len);

  private:
    int tail;
    int16_t* arr; //use malloc to create array of the proper size in .cpp
    int q_len; 
};


#endif
