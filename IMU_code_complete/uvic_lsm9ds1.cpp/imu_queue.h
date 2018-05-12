/*
 * Header file for the IMUQueue class
 * for use with LSM9DS1
 * Author: Andrew Rose
 */

#pragma once

#ifndef _imu_queue_h
#define _imu_queue_h

class IMUQueue{

  public:
    void enqueue(int16_t element);
    int16_t getTail();
    int32_t getAvg();
    IMUQueue(int size);

  private:
    int tail;
    int16_t* array; //use malloc to create array of the proper size in .cpp
    int size;
}


#endif
