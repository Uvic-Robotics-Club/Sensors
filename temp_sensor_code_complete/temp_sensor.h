/*
 * Header file for the TempSensor class
 * Author: Andrew Rose
 */
 
#pragma once //ensures that the file is included only once in a single compilation

#ifndef _temp_sensor_h
#define _temp_sensor_h

#include <Wire.h>

class TempSensor{
	 
	public:
		void begin(int addr);
		TempSensor();
		float bytes_to_celsius(uint8_t msb, uint8_t lsb);
		void command(int instruction);
		float get_temp();
    void write_float(float * temp_ptr);
    
  private:
		uint8_t data[2]; //temporary holding variable for each measurement
    int sensAddress; //address of sensor
};

#endif
