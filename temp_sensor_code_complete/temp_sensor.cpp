/*
 * Source file for the TempSensor class
 * Author: Andrew Rose
 */

#include <Wire.h>
#include <Arduino.h>
#include "temp_sensor.h"

//initializes the object; no setup required for this simple scenario
TempSensor::TempSensor(){
}

//stores the address of the sensor
void TempSensor::begin (int addr){
	sensAddress = addr;
}

//converts raw data to a Celsius reading
float TempSensor::bytes_to_celsius(uint8_t msb, uint8_t lsb){
	return (float)((msb*256.0 + lsb)*175.72/65536.0 - 46.85);
}

//issues a command to the sensor
void TempSensor::command (int instruction){
	Wire.beginTransmission(sensAddress);
	Wire.write(instruction);
	Wire.endTransmission();
	delay(500);
}

//calls several helper functions to get a Celsius measurement from the sensor
float TempSensor::get_temp(){
  //tell sensor to take a temperature measurement
	command(0xF3);
  //request n bytes of data
  Wire.requestFrom(sensAddress, 2);
  //wait for bytes to become available
  while (Wire.available()<2);
  data[0] = Wire.read();
  data[1] = Wire.read();
	return bytes_to_celsius(data[0], data[1]);
}

//given a pointer to a float, writes its four bytes to serial IN REVERSE ORDER
void TempSensor::write_float(float * temp_ptr){
  byte * tt = (byte*) temp_ptr;
  Serial.write(tt, 4);
}
