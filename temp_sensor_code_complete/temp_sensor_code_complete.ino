/*
 * Tests the TempSensor class
 * Contains four examples of how to write/print temperature data to serial
 * Degree-precision, single-byte writing is on by default
 * Float-precision, four-byte writing is also supported via the write_float function
 * Uncomment any alternative to try it out
 * Author: Andrew Rose
 */
 
#include <Wire.h>
#include "temp_sensor.h"

//address of temperature sensor
#define ADDR 0x40

//This makes the object global, simplifying the program
TempSensor sens1;
float tmptmp; //ha! temporary temperature! get it?
byte *tt; //byte pointer to the float so the float can be sent over serial as four bytes
 
void setup(){
	//set up sensor 1 with correct address
	sens1.begin(ADDR);
	//initialize I2C communication as master
	Wire.begin();
	//initialize serial communication with baud rate 9600 Hz
	Serial.begin(9600);
}
 
void loop(){
  
	tmptmp = sens1.get_temp();
  
  //Writing to serial (degree precision, 1 byte per reading)
  Serial.write(uint8_t(tmptmp));

  //Writing to serial in float precision, uncomment the line below:
  //sens1.write_float(&tmptmp);
  //note: this writes the 4 bytes of the float in REVERSE ORDER

  //Printing decimal representations of temperature
  //Uncomment for visible ease of testing
  /*
  Serial.println(tmptmp); //float precision
  Serial.println(uint8_t(tmptmp)); //degree precision
  */
  
  delay(500);
}
