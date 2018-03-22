#include <SparkFun_Si7021_Breakout_Library.h>

Weather humiditySensor;

#include <Wire.h>

#define TCAADDR 0x70

void setup ()
{
  // the number in the parameter of the 'tcaselect()' function is the port of the sensor that you want to set the connection to
  Serial.begin(9600);
  humiditySensor.begin();
}

void loop ()
{
  for (int sensornum = 0; sensornum < 8; sensornum++) // this loop goes through all the different ports collecting sensor data
  {
    tcaselect(sensornum);
    long data =  humiditySensor.getTemp();
      Serial.print("celsius degrees of sensor ");
      Serial.print(sensornum);
      Serial.print(": ");
      Serial.println(data);      
  }
  delay(500);
}


void tcaselect(uint8_t i) { // (if on port ex. 4, select 4 here)
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

