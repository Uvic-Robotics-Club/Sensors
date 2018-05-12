#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>

float humidity = 0;
float tempC = 0;


Weather humiditySensor;


void setup()
{
  Serial.begin(9600);

  humiditySensor.begin();
}


void loop()
{
  getWeather();
  printInfo();
  delay(1000);

}


//gets humidity and temp.
//get.Temp is celcius
//get.TempF is farenhight
void getWeather()
{
  humidity = humiditySensor.getRH();

  tempC = humiditySensor.getTemp();
}


//prints global values to serial.
void printInfo()
{
  Serial.print("Temp:");
  Serial.print(tempC);
  Serial.print("C, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}


