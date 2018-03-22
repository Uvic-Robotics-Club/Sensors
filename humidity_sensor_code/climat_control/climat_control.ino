#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>

float humidity = 0;
float tempC = 0;

Weather humiditySensor;

const int fanPin = 11;

uint8_t fanSpeed = 0;

void setup()
{
  Serial.begin(9600);

  humiditySensor.begin();
}


void loop()
{
  getWeather();

  fanSpeed = map(tempC, 10, 30, 0, 255); //10, 30 == sensor value range (that has been converted in celcius)   ||   0, 255 == pwm (fan control) range
  fanSpeed = constrain(fanSpeed, 0, 255);
  analogWrite(fanPin, fanSpeed);

  delay(10);
}


void getWeather()
{
  humidity = humiditySensor.getRH();

  tempC = humiditySensor.getTemp();//get.Temp == celcius  ||  get.TempF == farenhight
}
