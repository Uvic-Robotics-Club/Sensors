#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>
#define TCAADDR 0x70

float humidity;
float tempC;

Weather humiditySensor;

const uint8_t fanPin1 = 3;
const uint8_t fanPin2 = 5;
const uint8_t fanPin3 = 6;

uint8_t fanSpeed1 = 0;
uint8_t fanSpeed2 = 0;
uint8_t fanSpeed3 = 0;


void setup()
{
  Serial.begin(9600);

  humiditySensor.begin();
}


void loop()
{
  for (int sensornum = 0; sensornum < 8; sensornum++) //pings all sensors attached to i2c expander
  {
    tcaselect(sensornum);

    tempC = humiditySensor.getTemp();//get.Temp == celcius  ||  get.TempF == farenhight

    if (sensornum == 1) {
      fanSpeed1 = map(tempC, 10, 30, 0, 255); //maps fan1 to the port 1 of of i2c expander (sensor1)
      analogWrite(fanPin1, fanSpeed1);
    }

    else if (sensornum == 2) {
      fanSpeed2 = map(tempC, 10, 30, 0, 255); //maps both fan 2 & 3 to the port 2 of the i2c expander (sensor2)
      analogWrite(fanPin2, fanSpeed2);

      fanSpeed3 = map(tempC, 10, 30, 0, 255);
      analogWrite(fanPin3, fanSpeed3);

    }
  }
  delay(20);//short delay to slow things down
}


void tcaselect(uint8_t i) { // (if on port ex. 4, select 4 here)
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
