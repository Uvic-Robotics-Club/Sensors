#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>

float humidity = 0;
float tempf = 0;

/*

int power = A3;
int GND = A2;

*/

Weather sensor;


void setup()
{
  Serial.begin(9600);


/*

  pinMode(power, OUTPUT);
  pinMode(GND, OUTPUT);

  digitalWrite(power, HIGH);
  digitalWrite(GND, LOW);


*/

  sensor.begin();
}




void loop()
{
  getWeather();
  printInfo();
  delay(1000);

}




void getWeather()
{
  humidity = sensor.getRH();

  tempf = sensor.getTempF();
}




void printInfo()
{
  Serial.print("Temp:");
  Serial.print(tempf);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}


