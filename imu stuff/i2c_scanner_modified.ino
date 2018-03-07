
#include <Wire.h>
 
 
void setup()
{
  Wire.begin();
 
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
if (error == 0)
   {
     Serial.print("Error code 0 at 0x");
     if (address<16)
       Serial.print("0");
     Serial.print(address,HEX);
     Serial.println("  !");
   }

if (error == 1)
   {
     Serial.print("Error code 1 at 0x");
     if (address<16)
       Serial.print("0");
     Serial.print(address,HEX);
     Serial.println("  !");
   }


if (error == 2)
   {
     Serial.print("Error code 2 at 0x");
     if (address<16)
       Serial.print("0");
     Serial.print(address,HEX);
     Serial.println("  !");
   }

if (error == 3)
   {
     Serial.print("Error code 3 at 0x");
     if (address<16)
       Serial.print("0");
     Serial.print(address,HEX);
     Serial.println("  !");
   }
    
    
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}
