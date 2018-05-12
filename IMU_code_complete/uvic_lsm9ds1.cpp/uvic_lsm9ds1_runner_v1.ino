/*
 * The very first version of the runner for the very first version of my code
 * for the ST LSM9DS1 inertial measurement unit.
 * 
 * Just getting this thing up and running for the first time.
 * It prints the averaged-out data collected by the IMU at approximately 1 Hz.
 *
 * 
 * Author: Andrew Rose
 *
 * Interrupt configuration:
 * INT1 interrupts whenever there is new accelerometer and/or gyroscope data
 * INT2 interrupts whenever there is new temperature data
 * DRDY_M, by default, interrupts whenever there is new magnetometer data
 */

#include <Wire.h>
#include "uvic_lsm9ds1.h"
#include "imu_queue.h"

UVicLSM9DS1 imu;

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
// These can be swapped to any available digital pin:
const int INT1_PIN_AG = 3; // INT1 pin to D3 - interrupts when new accel/gyro data is available
const int INT2_PIN_TEMP = 4; // INT2 pin to D4 - interrupts when new temperature data is available
const int RDYM_PIN = 5;  // RDY pin to D5

//Variable to track when we print sensor readings
unsigned long lastPrint = 0;

//Sets up interface, sensor scales, and sample rates
//Temperature ODR is locked to 59.5 Hz while gyro is on, or 50 Hz when off
//(so 59.5 Hz)

void configInterrupts(){
  imu.configInt(XG_INT1, INT_DRDY_XL|INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);
  imu.configInt(XG_INT2, INT_DRDY_TEMP, INT_ACTIVE_LOW, INT_PUSH_PULL);
}

void setup(){
  Serial.begin(9600);

  //Set up interrupt pins
  //configured to be active-low
  pinMode(INT1_PIN_AG, INPUT_PULLUP);
  pinMode(INT2_PIN_TEMP, INPUT_PULLUP);
  //The magnetometer DRDY pin (RDY) is not configurable
  //It is active-high and always turned on
  pinMode(RDYM_PIN, INPUT);

  //Make sure everything is connected. If not, stop right here
  uint16_t status = begin();
  if (!status){
    Serial.print("Failed to connect to IMU: 0x");
    Setial.println(status, HEX);
    while(1);
  }
  configInterrupts();
}

void loop(){
  //Print the current measurements every second.
  //These are running averages taken by reading the entire queues
  if (millis() > (lastPrint + 1000){
    printStats();
    lastPrint = millis();
  }

  //INT1 fires when new accelerometer/gyroscope data is available
  if (digitalRead(INT1_PIN_AG)==LOW){
    imu.readAccel();
    imu.readGyro();
  }

  if (digitalRead(INT2_PIN_TEMP)==LOW){
    imu.readTemp();
  }
  
  if (digitalRead(RDYM_PIN) == HIGH){
    imu.readMag();
  }
}

void printStats(){
  Serial.println();
  Serial.print("Linear acceleration: ");
  Serial.print(imu.q_accel_x.getAvg()); Serial.print(", ");
  Serial.print(imu.q_accel_y.getAvg()); Serial.print(", ");
  Serial.println(imu.q_accel_z.getAvg());

  Serial.print("Angular velocity: ");
  Serial.print(imu.q_gyro_x.getAvg()); Serial.print(", ");
  Serial.print(imu.q_gyro_y.getAvg()); Serial.print(", ");
  Serial.println(imu.q_gyro_z.getAvg());

  Serial.print("Temperature: ");
  Serial.println(imu.q_temp.getAvg());

  Serial.print("Magnetic field: ");
  Serial.print(imu.q_mag_x.getAvg()); Serial.print(", ");
  Serial.print(imu.q_mag_ygetAvg()); Serial.print(", ");
  Serial.println(imu.q_mag_z.getAvg());
}
