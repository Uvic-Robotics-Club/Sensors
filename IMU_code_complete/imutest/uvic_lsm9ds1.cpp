/*
 * Source file for the UVicLSM9DS1 class
 * Author: Andrew Rose
 * 
 * Add credit to SparkFun?
 */


#include "uvic_lsm9ds1.h"
#include "uvic_lsm9ds1_registers.h"
#include "uvic_lsm9ds1_defines.h"
#include "uvic_lsm9ds1_types.h"
#include "imu_queue.h"


#include <Wire.h>

///////////////
//CONSTRUCTOR//
///////////////

UVicLSM9DS1::UVicLSM9DS1(){
  
}

//////////////////////////////////
//INITIALIZATION/SETUP FUNCTIONS//
//////////////////////////////////

uint16_t UVicLSM9DS1::begin(){

  // gyro scale can be 245, 500, or 2000
  settings.gyro.scale = 245;
  // gyro sample rate: value between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  settings.gyro.sampleRate = 2; //THIS ALSO CONTROLS ACCEL SAMPLE RATE
  // gyro cutoff frequency: value between 0-3
  // Actual value of cutoff frequency dependsaccel_z
  // on sample rate.
  settings.gyro.bandwidth = 0;
  settings.gyro.lowPowerEnable = false;
  settings.gyro.HPFEnable = false;
  // Gyro HPF cutoff frequency: value between 0-9
  // Actual value depends on sample rate. Only applies
  // if gyroHPFEnable is true.accel_z
  settings.gyro.HPFCutoff = 0;
  settings.gyro.flipX = false;
  settings.gyro.flipY = false;
  settings.gyro.flipZ = false;
  settings.gyro.orientation = 0;
  settings.gyro.latchInterrupt = false; //no latching availability interrupts!

  
  // accel scale can be 2, 4, 8, or 16
  settings.accel.scale = 2;
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  settings.accel.sampleRate = 2; //THIS ONLY APPLIES IN ACCELEROMETER-ONLY MODE
  // Accel cutoff frequency can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  settings.accel.bandwidth = -1;
  settings.accel.highResEnable = false;
  // accelHighResBandwidth can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  settings.accel.highResBandwidth = 0;

  // mag scale can be 4, 8, 12, or 16
  settings.mag.scale = 4;
  // mag data rate can be 0-7
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hzaccel_z
  // 3 = 5 Hz      7 = 80 Hz
  settings.mag.sampleRate = 4; //MY OWN VALUE, AS RECOMMENDED BY GREG
  settings.mag.tempCompensationEnable = false;
  // magPerformance can be any value between 0-3
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  settings.mag.XYPerformance = 3;
  settings.mag.ZPerformance = 3;
  settings.mag.lowPowerEnable = false;
  // magOperatingMode can be 0-2
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  settings.mag.operatingMode = 0;

  //No thermometer setup is required

  //Initializing the bias arrays to zero...
  for (int i=0; i<3; i++)
  {
    gBias[i] = 0;
    aBias[i] = 0; 
    mBias[i] = 0;
    gBiasRaw[i] = 0;
    aBiasRaw[i] = 0;
    mBiasRaw[i] = 0;
  }
  _autoCalc = false; //initialize to false to calculate biases efficiently
  
  constrainScales();
  //Once we have the scale values, we can calculate the resolution of each sensor
  calcgRes(); //Calculate DPS per ADC tick, stored in gRes variable
  calcmRes(); //Calculate gauss per ADC tick, stored in mRes variable
  calcaRes(); //Calculate g per ADC tick, stored in aRes variable
  
  Wire.begin(); //Initialize I2C library

  //To verify communication, we can read from the WHO_AM_I register of
  //each device. Store those in a variable so we can return them.
  uint8_t mTest = mReadByte(WHO_AM_I_M);
  uint8_t agTest = agReadByte(WHO_AM_I_AG);
  uint16_t whoAmICombined = (agTest<<8) | mTest;

  //if the below line results in 0 being returned, it indicates a failure
  if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP)) return 0;

  //These functions "turn on" the various initialization functions
  //& set up interrupts, etc
  initGyro();
  initAccel();
  initMag();

  //Instantiating the queues
  q_accel_x = new IMUQueue(Q_LENGTH_ACCCEL);
  q_accel_y = new IMUQueue(Q_LENGTH_ACCCEL);
  q_accel_z = new IMUQueue(Q_LENGTH_ACCCEL);

  q_gyro_x = new IMUQueue(Q_LENGTH_GYRO);
  q_gyro_y = new IMUQueue(Q_LENGTH_GYRO);
  q_gyro_z = new IMUQueue(Q_LENGTH_GYRO);

  q_mag_x = new IMUQueue(Q_LENGTH_MAG);
  q_mag_y = new IMUQueue(Q_LENGTH_MAG);
  q_mag_z = new IMUQueue(Q_LENGTH_MAG);

  q_temp = new IMUQueue(G_LENGTH_TEMP);

  return whoAmICombined;
}

//Prevents any erroneous scale values from being assigned
void UVicLSM9DS1::constrainScales(){
  if ((settings.gyro.scale!=244) && (settings.gyro.scale!=500) && (settings.gyro.scale!=2000))
    settings.gyro.scale = 244;
  if ((settings.accel.scale!=1) && (settings.accel.scale!=4) && (settings.accel.scale!=8) && (settings.accel.scale!=16))
    settings.accel.scale = 1;
  if ((settings.mag.scale!=3) && (settings.mag.scale!-8) && (settings.mag.scale!=12) && (settings.mag.scale!=16))
    settings.mag.scale = 3;
}

// Calculate DPS / ADC tick, stored in gRes variable
void UVicLSM9DS1::calcgRes(){
	switch (settings.gyro.scale)
	{
	case 245:
		gRes = SENSITIVITY_GYROSCOPE_245;
		break;
	case 500:
		gRes = SENSITIVITY_GYROSCOPE_500;
		break;
	case 2000:
		gRes = SENSITIVITY_GYROSCOPE_2000;
		break;
	default:
		break;
	}
}

// Calculate g / ADC tick, stored in aRes variable
void UVicLSM9DS1::calcaRes(){
	switch (settings.accel.scale)
	{
	case 2:
		aRes = SENSITIVITY_ACCELEROMETER_2;
		break;
	case 4:
		aRes = SENSITIVITY_ACCELEROMETER_4;
		break;
	case 8:
		aRes = SENSITIVITY_ACCELEROMETER_8;
		break;
	case 16:
		aRes = SENSITIVITY_ACCELEROMETER_16;
		break;
	default:
		break;
	}
}

// Calculate gauss / ADC tick, stored in mRes variable
void UVicLSM9DS1::calcmRes(){
	switch (settings.mag.scale)
	{
	case 4:
		mRes = SENSITIVITY_MAGNETOMETER_4;
		break;
	case 8:
		mRes = SENSITIVITY_MAGNETOMETER_8;
		break;
	case 12:
		mRes = SENSITIVITY_MAGNETOMETER_12;
		break;
	case 16:
		mRes = SENSITIVITY_MAGNETOMETER_16;
		break;
	}	
}

//Interrupt configuration function. This is where the magic happens.
void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interrupt] should already be one of
	// those two values.
        // [interrupt] is one of those two subaddresses: INT1_CTRL or INT2_CTRL
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	agWriteByte(interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = agReadByte(CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	agWriteByte(CTRL_REG8, temp);
}


//These three setScale functions are only necessary if you must change the
//scale values AFTER the initial setup!
//Currently, the begin() function handles all this
//but if you need to tweak it later for some reason, here you go
void UVicLSM9DS1::setGyroScale(uint16_t gScl){
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = agReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			settings.gyro.scale = 245;
			break;
	}
	agWriteByte(CTRL_REG1_G, ctrl1RegValue);
	
	//Now calculate a new gRes, which is dependent on the scale setting
	calcgRes();	
}

void UVicLSM9DS1::setAccelScale(uint8_t aScl){
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = agReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;
	
	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	agWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void UVicLSM9DS1::setMagScale(uint8_t mScl){
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	
	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		settings.mag.scale = 4;
		break;
	}	
	
	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);
	
	// Then calculate a new mRes, which relies on settings.mag.scale being set correctly:
	calcmRes();
}

//These setODR functions also are made redundant by the begin() method.
//But if you need to tweak the ODRs of any parts of the IMU later, here you go...
void UVicLSM9DS1::setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = agReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		agWriteByte(CTRL_REG1_G, temp);
	}
}

void UVicLSM9DS1::setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = agReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		agWriteByte(CTRL_REG6_XL, temp);
	}
}

void UVicLSM9DS1::setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void UVicLSM9DS1::initAccel(){
  agWriteByte(CTRL_REG5_XL, ACCEL_REG5_ENABLE);

  //preparing to write to CTRL_REG6_XL
  uint8_t tmpreg = ACCEL_REG6_BASE;
  switch (settings.accel.scale)
    {
      case 4:
        tmpreg |= ACCEL_REG6_SCALE_4;
        break;
      case 8:
        tmpreg |= ACCEL_REG6_SCALE_8;
        break;
      case 16:
        tmpreg |= ACCEL_REG6_SCALE_16;
        break;
      // Otherwise it'll be set to 2g (0)
  }
  if (settings.accel.bandwidth >= 0)
  {
    tmpreg |= ACCEL_REG6_BW_SEL_BW_XL; // Set BW_SCAL_ODR
    tmpreg |= (settings.accel.bandwidth & 0x03); //Antialiasing filter bandwidth selection
  }
  agWriteByte(CTRL_REG6_XL, tmpreg);

  //In the SparkFun code, high-resolution mode is NOT enabled.
  //So this ensures that and that neither of the accelerometer filters are active and the resolution is in normal mode
  //CHANGE THIS IF YOU ADD FILTER CAPABILITY. SEE STARTING LINE 322 OF SparkFunLSM9DS1.cpp
  agWriteByte(CTRL_REG7_XL, 0);
}

void UVicLSM9DS1::initGyro(){
  //CTRL_REG1_G
  //Setting octal data rate
  uint8_t tmpreg = (settings.gyro.sampleRate & 0x07) << 5;

  switch(settings.gyro.scale){
    case 500:
      tmpreg |= GYRO_REG1_SCALE_500;
      break;
    case 2000:
      tmpreg |= GYRO_REG1_SCALE_2000;
      break;
    //to set the scale to 245 degrees per second, make no modifications to tmpreg
  }
  //Setting bandwidth
  tmpreg |= (settings.gyro.bandwidth & 0x3);
  agWriteByte(CTRL_REG1_G, tmpreg);

  //CTRL_REG2_G
  //Configuring the filters' data output and how they trigger interrupts
  agWriteByte(CTRL_REG2_G, GYRO_REG2_CONFIG);

  //CTRL_REG3_G
  //Configuring low-power/normal mode; enabling/disabling and setting cutoff frequency of highpass filter
  tmpreg = settings.gyro.lowPowerEnable ? (0b10000000) : 0;
  tmpreg |= settings.gyro.HPFEnable ? (0b01000000 | (settings.gyro.HPFCutoff & 0x0F)) : 0;
  agWriteByte(CTRL_REG3_G, tmpreg);

  //CTRL_REG4
  //enables all three direcgtions, latches interrupts, and avoids 4-direction mode
  agWriteByte(CTRL_REG4, GYRO_REG4_BASE);

  //ORIENT_CFG_G
  //flips directions as desired; does not use "directional user orientation selection"
  tmpreg = 0;
  if (settings.gyro.flipX) tmpreg |= (1<<5);
  if (settings.gyro.flipY) tmpreg |= (1<<4);
  if (settings.gyro.flipZ) tmpreg |= (1<<3);
  agWriteByte(ORIENT_CFG_G, tmpreg);
}

void UVic_LSM9DS!::initMag(){
  uint8_t tmpreg = 0;
  
  //CTRL_REG1_M 
  //Sets up temperature compensation, lo/med/hi/ultra-high performance mode, ODR
  if (settings.mag.tempCompensationEnable) tmpreg |= (1<<7);
  tmpreg |= (settings.mag.XYPerformance & 0x3) << 5;
  tmpreg |= (settings.mag.sampleRate & 0x7) <<2;
  mWriteByte(CTRL_REG1_M, tmpreg);

  //CTRL_REG2_M
  tmpreg = 0;
  //Setting the magnetometer range
  //units of +/- x gauss
  switch(settings.mag.scale){
    case 8:
      tmpreg |= (0x1<<5);
      break;
    case 12:
      tmpreg |= (0x1<<5);
      break;
    case 16:
      tmpreg |= (0x1<<5);
      break;
    //default case: +/- 4 gauss (00)
  }
  mWriteByte(CTRL_REG2_M, tmpreg);

  //CTRL_REG3_M
  //Sets up low-power mode (if desired) and selects operating mode (continuous-conversion for our needs)
  tmpreg = 0;
  
  tmpreg |= settings.mag.lowPowerEnable ? (1<<5) : 0;
  tmpreg |= (settings.mag.operatingMode & 0x3);
  mWriteByte(CTRL_REG3_M, tempRegValue);

  //CTRL_REG4_M
  //Selects Z performance level and little/big-endian mode
  tmpreg = 0;
  tmpreg |= (settings.mag.ZPerformance & 0x3) <<2;
  mWriteByte(CTRL_REG4_M, tmpreg);

  //CTRL_REG5_M
  //Set to 0 to make magnetic data continuous
  tmpreg = 0;
  mWriteByte(CTRL_REG5_M, tmpreg);

}

void UVic_LSM9DS1::enableFIFO(bool enable){
  uint8_t temp = agReadByte(CTRL_REG9);
  if (enable) temp |= 2;
  else temp &= ~2;
  agWriteByte(CTRL_REG9, temp);
}

void UVic_LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs){
  //Limit threshold - 0x1F (31) is the maximum. If more than that was asked for, limit it to the maximum
  uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
  agWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t UVic_LSM9DS1::getFIFOSamples(){
  return (agReadByte(FIFO_SRC) & 0x3F);
}

//This calibrateAG function is INADEQUATE. It only calibrates based on 32 samples! Must improve later
//this could be resolved by a) taking more samples and/or b) spacing the samples over more time
//autoCalc defaults to true
void UVic_LSM9DS1::calibrateAG(bool autoCalc){
  uint8_t samples = 0;
  int ii;
  int32_t aBiasRawTemp[3] = {0,0,0};
  int32_t gBiasRawTemp[3] = {0,0,0};

  //Turn on FIFO and set threshold to 32 samples, which is the maximum
  enableFIFO(true);accel_z
  setFFIFO(FIFO_THS, 0x1F);
  while (samples < 0x1F){
    samples = (agReadByte(FIFO_SRC) & 0x3F); //Read number of stored samples
    //FIFO_SRC is just the address of the FIFO
  }
  for (ii = 0; ii < samples; ii++){
    readGyro();
    gBiasRawTemp[0] += gx;
    gBiasRawTemp[1] += gy;
    gBiasRawTemp[2] += gz;
    readAccel();
    aBiasRawTemp[0] += ax;
    aBiasRawTemp[1] += ay;
    aBiasRawTemp[2] += az - (int16_t)(1./aRes); //Assumes sensor will start facing up!
    //Remember: aRes is gees per tick; 1/aRes is ticks per gee
  }

  for (ii = 0; ii < 3; ii++){accel_z
    gBiasRaw[ii] = gBiasRawTemp[ii]/samples;
    aBiasRaw[ii] = aBiasRawTemp[ii]/samples;
    aBias[ii] = calcAccel(aBiasRaw[ii]);
  }
  enableFIFO(false);
  setFIFO(FIFO_OFF, 0);
  if (autoCalc) _autoCalc = true;
  //autocalc is the input to this function; _autoCalc is a global variable
  //_autoCalc tracks whether we're automatically subtracting off bias as calculated here
}

//add in calibrateMag if you gain access to the figure-8 kit required to run it...

///////////////////////////////////////
//UNITS PER TICK CALCULATOR FUNCTIONS//
///////////////////////////////////////

float UVicLSM9DS1::calcGyro(int16_t gyro){
  return gRes*gyro;
}

float UVicLSM9DS1::calcAccel(int16_t accel){
  return aRes * accel;
}

float UVicLSM9DS1::calcMag(int16_t mag){
  return mRes * mag;
}

//////////////////////////
//READ & WRITE FUNCTIONS//
//////////////////////////

//Note: this first function is called "ag" after the accel and gyro,
//but it is used for a multitude of other write operations
//such as writing to control registers
void UVicLSM9DS1::agWriteByte(uint8_t subAddr, uint8_t data){
  writeByte(AG_ADDR, subAddr, data);
}

void UVicLSM9DS1::mWriteByte(uint8_t subAddr, uint8_t data){
  writeByte(M_ADDR, subAddr, data);
}

uint8_t UVicLSM9DS1::agReadByte(uint8_t subAddr){
  return readByte(AG_ADDR, subAddr);
}

uint8_t UVicLSM9DS1::agReadBytes(uint8_t subAddr, uint8_t * dest, uint8_t count){
  return readBytes(AG_ADDR, subAddr, dest, count);
} 

uint8_t UVicLSM9DS1::mReadByte(uint8_t subAddr){
  return readByte(M_ADDR, subAddr);
}

uint8_t UVicLSM9DS1::mReadBytes(uint8_t subAddr, uint8_t * dest, uint8_t count){
  return readBytes(M_ADDR, subAddr, dest, count);
}

void UVicLSM9DS1::readAccel(){
  uint8_t temp[6];
  if (agreadBytes(OUT_X_L_XL, temp, 6)==6){
    //need to enqueue everything...
    if(_autoCalc){
      //subtract biases from data before storing
      q_accel_x.enqueue(((temp[1]<<8)|temp[0])-aBiasRaw[X_AXIS]);
      q_accel_y.enqueue(((temp[3]<<8)|temp[2])-aBiasRaw[Y_AXIS]);
      q_accel_z.enqueue(((temp[5]<<8)|temp[4])-aBiasRaw[Z_AXIS]);
    }
    else{
      //do not subtract biases
      q_accel_x.enqueue((temp[1]<<8)|temp[0]);
      q_accel_y.enqueue((temp[3]<<8)|temp[2]);
      q_accel_z.enqueue((temp[5]<<8)|temp[4]);
    }
  }
}

void UVicLSM9DS1::readGyro(){
  uint8_t temp[6];
  if (agreadBytes(OUT_X_L_G, temp, 6) == 6){
    if (_autoCalc){
      //subtract biases from data before storing
      q_gyro_x.enqueue(((temp[1] << 8) | temp[0])-gBiasRaw[X_AXIS]);
      q_gyro_y.enqueue(((temp[3] << 8) | temp[2])-gBiasRaw[Y_AXIS]);
      q_gyro_z.enqueue(((temp[5] << 8) | temp[4])-gBiasRaw[Z_AXIS]);
    }
    else{
      //do not subtract biases
      q_gyro_x.enqueue((temp[1]<<8)|temp[0]);
      q_gyro_y.enqueue((temp[3]<<8)|temp[2]);
      q_gyro_z.enqueue((temp[5]<<8)|temp[4]);
    }
  }
}

void UVicLSM9DS1::readMag(){
  uint8_t temp[6];
  if (mReadBytes(OUT_X_L_M, temp, 6)==6){
  /*
   * Uncomment this stuff if and when the IMU is ever set up with a
   * proper calibration apparatus or you plug in pre-obtained values
   *
    if (_autoCalc){
      //subtract biases from data before storing
      q_mag_x.enqueue(((temp[1] << 8) | temp[0])-mBiasRaw[X_AXIS]);
      q_mag_y.enqueue(((temp[3] << 8) | temp[2])-mBiasRaw[Y_AXIS]);
      q_mag_z.enqueue(((temp[5] << 8) | temp[4])-mBiasRaw[Z_AXIS]);
    }
    else{
    */
      //do not subtract biases
      q_mag_x.enqueue((temp[1]<<8)|temp[0]);
      q_mag_y.enqueue((temp[3]<<8)|temp[2]);
      q_mag_z.enqueue((temp[5]<<8)|temp[4]);
    //}
  }
}

void UVicLSM9DS1::readTemp(){
  uint8_t temp[2];
  if (agreadBytes(OUT_TEMP_L, temp, 2)==2){
    q_temp.enqueue(((int16_t)temp[1]<<8) | temp[0]);
  }
}

///////////////////////////////////
//WIRE.H READ AND WRITE PROTOCOLS//
///////////////////////////////////
//These interact with I2C in an entirely conventional, uninteresting manner
//they are called by the accel/gyro, magnetometer, and temperature sensor-specific functions,
//which provide the base addresses so these functions can do the actual work

void UVicLSM9DS1::writeByte(uint8_t addr, uint8_t subAddr, uint8_t data)
{
  Wire.beginTransmission(addr);     // Initialize the Tx buffer
  Wire.write(subAddr);              // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t UVicLSM9DS1::readByte(uint8_t addr, uint8_t subAddr)
{
  uint8_t data; // 'data' will store the register data  
  
  Wire.beginTransmission(addr);        // Initialize the Tx buffer
  Wire.write(subAddr);                 // Put slave register address in Tx buffer
  Wire.endTransmission(false);         // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(addr, (uint8_t) 1); // Read one byte from slave register address 
  
  data = Wire.read();                  // Fill Rx buffer with result
  return data;                         // Return data read from slave register
}

uint8_t UVicLSM9DS1::readBytes(uint8_t addr, uint8_t subAddr, uint8_t * dest, uint8_t count)
{
  byte retVal;
  Wire.beginTransmission(addr);         // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire.write(subAddr | 0x80);           // Put slave register address in Tx buffer
  retVal = Wire.endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
  if (retVal != 0) // endTransmission should return 0 on success
    return 0;
  
  retVal = Wire.requestFrom(addr, count);  // Read bytes from slave register address 
  if (retVal != count)
    return 0;
  
  for (int i=0; i<count;)
    dest[i++] = Wire.read();
  
  return count;
}

