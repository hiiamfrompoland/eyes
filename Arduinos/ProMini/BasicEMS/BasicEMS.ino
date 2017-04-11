/* EM7180_MPU9250_BMP280_M24512DFC Basic Example Code
 by: Kris Winer
 date: September 11, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the MPU9250 (with embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DRC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the MPU9250 9-axis motion sensor (accel/gyro/mag) as slave, a BMP280 pressure/temperature sensor, and an M24512DRC
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
 yet and there is a driver for the BMP280 in the SENtral firmware. 
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The BMP280 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+MPU9250+BMP280+M24512DRC Mini Add-On board for Teensy 3.1.
 
 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8
 
 Note: All the sensors n this board are I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
 Because the sensors are not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
 
#include "Globals.h"
#include <Wire.h>

#define SerialDebug true
//#define SerialDebug false

void setup()
{  
  // Setup for Master mode
  Wire.begin();
  delay(100);

  // Read Acc offset info
  //eeprom->readGlobalSet();
  delay(100);
  Serial.begin(9600);
  delay(5000);
      
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);

  // should detect SENtral at 0x28
  I2Cscan();
  
    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 
    
    // Insert Acc Cal upload here when the time comes...

    // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01);

    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 
  
    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41 Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 42 Hz
    
    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    
    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
    
    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);
  
    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A))
    {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B))
    {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
  
    //Disable stillness mode
    EM7180_set_integer_param (0x49, 0x00);
  
    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
    EM7180_set_gyro_FS (0x7D0); // 2000 dps
  
    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A))
    {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B))
    {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    delay(1000);
 
}

void loop()
{  
    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
  
    // Check for errors
    // Error detected, what is it?
    if(eventStatus & 0x02)
    { 
      uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
      if(!errorStatus)
      {
        Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
        if(errorStatus == 0x11) Serial.print("Magnetometer failure!");
        if(errorStatus == 0x12) Serial.print("Accelerometer failure!");
        if(errorStatus == 0x14) Serial.print("Gyro failure!");
        if(errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
        if(errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
        if(errorStatus == 0x24) Serial.print("Gyro initialization failure!");
        if(errorStatus == 0x30) Serial.print("Math error!");
        if(errorStatus == 0x80) Serial.print("Invalid sample rate!");
      }
      // Handle errors ToDo
    }
    
    if(eventStatus & 0x04)  // new quaternions available
    {
      readSENtralQuatData(Quat); 
    }

    // update LCD once per half-second independent of read rate

        Serial.println("Hardware quaternions (NED):"); 
        Serial.print("Q0 = "); Serial.print(Quat[0]);
        Serial.print(" Qx = "); Serial.print(Quat[1]); 
        Serial.print(" Qy = "); Serial.print(Quat[2]); 
        Serial.print(" Qz = "); Serial.println(Quat[3]); 
      
      
      //Hardware AHRS:
      float Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
      float Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
      float Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
      Pitch *= 180.0f / PI;
      Yaw   *= 180.0f / PI; 
      Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
      Roll  *= 180.0f / PI;
     
        Serial.print("Hardware Yaw, Pitch, Roll: ");
        Serial.print(Yaw, 2);
        Serial.print(", ");
        Serial.print(Pitch, 2);
        Serial.print(", ");
        Serial.println(Roll, 2);
      
      
  digitalWrite(myLed, !digitalRead(myLed));

    
  delay(500);
}

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

void EM7180_set_gyro_FS (uint16_t gyro_fs)
{
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused

  // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==0xCB)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB

  // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA);

  //Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==0xCA)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}
void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

}

void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}

//===================================================================================================================
//====== I2C Communication Support Functions
//===================================================================================================================

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
  Wire.beginTransmission(device_address);   // Initialize the Tx buffer
  Wire.write(data_address1);                // Put slave register address in Tx buffer
  Wire.write(data_address2);                // Put slave register address in Tx buffer
  Wire.write(data);                         // Put data in Tx buffer
  Wire.endTransmission();                   // Send the Tx buffer
}

void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
  if(count > 128)
  {
    count = 128;
    Serial.print("Page count cannot be more than 128 bytes!");
  }
  Wire.beginTransmission(device_address);   // Initialize the Tx buffer
  Wire.write(data_address1);                // Put slave register address in Tx buffer
  Wire.write(data_address2);                // Put slave register address in Tx buffer
  for(uint8_t i=0; i < count; i++)
  {
    Wire.write(dest[i]);                    // Put data in Tx buffer
  }
  Wire.endTransmission();                   // Send the Tx buffer
}

uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(device_address);         // Initialize the Tx buffer
  Wire.write(data_address1);                      // Put slave register address in Tx buffer
  Wire.write(data_address2);                      // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);               // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(device_address, (size_t)1);   // Read one byte from slave register address 
  data = Wire.read();                             // Fill Rx buffer with result
  return data;                                    // Return data read from slave register
}

void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(device_address);            // Initialize the Tx buffer
  Wire.write(data_address1);                         // Put slave register address in Tx buffer
  Wire.write(data_address2);                         // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);                  // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  }                                                   // Put read results in the Rx buffer
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
  // scan for i2c devices
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
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);            // Initialize the Tx buffer
  Wire.write(subAddress);                     // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);           // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  }                                           // Put read results in the Rx buffer
}

