/***************************************************************************

   Common definitions and global variablse

 ***************************************************************************/
 
#include <Time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include "Globals.h"

// Serial port connected to Bluetooth Mate Gold
#define BTserial  Serial1

// Serial port connected to OpenLog
#define SDserial  Serial2
#define SDgrn     11
#define SDdefBaud 9600
#define SDhispdBaud 57600

// Serial port connected to USB
#define DEBUG     Serial
#define DEBUBdefBaud  9600

// pulling this down will restart the whole device
#define RST_PIN   12
// Teensy led pin
#define LED_PIN   13
// i2c from sensor reset pin
#define INT_PIN   8
// buzzer pin
#define BUZ_PIN   2
// buzzer tone
#define BUZ_TONE  420

// EEPRROM addres of register with flags determining what was saved to the eeprom
#define SESS_ON_ADDR    0x048
#define SESS_ON             1   // if there was a session already set up or are we starting something new          
#define SESS_NAME_SET       2   // if in EEPROM there is name for sesison or should use default.log
#define SESS_CALIBRATED     4   // if in EEPROM there are reference quaternions
#define SESS_ALARM_SET      8   // if in EEPROM there is alarm level saved
#define SESS_POOLING_SET   16   // if custom pooling period was set
#define SESS_ALARM_EN      32   // if alarm is enabled

// EEPROM address of a session name of current measurement
#define SESS_SESSION_NAME_ADDR   0x050   // 8 bytes
// EEPROM address of an alarm level
#define SESS_ALARM_LEVEL_ADDR    0x60    // 4 bytes, float, alarm level set for this patient
// EEPROM address of 1st component of quaternions of initial position
#define SESS_INIT_Q1_ADDR        0x68    // 4 bytes, float
// EEPROM address of 2nd component of quaternions of initial position
#define SESS_INIT_Q2_ADDR        0x72    // 4 bytes, float
// EEPROM address of 3rd component of quaternions of initial position
#define SESS_INIT_Q3_ADDR        0x76    // 4 bytes, float
// EEPROM address of 4th component of quaternions of initial position
#define SESS_INIT_Q4_ADDR        0x80    // 4 bytes, float
// EEPROM address of period between measurements
#define SESS_POOLING_ADDR        0x84    // 4 bytes, uint32

// comment out if you don't want debug info in your stream
#define doDebug

// determines whether to log or not to log new data
boolean doRun = false;
// flag representing succesful initializaiton of all components
boolean initSuccess = true;

// name of the current session
String SessionName;
// 8 Flags indicating what was set up and what is lost
uint8_t SessionSet = 0;
#define SESSION_FORCED      1 // if stubborn to proceed without setting
#define SESSION_NAME_SET    2 // if the name/id was set, bit 1
#define SESSION_CALIBRATED  4 // if reference point was taken
#define SESSION_ALARM_SET   8 // if alarm level was set
#define SESSION_POOLING_SET   16 // if custom pooling period was set
#define SESSION_ALARM_EN      32   // if alarm is enabled
#define SESSION_EEPROMED    128   // if saved in EEPROM

// reference quaternions (representing head at null position), and it's conjugate
float refQuat[4];
float refQuatInv[4];
float rawQuat[4];
// Alarm level
float alarmLevel;
// period between two measurements, in miliseconds
uint32_t poolingPeriod = 500;

/**
 * Logs an error to the session file
 */
inline void errorLog(String e)
{
  SDserial.print("E ");
  SDserial.print(millis());
  SDserial.print(" ");
  SDserial.println(e);

  #ifdef doDebug
  DEBUG.print("ERROR ");
  DEBUG.println(e);
  #endif
}

void setup() {
  // setup default variables
  refQuat[0] = 0.0f;
  refQuat[1] = 0.0f;
  refQuat[2] = 0.0f;
  refQuat[3] = 1.0f;

  SessionName = "default.log";

  // diode is the most important
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // SENTRAL interrup pin
  pinMode(INT_PIN, INPUT);

  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);

  // setup debug stream
  DEBUG.begin(DEBUBdefBaud);

  // setup USB stream
  // nasty hack got to fix it some time some day
  if (&DEBUG != &Serial) {
    Serial.begin(DEBUBdefBaud);
  }

  // setup bluetooth, it is not critical component so failure doesn't change much
  // and besides we don't use it yet
  //BTsetup();
  // put bluetooth in low power consumption
  //BTlowmode(true);

  // setup sdcard, don't enter command mode yet
  if (!SDsetup(false)) {
    // if failed to open SDcard
    DEBUG.println("ERROR: SD card can't be operated properly, try to restart or investigate");
    initSuccess = false;
    return;
  }
  delay(100);

  // setup rtc stuff
  setupRTC();

  // Check if there is already a session running or if to setup new one
  // in case device restarts we will have track of it and pick up where
  // we finished so data would be more 'stable'
  uint8_t eeSess = EEPROM.read(SESS_ON_ADDR);
  if (eeSess & SESS_ON) {
    // apparently something was already going on, we start there

    // read session name
    if (eeSess & SESS_NAME_SET)
    {
      // tiny hack to get a 9 element char
      struct {
        char n[9];
      } sessname;
      EEPROM.get(SESS_SESSION_NAME_ADDR, sessname);
      sessname.n[8] = 0;  // terminate with 0 in a case
      SessionName = sessname.n; // save as a string

      SessionSet |= SESSION_NAME_SET;

      SDappend(SessionName);
      SDserial.print("I ");
      SDserial.print(millis());
      SDserial.print(" ");
      SDserial.print(SessionName);
      SDserial.print(" ");
      SDserial.println(now());

      DEBUG.print("EEPROM: loaded session name: ");
      DEBUG.println(SessionName);
    }

    // read alarm level
    if (eeSess & SESS_ALARM_SET)
    {
      EEPROM.get(SESS_ALARM_LEVEL_ADDR, alarmLevel);

      SessionSet |= SESSION_ALARM_SET;
      DEBUG.print("EEPROM: loaded alarm level: ");
      DEBUG.println(alarmLevel);
    }

    // read pooling period
    if (eeSess & SESS_POOLING_SET)
    {
      EEPROM.get(SESS_POOLING_ADDR, poolingPeriod);

      SessionSet |= SESSION_POOLING_SET;
      DEBUG.print("EEPROM: loaded pooling period: ");
      DEBUG.println(poolingPeriod);
    }
    // read reference quaternions
    if (eeSess & SESS_CALIBRATED)
    {
      EEPROM.get(SESS_INIT_Q1_ADDR, refQuat[0]);
      EEPROM.get(SESS_INIT_Q2_ADDR, refQuat[1]);
      EEPROM.get(SESS_INIT_Q3_ADDR, refQuat[2]);
      EEPROM.get(SESS_INIT_Q4_ADDR, refQuat[3]);

      float refQuatConj[4];
      quatConj(refQuat, refQuatConj);

      float norm = quatNorm(refQuatConj);
      refQuatInv[0] = refQuatConj[0] / norm;
      refQuatInv[1] = refQuatConj[1] / norm;
      refQuatInv[2] = refQuatConj[2] / norm;
      refQuatInv[3] = refQuatConj[3] / norm;
      #ifdef doDebug
      DEBUG.println("EEPROM: loaded calibrated data");
      DEBUG.print(refQuat[0]);
      DEBUG.print(" ");
      DEBUG.print(refQuat[1]);
      DEBUG.print(" ");
      DEBUG.print(refQuat[2]);
      DEBUG.print(" ");
      DEBUG.println(refQuat[3]);
      #endif

      SDserial.print("C ");
      SDserial.print(millis());
      SDserial.print(" ");
      SDserial.print(refQuat[0]);
      SDserial.print(" ");
      SDserial.print(refQuat[1]);
      SDserial.print(" ");
      SDserial.print(refQuat[2]);
      SDserial.print(" ");
      SDserial.println(refQuat[3]);
      SDserial.flush();

      SessionSet |= SESSION_CALIBRATED;
      DEBUG.print("EEPROM: loaded alarm level: ");
      DEBUG.println(alarmLevel);
    }

    // since everything was rad from EEPROM, flag it's synced
    SessionSet |= SESSION_EEPROMED;

    // if session was already initialized, just autostart
    doRun = true;

  } else {
    // start new session
    doRun = false;
  }

  // Just a delay to annoy people
  delay(500);

  // Try to initialize EMS, if fails repeat
  while ( EMSsetup() != true) {
    errorLog("EMS: Failed to initialize, trying again . . .");
    delay(500);
  }

  // remember time start
  timeStart = millis();

}


void loop() {

  // -----------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------------
  // if initialization failed, do nothing
  if (!initSuccess) {
    delay(1000);
    return;
  }

  // First of all - always check for incoming commands
  // need to check if this won't make BT wake up
  if (DEBUG.available())
  {
    readCmd();
  }

  // if it's not desired to collect data, do not
  if (!doRun)
  {
    return;
  }

  // Dummy software timer
  if (millis() - timeLast < poolingPeriod) {
    return;
  }
  timeLast = millis();

  // -----------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------------

  // Check event status register, way to check data ready by polling rather than interrupt
  uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

  // Check for errors
  // Error detected, what is it?
  if (eventStatus & 0x02)
  {
    uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
    if (!errorStatus)
    {
      if (errorStatus == 0x11) errorLog("EMS: Magnetometer failure!");
      if (errorStatus == 0x12) errorLog("EMS: Accelerometer failure!");
      if (errorStatus == 0x14) errorLog("EMS: Gyro failure!");
      if (errorStatus == 0x21) errorLog("EMS: Magnetometer initialization failure!");
      if (errorStatus == 0x22) errorLog("EMS: Accelerometer initialization failure!");
      if (errorStatus == 0x24) errorLog("EMS: Gyro initialization failure!");
      if (errorStatus == 0x30) errorLog("EMS: Math error!");
      if (errorStatus == 0x80) errorLog("EMS: Invalid sample rate!");

      // report that a measurement cycle is lost due to the error
      SDserial.print("E ");    // E stands for error
      SDserial.print(millis());
      SDserial.print(" ");
      SDserial.println(errorStatus);
      // in case of any of these severe errors quit current loop
      return;
    }
  }

  // if no errors, see if new data is ready
  // new acceleration data available
  if (eventStatus & 0x10)
  {
    readSENtralAccelData(accelCount);

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * 0.000488; // get actual g value
    ay = (float)accelCount[1] * 0.000488;
    az = (float)accelCount[2] * 0.000488;
  }

  if (eventStatus & 0x20)
  {
    // new gyro data available
    readSENtralGyroData(gyroCount);

    // Now we'll calculate the gyro value into actual dps's
    gx = (float)gyroCount[0] * 0.153; // get actual dps value
    gy = (float)gyroCount[1] * 0.153;
    gz = (float)gyroCount[2] * 0.153;
  }

  if (eventStatus & 0x08)
  {
    // new mag data available
    readSENtralMagData(magCount);

    // Now we'll calculate the mag value into actual G's
    // get actual G value
    mx = (float)magCount[0] * 0.305176;
    my = (float)magCount[1] * 0.305176;
    mz = (float)magCount[2] * 0.305176;
  }

  if (eventStatus & 0x04) // new quaternions available
  {
    readSENtralQuatData(rawQuat);

    // if there is reference quaternion, do stuff
    if (SessionSet & SESSION_CALIBRATED)
    {
      // substract reference quaternion to find relative quaternion
      quatProd(refQuatInv, rawQuat, Quat);
      // alarm level detection
    }
    else
    {
      Quat[0] = rawQuat[0];
      Quat[1] = rawQuat[1];
      Quat[2] = rawQuat[2];
      Quat[3] = rawQuat[3];
    }
  }

  float pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1])) * 180.0f / 3.14f;
  float yaw  = atan2(2.0f * (Quat[0] * Quat[1] + Quat[2] * Quat[3]), Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2] + Quat[3] * Quat[3]) * 180.0f / 3.14f;
  float roll = atan2(2.0f * (Quat[1] * Quat[2] + Quat[0] * Quat[3]), Quat[0] * Quat[0] + Quat[1] * Quat[1] - Quat[2] * Quat[2] - Quat[3] * Quat[3]) * 180.0f / 3.14f;   
  // if alarm anabled, do alarm stuff
  if(SessionSet & SESSION_ALARM_EN)
  {
    if (pitch < alarmLevel)
    {
        // turn on buzzer for duration of pooling period
        tone(BUZ_PIN, BUZ_TONE, poolingPeriod); 
    }
  }

  // -----------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------------
  // As we have data we wish to log it
  // write data to sd card, hopefully won't kill it that many in one sequence
  SDserial.print("M ");    // M stands for measurement
  SDserial.print(millis());
  SDserial.print(" ");
  SDserial.print(rawQuat[0]);
  SDserial.print(" ");
  SDserial.print(rawQuat[1]);
  SDserial.print(" ");
  SDserial.print(rawQuat[2]);
  SDserial.print(" ");
  SDserial.print(rawQuat[3]);
  SDserial.print(" ");
  SDserial.print(ax);
  SDserial.print(" ");
  SDserial.print(ay);
  SDserial.print(" ");
  SDserial.print(az);
  SDserial.print(" ");
  SDserial.print(gx);
  SDserial.print(" ");
  SDserial.print(gy);
  SDserial.print(" ");
  SDserial.print(gz);
  SDserial.print(" ");
  SDserial.print(mx);
  SDserial.print(" ");
  SDserial.print(my);
  SDserial.print(" ");
  SDserial.print(mz);
  SDserial.println("");

#ifdef doDebug
  DEBUG.print("Pitch:  ");
  DEBUG.print(pitch);
  DEBUG.print(" Roll:  ");
  DEBUG.print(roll);
  DEBUG.print(" QUAT:  ");
  DEBUG.print(Quat[0]);
  DEBUG.print(" ");
  DEBUG.print(Quat[1]);
  DEBUG.print(" ");
  DEBUG.print(Quat[2]);
  DEBUG.print(" ");
  DEBUG.print(Quat[3]);
  DEBUG.print(" Timestamp: ");
  DEBUG.println(millis());
#endif

}


