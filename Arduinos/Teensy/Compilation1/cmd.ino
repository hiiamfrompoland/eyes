/**
 * Command functions will will usualy have header as:
 * void cmdCommandName()
 * if they accept arguments then
 * void cmdCommandName(String* args, uint8_t argn)
 * where args is list of arguments (Strings) and argn is number of arguments
 */
void cmdStart()
{
    // if all was set or it's second time we ask for start
    if (((SessionSet & SESSION_NAME_SET)&& (SessionSet & SESSION_CALIBRATED1) && (SessionSet &  SESSION_CALIBRATED2)) || (SessionSet & SESSION_FORCED))
    {
      // start measurement
      doRun = true;
      DEBUG.println("CMD: measurement start");
    }
    else
    {
      DEBUG.println("CMD: name/calibration not set yet, type start to proceed or set missing properties");
      SessionSet |= SESSION_FORCED;
    }
}
void cmdStop()
{
    doRun = false;
    DEBUG.println("CMD: measurement stop");
}
void cmdSetTime(String* args, uint8_t argn)
{
    // synchronize RTC clock by providing data as dd/mm/yyyy
    if (argn < 2) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    // parse HH:mm:ss and dd/mm/yyyy to timestamp
    // and synchronize RTC

}
void cmdSetTimestamp(String* args, uint8_t argn)
{
    // syncrhonize RTC clock by providing data as a timestamp
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    // synchronize RTC
    uint32_t stamp = (args[0]).toInt();
    syncRTC((time_t) stamp); 
    DEBUG.println("CMD: new timestamp set");
}

void cmdReturnTimestamp()
{
    // display timestamp back
    DEBUG.print("CMD: current time ");
    DEBUG.println(now());
}
void cmdReturnTime()
{
    // return current time in nice format
    DEBUG.print("CMD: current time ");
    DEBUG.print(hour());
    DEBUG.print(":");
    DEBUG.print(minute());
    DEBUG.print(":");
    DEBUG.print(second());
    DEBUG.print(" ");
    DEBUG.print(day());
    DEBUG.print("/");
    DEBUG.print(month());
    DEBUG.print("/");
    DEBUG.println(year());
}
void cmdSetSessionName(String* args, uint8_t argn)
{
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    // set the name/id of current measurement so it's easier to find on sd disk
    SessionName = args[0];
    SessionSet &= ~SESSION_EEPROMED;

    SDappend(SessionName);
    SDserial.print("I ");
    SDserial.print(millis());
    SDserial.print(" ");
    SDserial.println(SessionName);


    DEBUG.print("CMD: new session name: ");
    DEBUG.println(SessionName);
}
void cmdSetAlarmLevel(String* args, uint8_t argn)
{
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    // set level of the alarm
    alarmLevel = (args[0]).toFloat();
    SessionSet &= ~SESSION_EEPROMED;
    DEBUG.print("CMD: new alarm level: ");
    DEBUG.println(alarmLevel);
}
void cmdSetAlarmEnabled(String* args, uint8_t argn)
{
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    if ( (args[0]).equals("true") )
    {
      SessionSet |= SESSION_ALARM_EN;
      DEBUG.print("CMD: alarm enabled");
    } 
    else 
    {
      SessionSet &= ~SESSION_ALARM_EN;
      DEBUG.print("CMD: alarm disabled");
    }
}
void cmdSetPoolingPeriod(String* args, uint8_t argn)
{
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    // set level of the alarm
    poolingPeriod = (args[0]).toInt();
    SessionSet &= ~SESSION_EEPROMED;
    SessionSet |= ~SESSION_POOLING_SET;
    DEBUG.print("CMD: new pooling period: ");
    DEBUG.println(poolingPeriod);
}
void cmdSave()
{
  
    // save session name
    struct {
      char n[9];
    } sessname;
    SessionName.toCharArray(sessname.n, 9);
    sessname.n[8] = 0;
    EEPROM.put(SESS_SESSION_NAME_ADDR, sessname);

    // save reference quaternions
    EEPROM.put(SESS_INIT_Q1_ADDR, refQuat1[0]);
    EEPROM.put(SESS_INIT_Q2_ADDR, refQuat1[1]);
    EEPROM.put(SESS_INIT_Q3_ADDR, refQuat1[2]);
    EEPROM.put(SESS_INIT_Q4_ADDR, refQuat1[3]);
    // save reference quaternions
    EEPROM.put(SESS_INIT_Q5_ADDR, refQuat2[0]);
    EEPROM.put(SESS_INIT_Q6_ADDR, refQuat2[1]);
    EEPROM.put(SESS_INIT_Q7_ADDR, refQuat2[2]);
    EEPROM.put(SESS_INIT_Q8_ADDR, refQuat2[3]);

    // save alarm level
    EEPROM.put(SESS_ALARM_LEVEL_ADDR, alarmLevel);
    
    // save pooling period
    EEPROM.put(SESS_POOLING_ADDR, poolingPeriod);

    // save flags
    EEPROM.put(SESS_ON_ADDR, SESS_ON | SESS_NAME_SET | SESS_CALIBRATED1 | SESS_CALIBRATED2 | SESS_ALARM_SET | SESS_POOLING_SET);

    SessionSet |= SESSION_EEPROMED;
    DEBUG.println("CMD: session saved to EEPROM");
}
void cmdForget()
{
    // forget the session so during next reset it will start from scratch
    doRun = false;
    EEPROM.write(SESS_ON_ADDR, 0x00);
    SessionSet = 0;
    DEBUG.println("CMD: session forgotten");
}
void cmdDump(String* args, uint8_t argn)
{
    // Just dump all the data from measurement into a DEBUG stream
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    if (argn == 3) {
      uint32_t start = (args[1]).toInt();
      uint32_t stop = (args[2]).toInt();
      SDdump2(args[0], start, stop);
      return;
    } 
    else 
    {
      SDdump(args[0]);
    }
    
    // set hi speed baudrate to receive loads of data
    //SDsetBaud( SDhispdBaud );
    // dump data
    // restore original baudrate
    //SDsetBaud( SDdefBaud );
}
void cmdDelete(String* args, uint8_t argn)
{
    // Just dump all the data from measurement into a DEBUG stream
    if (argn < 1) {
      DEBUG.println("CMD: Not enough arguments");
      return;
    }
    SDrm(args[0]);
}
void cmdCalibrate1()
{
    // perform a calibration
    // which is to remember quaternions at current position

    // in case something goes wrong, give them 10 attempts
    for (uint8_t i = 0; i < 10; i++)
    {
      uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

      // Check for errors
      if (eventStatus & 0x02)
      {
        DEBUG.print("CMD: error occured while calibration: ");
        DEBUG.println(eventStatus);
        DEBUG.println("CMD: Trying again...");
      }

      if (eventStatus & 0x04) // new quaternions available
      {
        // if there is quaternion data, read it, store it and exit loop
        readSENtralQuatData(refQuat1);
        float refQuat1Conj[4];
        quatConj(refQuat1, refQuat1Conj);

        float norm = quatNorm(refQuat1Conj);
        refQuat1Inv[0] = refQuat1Conj[0] / norm;
        refQuat1Inv[1] = refQuat1Conj[1] / norm;
        refQuat1Inv[2] = refQuat1Conj[2] / norm;
        refQuat1Inv[3] = refQuat1Conj[3] / norm;
        DEBUG.println("CMD: saved calibrated data 1");
        DEBUG.print(refQuat1[0]);
        DEBUG.print(" ");
        DEBUG.print(refQuat1[1]);
        DEBUG.print(" ");
        DEBUG.print(refQuat1[2]);
        DEBUG.print(" ");
        DEBUG.println(refQuat1[3]);

        SDserial.print("C1 ");
        SDserial.print(millis());
        SDserial.print(" ");
        SDserial.print(refQuat1[0]);
        SDserial.print(" ");
        SDserial.print(refQuat1[1]);
        SDserial.print(" ");
        SDserial.print(refQuat1[2]);
        SDserial.print(" ");
        SDserial.println(refQuat1[3]);
        SDserial.flush();

        // since we set new variable,
        SessionSet &= ~SESSION_EEPROMED;
        SessionSet |= SESSION_CALIBRATED1;
        // exit the loop
        break;
      }
      delay(250);
    }

}
void cmdCalibrate2()
{
    // perform a calibration
    // which is to remember quaternions at current position

    // in case something goes wrong, give them 10 attempts
    for (uint8_t i = 0; i < 10; i++)
    {
      uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

      // Check for errors
      if (eventStatus & 0x02)
      {
        DEBUG.print("CMD: error occured while calibration: ");
        DEBUG.println(eventStatus);
        DEBUG.println("CMD: Trying again...");
      }

      if (eventStatus & 0x04) // new quaternions available
      {
        // if there is quaternion data, read it, store it and exit loop
        readSENtralQuatData(refQuat2);
        float refQuat2Conj[4];
        quatConj(refQuat2, refQuat2Conj);

        float norm = quatNorm(refQuat2Conj);
        refQuat2Inv[0] = refQuat2Conj[0] / norm;
        refQuat2Inv[1] = refQuat2Conj[1] / norm;
        refQuat2Inv[2] = refQuat2Conj[2] / norm;
        refQuat2Inv[3] = refQuat2Conj[3] / norm;
        DEBUG.println("CMD: saved calibrated data 2");
        DEBUG.print(refQuat2[0]);
        DEBUG.print(" ");
        DEBUG.print(refQuat2[1]);
        DEBUG.print(" ");
        DEBUG.print(refQuat2[2]);
        DEBUG.print(" ");
        DEBUG.println(refQuat2[3]);

        SDserial.print("C2 ");
        SDserial.print(millis());
        SDserial.print(" ");
        SDserial.print(refQuat2[0]);
        SDserial.print(" ");
        SDserial.print(refQuat2[1]);
        SDserial.print(" ");
        SDserial.print(refQuat2[2]);
        SDserial.print(" ");
        SDserial.println(refQuat2[3]);
        SDserial.flush();

        // since we set new variable,
        SessionSet &= ~SESSION_EEPROMED;
        SessionSet |= SESSION_CALIBRATED2;
        // exit the loop
        break;
      }
      delay(250);
    }
}
void cmdSelfTest()
{
    // perform auto-test
    float dev[6];
    MPU9250SelfTest(dev);
    DEBUG.print("CMD: Self-test results: ");

    boolean testPassed = true;
    for (uint8_t i = 0; i < 6; i++)
    {
      if (dev[i] > 0.14f || dev[i] < -0.14f) testPassed = false;
      DEBUG.print(dev[i] * 100); DEBUG.print("%, ");
    }

    if (testPassed)
      DEBUG.println("Self test passed");
    else
      DEBUG.println("Self test failed");

}
void cmdSetBaudrate(String* args, uint8_t argn)
{
    // set the serial port baudrate. Might be useful
    // when downloading larger chunk of data
    if (argn < 1) {
      DEBUG.println("CMD: no baudrate specified");
      return;
    }
    DEBUG.flush();

    uint32_t baudrate = (args[0]).toInt();
    if(baudrate == 0) 
    {
      DEBUG.begin(9600);
    
      DEBUG.println("CMD: invalid baudrate");
      return;
    }
    DEBUG.end();
    DEBUG.begin(baudrate);
    delay(1000);
    DEBUG.println("CMD: new baudrate set");
}
/**
   Read command incoming on DEBUG stream and take an action
*/
void readCmd()
{
  // check if anything waiting for us
  if (!DEBUG.available())
  {
    return;
  }
  // each command terminated with newline
  String str = Serial.readStringUntil('\n');
  // name of the command
  String cmd;

  // no more than 8 arguments
  String args[8];
  // number of actually read arguments
  uint8_t argn   = 0;

  // read command
  if (str.length() < 0)
  {
    DEBUG.println("CMD: invalid request");
    return;
  }

  // take the command out of it
  if (str.indexOf(' ') > 0)
  {
    int8_t idx2 = str.indexOf(' ');
    int8_t idx1 = 0;
    uint8_t i   = 0;
    // if there are arguments after command, first extract the command itself
    cmd = str.substring(idx1, idx2);

    // then extract one parameter passed after another
    while ((idx1 = str.indexOf(' ', idx2 + 1)) > 0 && i < 6)
    {
      args[i] = str.substring(idx2 + 1, idx1);
      idx2 = idx1;
      i += 1;
    }

    // what left after last spacebar and before end is the last parameter
    if (idx2 - str.length() > 0)
    {
      args[i++] = str.substring(idx2 + 1);
    }
    // save the nmber of passed arguments
    argn = i;
  } else
  {
    // if no arguments are passed, then the whole line is just the command
    cmd = str;
  }

  // Now actually determine which command was that
  if (cmd.equals("setTime"))
  {
    cmdSetTime(args, argn);
  }
  else if (cmd.equals("setTimestamp"))
  {
    cmdSetTimestamp(args, argn);
  }
  else if (cmd.equals("returnTimestamp"))
  {
    cmdReturnTimestamp();
  }
  else if (cmd.equals("returnTime"))
  {
    cmdReturnTime();
  }
  else if (cmd.equals("setSessionName"))
  {
    cmdSetSessionName(args, argn);
  }
  else if (cmd.equals("setAlarmLevel"))
  {
    cmdSetAlarmLevel(args, argn);
  }
  else if (cmd.equals("setAlarmEnabled"))
  {
    cmdSetAlarmEnabled(args, argn);
  }
  else if (cmd.equals("setPoolingPeriod"))
  {
    cmdSetPoolingPeriod(args, argn);
  }
  else if (cmd.equals("save"))
  {
    cmdSave();
  }
  else if (cmd.equals("dump"))
  {
    cmdDump(args, argn);
  }
  else if (cmd.equals("calibrate1"))
  {
    cmdCalibrate1();
  }
  else if (cmd.equals("calibrate2"))
  {
    cmdCalibrate2();
  }
  else if (cmd.equals("list"))
  {
    // list all files in top level of sdcard
    SDlist();
  }
  else if (cmd.equals("clearSD"))
  {
    // remove everything but config from SD card

  }
  else if (cmd.equals("delete"))
  {
    // remove everything but config from SD card
    cmdDelete(args, argn);
  }
  else if (cmd.equals("start"))
  {
    cmdStart();
  }
  else if (cmd.equals("stop"))
  {
    // stop measurement
    cmdStop();
  }
  else if (cmd.equals("selftest"))
  {
    cmdSelfTest();
  }
  else if (cmd.equals("forget"))
  {
    cmdForget();
  }
  else if (cmd.equals("reset"))
  {
    // aaaand reset the device yay
    // doesn't work yet though
    digitalWrite(RST_PIN, LOW);
  }
  else if (cmd.equals("setBaudrate"))
  {
    cmdSetBaudrate(args, argn);
  }
  else
  {
    // no valid command apparently
    DEBUG.println("CMD: no such command");
  }


}
