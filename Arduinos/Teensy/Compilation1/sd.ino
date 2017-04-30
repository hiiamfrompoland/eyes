
/**
   Reset sdcard device
*/
inline void SDreset()
{
  digitalWrite(SDgrn, LOW);
  delay(100);
  digitalWrite(SDgrn, HIGH);
  delay(100);
}

/**
 * Change baudrate of the device and reestablish connection
 */
 void SDsetBaud(uint32_t baudrate)
 {  
    // enter cmd mode
    SDcmd(true);

    // send cmd
    SDserial.print("baud\r");

    // wait for prompt to input
    SDserial.print(baudrate);
    SDserial.print("\r");
    SDserial.flush();

    // delay a bit
    delay(500);
    SDserial.end();
  
    // reset SD thing
    SDreset();

    // wait a bit
    delay(500);

    // setup serial connection
    SDserial.begin(baudrate);

    // wait until it prints back stuff  
    while (!SDserial.available());


    // flush stuff
    while (SDserial.available()) SDserial.read();
 }

/**
   Setup SD card connectivity
   @return false if detected that SD card can't be operated properly
*/
boolean SDsetup(boolean cmd)
{
  // Set reset pin to output mode and keep high
  // driving it low will reset SDcard reader
  pinMode(SDgrn, OUTPUT);
  digitalWrite(SDgrn, HIGH);

  // setup serial connection to SDcard reader
  SDserial.begin(SDdefBaud);

  // if one want's to use SDcard just after setting it up, it's necessary
  // to wait a bit, thus we can set the delay right now
  while (!SDserial.available());

  // we should check for any error during opening

  // enter command mode
  if (cmd) {
    SDcmd(true);
    delay(100);
  }

  return true;
}

/**
   Enter command mode on SDcard
*/
void SDcmd(boolean doWait)
{
  // First we take everything out and discard, since
  // nobody bother to use that data
  while (SDserial.available()) SDserial.read();

  SDserial.write((char)26);
  SDserial.write((char)26);
  SDserial.write((char)26);
  SDserial.flush();

  // after sending '$$$' we need to wait for message back
  if (!doWait) return;
  // read all specila chars until '>'
  char c = 0;
  /*while (true) {
    while (!SDserial.available());
    if ( (c=SDserial.read()) == '>') break;
    Serial.print(c);
  }*/
  while ((c=SDserial.read()) != '>');
}
/**
   Quit SD command mode
*/
void SDcmdQuit()
{
  // First we take everything out and discard, since
  // nobody bother to use that data
  while (SDserial.available()) SDserial.read();
  SDserial.print("append ");
  SDserial.print(SessionName);  // SessionName is a global variable holding to which file we currently output
  SDserial.print("\r");

  while (true)
  {
    while (!SDserial.available());
    // wait for char indicating ready for writing data
    if ( SDserial.read() == '<' ) break;
  }
  
}
/**
   Enter command mode on SDcard
*/
void SDlist()
{
  char c;

  SDcmd(true);
  SDserial.print("ls\r");

  DEBUG.println("CMD: Reading list");
  while (!SDserial.available());
  while ((c = SDserial.read()) != '>')
  {
    DEBUG.write(c);
    while (!SDserial.available());
  }
  DEBUG.println("CMD: End of list");
  DEBUG.flush();

  // quit command mode
  SDcmdQuit();
}
/**
   Enter command mode on SDcard
*/
void SDrm(String filename)
{
  char c;

  SDcmd(true);
  SDserial.print("rm ");
  SDserial.print(filename);
  SDserial.print("\r");

  DEBUG.println("CMD: removing file");
  while (!SDserial.available());
  while ((c = SDserial.read()) != '>')
  {
    DEBUG.write(c);
    while (!SDserial.available());
  }
  DEBUG.flush();

  // quit command mode
  SDcmdQuit();
}
/**
   Dump data to stream
*/
void SDdump(String filename)
{
  SDcmd(true);
  SDserial.print("read ");
  SDserial.print(filename);
  SDserial.print("\r");

  DEBUG.print("CMD: Dumping ");
  DEBUG.println(filename);
  char c;
  while (true)
  {
    while (!SDserial.available());
    c = SDserial.read();
    // everything read, back in cmd mode
    if (c == '>') break;
    DEBUG.write(c);
  }
  DEBUG.println("CMD: Dumped");
  DEBUG.flush();

  SDcmdQuit();
}

/**
   Dump data to stream
*/
void SDdump2(String filename, uint32_t start, uint32_t stop)
{
  SDcmd(true);
  SDserial.print("read ");
  SDserial.print(filename);
  SDserial.print(" ");
  SDserial.print(start);
  SDserial.print(" ");
  SDserial.print(stop);
  SDserial.print("\r");

  DEBUG.print("CMD: Dumping ");
  DEBUG.println(filename);
  char c;
  while (true)
  {
    while (!SDserial.available());
    c = SDserial.read();
    // everything read, back in cmd mode
    if (c == '>') break;
    DEBUG.write(c);
  }
  DEBUG.println("CMD: Dumped");
  DEBUG.flush();

  SDcmdQuit();
}
/**
   Set to which file data will be appended
*/
void SDappend(String filename)
{
  SDcmd(true);

  // continue session
  SDserial.print("append ");
  SDserial.print(filename);
  SDserial.print("\r");

  while (true)
  {
    while (!SDserial.available());
    // wait for char indicating ready for writing data
    if ( SDserial.read() == '<' ) break;
  }

}
