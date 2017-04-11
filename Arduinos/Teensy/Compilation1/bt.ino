
void BTsetup()
{

  // Teensy can handle high baudrates, but let's see if in low power mode
  // it will be still fine
  BTserial.begin(115200);

}
void BTcmd(boolean s)
{
  if (s)
  {
    BTserial.print("$");
    BTserial.print("$");
    BTserial.print("$");
  } else {
    BTserial.println("R,1");
  }
  delay(100);
}
void BTlowmode(boolean s)
{
  BTcmd(true);

  if (s) {
    // lowpower = true
    BTserial.println("SY,0000");  // set lower output power, need to check empirically which is good enough
    BTserial.println("S|,FF01");  // practically always off
  } else {
    // lowpower = false
    BTserial.println("SY,0010");
    BTserial.println("S|,0000");
  }

  BTcmd(false);
}
void BTsetName(char* newName)
{
  BTcmd(true);
  BTserial.print("SN,");
  BTserial.println(newName);
  BTcmd(false);
}
