#include <SoftwareSerial.h>  

#define BTserial  Serial1

void setup()
{
  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  BTserial.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  delay(100);  // Short delay, wait for the Mate to send back CMD
}

void loop()
{
  if(BTserial.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)BTserial.read());  
  }
  if(Serial.available())  // If stuff was typed in the serial monitor
  {
    // Send any characters the Serial monitor prints to the bluetooth
    BTserial.print((char)Serial.read());
  }
  // and loop forever and ever!
}

