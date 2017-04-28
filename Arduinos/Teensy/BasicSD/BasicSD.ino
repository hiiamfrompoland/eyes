#define SDserial  Serial2
#define SDgrn     12

void setup()
{
  // put your setup code here, to run once:

  // Reset SD card
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(100);
  digitalWrite(12, HIGH);
  delay(100);

  // Setup
  Serial.begin(9600);
  SDserial.begin(9600);
  delay(1000);

  while (!SDserial.available());

  SDserial.write((char)26);
  SDserial.write((char)26);
  SDserial.write((char)26);

  Serial.println("Hit me!");
}

void loop()
{
  char c;
  if (SDserial.available())
  {
    Serial.print((char)SDserial.read());
  }
  if (Serial.available())
  {
    c = (char)Serial.read();
    if (c == '%')
    {
      SDserial.write((char)26);
    } else {
      SDserial.print(c);
      Serial.print(c);
    }
  }
}

