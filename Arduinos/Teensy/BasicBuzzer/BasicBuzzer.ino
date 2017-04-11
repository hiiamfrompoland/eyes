
#define buzzerPin   10
int buzzerFreq  = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(250);
  tone(buzzerPin, buzzerFreq);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  noTone(buzzerPin);
  buzzerFreq = buzzerFreq + 100;

  if(buzzerFreq > 20000) {
    buzzerFreq = 100;
  }
  
  tone(buzzerPin, buzzerFreq);
  Serial.print("Frequency ");
  Serial.print(buzzerFreq);
  Serial.println("Hz");
}
