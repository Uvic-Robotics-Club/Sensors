int fanPin = 11;

int fanSpeed = 150;


void setup() {
Serial.begin(9600);
}


void loop() {
  analogWrite(fanPin, fanSpeed);
 Serial.println(fanSpeed);
}
