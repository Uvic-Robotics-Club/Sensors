int fanPin = 11;

int full = 255;
int half = 128;
int quater = 63.75; //1/3 == 85 || 1/4 == 63.75


void setup() {
  Serial.begin(9600);
}


void loop() {
  analogWrite(fanPin, full);
  delay(5000);


  analogWrite(fanPin, half);
  delay(5000);


  analogWrite(fanPin, quater);
  delay(5000);
}
