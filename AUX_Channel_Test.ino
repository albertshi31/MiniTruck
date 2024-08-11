const int inputPin = 10;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  unsigned long highTime = pulseIn(inputPin, HIGH);
  unsigned long lowTime = pulseIn(inputPin, LOW);

  unsigned long cycleTime = highTime + lowTime;
  float dutyCycle = (float)highTime / float(cycleTime);

  Serial.println(dutyCycle);

  // if (dutyCycle < 0.07) {
  //   Serial.println("speak");
  //   delay(5000);  // Add delay to avoid spamming
  // }
  // // else if (dutyCycle > 0.07 && dutyCycle < 0.10) {
  // else if (dutyCycle > 0.09) {
  //   Serial.println("video");
  //   delay(5000);
  // }
}
