const int phaseAPin = 2; // Digital pin connected to Phase A
const int phaseBPin = 3; // Digital pin connected to Phase B
const int phaseCPin = 4; // Digital pin connected to Phase C
const int pwmPin = 10;    // PWM output pin (you can use 3, 5, 6, 9, 10, or 11 on most Arduinos)
const int frequency = 50; // Desired frequency in Hz

const float radius = 0.05; // Radius of the wheel/pulley in meters (example value)

volatile int currentState = 0;
volatile int lastState = 0;
volatile unsigned long lastTime = 0;
float velocity = 0;
float linearVelocity = 0;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 bps

  pinMode(phaseAPin, INPUT); // Set Phase A pin as input
  pinMode(phaseBPin, INPUT); // Set Phase B pin as input
  pinMode(phaseCPin, INPUT); // Set Phase C pin as input

  // Set the PWM pin as an output
  pinMode(pwmPin, OUTPUT);

  // Configure the PWM frequency
  setPwmFrequency(pwmPin, frequency);

  // Attach interrupts to the Hall sensor pins
  attachInterrupt(digitalPinToInterrupt(phaseAPin), updateState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseBPin), updateState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseCPin), updateState, CHANGE);

  lastTime = micros(); // Initialize time
}

void loop() {
  // Set the duty cycle to 7.5%
  analogWrite(pwmPin, 19); // 7.5% duty cycle

  delay(200);

  analogWrite(pwmPin, 0);

  delay(2000); // 20ms delay to match the period of the PWM signal
}

void updateState() {
  // Combine the sensor states into a single number
  int phaseA = digitalRead(phaseAPin); // Read Phase A
  int phaseB = digitalRead(phaseBPin); // Read Phase B
  int phaseC = digitalRead(phaseCPin); // Read Phase C

  currentState = (phaseA << 2) | (phaseB << 1) | phaseC;

  if (currentState != lastState) {
    unsigned long currentTime = micros();
    unsigned long timeDiff;

    // Calculate the time difference considering overflow
    if (currentTime >= lastTime) {
      timeDiff = currentTime - lastTime;
    } else {
      timeDiff = (4294967295UL - lastTime) + currentTime + 1;
    }

    if (timeDiff > 0) {
      // Calculate velocity (angle change per unit time)
      velocity = 60000000.0 / timeDiff; // Degrees per second (assuming 60-degree steps)
      float angularVelocityRad = velocity * (PI / 180.0); // Convert to radians per second
      linearVelocity = angularVelocityRad * radius; // Calculate linear velocity

      Serial.println("<<<<<<<<<<<<<<");
      Serial.print("Phase A: "); Serial.print(phaseA);
      Serial.print(", Phase B: "); Serial.print(phaseB);
      Serial.print(", Phase C: "); Serial.println(phaseC);
      Serial.print("Current State: "); Serial.println(currentState, BIN);
      Serial.print("Time Diff: "); Serial.println(timeDiff);
      Serial.print("Linear Velocity: "); Serial.print(linearVelocity);
      Serial.println(" m/s");
    }

    lastTime = currentTime;
    lastState = currentState;
  }
}

// Function to estimate angle based on Hall sensor state
int getAngle(int state) {
  switch (state) {
    case 0b101: return 0;
    case 0b100: return 60;
    case 0b110: return 120;
    case 0b010: return 180;
    case 0b011: return 240;
    case 0b001: return 300;
    default: return -1; // Invalid state
  }
}

void setPwmFrequency(int pin, int frequency) {
  int prescaleValue;
  int timer;

  if (pin == 5 || pin == 6) {
    timer = 0;
  } else if (pin == 9 || pin == 10) {
    timer = 1;
  } else if (pin == 3 || pin == 11) {
    timer = 2;
  } else {
    return; // Not a valid PWM pin
  }

  switch (frequency) {
    case 50:
      prescaleValue = 0x05; // 1024 prescale value for Timer1 (50Hz approx)
      break;
    default:
      return; // Unsupported frequency
  }

  if (timer == 1) {
    TCCR1B = TCCR1B & 0b11111000 | prescaleValue;
  }
}
