/***************************************************************************
  AS5600_simple.ino
  Modified by ChatGPT

  This code reads the angle from the AS5600 magnetic encoder and prints the tared angle.
***************************************************************************/

// ----- Libraries
#include <Wire.h>  // This is for I2C

// ----- AS5600 Magnetic sensor
int magnetStatus = 0;  // value of the status register (MD, ML, MH)

int lowbyte;  // raw angle bits[7:0]
word highbyte;  // raw angle bits[11:8]
int rawAngle;  // final raw angle bits[11:0]
float degAngle;  // raw angle in degrees (360/4096 * [value between 0-4095])

float startAngle = 0;  // starting angle
float taredAngle = 0;  // tared angle - based on the startup value

// ----------------
// setup()
// ----------------
void setup() {
  Serial.begin(115200);  // start serial
  Wire.begin();  // start I2C
  Wire.setClock(400000L);  // fast clock

  // ----- flush the buffers
  Serial.flush();  // clear TX buffer
  while (Serial.available()) Serial.read();  // clear RX buffer

  Serial.println("Initializing...");

  checkMagnetPresence();  // check the magnet (blocks until magnet is found)

  Serial.println("Calibrating");

  readRawAngle();  // make initial reading so the degAngle gets updated
  startAngle = degAngle;  // update startAngle with degAngle - for taring

  Serial.print("Start angle: ");
  Serial.println(startAngle);
}

// ----------------
// loop()
// ----------------
void loop() {
  display_angle();  // display the current tared angle
  delay(100);  // delay for readability
}

// ==========================================================================
// display_angle()
// ==========================================================================
void display_angle() {
  readRawAngle();  // ask the value from the sensor
  tareAngle();  // tare the value

  Serial.print("Tared angle: ");
  Serial.println(taredAngle);

  Serial.print("Angle:");
  Serial.println(taredAngle);
}

// ----------------
// readRawAngle()
// ----------------
void readRawAngle() {
  //----- read low-order bits 7:0
  Wire.beginTransmission(0x36);  // connect to the sensor
  Wire.write(0x0D);  // register map: Raw angle (7:0)
  Wire.endTransmission();  // end transmission
  Wire.requestFrom(0x36, 1);  // request from the sensor
  while (Wire.available() == 0);  // wait until it becomes available
  lowbyte = Wire.read();  // Reading the data after the request

  // ----- read high-order bits 11:8
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);  // register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  highbyte = Wire.read();

  // ----- combine bytes
  highbyte = highbyte << 8;  // shift highbyte to left
  rawAngle = highbyte | lowbyte;  // combine bytes to get 12-bit value 11:0
  degAngle = rawAngle * 0.087890625;  // 360/4096 = 0.087890625
}

// ----------------
// tareAngle()
// ----------------
void tareAngle() {
  // ----- recalculate angle
  taredAngle = degAngle - startAngle;  // this tares the position

  if (taredAngle < 0) {  // if the calculated angle is negative, we need to "normalize" it
    taredAngle = taredAngle + 360;  // correction for negative numbers (i.e. -15 becomes +345)
  }
}

// ----------------------
// checkMagnetPresence()
// ----------------------
void checkMagnetPresence() {
  // Status register output: 0 0 MD ML MH 0 0 0
  // MH: Too strong magnet - 100111 - DEC: 39
  // ML: Too weak magnet - 10111 - DEC: 23
  // MD: OK magnet - 110111 - DEC: 55

  // ---- Check MD status bit (magnet detected)
  while ((magnetStatus & B00100000) != 32) {  // locks MCU until magnet correctly positioned
    magnetStatus = 0;  // reset reading

    Wire.beginTransmission(0x36);  // connect to the sensor
    Wire.write(0x0B);  // register map: Status: MD ML MH
    Wire.endTransmission();  // end transmission
    Wire.requestFrom(0x36, 1);  // request from the sensor
    while (Wire.available() == 0);  // wait until it becomes available
    magnetStatus = Wire.read();  // Reading the data after the request
    Serial.print("Magnet Status: ");
    Serial.println(magnetStatus, BIN);
    Serial.println(" ");
  }
  delay(1000);
}
