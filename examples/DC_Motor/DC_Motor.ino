#include <SPI.h>
#include <TMC5160.h>

TMC5160 stepper;

// Define SPI pins for Arduino Uno
// #define SPI_MOSI_PIN  11  // MOSI (Master Out Slave In)
// #define SPI_MISO_PIN  12  // MISO (Master In Slave Out)
// #define SPI_SCK_PIN   13  // SCK (Serial Clock)
#define SPI_SS_PIN 7

void setup() {
   /* TMC5160 as DC motor driver */
  //  SPI.begin();
  //  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3));

   pinMode(SPI_SS_PIN, OUTPUT);  // Set Slave Select (SS) pin as output
   digitalWrite(SPI_SS_PIN, LOW);  // Deselect the TMC5160 initially

   stepper.begin(&SPI, true, SPI_SS_PIN);  // Initialize TMC5160 with SPI, Slave Select (SS) pin, and default CS setting
   stepper.setDCMotorMode(false);  // No torque limit
   stepper.setDCMotor(0, 0);  // Set DC Torque Mode
  //  stepper.setEncoder(0);  // Enable Encoder
}

void loop() {
   // Set Motor 1 to Max
   stepper.setDCMotor(255, 0);

   delay(5000);

   // Set Motor 2 to Max
   stepper.setDCMotor(0, 255);

   delay(5000);

   // Set Motor 2 to -Max
   stepper.setDCMotor(0, -255);

   delay(5000);

   // All motors off
   stepper.setDCMotor(0, 0);
}
