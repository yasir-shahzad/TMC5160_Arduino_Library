/* TMC5160 SPI example

This code demonstrates the usage of a Trinamic TMC5160 stepper driver in SPI mode.

Hardware setup :
Connect the following lines between the microcontroller board and the TMC5160 driver
(Tested with a Teensy 3.2 and a TMC5160-BOB)

  MOSI (Teensy : 11)  <=> SDI
  MISO (Teensy : 12)  <=> SDO
  SCK (Teensy : 13)   <=> SCK
  5                   <=> CSN
  8                   <=> DRV_ENN (optional, tie to GND if not used)
  GND                 <=> GND
  3.3V/5V             <=> VCC_IO (depending on the processor voltage)

The TMC5160 VS pin must also be powered.
Tie CLK16 to GND to use the TMC5160 internal clock.
Tie SPI_MODE to VCC_IO, SD_MODE to GND.

Please run the Config Wizard for power stage fine tuning / motor calibration (this code uses the default parameters and auto calibration).

Copyright (c) 2020 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include "src/TMC5160.h"

const uint8_t SPI_CS = SS;       // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 9;  // DRV_ENN pin in SPI mode

TMC5160_SPI motor = TMC5160_SPI(SPI_CS); //Use default SPI peripheral and SPI settings.
unsigned long t_dirchange = 0;
unsigned long t_echo;

void setup()
{
    delay(3000);
    // USB/debug serial coms
    Serial.begin(115200);
    Serial.print("CS:");
    Serial.println(SPI_CS);
    //pinMode(7, INPUT);
    //pinMode(8, INPUT);
    pinMode(SPI_DRV_ENN, OUTPUT);
    digitalWrite(SPI_DRV_ENN, LOW); // Active low

    // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
    TMC5160::PowerStageParameters powerStageParams; // defaults.
    TMC5160::MotorParameters motorParams;
    motorParams.globalScaler = 96; // Adapt to your driver and motor ("Selecting sense resistors")
    motorParams.irun = 15;
    motorParams.ihold = 5;

    SPI.begin();
    if(!motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION)){

    Serial.println("TMC5160 not detected, Please check the wiring diagram");
    }

    // ramp definition
    motor.setRampMode(TMC5160::VELOCITY_MODE);
    motor.setMaxSpeed(-266);   // tics/sec  1rpm
    motor.setAcceleration(500);

    //motor.writeRegister(TMC5160_Reg::TPWMTHRS, 0);
   //motor.setModeChangeSpeeds(100, 10, 10);

    //Serial.println("starting up");

    delay(1000); // Standstill for automatic tuning
}

void loop()
{
    int a = motor.getDriverStatus();
    const char* myString = motor.getDriverStatusDescription(a);
    Serial.println(myString);
}
