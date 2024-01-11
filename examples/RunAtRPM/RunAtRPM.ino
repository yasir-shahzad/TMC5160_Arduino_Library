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

//#include <Arduino.h>
#include "src/TMC5160.h"

const uint8_t SPI_CS = SS;      // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 7;  // DRV_ENN pin in SPI mode
uint32_t currentTime = millis();

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);  //Use default SPI peripheral and SPI settings.

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams;  // defaults.
  TMC5160::MotorParameters motorParams;
unsigned long start_time=0;
void setup() {
  // delay(3000);
  // USB/debug serial coms
  Serial.begin(115200);
//  while(!Serial);
//  delay(1000);
  Serial.print("CS:");
  Serial.println(SPI_CS);

  // pinMode(7, INPUT);
  // pinMode(8, INPUT);
  pinMode(SPI_DRV_ENN, OUTPUT);
  digitalWrite(SPI_DRV_ENN, LOW);  // Active low1
   motorParams.globalScaler = 136; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 16;

  SPI.begin();
  if (!motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION)) {
 
    Serial.println("TMC5160 not detected, Please check the wiring diagram");
  } else {
    Serial.println("TMC5160 detected");
  }

  // ramp definition
  motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(0);  // tics/sec  1rpm
  motor.setAcceleration(1500);
  //motor.writeRegister(TMC5160_Reg::TPWMTHRS, 0);
  //motor.setModeChangeSpeeds(100, 10, 10);

  //Serial.println("starting up");

  // Standstill for automatic tuning

  // motor.setMaxSpeed(200);  // tics/sec  1rpm
  // delay(8000);
  // motor.setMaxSpeed(-200);  //tics/sec  1rpm
  // delay(8000);
  start_time=millis();
  Serial.print("Starting Current is 2000 mA\n");
}
bool run=0;
bool countonce=0;
int speed=1000;
int value=300;
//-50;
//-50
//133

int multiplyFactor = 2;
uint32_t delayTime = 15000;

void loop() { 
  // Serial.println("RPM120");
  // motor.setMaxSpeed(120*multiplyFactor); 
  // delay(delayTime);
  Serial.println("RPM100");
  motor.setMaxSpeed(100*multiplyFactor); 
  // delay(delayTime);
  // Serial.println("RPM80");
  // motor.setMaxSpeed(80*multiplyFactor); 
  while(1){

      motor.getDriverStatus();
      // Serial.println(currentSpeed);
      delay(500);
      //  motor.setMaxSpeed(0*multiplyFactor);
  }
  delay(delayTime);
  Serial.println("RPM50");
  motor.setMaxSpeed(50*multiplyFactor); 
  delay(delayTime);
  Serial.println("RPM30");
  motor.setMaxSpeed(30*multiplyFactor); 
  delay(delayTime);
    // motor.setMaxSpeed(100*multiplyFactor); 
  // /delay(delayTime);
  // int vmax = 120;
  // for(int i = 0; i<= vmax; i++)
  // {
  // motor.setMaxSpeed(i*multiplyFactor); 
  // delay(200);
  // Serial.println(i);
  // }
  // delay(delayTime);

  if(motor.isIcRest()) {
      motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
        motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(0);  // tics/sec  1rpm
  motor.setAcceleration(1500);
  }
  //Serial.println("I am inside the loop");
//      if(Serial.available()>0)
// {
//   String data = Serial.readString();
//    // Convert the string to an integer
//     value = data.toInt();
//     // Print the converted value
//     Serial.print("Received integer value: ");
//     Serial.println(value);
// }

// delay(15000);
//  motor.setMaxSpeed(-133);
//  delay(15000);
  // if (Serial.available() > 0) {
  //   char input = Serial.read();
  //   if (input == '1') {
  //     speed = speed + 100;
  //     Serial.print("Motor Speed is  "); // Use print instead of println
  //     Serial.println(speed);
  //     // Set the motor speed using your motor controller library
  //   }
  //   if (input == '0') {
  //     speed = 0;
  //     Serial.println("Motor Stopped");
  // //    motor.setMaxSpeed(speed);
  //     // Set the motor speed to stop using your motor controller library
  //   }
  //       if (input == '2') {
  //     speed = speed-100;
  //     Serial.println("Motor Speed is:");
  //        Serial.println(speed);
    //  motor.setMaxSpeed(speed);
      // Set the motor speed to stop using your motor controller library
    }
  
  //}
//}

// void loop() {
// if(Serial.available()>0)
// {
// char input=Serial.read();
// if(input=='1')
// {
//   speed=speed+10;
//   Serial.println("Motor Speed is "+String(speed));
// motor.setMaxSpeed(speed);
// }
// if(input=='0')
// {
// speed=0;
//   Serial.println("Motor Stopped);
// motor.setMaxSpeed(speed); 
// }
// }
// }


/*if(millis()-start_time>5000 && countonce==0)
{
countonce=1;
Serial.print("Starting Current is 1600 mA");
motorParams.globalScaler = 136;  // Adapt to your driver and motor ("Selecting sense resistors")
}if(Serial.available()>0)
{
char input=Serial.read();
if(input=='0')
{
Serial.println("Zero obtained, MOTOR IS GOING TO STOP");
  motor.setMaxSpeed(0);
}
else if(input=='1')
{
Serial.println("One obtained Motor is going to start");
  motor.setMaxSpeed(-80);
}
else if(input=='3')
{
motor.setMaxSpeed(150);
}
else if(input=='4')
{
  Serial.println("Current is decreased now");
  motorParams.globalScaler = 136;  // Adapt to your driver and motor ("Selecting sense resistors")
 // motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
   motor.setcurrent(136);
   motor.setMaxSpeed(-80);

}
else if(input=='5')
{
  Serial.println("Current is increased again");
 // motorParams.globalScaler = 174;  // Adapt to your driver and motor ("Selecting sense resistors")
   motor.setcurrent(174);
  //motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  //writeRegister(TMC5160_Reg::GLOBAL_SCALER, constrain(motorParams.globalScaler, 32, 256));
   motor.setMaxSpeed(-80);
}
}*/

  // if(motor.isIcRest()) {
  //      TMC5160::PowerStageParameters powerStageParams; // defaults.
  //     TMC5160::MotorParameters motorParams;
  //     motorParams.globalScaler = 90; // Adapt to your driver and motor ("Selecting sense resistors")
  //     motorParams.irun = 25;
  //     motorParams.ihold = 1;
  //     motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  //         // ramp definition
  //     motor.setRampMode(TMC5160::VELOCITY_MODE);
  //     motor.setMaxSpeed(0);   // tics/sec  1rpm
  //     motor.setAcceleration(120);
  // }
  // int a = motor.getDriverStatus();

  // motor.setMaxSpeed(20);  // tics/sec  1rpm
  // currentTime = millis();
  // while(millis()-currentTime > 8000) {
  // motor.getDriverStatus();
  // delay(100);
  // }

  // motor.setMaxSpeed(200);  //tics/sec  1rpm
  // currentTime = millis();
  // while (millis() - currentTime > 8000) {
  // motor.getDriverStatus();
  // delay(100);
  // }

//  motor.setMaxSpeed(-80);
  // motor.setMaxSpeed(0);
  // delay(1000);
  //   delay(3000);

  // const char *myString = motor.getDriverStatusDescription(a);
  //  Serial.println(myString);
  /*
  if(Serial.available()>0)
{
char input=Serial.read();
if(input=='0')
{
Serial.println("Zero obtained, MOTOR IS GOING TO STOP");
  speed=0;
}
else if(input=='1')
{
//Serial.println("One obtained Motor is going to start");
 //motor.setMaxSpeed(-80);
Serial.println("Motor Speed=-40");
speed=-40;
}
else if(input=='3')
{
//motor.setMaxSpeed(150);
Serial.println("Motor Speed=40");
speed=40;
}
else if(input=='4')
{
  //Serial.println("Current is decreased now");
  //motorParams.globalScaler = 136;  // Adapt to your driver and motor ("Selecting sense resistors")
 // motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  // motor.setcurrent(136);
   Serial.println("Motor Speed=80");
speed=80;
}
else if(input=='5')
{
  Serial.println("Current is increased again");
 // motorParams.globalScaler = 174;  // Adapt to your driver and motor ("Selecting sense resistors")
 //  motor.setcurrent(174);
  //motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  //writeRegister(TMC5160_Reg::GLOBAL_SCALER, constrain(motorParams.globalScaler, 32, 256));
   Serial.println("Motor Speed=-80");
speed=-80;
}
}*/