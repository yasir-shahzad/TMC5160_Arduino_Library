

//#include <Arduino.h>
#include "TMC5160.h"

const uint8_t SPI_CS = SS;      // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 7;  // DRV_ENN pin in SPI mode
uint32_t currentTime = millis();

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);  //Use default SPI peripheral and SPI settings.

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !

unsigned long start_time=0;
void setup() {
  // delay(3000);
  // USB/debug serial coms
  Serial.begin(115200);
//  while(!Serial);
//  delay(1000);
  Serial.print("CS:");
  Serial.println(SPI_CS);


  pinMode(SPI_DRV_ENN, OUTPUT);
  digitalWrite(SPI_DRV_ENN, LOW);  // Active low1


  SPI.begin();
  if (!motor.begin()) {
      Serial.println("TMC5160 not detected, Please check the wiring diagram");
  }
  else {
      Serial.println("TMC5160 detected");
  }

  // ramp definition
  motor.setRampMode(VELOCITY_MODE);
  motor.setMaxSpeed(0);  // tics/sec  1rpm
  motor.setAcceleration(1500);
  //motor.writeRegister(ADDRESS_TPWMTHRS, 0);
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
      motor.begin();
        motor.setRampMode(VELOCITY_MODE);
  motor.setMaxSpeed(0);  // tics/sec  1rpm
  motor.setAcceleration(1500);
  }
}