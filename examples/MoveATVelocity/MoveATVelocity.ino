

#include<SPI.h>
#include <TMC5160.h>

const uint8_t SPI_CS = SS;      // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 7;  // DRV_ENN pin in SPI mode
uint32_t currentTime = millis();

TMC5160_SPI motor = TMC5160_SPI(SPI_CS);  //Use default SPI peripheral and SPI settings.

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !

unsigned long start_time=0;
void setup() {
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
  motor.moveAtVelocity(400);  // tics/sec  1rpm
  motor.setAcceleration(500);
  delay(1000);  //Standstill for automatic tuning
}

void loop() { 

  Serial.println("RPM100");
  motor.moveAtVelocity(100); 

  if(motor.isResetOccurred()) {
      motor.begin();
        motor.setRampMode(VELOCITY_MODE);
  motor.moveAtVelocity(0);  // tics/sec  1rpm
  motor.setAcceleration(1500);
  }
}