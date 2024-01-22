# TMC5160
Arduino Library to control the Trinamic TMC5160 stepper motor driver


const unsigned long INTERVAL = 100;  // Check every 100ms
unsigned long nextCheckTime = 0;

void loop() {
  unsigned long currentTime = millis();

  // Perform other tasks
  // ...

  // Check for tick update time
  if (currentTime >= nextCheckTime) {
    uint32_t currentTicks = getRpmTicks();
    ticksWindow[windowIndex] = currentTicks;
    totalTicks += currentTicks - ticksWindow[(windowIndex + WINDOW_SIZE - 1) % WINDOW_SIZE];
    windowIndex = (windowIndex + 1) % WINDOW_SIZE;

    // Check motor status
    if (totalTicks >= RUNNING_THRESHOLD) {
      Serial.println("Motor is running");
    } else {
      Serial.println("Motor has a problem");
    }

    nextCheckTime += INTERVAL;  // Schedule next check
  }
}
