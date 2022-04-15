#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/STS3x.h"

STS3x sensor(Wire); // To use a non-default address use STS3x sensor(Wire, address);

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  // The call to readTemp() will automatically block for the time the measurement takes to complete
  Serial.print(sensor.readTemp());  // Read the temperarture as a float in °C
  Serial.println(" °C");
  delay(2000);
}
