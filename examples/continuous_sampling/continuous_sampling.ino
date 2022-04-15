#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/STS3x.h"

STS3x sensor(Wire); // To use a non-default address use STS3x sensor(Wire, address);

void setup() {
  Wire.begin();
  sensor.setContinousSampling(MPS_05, REP_HIGH);  // One measurement every 2 seconds, high repeatability
  Serial.begin(115200);
}

void loop() {
  Serial.print(sensor.fetchTemp());  // Read the temperarture as a float in °C
  Serial.println(" °C");
  delay(2000);
}
