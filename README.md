Sensirion STS3x Arduino Library
===================

This repository contains an Arduino library for the following Sensirion devices:
 * [STS30A-DIS](https://sensirion.com/products/catalog/STS30A-DIS/)
 * [STS30-DIS](https://sensirion.com/products/catalog/STS30-DIS/)
 * [STS31A-DIS](https://sensirion.com/products/catalog/STS31A-DIS/)
 * [STS31-DIS](https://sensirion.com/products/catalog/STS31-DIS/)
 * [STS32-DIS](https://sensirion.com/products/catalog/STS32-DIS/)
 * [STS33-DIS](https://sensirion.com/products/catalog/STS33-DIS/)
 * [STS35-DIS](https://sensirion.com/products/catalog/STS35-DIS/)

The library uses I²C transactions and allows to select the I²C controller if there is more than one.

Usage
-----
```cpp
#include <Wire.h>   // #include <i2c_t3.h> on the Teensy platform.
#include "src/STS3x.h"

STS3x sensor(Wire); // To use a non-default address use STS3x sensor(Wire, address);

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  while (true) {
    // The call to readTemp() will automatically block for the time the measurement takes to complete
    Serial.print(sensor.readTemp());  // Read the temperarture as a float in °C
    Serial.println(" °C");
    delay(2000);
  }
}
```

Installation
-----
Currently the library does not support the Arduino library manager, so it is highly recommended to copy the full library to a subfolder called
```
src/
```
within your Arduino project.
