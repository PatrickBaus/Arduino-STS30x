/*
 *  The STS3x library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "STS3x.h"

#if defined(__arm__) && defined(TEENSYDUINO)
STS3x::STS3x(i2c_t3& i2cBus, const uint8_t address) : _pI2cBus(&i2cBus), _i2c_addr(address) {};
#else
STS3x::STS3x(TwoWire& i2cBus, const uint8_t address) : _pI2cBus(&i2cBus), _i2c_addr(address) {};
#endif

uint8_t STS3x::crc8(const uint8_t* data, const size_t len) {
  // Calculate CRC with the following parameters (datasheet p. 10):
  // Type: CRC-8
  // Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
  // Reflextion in/out: false/false
  // Init: 0xFF
  // Final XOR: 0x00

  uint8_t crc = 0xFF; // init to starting value
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t crcBit = 8; crcBit > 0; crcBit--) {
      // Apply the polynomial
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc; // No final XOR needed since it is 0x00;
}

float STS3x::convertToCelsius(const uint16_t value) {
  return (float)((uint32_t)value * 175) / 65535.0f - 45;
}

uint16_t STS3x::convertToRaw(const float value) {
  return (value + 45.0f) * 65535.0f / 175.0f;
}

bool STS3x::writeCommand(const uint16_t command, uint8_t* data, const size_t len) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  _pI2cBus->beginTransmission(_i2c_addr);
  bool result = _pI2cBus->write(cmd, sizeof(cmd)) == sizeof(cmd);
  if (len) {
    result = result & (_pI2cBus->write(data, len) == len);
  }
  _pI2cBus->endTransmission();
  return result;
}

bool STS3x::queryCommand(const uint16_t command, uint8_t* result, const size_t len, const uint16_t delayus) {
  this->writeCommand(command);
  delayMicroseconds(delayus);

  _pI2cBus->requestFrom(_i2c_addr, len);
  size_t numberOfBytesread = _pI2cBus->readBytes(result, len);

  // Check data for validity
  if ((numberOfBytesread != len) or (result[len-1] != STS3x::crc8(result, len - 1))) {
    return false;
  }
  return true;
}

void STS3x::stopConversion(void) {
  this->writeCommand(STS3x::CMD_BREAK);
  delay(1);
}

bool STS3x::readStatus(SensorStatus& status) {
  uint8_t data[3] = {0};
  bool result = this->queryCommand(STS3x::CMD_READ_STATUS, data, sizeof(data));

  status.checksumError = data[1] &  STS3x::STATUS_CRC_ERROR;
  status.commandError = data[1] & STS3x::STATUS_COMMAND_ERROR;
  status.systemReset = data[1] & STS3x::STATUS_SYSTEM_RESET;
  status.alertTemperature = data[0] & STS3x::STATUS_ALERT_TEMPERATURE;
  status.heaterEnabled = data[0] & STS3x::STATUS_HEATER;
  status.alertPending = data[0] & STS3x::STATUS_ALERT_PENDING;

  return result;
}

void STS3x::clearStatus(void) {
  this->writeCommand(STS3x::CMD_CLEAR_STATUS);
  delayMicroseconds(10);
}

uint32_t STS3x::readSerial(void) {
  uint8_t data[4] = {0};
  this->queryCommand(STS3x::CMD_READ_SERIAL, data, sizeof(data), 1000 /*µs*/);

  return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3] << 0;
}

// Soft-Resets the sensor. Do make sure to call stopConversion() before calling
// reset, because the sensor will not reset during conversions.
void STS3x::reset(void) {
  this->writeCommand(STS3x::CMD_SOFTRESET);
  delay(1); // The sensor needs 1 ms max to return to idle (see datasheet p. 3)
}

void STS3x::setHeaterState(const bool enable) {
  if (enable) {
    this->writeCommand(STS3x::CMD_HEATER_ENABLE);
  } else {
    this->writeCommand(STS3x::CMD_HEATER_DISABLE);
  }
  delayMicroseconds(10);
}

float STS3x::readTemp(const Repeatability repeatability) {
  uint16_t value;
  if (this->readTempRaw(value, repeatability)) {
    return STS3x::convertToCelsius(value);
  } else {
    return nan("");
  }
}

float STS3x::fetchTemp(void) {
  uint16_t value;
  if (this->fetchTempRaw(value)) {
    return STS3x::convertToCelsius(value);
  } else {
    return nan("");
  }
}


uint16_t STS3x::readTempRaw(const Repeatability repeatability) {
  uint16_t value;
  this->readTempRaw(value, repeatability);
  return value;
}

uint16_t STS3x::fetchTempRaw(void) {
  uint16_t value;
  this->fetchTempRaw(value);
  return value;
}

bool STS3x::readTempRaw(uint16_t &value, const Repeatability repeatability) {
  uint8_t data[3] = {0};
  bool result;
  switch (repeatability) {
    case REP_HIGH:
      result = this->queryCommand(STS3x::CMD_MEASURE_HIGH_REAPEATABILITY, data, sizeof(data), 15000 /*µs*/);
      break;
    case REP_MEDIUM:
      result = this->queryCommand(STS3x::CMD_MEASURE_MEDIUM_REAPEATABILITY, data, sizeof(data), 6000 /*µs*/);
      break;
    case REP_LOW:
      result = this->queryCommand(STS3x::CMD_MEASURE_LOW_REAPEATABILITY, data, sizeof(data), 4000 /*µs*/);
      break;
    default:
      // No default needed. All cases are handled above
      result = false;
      break;
  }

  value = ((uint16_t)data[0] << 8) | data[1] << 0;
  return result;
}

bool STS3x::fetchTempRaw(uint16_t &value) {
  uint8_t data[3] = {0};
  bool result = this->queryCommand(STS3x::CMD_FETCH_PERIODIC_RESULT, data, sizeof(data));

  value = ((uint16_t)data[0] << 8) | data[1] << 0;
  return result;
}

void STS3x::setContinousSampling(const MeasurementsPerSecond mps, const Repeatability rep) {
  uint16_t cmd = (uint8_t)mps << 8;

  switch (mps) {
    case MPS_05:
      switch(rep) {
        case REP_LOW:
          cmd |= 0x2F;
          break;
        case REP_MEDIUM:
          cmd |= 0x24;
          break;
        case REP_HIGH:
          cmd |= 0x32;
          break;
      }
      break;
    case MPS_1:
      switch(rep) {
        case REP_LOW:
          cmd |= 0x2D;
          break;
        case REP_MEDIUM:
          cmd |= 0x26;
          break;
        case REP_HIGH:
          cmd |= 0x30;
          break;
      }
      break;
    case MPS_2:
      switch(rep) {
        case REP_LOW:
          cmd |= 0x2B;
          break;
        case REP_MEDIUM:
          cmd |= 0x20;
          break;
        case REP_HIGH:
          cmd |= 0x36;
          break;
      }
      break;
    case MPS_4:
      switch(rep) {
        case REP_LOW:
          cmd |= 0x29;
          break;
        case REP_MEDIUM:
          cmd |= 0x22;
          break;
        case REP_HIGH:
          cmd |= 0x34;
          break;
      }
      break;
    case MPS_10:
      switch(rep) {
        case REP_LOW:
          cmd |= 0x2A;
          break;
        case REP_MEDIUM:
          cmd |= 0x21;
          break;
        case REP_HIGH:
          cmd |= 0x37;
          break;
      }
      break;
  }
  this->writeCommand(cmd);
}

bool STS3x::readAlertLimitsHigh(uint16_t& set, uint16_t& clear) {
  uint8_t data[3] = {0};

  bool result = this->queryCommand(STS3x::CMD_READ_ALERT_LIMIT_HIGH_SET, data, sizeof(data));
  set = ((uint16_t)data[0] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH) | (uint16_t)data[1] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  if (!result) {
    return false;
  }

  result = this->queryCommand(STS3x::CMD_READ_ALERT_LIMIT_HIGH_CLEAR, data, sizeof(data));
  clear = ((uint16_t)data[0] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH) | (uint16_t)data[1] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;

  return result;
}

bool STS3x::setAlertLimitsHigh(const uint16_t set, const uint16_t clear) {
  uint8_t data[2] = {0};

  data[0] = set >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH;
  data[1] = set >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  bool result = this->writeCommand(STS3x::CMD_WRITE_ALERT_LIMIT_HIGH_SET, data, sizeof(data));
  if (!result) {
    return false;
  }

  data[0] = clear >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH;
  data[1] = clear >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  result = this->writeCommand(STS3x::CMD_WRITE_ALERT_LIMIT_HIGH_CLEAR, data, sizeof(data));

  return result;
}

bool STS3x::readAlertLimitsLow(uint16_t& set, uint16_t& clear) {
  uint8_t data[3] = {0};

  bool result = this->queryCommand(STS3x::CMD_READ_ALERT_LIMIT_LOW_SET, data, sizeof(data));
  set = ((uint16_t)data[0] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH) | (uint16_t)data[1] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  if (!result) {
    return false;
  }

  result = this->queryCommand(STS3x::CMD_READ_ALERT_LIMIT_LOW_CLEAR, data, sizeof(data));
  clear = ((uint16_t)data[0] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH) | (uint16_t)data[1] << STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;

  return result;
}

bool STS3x::setAlertLimitsLow(const uint16_t set, const uint16_t clear) {
  uint8_t data[2] = {0};

  data[0] = set >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH;
  data[1] = set >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  bool result = this->writeCommand(STS3x::CMD_WRITE_ALERT_LIMIT_LOW_SET, data, sizeof(data));
  if (!result) {
    return false;
  }

  data[0] = clear >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH;
  data[1] = clear >> STS3x::OFFSET_ALERT_LIMIT_TEMPERATURE_LOW;
  result = this->writeCommand(STS3x::CMD_WRITE_ALERT_LIMIT_LOW_CLEAR, data, sizeof(data));

  return result;
}
