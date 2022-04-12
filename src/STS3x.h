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
#ifndef STS3x_H
#define STS3x_H

#include <stdint.h>
#include <stdbool.h>

#if defined(__arm__) && defined(TEENSYDUINO)
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

enum MeasurementsPerSecond {
  MPS_05 = 0x20,
  MPS_1  = 0x21,
  MPS_2  = 0x22,
  MPS_4  = 0x23,
  MPS_10 = 0x27,
};

enum Repeatability {
  REP_LOW,
  REP_MEDIUM,
  REP_HIGH,
};

struct SensorStatus {
   bool checksumError;
   bool commandError;
   bool systemReset;
   bool alertTemperature;
   bool alertPending;
   bool heaterEnabled;
};

class STS3x {
  public:
    #if defined(__arm__) && defined(TEENSYDUINO)
    STS3x(i2c_t3& i2cBus, const uint8_t address=0x4A);
    #else
    STS3x(TwoWire& i2cBus, const uint8_t address=0x4A);
    #endif
    void setHeaterState(const bool enable);
    void setContinousSampling(const MeasurementsPerSecond mps, const Repeatability rep);
    bool readStatus(SensorStatus& status);
    void clearStatus(void);
    void reset(void);
    void stopConversion(void);
    float readTemp(const Repeatability repeatability=REP_HIGH);
    float fetchTemp(void);
    uint16_t readTempRaw(const Repeatability repeatability=REP_HIGH);
    uint16_t fetchTempRaw(void);
    uint32_t readSerial(void);
    bool readAlertLimitsHigh(uint16_t& set, uint16_t& clear);
    bool setAlertLimitsHigh(const uint16_t set, const uint16_t clear);
    bool readAlertLimitsLow(uint16_t& set, uint16_t& clear);
    bool setAlertLimitsLow(const uint16_t set, const uint16_t clear);
    static float convertToCelsius(const uint16_t value);
    static uint16_t convertToRaw(const float value);
  private:
    #if defined(__arm__) && defined(TEENSYDUINO)
    i2c_t3* _pI2cBus = NULL;
    #else
    TwoWire* _pI2cBus = NULL;
    #endif
    uint8_t _i2c_addr;

    static const uint16_t CMD_SOFTRESET = 0x30A2;
    static const uint16_t CMD_BREAK = 0x3093;
    static const uint16_t CMD_HEATER_ENABLE = 0x306D;
    static const uint16_t CMD_HEATER_DISABLE = 0x3066;
    static const uint16_t CMD_READ_STATUS = 0xF32D;
    static const uint16_t CMD_CLEAR_STATUS = 0x3041;
    static const uint16_t CMD_READ_SERIAL = 0x3780;
    static const uint16_t CMD_MEASURE_HIGH_REAPEATABILITY = 0x2400;
    static const uint16_t CMD_MEASURE_MEDIUM_REAPEATABILITY = 0x240B;
    static const uint16_t CMD_MEASURE_LOW_REAPEATABILITY = 0x2416;
    static const uint16_t CMD_FETCH_PERIODIC_RESULT = 0xE000;
    static const uint16_t CMD_READ_ALERT_LIMIT_HIGH_SET = 0xE11F;
    static const uint16_t CMD_READ_ALERT_LIMIT_HIGH_CLEAR = 0xE114;
    static const uint16_t CMD_READ_ALERT_LIMIT_LOW_SET = 0xE102;
    static const uint16_t CMD_READ_ALERT_LIMIT_LOW_CLEAR = 0xE109;
    static const uint16_t CMD_WRITE_ALERT_LIMIT_HIGH_SET = 0x611D;
    static const uint16_t CMD_WRITE_ALERT_LIMIT_HIGH_CLEAR = 0x6116;
    static const uint16_t CMD_WRITE_ALERT_LIMIT_LOW_SET = 0x6100;
    static const uint16_t CMD_WRITE_ALERT_LIMIT_LOW_CLEAR = 0x610B;
    static const uint8_t OFFSET_ALERT_LIMIT_TEMPERATURE_LOW = 7;
    static const uint8_t OFFSET_ALERT_LIMIT_TEMPERATURE_HIGH = OFFSET_ALERT_LIMIT_TEMPERATURE_LOW+8;
    static const uint8_t STATUS_HEATER = 1 << (13-8);
    static const uint8_t STATUS_ALERT_PENDING = 1 << (15-8);
    static const uint8_t STATUS_ALERT_TEMPERATURE = 1 << (10-8);
    static const uint8_t STATUS_SYSTEM_RESET = 1 << 4;
    static const uint8_t STATUS_COMMAND_ERROR = 1 << 1;
    static const uint8_t STATUS_CRC_ERROR = 1 << 0;

    bool writeCommand(const uint16_t command, const uint8_t* data=NULL, const size_t len=0);
    bool queryCommand(const uint16_t command, uint8_t* result, const size_t len, const uint16_t delayus=0);
    bool readTempRaw(uint16_t &value, const Repeatability repeatability=REP_HIGH);
    bool fetchTempRaw(uint16_t &value);
    static uint8_t crc8(const uint8_t* data, const size_t len);
};
#endif
