/**
 * @file rtc.cpp
 * @brief DS3231 RTC implementation
 */

#include "rtc.h"
#include "config.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#else
#include "mock_arduino.h"
#endif

// DS3231 I2C address
#define DS3231_ADDRESS 0x68

// DS3231 register addresses
#define DS3231_REG_SECONDS    0x00
#define DS3231_REG_MINUTES    0x01
#define DS3231_REG_HOURS      0x02
#define DS3231_REG_DAY        0x03
#define DS3231_REG_DATE       0x04
#define DS3231_REG_MONTH      0x05
#define DS3231_REG_YEAR       0x06
#define DS3231_REG_CONTROL    0x0E
#define DS3231_REG_STATUS     0x0F
#define DS3231_REG_TEMP_MSB   0x11
#define DS3231_REG_TEMP_LSB   0x12

// ============================================================================
// CONSTRUCTOR
// ============================================================================

RealTimeClock::RealTimeClock()
    : running_(false), lostPower_(false), useInjectedTime_(false) {
    injectedTime_ = {0, 0, 0, 1, 1, 1, 2024};
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool RealTimeClock::begin() {
#ifdef ARDUINO
    Wire.begin();

    // Check if DS3231 responds
    Wire.beginTransmission(DS3231_ADDRESS);
    if (Wire.endTransmission() != 0) {
        running_ = false;
        return false;
    }

    // Check oscillator stop flag (bit 7 of status register)
    uint8_t status = readRegister(DS3231_REG_STATUS);
    lostPower_ = (status & 0x80) != 0;

    if (lostPower_) {
        // Clear the oscillator stop flag
        writeRegister(DS3231_REG_STATUS, status & ~0x80);

        // If lost power, set a default time
        Serial.println(F("RTC lost power! Time may be incorrect."));
        Serial.println(F("Consider running setFromCompileTime() or setting manually."));
    }

    // Enable oscillator (clear EOSC bit in control register)
    uint8_t ctrl = readRegister(DS3231_REG_CONTROL);
    writeRegister(DS3231_REG_CONTROL, ctrl & ~0x80);

    running_ = true;
    return true;
#else
    // For native testing, always succeed
    running_ = true;
    return true;
#endif
}

// ============================================================================
// TIME OPERATIONS
// ============================================================================

TimeInfo RealTimeClock::getTime() {
    if (useInjectedTime_) {
        return injectedTime_;
    }

    TimeInfo t = {0, 0, 0, 1, 1, 1, 2024};

#ifdef ARDUINO
    if (!running_) return t;

    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(DS3231_REG_SECONDS);
    Wire.endTransmission();

    Wire.requestFrom(DS3231_ADDRESS, 7);
    if (Wire.available() >= 7) {
        t.second = bcdToDec(Wire.read() & 0x7F);
        t.minute = bcdToDec(Wire.read());
        t.hour = bcdToDec(Wire.read() & 0x3F);  // 24-hour mode
        t.dayOfWeek = bcdToDec(Wire.read());
        t.day = bcdToDec(Wire.read());
        t.month = bcdToDec(Wire.read() & 0x1F);
        t.year = 2000 + bcdToDec(Wire.read());
    }
#endif

    return t;
}

void RealTimeClock::setTime(const TimeInfo& time) {
#ifdef ARDUINO
    if (!running_) return;

    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(DS3231_REG_SECONDS);
    Wire.write(decToBcd(time.second));
    Wire.write(decToBcd(time.minute));
    Wire.write(decToBcd(time.hour));
    Wire.write(decToBcd(time.dayOfWeek));
    Wire.write(decToBcd(time.day));
    Wire.write(decToBcd(time.month));
    Wire.write(decToBcd(time.year - 2000));
    Wire.endTransmission();

    // Clear oscillator stop flag
    uint8_t status = readRegister(DS3231_REG_STATUS);
    writeRegister(DS3231_REG_STATUS, status & ~0x80);
    lostPower_ = false;
#endif
}

void RealTimeClock::setFromCompileTime() {
    // Parse __DATE__ and __TIME__ macros
    // __DATE__ = "Jan  1 2024"
    // __TIME__ = "12:34:56"

    TimeInfo t;

#ifdef ARDUINO
    const char* date = __DATE__;
    const char* time = __TIME__;

    // Parse month
    const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char monthStr[4] = {date[0], date[1], date[2], 0};
    const char* found = strstr(months, monthStr);
    t.month = found ? ((found - months) / 3) + 1 : 1;

    // Parse day
    t.day = (date[4] == ' ') ? (date[5] - '0') : ((date[4] - '0') * 10 + (date[5] - '0'));

    // Parse year
    t.year = (date[7] - '0') * 1000 + (date[8] - '0') * 100 +
             (date[9] - '0') * 10 + (date[10] - '0');

    // Parse time
    t.hour = (time[0] - '0') * 10 + (time[1] - '0');
    t.minute = (time[3] - '0') * 10 + (time[4] - '0');
    t.second = (time[6] - '0') * 10 + (time[7] - '0');

    t.dayOfWeek = 1;  // Not calculated, can be computed if needed

    setTime(t);

    Serial.print(F("RTC set to compile time: "));
    Serial.print(t.month);
    Serial.print('/');
    Serial.print(t.day);
    Serial.print('/');
    Serial.print(t.year);
    Serial.print(' ');
    Serial.print(t.hour);
    Serial.print(':');
    Serial.println(t.minute);
#endif
}

uint8_t RealTimeClock::getHour() {
    return getTime().hour;
}

uint8_t RealTimeClock::getMinute() {
    return getTime().minute;
}

float RealTimeClock::getTemperature() {
#ifdef ARDUINO
    if (!running_) return 0.0f;

    uint8_t msb = readRegister(DS3231_REG_TEMP_MSB);
    uint8_t lsb = readRegister(DS3231_REG_TEMP_LSB);

    // Temperature is in 0.25C increments
    // MSB is integer part (signed), upper 2 bits of LSB are fractional
    int16_t temp = (msb << 2) | (lsb >> 6);
    if (msb & 0x80) {
        temp |= 0xFC00;  // Sign extend for negative temps
    }
    return temp * 0.25f;
#else
    return 25.0f;  // Default for testing
#endif
}

// ============================================================================
// TESTING HELPERS
// ============================================================================

void RealTimeClock::injectTime(uint8_t hour, uint8_t minute, uint8_t second) {
    useInjectedTime_ = true;
    injectedTime_.hour = hour;
    injectedTime_.minute = minute;
    injectedTime_.second = second;
}

void RealTimeClock::clearInjectedTime() {
    useInjectedTime_ = false;
}

// ============================================================================
// I2C HELPERS
// ============================================================================

uint8_t RealTimeClock::bcdToDec(uint8_t bcd) {
    return (bcd / 16 * 10) + (bcd % 16);
}

uint8_t RealTimeClock::decToBcd(uint8_t dec) {
    return (dec / 10 * 16) + (dec % 10);
}

void RealTimeClock::writeRegister(uint8_t reg, uint8_t value) {
#ifdef ARDUINO
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#endif
}

uint8_t RealTimeClock::readRegister(uint8_t reg) {
#ifdef ARDUINO
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(DS3231_ADDRESS, 1);
    if (Wire.available()) {
        return Wire.read();
    }
#endif
    return 0;
}

// ============================================================================
// NTP SYNC (ESP32 ONLY)
// ============================================================================

#ifdef ESP32
#include <WiFi.h>
#include <time.h>

bool syncRtcWithNtp(RealTimeClock& rtc, const char* timezone) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("NTP sync failed: WiFi not connected"));
        return false;
    }

    // Configure NTP
    configTzTime(timezone, "pool.ntp.org", "time.nist.gov");

    // Wait for time sync (max 10 seconds)
    struct tm timeinfo;
    int attempts = 0;
    while (!getLocalTime(&timeinfo) && attempts < 20) {
        delay(500);
        attempts++;
    }

    if (attempts >= 20) {
        Serial.println(F("NTP sync failed: timeout"));
        return false;
    }

    // Set RTC from NTP time
    TimeInfo t;
    t.second = timeinfo.tm_sec;
    t.minute = timeinfo.tm_min;
    t.hour = timeinfo.tm_hour;
    t.dayOfWeek = timeinfo.tm_wday + 1;
    t.day = timeinfo.tm_mday;
    t.month = timeinfo.tm_mon + 1;
    t.year = timeinfo.tm_year + 1900;

    rtc.setTime(t);

    Serial.print(F("RTC synced with NTP: "));
    Serial.print(t.month);
    Serial.print('/');
    Serial.print(t.day);
    Serial.print('/');
    Serial.print(t.year);
    Serial.print(' ');
    Serial.print(t.hour);
    Serial.print(':');
    Serial.println(t.minute);

    return true;
}
#endif
