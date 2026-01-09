/**
 * @file rtc.h
 * @brief Real-Time Clock (DS3231) interface for light scheduling
 */

#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// TIME STRUCTURE
// ============================================================================

struct TimeInfo {
    uint8_t hour;      // 0-23
    uint8_t minute;    // 0-59
    uint8_t second;    // 0-59
    uint8_t dayOfWeek; // 1-7 (Sunday = 1)
    uint8_t day;       // 1-31
    uint8_t month;     // 1-12
    uint16_t year;     // 2000-2099

    // Helper to get minutes since midnight
    uint16_t minutesSinceMidnight() const {
        return hour * 60 + minute;
    }

    // Helper to get total seconds since midnight
    uint32_t secondsSinceMidnight() const {
        return hour * 3600UL + minute * 60UL + second;
    }
};

// ============================================================================
// RTC CLASS
// ============================================================================

class RealTimeClock {
public:
    RealTimeClock();

    /**
     * Initialize the RTC
     * @return true if RTC found and running
     */
    bool begin();

    /**
     * Check if RTC is available and running
     */
    bool isRunning() const { return running_; }

    /**
     * Check if RTC lost power (battery dead/missing)
     */
    bool lostPower() const { return lostPower_; }

    /**
     * Get current time from RTC
     */
    TimeInfo getTime();

    /**
     * Set the RTC time
     */
    void setTime(const TimeInfo& time);

    /**
     * Set time from compile timestamp (useful for initial setup)
     */
    void setFromCompileTime();

    /**
     * Get current hour (0-23)
     */
    uint8_t getHour();

    /**
     * Get current minute (0-59)
     */
    uint8_t getMinute();

    /**
     * Get RTC temperature (DS3231 has built-in temp sensor)
     * @return Temperature in Celsius
     */
    float getTemperature();

    /**
     * For testing - inject a fake time
     */
    void injectTime(uint8_t hour, uint8_t minute, uint8_t second = 0);
    void clearInjectedTime();

private:
    bool running_;
    bool lostPower_;

    // For testing
    bool useInjectedTime_;
    TimeInfo injectedTime_;

    // I2C communication helpers
    uint8_t bcdToDec(uint8_t bcd);
    uint8_t decToBcd(uint8_t dec);
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

// ============================================================================
// NTP SYNC (ESP32 ONLY)
// ============================================================================

#ifdef ESP32
/**
 * Sync RTC with NTP time server
 * @param rtc Reference to RTC object
 * @param timezone Timezone offset string (e.g., "EST5EDT")
 * @return true if sync successful
 */
bool syncRtcWithNtp(RealTimeClock& rtc, const char* timezone = "EST5EDT,M3.2.0,M11.1.0");
#endif

#endif // RTC_H
