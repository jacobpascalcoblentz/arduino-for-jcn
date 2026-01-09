/**
 * @file lights.h
 * @brief Light controller with scheduling and dimming support
 */

#ifndef LIGHTS_H
#define LIGHTS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "rtc.h"

// ============================================================================
// LIGHT ZONE CONFIGURATION
// ============================================================================

struct LightSchedule {
    uint8_t onHour;        // 0-23 (hour to turn on)
    uint8_t onMinute;      // 0-59 (minute to turn on)
    uint8_t offHour;       // 0-23 (hour to turn off, 24 = midnight next day)
    uint8_t offMinute;     // 0-59 (minute to turn off)
    uint8_t sunriseMins;   // Minutes to ramp up (0 = instant on)
    uint8_t sunsetMins;    // Minutes to ramp down (0 = instant off)
    bool enabled;          // Zone enabled

    // Calculate photoperiod in hours
    float getPhotoperiod() const {
        int onMins = onHour * 60 + onMinute;
        int offMins = offHour * 60 + offMinute;
        if (offMins <= onMins) offMins += 24 * 60;  // Crosses midnight
        return (offMins - onMins) / 60.0f;
    }
};

enum class LightState : uint8_t {
    OFF = 0,
    SUNRISE,       // Ramping up
    ON,
    SUNSET,        // Ramping down
    MANUAL_ON,     // Manual override - on
    MANUAL_OFF     // Manual override - off
};

struct LightZoneStatus {
    LightState state;
    uint8_t brightness;     // 0-255 (for PWM dimming)
    bool relayOn;           // Relay state
    float powerWatts;       // Measured power (from current sensor)
    bool powerVerified;     // Power matches expected
    uint16_t minutesUntilChange;  // Minutes until next state change
};

// ============================================================================
// LIGHT CONTROLLER CLASS
// ============================================================================

class LightController {
public:
    LightController(RealTimeClock* rtc);

    /**
     * Initialize light controller
     * @param zone1RelayPin Relay pin for zone 1
     * @param zone2RelayPin Relay pin for zone 2
     * @param zone1DimPin PWM pin for zone 1 dimming (255 = no dimming)
     * @param zone2DimPin PWM pin for zone 2 dimming (255 = no dimming)
     */
    bool begin(uint8_t zone1RelayPin, uint8_t zone2RelayPin,
               uint8_t zone1DimPin = 255, uint8_t zone2DimPin = 255);

    /**
     * Update light states based on schedule
     * MUST be called frequently (at least once per second)
     */
    void update();

    /**
     * Set schedule for a zone
     */
    void setSchedule(uint8_t zone, const LightSchedule& schedule);

    /**
     * Get current schedule for a zone
     */
    LightSchedule getSchedule(uint8_t zone) const;

    /**
     * Get current status of a zone
     */
    LightZoneStatus getStatus(uint8_t zone) const;

    /**
     * Manual override controls
     */
    void setManualOverride(uint8_t zone, bool on);
    void clearManualOverride(uint8_t zone);
    bool isManualOverride(uint8_t zone) const;

    /**
     * Set brightness directly (0-255)
     * Only works if dimming pin is configured
     */
    void setBrightness(uint8_t zone, uint8_t brightness);

    /**
     * Enable/disable a zone
     */
    void setZoneEnabled(uint8_t zone, bool enabled);

    /**
     * Report measured power for a zone (called by PowerMonitor)
     */
    void reportPower(uint8_t zone, float watts);

    /**
     * Set expected power for verification
     */
    void setExpectedPower(uint8_t zone, float watts, float tolerancePercent = 20.0f);

    /**
     * Check if any lights have power anomalies
     */
    bool hasPowerAnomaly() const;

    /**
     * Get formatted status string
     */
    void getStatusString(uint8_t zone, char* buffer, size_t bufferSize) const;

    // For testing
    void injectCurrentTime(uint8_t hour, uint8_t minute);

private:
    RealTimeClock* rtc_;

    // Pin configuration
    uint8_t relayPins_[2];
    uint8_t dimPins_[2];
    bool hasDimming_[2];

    // Zone data
    LightSchedule schedules_[2];
    LightZoneStatus status_[2];
    float expectedPower_[2];
    float powerTolerance_[2];

    // Timing
    uint32_t lastUpdateMs_;

    // Internal methods
    void updateZone(uint8_t zone);
    LightState calculateState(uint8_t zone, uint16_t currentMins);
    uint8_t calculateBrightness(uint8_t zone, uint16_t currentMins, LightState state);
    void setRelay(uint8_t zone, bool on);
    void setPwm(uint8_t zone, uint8_t value);
    uint16_t getMinutesUntilChange(uint8_t zone, uint16_t currentMins, LightState state);
};

// ============================================================================
// PRESET SCHEDULES
// ============================================================================

namespace LightPresets {
    // Vegetative growth: 18 hours on
    const LightSchedule VEG_18_6 = {
        .onHour = 6, .onMinute = 0,
        .offHour = 24, .offMinute = 0,  // Midnight
        .sunriseMins = 30, .sunsetMins = 30,
        .enabled = true
    };

    // Flowering: 12 hours on
    const LightSchedule FLOWER_12_12 = {
        .onHour = 6, .onMinute = 0,
        .offHour = 18, .offMinute = 0,
        .sunriseMins = 30, .sunsetMins = 30,
        .enabled = true
    };

    // Seedlings: 16 hours on
    const LightSchedule SEEDLING_16_8 = {
        .onHour = 6, .onMinute = 0,
        .offHour = 22, .offMinute = 0,
        .sunriseMins = 15, .sunsetMins = 15,
        .enabled = true
    };

    // Lettuce/greens: 14 hours
    const LightSchedule LETTUCE_14_10 = {
        .onHour = 6, .onMinute = 0,
        .offHour = 20, .offMinute = 0,
        .sunriseMins = 20, .sunsetMins = 20,
        .enabled = true
    };

    // Always on (for propagation)
    const LightSchedule ALWAYS_ON = {
        .onHour = 0, .onMinute = 0,
        .offHour = 24, .offMinute = 0,
        .sunriseMins = 0, .sunsetMins = 0,
        .enabled = true
    };
}

#endif // LIGHTS_H
