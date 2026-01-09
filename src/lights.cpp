/**
 * @file lights.cpp
 * @brief Light controller implementation
 */

#include "lights.h"
#include "config.h"

#include <stdio.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "mock_arduino.h"
#endif

// ============================================================================
// CONSTRUCTOR
// ============================================================================

LightController::LightController(RealTimeClock* rtc)
    : rtc_(rtc), lastUpdateMs_(0) {

    // Initialize pins to invalid
    for (int i = 0; i < 2; i++) {
        relayPins_[i] = 255;
        dimPins_[i] = 255;
        hasDimming_[i] = false;
        expectedPower_[i] = 0.0f;
        powerTolerance_[i] = POWER_TOLERANCE_PERCENT;

        // Default status
        status_[i].state = LightState::OFF;
        status_[i].brightness = 0;
        status_[i].relayOn = false;
        status_[i].powerWatts = 0.0f;
        status_[i].powerVerified = true;
        status_[i].minutesUntilChange = 0;

        // Default schedules
        schedules_[i].onHour = 6;
        schedules_[i].onMinute = 0;
        schedules_[i].offHour = (i == 0) ? 24 : 18;  // Veg vs Flower default
        schedules_[i].offMinute = 0;
        schedules_[i].sunriseMins = SUNRISE_DURATION_MIN;
        schedules_[i].sunsetMins = SUNSET_DURATION_MIN;
        schedules_[i].enabled = true;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool LightController::begin(uint8_t zone1RelayPin, uint8_t zone2RelayPin,
                            uint8_t zone1DimPin, uint8_t zone2DimPin) {
    relayPins_[0] = zone1RelayPin;
    relayPins_[1] = zone2RelayPin;
    dimPins_[0] = zone1DimPin;
    dimPins_[1] = zone2DimPin;

    hasDimming_[0] = (zone1DimPin != 255);
    hasDimming_[1] = (zone2DimPin != 255);

#ifdef ARDUINO
    // Setup relay pins
    for (int i = 0; i < 2; i++) {
        if (relayPins_[i] != 255) {
            pinMode(relayPins_[i], OUTPUT);
            digitalWrite(relayPins_[i], LOW);  // Start with lights off
        }
        if (dimPins_[i] != 255) {
            pinMode(dimPins_[i], OUTPUT);
            analogWrite(dimPins_[i], 0);
        }
    }
#endif

    return true;
}

// ============================================================================
// UPDATE (CALL FREQUENTLY)
// ============================================================================

void LightController::update() {
    uint32_t now = millis();

    // Only update once per second
    if (now - lastUpdateMs_ < LIGHT_CHECK_INTERVAL_MS) {
        return;
    }
    lastUpdateMs_ = now;

    updateZone(0);
    updateZone(1);
}

void LightController::updateZone(uint8_t zone) {
    if (zone >= 2) return;

    // Get current time
    TimeInfo time = rtc_->getTime();
    uint16_t currentMins = time.minutesSinceMidnight();

    // Check for manual override
    if (status_[zone].state == LightState::MANUAL_ON ||
        status_[zone].state == LightState::MANUAL_OFF) {
        // Manual override active - don't change state
        return;
    }

    // Check if zone is enabled
    if (!schedules_[zone].enabled) {
        setRelay(zone, false);
        setPwm(zone, 0);
        status_[zone].state = LightState::OFF;
        status_[zone].brightness = 0;
        status_[zone].relayOn = false;
        return;
    }

    // Calculate what state we should be in
    LightState newState = calculateState(zone, currentMins);
    uint8_t newBrightness = calculateBrightness(zone, currentMins, newState);

    // Update hardware
    bool shouldRelayBeOn = (newState == LightState::SUNRISE ||
                            newState == LightState::ON ||
                            newState == LightState::SUNSET);

    setRelay(zone, shouldRelayBeOn);
    setPwm(zone, newBrightness);

    // Update status
    status_[zone].state = newState;
    status_[zone].brightness = newBrightness;
    status_[zone].relayOn = shouldRelayBeOn;
    status_[zone].minutesUntilChange = getMinutesUntilChange(zone, currentMins, newState);

    // Verify power if expected power is configured
    if (expectedPower_[zone] > 0.0f && shouldRelayBeOn) {
        float expectedMin = expectedPower_[zone] * (1.0f - powerTolerance_[zone] / 100.0f);
        float expectedMax = expectedPower_[zone] * (1.0f + powerTolerance_[zone] / 100.0f);

        // Scale by brightness for dimmed lights
        if (hasDimming_[zone]) {
            float scale = newBrightness / 255.0f;
            expectedMin *= scale;
            expectedMax *= scale;
        }

        status_[zone].powerVerified =
            (status_[zone].powerWatts >= expectedMin * 0.5f);  // 50% threshold for "on"
    } else if (!shouldRelayBeOn) {
        // When off, power should be near zero
        status_[zone].powerVerified = (status_[zone].powerWatts < 10.0f);
    }
}

// ============================================================================
// STATE CALCULATION
// ============================================================================

LightState LightController::calculateState(uint8_t zone, uint16_t currentMins) {
    const LightSchedule& s = schedules_[zone];

    uint16_t onMins = s.onHour * 60 + s.onMinute;
    uint16_t offMins = s.offHour * 60 + s.offMinute;

    // Handle midnight crossing
    bool crossesMidnight = (offMins <= onMins);
    if (crossesMidnight && offMins == 0) {
        offMins = 24 * 60;  // Treat as 24:00
    }

    // Calculate sunrise/sunset boundaries
    uint16_t sunriseEnd = onMins + s.sunriseMins;
    uint16_t sunsetStart = offMins - s.sunsetMins;

    // Determine state based on current time
    if (crossesMidnight) {
        // Light period crosses midnight
        if (currentMins >= onMins) {
            // After on time today
            if (currentMins < sunriseEnd) return LightState::SUNRISE;
            return LightState::ON;
        } else if (currentMins < offMins) {
            // Before off time (after midnight)
            if (currentMins >= offMins - s.sunsetMins) return LightState::SUNSET;
            return LightState::ON;
        }
        return LightState::OFF;
    } else {
        // Normal schedule (same day)
        if (currentMins < onMins) {
            return LightState::OFF;
        } else if (currentMins < sunriseEnd) {
            return LightState::SUNRISE;
        } else if (currentMins < sunsetStart) {
            return LightState::ON;
        } else if (currentMins < offMins) {
            return LightState::SUNSET;
        }
        return LightState::OFF;
    }
}

uint8_t LightController::calculateBrightness(uint8_t zone, uint16_t currentMins, LightState state) {
    const LightSchedule& s = schedules_[zone];

    switch (state) {
        case LightState::OFF:
        case LightState::MANUAL_OFF:
            return 0;

        case LightState::ON:
        case LightState::MANUAL_ON:
            return 255;

        case LightState::SUNRISE: {
            if (s.sunriseMins == 0) return 255;
            uint16_t onMins = s.onHour * 60 + s.onMinute;
            uint16_t elapsed = currentMins - onMins;
            uint16_t progress = (elapsed * 255) / s.sunriseMins;
            return (progress > 255) ? 255 : progress;
        }

        case LightState::SUNSET: {
            if (s.sunsetMins == 0) return 0;
            uint16_t offMins = s.offHour * 60 + s.offMinute;
            uint16_t remaining = offMins - currentMins;
            uint16_t progress = (remaining * 255) / s.sunsetMins;
            return (progress > 255) ? 255 : progress;
        }
    }
    return 0;
}

uint16_t LightController::getMinutesUntilChange(uint8_t zone, uint16_t currentMins, LightState state) {
    const LightSchedule& s = schedules_[zone];

    uint16_t onMins = s.onHour * 60 + s.onMinute;
    uint16_t offMins = s.offHour * 60 + s.offMinute;
    if (offMins == 0) offMins = 24 * 60;

    switch (state) {
        case LightState::OFF:
            // Minutes until lights turn on
            if (currentMins < onMins) {
                return onMins - currentMins;
            }
            return (24 * 60 - currentMins) + onMins;  // Next day

        case LightState::SUNRISE:
            return (onMins + s.sunriseMins) - currentMins;

        case LightState::ON:
            return (offMins - s.sunsetMins) - currentMins;

        case LightState::SUNSET:
            return offMins - currentMins;

        default:
            return 0;
    }
}

// ============================================================================
// HARDWARE CONTROL
// ============================================================================

void LightController::setRelay(uint8_t zone, bool on) {
    if (zone >= 2 || relayPins_[zone] == 255) return;

#ifdef ARDUINO
    digitalWrite(relayPins_[zone], on ? HIGH : LOW);
#endif
}

void LightController::setPwm(uint8_t zone, uint8_t value) {
    if (zone >= 2) return;

    if (dimPins_[zone] != 255) {
#ifdef ARDUINO
        analogWrite(dimPins_[zone], value);
#endif
    }
}

// ============================================================================
// SCHEDULE MANAGEMENT
// ============================================================================

void LightController::setSchedule(uint8_t zone, const LightSchedule& schedule) {
    if (zone >= 2) return;
    schedules_[zone] = schedule;
}

LightSchedule LightController::getSchedule(uint8_t zone) const {
    if (zone >= 2) return schedules_[0];
    return schedules_[zone];
}

LightZoneStatus LightController::getStatus(uint8_t zone) const {
    if (zone >= 2) return status_[0];
    return status_[zone];
}

// ============================================================================
// MANUAL OVERRIDE
// ============================================================================

void LightController::setManualOverride(uint8_t zone, bool on) {
    if (zone >= 2) return;

    if (on) {
        status_[zone].state = LightState::MANUAL_ON;
        setRelay(zone, true);
        setPwm(zone, 255);
        status_[zone].brightness = 255;
        status_[zone].relayOn = true;
    } else {
        status_[zone].state = LightState::MANUAL_OFF;
        setRelay(zone, false);
        setPwm(zone, 0);
        status_[zone].brightness = 0;
        status_[zone].relayOn = false;
    }
}

void LightController::clearManualOverride(uint8_t zone) {
    if (zone >= 2) return;

    // Clear override - next update() will set correct state
    status_[zone].state = LightState::OFF;
}

bool LightController::isManualOverride(uint8_t zone) const {
    if (zone >= 2) return false;
    return (status_[zone].state == LightState::MANUAL_ON ||
            status_[zone].state == LightState::MANUAL_OFF);
}

// ============================================================================
// BRIGHTNESS CONTROL
// ============================================================================

void LightController::setBrightness(uint8_t zone, uint8_t brightness) {
    if (zone >= 2 || !hasDimming_[zone]) return;

    setPwm(zone, brightness);
    status_[zone].brightness = brightness;
}

void LightController::setZoneEnabled(uint8_t zone, bool enabled) {
    if (zone >= 2) return;
    schedules_[zone].enabled = enabled;
}

// ============================================================================
// POWER MONITORING
// ============================================================================

void LightController::reportPower(uint8_t zone, float watts) {
    if (zone >= 2) return;
    status_[zone].powerWatts = watts;
}

void LightController::setExpectedPower(uint8_t zone, float watts, float tolerancePercent) {
    if (zone >= 2) return;
    expectedPower_[zone] = watts;
    powerTolerance_[zone] = tolerancePercent;
}

bool LightController::hasPowerAnomaly() const {
    for (int i = 0; i < 2; i++) {
        if (schedules_[i].enabled && !status_[i].powerVerified) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// STATUS STRING
// ============================================================================

void LightController::getStatusString(uint8_t zone, char* buffer, size_t bufferSize) const {
    if (zone >= 2 || buffer == nullptr) return;

    const LightZoneStatus& s = status_[zone];
    const char* stateStr = "???";

    switch (s.state) {
        case LightState::OFF:        stateStr = "OFF"; break;
        case LightState::SUNRISE:    stateStr = "SUNRISE"; break;
        case LightState::ON:         stateStr = "ON"; break;
        case LightState::SUNSET:     stateStr = "SUNSET"; break;
        case LightState::MANUAL_ON:  stateStr = "MANUAL-ON"; break;
        case LightState::MANUAL_OFF: stateStr = "MANUAL-OFF"; break;
    }

    snprintf(buffer, bufferSize, "Z%d: %s %3d%% %.0fW %s",
             zone + 1,
             stateStr,
             (s.brightness * 100) / 255,
             s.powerWatts,
             s.powerVerified ? "OK" : "!PWR");
}

// ============================================================================
// TESTING
// ============================================================================

void LightController::injectCurrentTime(uint8_t hour, uint8_t minute) {
    rtc_->injectTime(hour, minute, 0);
}
