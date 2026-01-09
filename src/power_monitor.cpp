/**
 * @file power_monitor.cpp
 * @brief ACS712 current sensor implementation
 */

#include "power_monitor.h"
#include "config.h"

#include <stdio.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "mock_arduino.h"
#endif

// Number of samples to average for stable reading
#define POWER_SAMPLE_COUNT 20

// ============================================================================
// POWER MONITOR IMPLEMENTATION
// ============================================================================

PowerMonitor::PowerMonitor(uint8_t numChannels)
    : numChannels_(numChannels > MAX_CHANNELS ? MAX_CHANNELS : numChannels) {

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        channels_[i].pin = 255;
        channels_[i].sensitivity = ACS712_SENSITIVITY;
        channels_[i].zeroOffset = ACS712_ZERO_POINT;
        channels_[i].voltage = POWER_VOLTAGE;
        channels_[i].threshold = 0.1f;  // 100mA default threshold
        channels_[i].configured = false;

        readings_[i].currentAmps = 0.0f;
        readings_[i].powerWatts = 0.0f;
        readings_[i].voltage = POWER_VOLTAGE;
        readings_[i].rawAdc = 0;
        readings_[i].valid = false;
        readings_[i].lightDetected = false;

        useInjected_[i] = false;
        injectedAmps_[i] = 0.0f;
    }
}

bool PowerMonitor::begin(uint8_t channel, uint8_t analogPin,
                         ACS712Variant variant, float voltage) {
    if (channel >= numChannels_) return false;

    channels_[channel].pin = analogPin;
    channels_[channel].sensitivity = getSensitivity(variant);
    channels_[channel].voltage = voltage;
    channels_[channel].configured = true;

#ifdef ARDUINO
    pinMode(analogPin, INPUT);
#endif

    return true;
}

float PowerMonitor::getSensitivity(ACS712Variant variant) {
    switch (variant) {
        case ACS712Variant::ACS712_5A:  return 0.185f;
        case ACS712Variant::ACS712_20A: return 0.100f;
        case ACS712Variant::ACS712_30A: return 0.066f;
        default: return 0.185f;
    }
}

int PowerMonitor::readAnalog(uint8_t pin) {
#ifdef ARDUINO
    return analogRead(pin);
#else
    return 512;  // Mid-point for testing
#endif
}

PowerReading PowerMonitor::read(uint8_t channel) {
    PowerReading result = {0, 0, 0, 0, false, false};

    if (channel >= numChannels_ || !channels_[channel].configured) {
        return result;
    }

    // Check for injected test value
    if (useInjected_[channel]) {
        result.currentAmps = injectedAmps_[channel];
        result.voltage = channels_[channel].voltage;
        result.powerWatts = result.currentAmps * result.voltage;
        result.valid = true;
        result.lightDetected = (result.currentAmps >= channels_[channel].threshold);
        readings_[channel] = result;
        return result;
    }

    // Average multiple samples for stability
    uint32_t adcSum = 0;
    for (int i = 0; i < POWER_SAMPLE_COUNT; i++) {
        adcSum += readAnalog(channels_[channel].pin);
#ifdef ARDUINO
        delayMicroseconds(100);
#endif
    }
    uint16_t avgAdc = adcSum / POWER_SAMPLE_COUNT;

    // Convert ADC to voltage
    float voltage = (avgAdc * ADC_VREF) / ADC_RESOLUTION;

    // Convert voltage to current
    // ACS712 output: 2.5V at 0A, sensitivity V/A
    float current = (voltage - channels_[channel].zeroOffset) / channels_[channel].sensitivity;

    // For AC measurement, we're measuring DC offset which represents RMS
    // Take absolute value since we don't care about direction
    current = (current < 0) ? -current : current;

    // Calculate power
    float power = current * channels_[channel].voltage;

    // Populate result
    result.currentAmps = current;
    result.powerWatts = power;
    result.voltage = channels_[channel].voltage;
    result.rawAdc = avgAdc;
    result.valid = true;
    result.lightDetected = (current >= channels_[channel].threshold);

    readings_[channel] = result;
    return result;
}

void PowerMonitor::readAll() {
    for (uint8_t i = 0; i < numChannels_; i++) {
        if (channels_[i].configured) {
            read(i);
        }
    }
}

PowerReading PowerMonitor::getReading(uint8_t channel) const {
    if (channel >= numChannels_) {
        return PowerReading{0, 0, 0, 0, false, false};
    }
    return readings_[channel];
}

float PowerMonitor::getTotalPower() const {
    float total = 0.0f;
    for (uint8_t i = 0; i < numChannels_; i++) {
        if (readings_[i].valid) {
            total += readings_[i].powerWatts;
        }
    }
    return total;
}

void PowerMonitor::calibrateZero(uint8_t channel) {
    if (channel >= numChannels_ || !channels_[channel].configured) return;

    // Average many samples with no load
    uint32_t adcSum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        adcSum += readAnalog(channels_[channel].pin);
#ifdef ARDUINO
        delay(10);
#endif
    }

    float avgVoltage = ((adcSum / samples) * ADC_VREF) / ADC_RESOLUTION;
    channels_[channel].zeroOffset = avgVoltage;

#ifdef ARDUINO
    Serial.print(F("Channel "));
    Serial.print(channel);
    Serial.print(F(" zero calibrated to "));
    Serial.print(avgVoltage, 3);
    Serial.println(F("V"));
#endif
}

void PowerMonitor::setZeroOffset(uint8_t channel, float offsetVolts) {
    if (channel >= numChannels_) return;
    channels_[channel].zeroOffset = offsetVolts;
}

void PowerMonitor::setDetectionThreshold(uint8_t channel, float amps) {
    if (channel >= numChannels_) return;
    channels_[channel].threshold = amps;
}

bool PowerMonitor::isAnyLightOn() const {
    for (uint8_t i = 0; i < numChannels_; i++) {
        if (readings_[i].valid && readings_[i].lightDetected) {
            return true;
        }
    }
    return false;
}

bool PowerMonitor::isLightOn(uint8_t channel) const {
    if (channel >= numChannels_) return false;
    return readings_[channel].valid && readings_[channel].lightDetected;
}

void PowerMonitor::injectReading(uint8_t channel, float amps) {
    if (channel >= numChannels_) return;
    useInjected_[channel] = true;
    injectedAmps_[channel] = amps;
}

void PowerMonitor::clearInjectedReadings() {
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        useInjected_[i] = false;
    }
}

// ============================================================================
// LIGHT POWER VERIFIER IMPLEMENTATION
// ============================================================================

LightPowerVerifier::LightPowerVerifier()
    : monitor_(2), initialized_(false) {
}

bool LightPowerVerifier::begin(uint8_t zone1Pin, uint8_t zone2Pin) {
    bool ok = true;
    ok &= monitor_.begin(0, zone1Pin, ACS712Variant::ACS712_5A, POWER_VOLTAGE);
    ok &= monitor_.begin(1, zone2Pin, ACS712Variant::ACS712_5A, POWER_VOLTAGE);

    // Set detection thresholds based on expected power
    // If expecting 400W at 120V, that's ~3.3A
    // Detect at 10% of expected = 0.33A
    monitor_.setDetectionThreshold(0, LIGHT_ZONE1_EXPECTED_WATTS / POWER_VOLTAGE * 0.1f);
    monitor_.setDetectionThreshold(1, LIGHT_ZONE2_EXPECTED_WATTS / POWER_VOLTAGE * 0.1f);

    initialized_ = ok;
    return ok;
}

bool LightPowerVerifier::isLightOn(uint8_t zone) {
    if (!initialized_ || zone >= 2) return false;
    monitor_.read(zone);
    return monitor_.isLightOn(zone);
}

float LightPowerVerifier::getPower(uint8_t zone) {
    if (!initialized_ || zone >= 2) return 0.0f;
    monitor_.read(zone);
    return monitor_.getReading(zone).powerWatts;
}

bool LightPowerVerifier::verify(uint8_t zone, bool expectedOn) {
    if (!initialized_ || zone >= 2) return true;  // Can't verify, assume OK

    bool actualOn = isLightOn(zone);
    return (actualOn == expectedOn);
}

void LightPowerVerifier::getStatusString(char* buffer, size_t size) {
    if (!initialized_) {
        snprintf(buffer, size, "Power monitor not initialized");
        return;
    }

    monitor_.readAll();
    PowerReading r0 = monitor_.getReading(0);
    PowerReading r1 = monitor_.getReading(1);

    snprintf(buffer, size, "Z1:%.0fW(%s) Z2:%.0fW(%s)",
             r0.powerWatts, r0.lightDetected ? "ON" : "off",
             r1.powerWatts, r1.lightDetected ? "ON" : "off");
}
