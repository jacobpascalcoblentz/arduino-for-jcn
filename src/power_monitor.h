/**
 * @file power_monitor.h
 * @brief Power/current monitoring using ACS712 current sensors
 */

#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// POWER READING STRUCTURE
// ============================================================================

struct PowerReading {
    float currentAmps;      // Measured current (A)
    float powerWatts;       // Calculated power (W)
    float voltage;          // System voltage (configured)
    uint16_t rawAdc;        // Raw ADC reading
    bool valid;             // Reading is valid
    bool lightDetected;     // Significant current detected
};

// ============================================================================
// ACS712 VARIANTS
// ============================================================================

enum class ACS712Variant {
    ACS712_5A,    // 185 mV/A
    ACS712_20A,   // 100 mV/A
    ACS712_30A    // 66 mV/A
};

// ============================================================================
// POWER MONITOR CLASS
// ============================================================================

class PowerMonitor {
public:
    /**
     * Constructor
     * @param numChannels Number of current sensors (1-4)
     */
    explicit PowerMonitor(uint8_t numChannels = 2);

    /**
     * Initialize a current sensor channel
     * @param channel Channel index (0-based)
     * @param analogPin Analog pin connected to ACS712
     * @param variant ACS712 variant (5A, 20A, 30A)
     * @param voltage System voltage for power calculation
     */
    bool begin(uint8_t channel, uint8_t analogPin,
               ACS712Variant variant = ACS712Variant::ACS712_5A,
               float voltage = 120.0f);

    /**
     * Read current from a channel
     * @param channel Channel index
     * @return PowerReading with current and power
     */
    PowerReading read(uint8_t channel);

    /**
     * Read all channels
     */
    void readAll();

    /**
     * Get last reading for a channel
     */
    PowerReading getReading(uint8_t channel) const;

    /**
     * Get total power across all channels
     */
    float getTotalPower() const;

    /**
     * Calibrate zero point for a channel
     * Call with no load to set the zero current offset
     */
    void calibrateZero(uint8_t channel);

    /**
     * Set custom zero offset
     */
    void setZeroOffset(uint8_t channel, float offsetVolts);

    /**
     * Set minimum current threshold for "light detected"
     * @param amps Minimum current to consider light as on
     */
    void setDetectionThreshold(uint8_t channel, float amps);

    /**
     * Check if light is detected on any channel
     */
    bool isAnyLightOn() const;

    /**
     * Check if light is detected on specific channel
     */
    bool isLightOn(uint8_t channel) const;

    // For testing
    void injectReading(uint8_t channel, float amps);
    void clearInjectedReadings();

private:
    static const uint8_t MAX_CHANNELS = 4;

    struct ChannelConfig {
        uint8_t pin;
        float sensitivity;     // V/A
        float zeroOffset;      // Zero current voltage
        float voltage;         // System voltage
        float threshold;       // Detection threshold (A)
        bool configured;
    };

    uint8_t numChannels_;
    ChannelConfig channels_[MAX_CHANNELS];
    PowerReading readings_[MAX_CHANNELS];

    // Testing
    bool useInjected_[MAX_CHANNELS];
    float injectedAmps_[MAX_CHANNELS];

    float getSensitivity(ACS712Variant variant);
    int readAnalog(uint8_t pin);
};

// ============================================================================
// SIMPLE POWER VERIFIER
// ============================================================================

/**
 * Simplified class for just verifying lights are on/off
 */
class LightPowerVerifier {
public:
    LightPowerVerifier();

    /**
     * Initialize with analog pins
     */
    bool begin(uint8_t zone1Pin, uint8_t zone2Pin);

    /**
     * Read and check if light is drawing power
     */
    bool isLightOn(uint8_t zone);

    /**
     * Get measured power for zone
     */
    float getPower(uint8_t zone);

    /**
     * Verify light state matches expected
     * @param zone Zone index
     * @param expectedOn Whether light should be on
     * @return true if actual matches expected
     */
    bool verify(uint8_t zone, bool expectedOn);

    /**
     * Get verification status string
     */
    void getStatusString(char* buffer, size_t size);

private:
    PowerMonitor monitor_;
    bool initialized_;
};

#endif // POWER_MONITOR_H
