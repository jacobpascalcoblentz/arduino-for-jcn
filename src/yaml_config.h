/**
 * @file yaml_config.h
 * @brief Simple YAML configuration parser for Arduino
 *
 * Parses a subset of YAML from SD card for easy configuration by non-programmers.
 * Supports nested keys (e.g., "setpoints.ph") and basic types (float, int, bool, string).
 */

#ifndef YAML_CONFIG_H
#define YAML_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Maximum configuration values
#define MAX_CONFIG_KEYS 64
#define MAX_KEY_LENGTH 48
#define MAX_VALUE_LENGTH 32

// ============================================================================
// RUNTIME CONFIGURATION STRUCTURE
// ============================================================================

/**
 * @brief All configurable parameters loaded from config.yaml
 *
 * This structure holds all runtime-configurable values.
 * Default values match config.h compile-time defaults.
 */
struct HydroConfig {
    // Setpoints
    float phSetpoint;
    float tdsSetpoint;
    float temperatureSetpoint;
    float waterLevelSetpoint;

    // Deadbands
    float phDeadband;
    float tdsDeadband;
    float temperatureDeadband;
    float waterLevelDeadband;

    // Safety limits
    float phMin;
    float phMax;
    float tdsMin;
    float tdsMax;
    float tempMin;
    float tempMax;
    float levelMin;
    float levelMax;

    // Dosing limits
    float maxPhDoseMl;
    float maxNutrientDoseMl;
    float maxWaterMl;
    uint32_t minDoseIntervalSec;

    // System parameters
    float systemVolumeLiters;
    float tankHeightCm;
    float tankAreaCm2;

    // Pump flow rates
    float phDownFlowRate;
    float phUpFlowRate;
    float nutrientAFlowRate;
    float nutrientBFlowRate;
    float waterFlowRate;

    // Chemical strengths
    float phDownStrength;
    float phUpStrength;
    float nutrientATdsPerMl;
    float nutrientBTdsPerMl;

    // Sensor calibration
    float phNeutralMv;
    float phAcidMv;
    float tdsCalibrationFactor;
    float levelSensorOffsetCm;

    // Controller tuning
    float ffPhGain;
    float ffTdsGain;
    float ffLevelGain;
    float kiPh;
    float kiTds;
    float kiLevel;

    // Timing
    uint32_t sensorReadMs;
    uint32_t controlUpdateMs;
    uint32_t logIntervalMs;
    uint32_t displayUpdateMs;

    // Logging
    bool loggingEnabled;
    char logFilename[32];

    // Light Control - Zone 1 (typically veg)
    bool lightZone1Enabled;
    uint8_t lightZone1OnHour;
    uint8_t lightZone1OnMinute;
    uint8_t lightZone1OffHour;
    uint8_t lightZone1OffMinute;
    uint8_t lightZone1SunriseMins;
    uint8_t lightZone1SunsetMins;
    float lightZone1ExpectedWatts;

    // Light Control - Zone 2 (typically flower)
    bool lightZone2Enabled;
    uint8_t lightZone2OnHour;
    uint8_t lightZone2OnMinute;
    uint8_t lightZone2OffHour;
    uint8_t lightZone2OffMinute;
    uint8_t lightZone2SunriseMins;
    uint8_t lightZone2SunsetMins;
    float lightZone2ExpectedWatts;

    // Power monitoring
    bool powerMonitorEnabled;
    float powerVoltage;          // Mains voltage (120 or 230)
    float powerTolerance;        // Alert threshold percent

    // RTC/Time settings
    char timezone[32];           // Timezone for NTP sync (ESP32)
    bool ntpSyncEnabled;         // Auto-sync with NTP (ESP32)

    // Load status
    bool loaded;
    char loadError[64];

    // Initialize with defaults
    void setDefaults();
};

// ============================================================================
// YAML PARSER CLASS
// ============================================================================

class YamlConfigParser {
public:
    YamlConfigParser();

    /**
     * @brief Load configuration from SD card
     * @param filename Path to YAML file on SD card
     * @param config Output configuration structure
     * @return true if loaded successfully
     */
    bool load(const char* filename, HydroConfig& config);

    /**
     * @brief Get last error message
     */
    const char* getError() const { return errorMsg_; }

    /**
     * @brief Check if a specific key was found in the file
     */
    bool hasKey(const char* key) const;

    /**
     * @brief Get a float value by key path (e.g., "setpoints.ph")
     */
    float getFloat(const char* key, float defaultValue = 0.0f) const;

    /**
     * @brief Get an integer value by key path
     */
    int32_t getInt(const char* key, int32_t defaultValue = 0) const;

    /**
     * @brief Get a boolean value by key path
     */
    bool getBool(const char* key, bool defaultValue = false) const;

    /**
     * @brief Get a string value by key path
     */
    const char* getString(const char* key, const char* defaultValue = "") const;

private:
    // Key-value storage
    struct KeyValue {
        char key[MAX_KEY_LENGTH];
        char value[MAX_VALUE_LENGTH];
    };

    KeyValue entries_[MAX_CONFIG_KEYS];
    int entryCount_;
    char errorMsg_[64];
    char currentSection_[MAX_KEY_LENGTH];

    // Parsing helpers
    bool parseLine(const char* line);
    void trimWhitespace(char* str);
    bool isComment(const char* line);
    bool isSectionHeader(const char* line);
    bool isKeyValue(const char* line);
    void buildFullKey(const char* section, const char* key, char* fullKey);
    int findKey(const char* key) const;
};

// ============================================================================
// CONFIGURATION LOADER
// ============================================================================

/**
 * @brief High-level configuration loader
 *
 * Handles SD card initialization and applies config to system components.
 */
class ConfigLoader {
public:
    ConfigLoader();

    /**
     * @brief Load config from SD card and apply to system
     * @param config Output configuration
     * @return true if successful
     */
    bool loadFromSD(HydroConfig& config);

    /**
     * @brief Save current config back to SD card
     * @param config Configuration to save
     * @return true if successful
     */
    bool saveToSD(const HydroConfig& config);

    /**
     * @brief Print current configuration to Serial
     */
    static void printConfig(const HydroConfig& config);

    /**
     * @brief Validate configuration values
     * @return true if all values are within reasonable ranges
     */
    static bool validateConfig(const HydroConfig& config, char* errorMsg);

private:
    bool sdInitialized_;
};

// ============================================================================
// GLOBAL CONFIG INSTANCE
// ============================================================================

extern HydroConfig gConfig;

#endif // YAML_CONFIG_H
