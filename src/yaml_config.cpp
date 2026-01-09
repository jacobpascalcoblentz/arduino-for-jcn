/**
 * @file yaml_config.cpp
 * @brief YAML configuration parser implementation
 */

#include "yaml_config.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <SD.h>
#endif

// Global config instance
HydroConfig gConfig;

// ============================================================================
// HYDRO CONFIG DEFAULTS
// ============================================================================

void HydroConfig::setDefaults() {
    // Setpoints
    phSetpoint = PH_SETPOINT;
    tdsSetpoint = TDS_SETPOINT;
    temperatureSetpoint = TEMP_SETPOINT;
    waterLevelSetpoint = LEVEL_SETPOINT;

    // Deadbands
    phDeadband = PH_DEADBAND;
    tdsDeadband = TDS_DEADBAND;
    temperatureDeadband = TEMP_DEADBAND;
    waterLevelDeadband = LEVEL_DEADBAND;

    // Safety limits
    phMin = PH_MIN_SAFE;
    phMax = PH_MAX_SAFE;
    tdsMin = TDS_MIN_SAFE;
    tdsMax = TDS_MAX_SAFE;
    tempMin = TEMP_MIN_SAFE;
    tempMax = TEMP_MAX_SAFE;
    levelMin = LEVEL_MIN_SAFE;
    levelMax = LEVEL_MAX_SAFE;

    // Dosing limits
    maxPhDoseMl = MAX_PH_DOSE_ML;
    maxNutrientDoseMl = MAX_NUTRIENT_DOSE_ML;
    maxWaterMl = 1000.0f;
    minDoseIntervalSec = MIN_DOSE_INTERVAL_SEC;

    // System parameters
    systemVolumeLiters = SYSTEM_VOLUME_LITERS;
    tankHeightCm = WATER_LEVEL_TANK_HEIGHT_CM;
    tankAreaCm2 = 2500.0f;

    // Pump flow rates
    phDownFlowRate = PUMP_PH_DOWN_FLOW_RATE;
    phUpFlowRate = PUMP_PH_UP_FLOW_RATE;
    nutrientAFlowRate = PUMP_NUTRIENT_A_FLOW_RATE;
    nutrientBFlowRate = PUMP_NUTRIENT_B_FLOW_RATE;
    waterFlowRate = VALVE_WATER_FLOW_RATE;

    // Chemical strengths
    phDownStrength = PH_DOWN_CONCENTRATION;
    phUpStrength = PH_UP_CONCENTRATION;
    nutrientATdsPerMl = NUTRIENT_A_TDS_PER_ML;
    nutrientBTdsPerMl = NUTRIENT_B_TDS_PER_ML;

    // Sensor calibration
    phNeutralMv = PH_CALIBRATION_VOLTAGE_NEUTRAL;
    phAcidMv = PH_CALIBRATION_VOLTAGE_ACID;
    tdsCalibrationFactor = TDS_CALIBRATION_FACTOR;
    levelSensorOffsetCm = WATER_LEVEL_SENSOR_OFFSET_CM;

    // Controller tuning
    ffPhGain = FF_PH_GAIN;
    ffTdsGain = FF_TDS_GAIN;
    ffLevelGain = FF_LEVEL_GAIN;
    kiPh = 0.001f;
    kiTds = 0.01f;
    kiLevel = 0.05f;

    // Timing
    sensorReadMs = SENSOR_READ_INTERVAL_MS;
    controlUpdateMs = CONTROL_UPDATE_INTERVAL_MS;
    logIntervalMs = LOG_INTERVAL_MS;
    displayUpdateMs = DISPLAY_UPDATE_INTERVAL_MS;

    // Logging
    loggingEnabled = true;
    strcpy(logFilename, "hydro.csv");

    // Status
    loaded = false;
    loadError[0] = '\0';
}

// ============================================================================
// YAML PARSER IMPLEMENTATION
// ============================================================================

YamlConfigParser::YamlConfigParser() {
    entryCount_ = 0;
    errorMsg_[0] = '\0';
    currentSection_[0] = '\0';
}

bool YamlConfigParser::load(const char* filename, HydroConfig& config) {
    // Start with defaults
    config.setDefaults();
    entryCount_ = 0;

#ifdef ARDUINO
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        strcpy(errorMsg_, "Failed to open config file");
        strcpy(config.loadError, errorMsg_);
        return false;
    }

    char line[128];
    int lineNum = 0;

    while (file.available() && entryCount_ < MAX_CONFIG_KEYS) {
        // Read line
        int i = 0;
        while (file.available() && i < 127) {
            char c = file.read();
            if (c == '\n' || c == '\r') break;
            line[i++] = c;
        }
        line[i] = '\0';
        lineNum++;

        // Skip empty lines and comments
        trimWhitespace(line);
        if (line[0] == '\0' || isComment(line)) {
            continue;
        }

        // Parse line
        if (!parseLine(line)) {
            snprintf(errorMsg_, sizeof(errorMsg_), "Parse error line %d", lineNum);
        }
    }

    file.close();
#else
    // For testing: try to read from filesystem
    FILE* file = fopen(filename, "r");
    if (!file) {
        strcpy(errorMsg_, "Failed to open config file");
        strcpy(config.loadError, errorMsg_);
        return false;
    }

    char line[128];
    int lineNum = 0;

    while (fgets(line, sizeof(line), file) && entryCount_ < MAX_CONFIG_KEYS) {
        lineNum++;

        trimWhitespace(line);
        if (line[0] == '\0' || isComment(line)) {
            continue;
        }

        if (!parseLine(line)) {
            snprintf(errorMsg_, sizeof(errorMsg_), "Parse error line %d", lineNum);
        }
    }

    fclose(file);
#endif

    // Apply parsed values to config
    // Setpoints
    config.phSetpoint = getFloat("setpoints.ph", config.phSetpoint);
    config.tdsSetpoint = getFloat("setpoints.tds", config.tdsSetpoint);
    config.temperatureSetpoint = getFloat("setpoints.temperature", config.temperatureSetpoint);
    config.waterLevelSetpoint = getFloat("setpoints.water_level", config.waterLevelSetpoint);

    // Deadbands
    config.phDeadband = getFloat("deadbands.ph", config.phDeadband);
    config.tdsDeadband = getFloat("deadbands.tds", config.tdsDeadband);
    config.temperatureDeadband = getFloat("deadbands.temperature", config.temperatureDeadband);
    config.waterLevelDeadband = getFloat("deadbands.water_level", config.waterLevelDeadband);

    // Safety
    config.phMin = getFloat("safety.ph_min", config.phMin);
    config.phMax = getFloat("safety.ph_max", config.phMax);
    config.tdsMin = getFloat("safety.tds_min", config.tdsMin);
    config.tdsMax = getFloat("safety.tds_max", config.tdsMax);
    config.tempMin = getFloat("safety.temp_min", config.tempMin);
    config.tempMax = getFloat("safety.temp_max", config.tempMax);
    config.levelMin = getFloat("safety.level_min", config.levelMin);
    config.levelMax = getFloat("safety.level_max", config.levelMax);

    // Dosing
    config.maxPhDoseMl = getFloat("dosing.max_ph_dose_ml", config.maxPhDoseMl);
    config.maxNutrientDoseMl = getFloat("dosing.max_nutrient_dose_ml", config.maxNutrientDoseMl);
    config.maxWaterMl = getFloat("dosing.max_water_ml", config.maxWaterMl);
    config.minDoseIntervalSec = (uint32_t)getInt("dosing.min_dose_interval_sec", config.minDoseIntervalSec);

    // System
    config.systemVolumeLiters = getFloat("system.volume_liters", config.systemVolumeLiters);
    config.tankHeightCm = getFloat("system.tank_height_cm", config.tankHeightCm);
    config.tankAreaCm2 = getFloat("system.tank_area_cm2", config.tankAreaCm2);

    // Pumps
    config.phDownFlowRate = getFloat("pumps.ph_down_ml_per_sec", config.phDownFlowRate);
    config.phUpFlowRate = getFloat("pumps.ph_up_ml_per_sec", config.phUpFlowRate);
    config.nutrientAFlowRate = getFloat("pumps.nutrient_a_ml_per_sec", config.nutrientAFlowRate);
    config.nutrientBFlowRate = getFloat("pumps.nutrient_b_ml_per_sec", config.nutrientBFlowRate);
    config.waterFlowRate = getFloat("pumps.water_ml_per_sec", config.waterFlowRate);

    // Chemicals
    config.phDownStrength = getFloat("chemicals.ph_down_strength", config.phDownStrength);
    config.phUpStrength = getFloat("chemicals.ph_up_strength", config.phUpStrength);
    config.nutrientATdsPerMl = getFloat("chemicals.nutrient_a_tds_per_ml", config.nutrientATdsPerMl);
    config.nutrientBTdsPerMl = getFloat("chemicals.nutrient_b_tds_per_ml", config.nutrientBTdsPerMl);

    // Calibration
    config.phNeutralMv = getFloat("calibration.ph_neutral_mv", config.phNeutralMv);
    config.phAcidMv = getFloat("calibration.ph_acid_mv", config.phAcidMv);
    config.tdsCalibrationFactor = getFloat("calibration.tds_factor", config.tdsCalibrationFactor);
    config.levelSensorOffsetCm = getFloat("calibration.level_sensor_offset_cm", config.levelSensorOffsetCm);

    // Controller
    config.ffPhGain = getFloat("controller.ff_ph_gain", config.ffPhGain);
    config.ffTdsGain = getFloat("controller.ff_tds_gain", config.ffTdsGain);
    config.ffLevelGain = getFloat("controller.ff_level_gain", config.ffLevelGain);
    config.kiPh = getFloat("controller.ki_ph", config.kiPh);
    config.kiTds = getFloat("controller.ki_tds", config.kiTds);
    config.kiLevel = getFloat("controller.ki_level", config.kiLevel);

    // Timing
    config.sensorReadMs = (uint32_t)getInt("timing.sensor_read_ms", config.sensorReadMs);
    config.controlUpdateMs = (uint32_t)getInt("timing.control_update_ms", config.controlUpdateMs);
    config.logIntervalMs = (uint32_t)getInt("timing.log_interval_ms", config.logIntervalMs);
    config.displayUpdateMs = (uint32_t)getInt("timing.display_update_ms", config.displayUpdateMs);

    // Logging
    config.loggingEnabled = getBool("logging.enabled", config.loggingEnabled);
    const char* logFile = getString("logging.filename", config.logFilename);
    strncpy(config.logFilename, logFile, sizeof(config.logFilename) - 1);

    config.loaded = true;
    return true;
}

bool YamlConfigParser::parseLine(const char* line) {
    // Check for section header (e.g., "setpoints:")
    if (isSectionHeader(line)) {
        // Extract section name
        char temp[MAX_KEY_LENGTH];
        strncpy(temp, line, sizeof(temp) - 1);
        temp[sizeof(temp) - 1] = '\0';

        // Remove trailing colon
        char* colon = strchr(temp, ':');
        if (colon) *colon = '\0';

        // Count leading spaces to determine nesting level
        int indent = 0;
        const char* p = line;
        while (*p == ' ') { indent++; p++; }

        if (indent == 0) {
            // Top-level section
            trimWhitespace(temp);
            strncpy(currentSection_, temp, sizeof(currentSection_) - 1);
        }
        // Nested sections not fully supported, but we handle simple nesting
        return true;
    }

    // Check for key-value pair
    if (isKeyValue(line)) {
        char* colonPos = strchr((char*)line, ':');
        if (!colonPos) return false;

        // Split into key and value
        char key[MAX_KEY_LENGTH];
        char value[MAX_VALUE_LENGTH];

        int keyLen = colonPos - line;
        if (keyLen >= MAX_KEY_LENGTH) keyLen = MAX_KEY_LENGTH - 1;
        strncpy(key, line, keyLen);
        key[keyLen] = '\0';
        trimWhitespace(key);

        strncpy(value, colonPos + 1, MAX_VALUE_LENGTH - 1);
        value[MAX_VALUE_LENGTH - 1] = '\0';
        trimWhitespace(value);

        // Remove inline comments
        char* commentPos = strchr(value, '#');
        if (commentPos) {
            *commentPos = '\0';
            trimWhitespace(value);
        }

        // Build full key with section prefix
        char fullKey[MAX_KEY_LENGTH];
        buildFullKey(currentSection_, key, fullKey);

        // Store entry
        if (entryCount_ < MAX_CONFIG_KEYS) {
            strncpy(entries_[entryCount_].key, fullKey, MAX_KEY_LENGTH - 1);
            strncpy(entries_[entryCount_].value, value, MAX_VALUE_LENGTH - 1);
            entryCount_++;
        }

        return true;
    }

    return true;  // Skip unknown lines
}

void YamlConfigParser::trimWhitespace(char* str) {
    if (!str || !*str) return;

    // Trim leading
    char* start = str;
    while (*start && isspace((unsigned char)*start)) start++;

    // Trim trailing
    char* end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) end--;
    *(end + 1) = '\0';

    // Move to beginning
    if (start != str) {
        memmove(str, start, strlen(start) + 1);
    }
}

bool YamlConfigParser::isComment(const char* line) {
    const char* p = line;
    while (*p && isspace((unsigned char)*p)) p++;
    return *p == '#';
}

bool YamlConfigParser::isSectionHeader(const char* line) {
    // Section header: "word:" with no value after colon (or only whitespace)
    const char* colon = strchr(line, ':');
    if (!colon) return false;

    // Check if there's anything meaningful after the colon
    const char* afterColon = colon + 1;
    while (*afterColon && isspace((unsigned char)*afterColon)) afterColon++;

    // If nothing after colon (or just comment), it's a section header
    return *afterColon == '\0' || *afterColon == '#';
}

bool YamlConfigParser::isKeyValue(const char* line) {
    const char* colon = strchr(line, ':');
    if (!colon) return false;

    // Check if there's a value after the colon
    const char* afterColon = colon + 1;
    while (*afterColon && isspace((unsigned char)*afterColon)) afterColon++;

    return *afterColon != '\0' && *afterColon != '#';
}

void YamlConfigParser::buildFullKey(const char* section, const char* key, char* fullKey) {
    if (section && section[0] != '\0') {
        snprintf(fullKey, MAX_KEY_LENGTH, "%s.%s", section, key);
    } else {
        strncpy(fullKey, key, MAX_KEY_LENGTH - 1);
        fullKey[MAX_KEY_LENGTH - 1] = '\0';
    }
}

int YamlConfigParser::findKey(const char* key) const {
    for (int i = 0; i < entryCount_; i++) {
        if (strcmp(entries_[i].key, key) == 0) {
            return i;
        }
    }
    return -1;
}

bool YamlConfigParser::hasKey(const char* key) const {
    return findKey(key) >= 0;
}

float YamlConfigParser::getFloat(const char* key, float defaultValue) const {
    int idx = findKey(key);
    if (idx < 0) return defaultValue;

    return (float)atof(entries_[idx].value);
}

int32_t YamlConfigParser::getInt(const char* key, int32_t defaultValue) const {
    int idx = findKey(key);
    if (idx < 0) return defaultValue;

    return atoi(entries_[idx].value);
}

bool YamlConfigParser::getBool(const char* key, bool defaultValue) const {
    int idx = findKey(key);
    if (idx < 0) return defaultValue;

    const char* val = entries_[idx].value;
    return (strcmp(val, "true") == 0 ||
            strcmp(val, "True") == 0 ||
            strcmp(val, "TRUE") == 0 ||
            strcmp(val, "yes") == 0 ||
            strcmp(val, "Yes") == 0 ||
            strcmp(val, "1") == 0);
}

const char* YamlConfigParser::getString(const char* key, const char* defaultValue) const {
    int idx = findKey(key);
    if (idx < 0) return defaultValue;

    // Remove quotes if present
    static char unquoted[MAX_VALUE_LENGTH];
    const char* val = entries_[idx].value;

    if ((val[0] == '"' || val[0] == '\'') && strlen(val) > 2) {
        strncpy(unquoted, val + 1, MAX_VALUE_LENGTH - 1);
        size_t len = strlen(unquoted);
        if (len > 0 && (unquoted[len-1] == '"' || unquoted[len-1] == '\'')) {
            unquoted[len-1] = '\0';
        }
        return unquoted;
    }

    return val;
}

// ============================================================================
// CONFIG LOADER IMPLEMENTATION
// ============================================================================

ConfigLoader::ConfigLoader() : sdInitialized_(false) {
}

bool ConfigLoader::loadFromSD(HydroConfig& config) {
#ifdef ARDUINO
    if (!sdInitialized_) {
        if (!SD.begin(PIN_SD_CS)) {
            strcpy(config.loadError, "SD card init failed");
            config.setDefaults();
            return false;
        }
        sdInitialized_ = true;
    }

    // Check if config file exists
    if (!SD.exists("config.yaml")) {
        strcpy(config.loadError, "config.yaml not found, using defaults");
        config.setDefaults();
        return false;
    }
#endif

    YamlConfigParser parser;
    bool success = parser.load("config.yaml", config);

    if (!success) {
        strncpy(config.loadError, parser.getError(), sizeof(config.loadError) - 1);
    }

    // Validate loaded config
    char validationError[64];
    if (!validateConfig(config, validationError)) {
        strncpy(config.loadError, validationError, sizeof(config.loadError) - 1);
        // Don't fail, just warn - use the loaded values anyway
    }

    return success;
}

bool ConfigLoader::saveToSD(const HydroConfig& config) {
#ifdef ARDUINO
    if (!sdInitialized_) return false;

    // For simplicity, we don't implement full YAML writing
    // Just save critical setpoints
    File file = SD.open("config.yaml", FILE_WRITE);
    if (!file) return false;

    file.println("# Auto-saved configuration");
    file.println("setpoints:");
    file.print("  ph: ");
    file.println(config.phSetpoint, 2);
    file.print("  tds: ");
    file.println(config.tdsSetpoint, 0);
    file.print("  temperature: ");
    file.println(config.temperatureSetpoint, 1);
    file.print("  water_level: ");
    file.println(config.waterLevelSetpoint, 0);

    file.close();
    return true;
#else
    (void)config;
    return false;
#endif
}

void ConfigLoader::printConfig(const HydroConfig& config) {
#ifdef ARDUINO
    Serial.println(F("\n=== LOADED CONFIGURATION ===\n"));

    Serial.println(F("Setpoints:"));
    Serial.print(F("  pH:          ")); Serial.println(config.phSetpoint, 2);
    Serial.print(F("  TDS:         ")); Serial.print(config.tdsSetpoint, 0); Serial.println(F(" ppm"));
    Serial.print(F("  Temperature: ")); Serial.print(config.temperatureSetpoint, 1); Serial.println(F(" C"));
    Serial.print(F("  Water Level: ")); Serial.print(config.waterLevelSetpoint, 0); Serial.println(F(" %"));

    Serial.println(F("\nSafety Limits:"));
    Serial.print(F("  pH:    ")); Serial.print(config.phMin, 1);
    Serial.print(F(" - ")); Serial.println(config.phMax, 1);
    Serial.print(F("  TDS:   ")); Serial.print(config.tdsMin, 0);
    Serial.print(F(" - ")); Serial.print(config.tdsMax, 0); Serial.println(F(" ppm"));
    Serial.print(F("  Temp:  ")); Serial.print(config.tempMin, 0);
    Serial.print(F(" - ")); Serial.print(config.tempMax, 0); Serial.println(F(" C"));

    Serial.println(F("\nSystem:"));
    Serial.print(F("  Volume: ")); Serial.print(config.systemVolumeLiters, 0); Serial.println(F(" L"));

    if (config.loadError[0] != '\0') {
        Serial.print(F("\nWarning: "));
        Serial.println(config.loadError);
    }

    Serial.println(F("\n============================\n"));
#else
    printf("\n=== LOADED CONFIGURATION ===\n");
    printf("pH Setpoint: %.2f\n", config.phSetpoint);
    printf("TDS Setpoint: %.0f ppm\n", config.tdsSetpoint);
    printf("Temperature: %.1f C\n", config.temperatureSetpoint);
    printf("Water Level: %.0f%%\n", config.waterLevelSetpoint);
    if (config.loadError[0] != '\0') {
        printf("Warning: %s\n", config.loadError);
    }
#endif
}

bool ConfigLoader::validateConfig(const HydroConfig& config, char* errorMsg) {
    // pH validation
    if (config.phSetpoint < 3.0f || config.phSetpoint > 10.0f) {
        strcpy(errorMsg, "pH setpoint out of range (3-10)");
        return false;
    }
    if (config.phMin >= config.phMax) {
        strcpy(errorMsg, "pH min must be less than max");
        return false;
    }

    // TDS validation
    if (config.tdsSetpoint < 0.0f || config.tdsSetpoint > 3000.0f) {
        strcpy(errorMsg, "TDS setpoint out of range (0-3000)");
        return false;
    }

    // Temperature validation
    if (config.temperatureSetpoint < 10.0f || config.temperatureSetpoint > 40.0f) {
        strcpy(errorMsg, "Temperature setpoint out of range (10-40)");
        return false;
    }

    // Level validation
    if (config.waterLevelSetpoint < 0.0f || config.waterLevelSetpoint > 100.0f) {
        strcpy(errorMsg, "Water level setpoint out of range (0-100)");
        return false;
    }

    // System volume validation
    if (config.systemVolumeLiters <= 0.0f) {
        strcpy(errorMsg, "System volume must be positive");
        return false;
    }

    errorMsg[0] = '\0';
    return true;
}
