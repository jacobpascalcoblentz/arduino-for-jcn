/**
 * @file main.cpp
 * @brief Hydroponics Monitoring and Control System - Main Application
 *
 * Advanced feedforward controller for automated hydroponics management.
 * Monitors pH, EC/TDS, temperature, and water level with automatic
 * nutrient dosing and pH adjustment.
 */

#include "config.h"
#include "sensors.h"
#include "controller.h"
#include "yaml_config.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>

// LCD Display (20x4 I2C)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// SD Card logging
File logFile;
bool sdInitialized = false;

// Configuration loader
ConfigLoader configLoader;
#endif

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

SensorManager sensors;
FeedforwardController controller;
ActuatorDriver actuators;
SafetyMonitor safety;

// State tracking
SensorReadings currentReadings;
SensorReadings previousReadings;
ControlAction lastAction;

// Timing
uint32_t lastSensorRead = 0;
uint32_t lastControlUpdate = 0;
uint32_t lastLogWrite = 0;
uint32_t lastDisplayUpdate = 0;

// System state
bool systemRunning = true;
bool calibrationMode = false;
uint32_t uptimeSeconds = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSystem();
void readSensors();
void runController();
void updateDisplay();
void logData();
void handleSerialCommands();
void printStatus();
void enterCalibrationMode();
void runCalibration();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
#ifdef ARDUINO
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait for serial (timeout 3s)

    Serial.println(F(""));
    Serial.println(F("==========================================="));
    Serial.println(F("  HYDROPONICS CONTROL SYSTEM v1.0"));
    Serial.println(F("  Feedforward + Feedback Controller"));
    Serial.println(F("==========================================="));
    Serial.println(F(""));

    initializeSystem();
#endif
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
#ifdef ARDUINO
    uint32_t now = millis();

    // Handle serial commands
    handleSerialCommands();

    // Calibration mode
    if (calibrationMode) {
        runCalibration();
        return;
    }

    // Sensor reading (uses config timing)
    if (now - lastSensorRead >= gConfig.sensorReadMs) {
        lastSensorRead = now;
        readSensors();
    }

    // Control update (uses config timing)
    if (now - lastControlUpdate >= gConfig.controlUpdateMs) {
        float dt = (now - lastControlUpdate) / 1000.0f;
        lastControlUpdate = now;
        runController();

        // Update disturbance estimates for adaptive feedforward
        controller.updateDisturbanceEstimates(previousReadings, currentReadings, dt);
        previousReadings = currentReadings;
    }

    // Display update (uses config timing)
    if (now - lastDisplayUpdate >= gConfig.displayUpdateMs) {
        lastDisplayUpdate = now;
        updateDisplay();
    }

    // Data logging (uses config timing)
    if (now - lastLogWrite >= gConfig.logIntervalMs) {
        lastLogWrite = now;
        logData();
        uptimeSeconds += gConfig.logIntervalMs / 1000;
    }
#endif
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializeSystem() {
#ifdef ARDUINO
    Serial.println(F("Initializing..."));

    // Initialize I2C
    Wire.begin();

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HYDROPONICS CTRL"));
    lcd.setCursor(0, 1);
    lcd.print(F("Initializing..."));

    // Initialize SD card FIRST (needed to read config)
    if (SD.begin(PIN_SD_CS)) {
        sdInitialized = true;
        Serial.println(F("SD card initialized"));
    } else {
        Serial.println(F("SD card failed - using defaults"));
    }

    // =========================================================================
    // LOAD CONFIGURATION FROM config.yaml
    // =========================================================================
    lcd.setCursor(0, 2);
    lcd.print(F("Loading config..."));

    if (configLoader.loadFromSD(gConfig)) {
        Serial.println(F("Configuration loaded from config.yaml"));
    } else {
        Serial.println(F("Using default configuration"));
        Serial.print(F("Reason: "));
        Serial.println(gConfig.loadError);
    }

    // Print loaded configuration
    ConfigLoader::printConfig(gConfig);

    // =========================================================================
    // APPLY CONFIGURATION TO SYSTEM COMPONENTS
    // =========================================================================

    // Configure controller with loaded setpoints
    controller.setSetpoints(gConfig.phSetpoint, gConfig.tdsSetpoint, gConfig.waterLevelSetpoint);
    controller.setDeadbands(gConfig.phDeadband, gConfig.tdsDeadband, gConfig.waterLevelDeadband);
    controller.setFeedforwardGains(gConfig.ffPhGain, gConfig.ffTdsGain, gConfig.ffLevelGain);
    controller.setIntegralGains(gConfig.kiPh, gConfig.kiTds, gConfig.kiLevel);
    controller.setDoseLimits(gConfig.maxPhDoseMl, gConfig.maxNutrientDoseMl, gConfig.maxWaterMl);
    controller.setMinDoseInterval(gConfig.minDoseIntervalSec * 1000);

    // Configure system model
    SystemModel model;
    model.setDefaults();
    model.systemVolumeLiters = gConfig.systemVolumeLiters;
    model.tankAreaCm2 = gConfig.tankAreaCm2;
    model.phDownStrength = gConfig.phDownStrength;
    model.phUpStrength = gConfig.phUpStrength;
    model.nutrientATdsFactor = gConfig.nutrientATdsPerMl;
    model.nutrientBTdsFactor = gConfig.nutrientBTdsPerMl;
    controller.setSystemModel(model);

    // Set safety limits from config
    SafetyLimits limits;
    limits.phMin = gConfig.phMin;
    limits.phMax = gConfig.phMax;
    limits.tdsMin = gConfig.tdsMin;
    limits.tdsMax = gConfig.tdsMax;
    limits.tempMin = gConfig.tempMin;
    limits.tempMax = gConfig.tempMax;
    limits.levelMin = gConfig.levelMin;
    limits.levelMax = gConfig.levelMax;
    safety.setLimits(limits);

    // =========================================================================
    // INITIALIZE SENSORS
    // =========================================================================
    Serial.println(F("Initializing sensors..."));
    lcd.setCursor(0, 2);
    lcd.print(F("Sensors...      "));

    if (!sensors.begin()) {
        Serial.println(F("WARNING: Some sensors failed to initialize"));
        lcd.setCursor(0, 3);
        lcd.print(F("SENSOR ERROR"));
    } else {
        Serial.println(F("All sensors initialized"));
    }

    // Apply sensor calibration from config
    sensors.getPhSensor()->setCalibration(gConfig.phNeutralMv, gConfig.phAcidMv);
    sensors.getTdsSensor()->setCalibrationFactor(gConfig.tdsCalibrationFactor);
    sensors.getLevelSensor()->setTankHeight(gConfig.tankHeightCm);
    sensors.getLevelSensor()->setSensorOffset(gConfig.levelSensorOffsetCm);

    // Initialize actuators
    actuators.begin();
    Serial.println(F("Actuators initialized"));

    // Create/open log file with header
    if (sdInitialized && gConfig.loggingEnabled) {
        logFile = SD.open(gConfig.logFilename, FILE_WRITE);
        if (logFile) {
            if (logFile.size() == 0) {
                logFile.println(F("timestamp,ph,temp_c,tds_ppm,level_pct,ph_dose,nutrient_dose,status"));
            }
            logFile.close();
        }
    }

    // Initial sensor read
    delay(1000);
    sensors.readAll();
    currentReadings = sensors.getReadings();
    previousReadings = currentReadings;

    Serial.println(F(""));
    Serial.println(F("System ready. Commands:"));
    Serial.println(F("  s - Print status"));
    Serial.println(F("  c - Enter calibration mode"));
    Serial.println(F("  r - Reset integrators"));
    Serial.println(F("  p - Pause/resume control"));
    Serial.println(F("  e - Emergency stop"));
    Serial.println(F(""));

    // Update display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("System Ready"));
    lcd.setCursor(0, 1);
    lcd.print(F("pH:"));
    lcd.print(gConfig.phSetpoint, 1);
    lcd.print(F(" TDS:"));
    lcd.print((int)gConfig.tdsSetpoint);
    delay(2000);
#endif
}

// ============================================================================
// SENSOR READING
// ============================================================================

void readSensors() {
#ifdef ARDUINO
    if (!sensors.readAll()) {
        Serial.println(F("Warning: Sensor read error"));
    }
    currentReadings = sensors.getReadings();

    // Check safety
    SafetyState safetyState = safety.check(currentReadings);

    if (safetyState == SafetyState::EMERGENCY_STOP) {
        Serial.println(F("!!! EMERGENCY STOP !!!"));
        controller.setEmergencyStop(true);
        actuators.stopAllPumps();
        systemRunning = false;
    } else if (safetyState == SafetyState::ALARM) {
        Serial.print(F("ALARM: "));
        Serial.println(safety.getAlarmReason());
    }
#endif
}

// ============================================================================
// CONTROLLER EXECUTION
// ============================================================================

void runController() {
#ifdef ARDUINO
    if (!systemRunning) return;

    // Update pump timers (non-blocking pump control)
    actuators.update();

    float dt = gConfig.controlUpdateMs / 1000.0f;

    // Get control action from feedforward controller
    lastAction = controller.update(currentReadings, dt);

    // Execute control action
    actuators.execute(lastAction);

    // Log if dosing occurred
    if (lastAction.phDownDoseMl > 0) {
        Serial.print(F("Dosing pH-: "));
        Serial.print(lastAction.phDownDoseMl, 2);
        Serial.println(F(" mL"));
    }
    if (lastAction.phUpDoseMl > 0) {
        Serial.print(F("Dosing pH+: "));
        Serial.print(lastAction.phUpDoseMl, 2);
        Serial.println(F(" mL"));
    }
    if (lastAction.nutrientADoseMl > 0) {
        Serial.print(F("Dosing Nutrient A: "));
        Serial.print(lastAction.nutrientADoseMl, 2);
        Serial.println(F(" mL"));
    }
    if (lastAction.nutrientBDoseMl > 0) {
        Serial.print(F("Dosing Nutrient B: "));
        Serial.print(lastAction.nutrientBDoseMl, 2);
        Serial.println(F(" mL"));
    }
    if (lastAction.freshWaterMl > 0) {
        Serial.print(F("Adding water: "));
        Serial.print(lastAction.freshWaterMl, 2);
        Serial.println(F(" mL"));
    }
#endif
}

// ============================================================================
// DISPLAY UPDATE
// ============================================================================

void updateDisplay() {
#ifdef ARDUINO
    // Line 0: pH and Temperature
    lcd.setCursor(0, 0);
    lcd.print(F("pH:"));
    lcd.print(currentReadings.ph, 2);
    lcd.print(F(" T:"));
    lcd.print(currentReadings.temperatureC, 1);
    lcd.print(F("C  "));

    // Line 1: TDS/EC
    lcd.setCursor(0, 1);
    lcd.print(F("TDS:"));
    lcd.print((int)currentReadings.tdsPpm);
    lcd.print(F("ppm EC:"));
    lcd.print(currentReadings.tdsPpm / 500.0f, 2);
    lcd.print(F("   "));

    // Line 2: Water level and status
    lcd.setCursor(0, 2);
    lcd.print(F("Lvl:"));
    lcd.print((int)currentReadings.waterLevelPercent);
    lcd.print(F("% "));
    lcd.print(safety.getStateString());
    lcd.print(F("      "));

    // Line 3: Controller state
    lcd.setCursor(0, 3);
    if (controller.isInDeadband()) {
        lcd.print(F("Stable          "));
    } else {
        lcd.print(F("Err pH:"));
        lcd.print(controller.getPhError(), 2);
        lcd.print(F("       "));
    }
#endif
}

// ============================================================================
// DATA LOGGING
// ============================================================================

void logData() {
#ifdef ARDUINO
    if (!sdInitialized || !gConfig.loggingEnabled) return;

    logFile = SD.open(gConfig.logFilename, FILE_WRITE);
    if (logFile) {
        // timestamp,ph,temp_c,tds_ppm,level_pct,ph_dose,nutrient_dose,status
        logFile.print(uptimeSeconds);
        logFile.print(F(","));
        logFile.print(currentReadings.ph, 3);
        logFile.print(F(","));
        logFile.print(currentReadings.temperatureC, 2);
        logFile.print(F(","));
        logFile.print(currentReadings.tdsPpm, 1);
        logFile.print(F(","));
        logFile.print(currentReadings.waterLevelPercent, 1);
        logFile.print(F(","));
        logFile.print(lastAction.phDownDoseMl - lastAction.phUpDoseMl, 2);
        logFile.print(F(","));
        logFile.print(lastAction.nutrientADoseMl + lastAction.nutrientBDoseMl, 2);
        logFile.print(F(","));
        logFile.println(safety.getStateString());
        logFile.close();
    }
#endif
}

// ============================================================================
// SERIAL COMMAND HANDLING
// ============================================================================

void handleSerialCommands() {
#ifdef ARDUINO
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case 's':
            case 'S':
                printStatus();
                break;

            case 'c':
            case 'C':
                enterCalibrationMode();
                break;

            case 'r':
            case 'R':
                controller.resetIntegrators();
                Serial.println(F("Integrators reset"));
                break;

            case 'p':
            case 'P':
                systemRunning = !systemRunning;
                if (!systemRunning) {
                    actuators.stopAllPumps();
                    Serial.println(F("System PAUSED"));
                } else {
                    Serial.println(F("System RESUMED"));
                }
                break;

            case 'e':
            case 'E':
                controller.setEmergencyStop(true);
                actuators.stopAllPumps();
                systemRunning = false;
                Serial.println(F("!!! EMERGENCY STOP ACTIVATED !!!"));
                break;

            case 'x':
            case 'X':
                controller.setEmergencyStop(false);
                safety.reset();
                systemRunning = true;
                Serial.println(F("Emergency stop cleared"));
                break;
        }
    }
#endif
}

// ============================================================================
// STATUS PRINTING
// ============================================================================

void printStatus() {
#ifdef ARDUINO
    Serial.println(F(""));
    Serial.println(F("========== SYSTEM STATUS =========="));
    Serial.println(F(""));

    Serial.println(F("--- Sensor Readings ---"));
    Serial.print(F("  pH:          "));
    Serial.println(currentReadings.ph, 3);
    Serial.print(F("  Temperature: "));
    Serial.print(currentReadings.temperatureC, 2);
    Serial.println(F(" C"));
    Serial.print(F("  TDS:         "));
    Serial.print(currentReadings.tdsPpm, 1);
    Serial.println(F(" ppm"));
    Serial.print(F("  EC:          "));
    Serial.print(currentReadings.tdsPpm / 500.0f, 3);
    Serial.println(F(" mS/cm"));
    Serial.print(F("  Water Level: "));
    Serial.print(currentReadings.waterLevelPercent, 1);
    Serial.println(F(" %"));

    Serial.println(F(""));
    Serial.println(F("--- Controller State ---"));
    Serial.print(F("  pH Error:    "));
    Serial.println(controller.getPhError(), 3);
    Serial.print(F("  TDS Error:   "));
    Serial.println(controller.getTdsError(), 1);
    Serial.print(F("  Level Error: "));
    Serial.println(controller.getLevelError(), 1);
    Serial.print(F("  In Deadband: "));
    Serial.println(controller.isInDeadband() ? "Yes" : "No");

    Serial.println(F(""));
    Serial.println(F("--- Integrator States ---"));
    Serial.print(F("  pH Integral:    "));
    Serial.println(controller.getPhIntegral(), 4);
    Serial.print(F("  TDS Integral:   "));
    Serial.println(controller.getTdsIntegral(), 4);
    Serial.print(F("  Level Integral: "));
    Serial.println(controller.getLevelIntegral(), 4);

    Serial.println(F(""));
    Serial.println(F("--- Disturbance Estimates ---"));
    Serial.print(F("  pH Drift:    "));
    Serial.print(controller.getEstimatedPhDrift(), 4);
    Serial.println(F(" pH/hr"));
    Serial.print(F("  TDS Uptake:  "));
    Serial.print(controller.getEstimatedTdsUptake(), 2);
    Serial.println(F(" ppm/hr"));

    Serial.println(F(""));
    Serial.print(F("Safety State: "));
    Serial.println(safety.getStateString());
    Serial.print(F("System Running: "));
    Serial.println(systemRunning ? "Yes" : "No");
    Serial.print(F("Uptime: "));
    Serial.print(uptimeSeconds / 3600);
    Serial.print(F("h "));
    Serial.print((uptimeSeconds % 3600) / 60);
    Serial.println(F("m"));

    Serial.println(F(""));
    Serial.println(F("==================================="));
    Serial.println(F(""));
#endif
}

// ============================================================================
// CALIBRATION MODE
// ============================================================================

void enterCalibrationMode() {
#ifdef ARDUINO
    calibrationMode = true;
    systemRunning = false;
    actuators.stopAllPumps();

    Serial.println(F(""));
    Serial.println(F("=== CALIBRATION MODE ==="));
    Serial.println(F(""));
    Serial.println(F("Commands:"));
    Serial.println(F("  1 - Calibrate pH (pH 7.0 buffer)"));
    Serial.println(F("  2 - Calibrate pH (pH 4.0 buffer)"));
    Serial.println(F("  3 - Calibrate TDS (known solution)"));
    Serial.println(F("  4 - Calibrate water level (empty)"));
    Serial.println(F("  5 - Calibrate water level (full)"));
    Serial.println(F("  6 - Test pH down pump (1 sec)"));
    Serial.println(F("  7 - Test pH up pump (1 sec)"));
    Serial.println(F("  8 - Test nutrient pump (1 sec)"));
    Serial.println(F("  q - Exit calibration mode"));
    Serial.println(F(""));
#endif
}

void runCalibration() {
#ifdef ARDUINO
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1': {
                // Read current voltage for pH 7.0 calibration
                sensors.readAll();
                float voltage = sensors.getPhSensor()->getRawVoltage();
                sensors.getPhSensor()->calibratePoint(7.0f, voltage);
                Serial.print(F("pH 7.0 calibrated at "));
                Serial.print(voltage, 2);
                Serial.println(F(" mV"));
                break;
            }

            case '2': {
                sensors.readAll();
                float voltage = sensors.getPhSensor()->getRawVoltage();
                sensors.getPhSensor()->calibratePoint(4.0f, voltage);
                Serial.print(F("pH 4.0 calibrated at "));
                Serial.print(voltage, 2);
                Serial.println(F(" mV"));
                break;
            }

            case '3': {
                Serial.println(F("Enter known TDS value (ppm):"));
                while (!Serial.available());
                float knownTds = Serial.parseFloat();
                sensors.readAll();
                float measuredTds = sensors.getTdsSensor()->getValue();

                // Guard against division by zero
                if (measuredTds < 0.01f) {
                    Serial.println(F("ERROR: Measured TDS too low or sensor error"));
                    Serial.println(F("Check sensor connection and try again"));
                    break;
                }

                float factor = knownTds / measuredTds;
                sensors.getTdsSensor()->setCalibrationFactor(factor);
                Serial.print(F("TDS calibration factor set to "));
                Serial.println(factor, 4);
                break;
            }

            case '4': {
                sensors.readAll();
                float dist = sensors.getLevelSensor()->getDistanceCm();
                Serial.print(F("Empty tank distance: "));
                Serial.print(dist, 1);
                Serial.println(F(" cm"));
                // Would store this as max distance (empty = max ultrasonic distance)
                break;
            }

            case '5': {
                sensors.readAll();
                float dist = sensors.getLevelSensor()->getDistanceCm();
                Serial.print(F("Full tank distance: "));
                Serial.print(dist, 1);
                Serial.println(F(" cm"));
                break;
            }

            case '6':
                Serial.println(F("Testing pH DOWN pump..."));
                actuators.runPump(PIN_PUMP_PH_DOWN, 1.0f);
                Serial.println(F("Done"));
                break;

            case '7':
                Serial.println(F("Testing pH UP pump..."));
                actuators.runPump(PIN_PUMP_PH_UP, 1.0f);
                Serial.println(F("Done"));
                break;

            case '8':
                Serial.println(F("Testing Nutrient A pump..."));
                actuators.runPump(PIN_PUMP_NUTRIENT_A, 1.0f);
                Serial.println(F("Done"));
                break;

            case 'q':
            case 'Q':
                calibrationMode = false;
                systemRunning = true;
                Serial.println(F("Exiting calibration mode"));
                break;
        }
    }
#endif
}

// ============================================================================
// NON-ARDUINO BUILD (for testing)
// ============================================================================

#ifndef ARDUINO
int main() {
    // Test harness for non-Arduino builds
    printf("Hydroponics Control System - Test Build\n");
    printf("Run unit tests with: ./test_runner\n");
    return 0;
}
#endif
