/**
 * @file calibration.h
 * @brief Interactive sensor calibration wizard for hydroponics controller
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sensors.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <EEPROM.h>
#endif

// ============================================================================
// CALIBRATION STATE MACHINE
// ============================================================================

enum class CalibrationState : uint8_t {
    IDLE = 0,
    MAIN_MENU,

    // pH Calibration States
    PH_MENU,
    PH_NEUTRAL_WAIT,
    PH_NEUTRAL_SAMPLE,
    PH_ACID_WAIT,
    PH_ACID_SAMPLE,
    PH_COMPLETE,

    // TDS Calibration States
    TDS_MENU,
    TDS_SOLUTION_WAIT,
    TDS_SOLUTION_SAMPLE,
    TDS_COMPLETE,

    // Water Level Calibration States
    LEVEL_MENU,
    LEVEL_EMPTY_WAIT,
    LEVEL_EMPTY_SAMPLE,
    LEVEL_FULL_WAIT,
    LEVEL_FULL_SAMPLE,
    LEVEL_COMPLETE,

    // Pump Test States
    PUMP_MENU,
    PUMP_TESTING,

    // Exit
    SAVE_CONFIRM,
    EXIT
};

// ============================================================================
// CALIBRATION DATA STORAGE
// ============================================================================

// EEPROM addresses for calibration data
#define EEPROM_MAGIC_ADDR       0
#define EEPROM_MAGIC_VALUE      0xCA1B  // "CALIB" marker
#define EEPROM_PH_ADDR          4
#define EEPROM_TDS_ADDR         20
#define EEPROM_LEVEL_ADDR       32

struct CalibrationData {
    // pH sensor
    float phNeutralVoltage;     // Voltage at pH 7.0
    float phAcidVoltage;        // Voltage at pH 4.0

    // TDS sensor
    float tdsCalibrationFactor;
    float tdsReferenceValue;    // Known TDS value used for calibration

    // Water level sensor
    float levelEmptyDistance;   // Distance when tank is empty
    float levelFullDistance;    // Distance when tank is full
    float levelTankHeight;      // Tank height in cm

    // Validity flags
    bool phCalibrated;
    bool tdsCalibrated;
    bool levelCalibrated;

    // Checksum for validation
    uint16_t checksum;
};

// ============================================================================
// CALIBRATION WIZARD CLASS
// ============================================================================

class CalibrationWizard {
public:
    CalibrationWizard(SensorManager* sensors);

    // Start/stop wizard
    void begin();
    void end();
    bool isActive() const { return state_ != CalibrationState::IDLE; }

    // Process input and update state
    void processInput(char input);
    void update();

    // Get current prompt/message for display
    const char* getCurrentPrompt() const;
    const char* getCurrentInstructions() const;

    // Get calibration data
    CalibrationData getCalibrationData() const { return data_; }

    // Save/load calibration to EEPROM
    bool saveToEeprom();
    bool loadFromEeprom();

    // Apply calibration to sensors
    void applyCalibration();

    // Get current state for display
    CalibrationState getState() const { return state_; }

    // Get current sampling progress (0-100)
    uint8_t getSamplingProgress() const;

    // Get current reading for display
    float getCurrentReading() const { return currentReading_; }

private:
    SensorManager* sensors_;
    CalibrationState state_;
    CalibrationData data_;

    // Sampling state
    uint32_t sampleStartTime_;
    uint32_t lastSampleTime_;
    float sampleSum_;
    uint16_t sampleCount_;
    static const uint16_t SAMPLES_REQUIRED = 30;
    static const uint32_t SAMPLE_INTERVAL_MS = 100;
    static const uint32_t STABILIZATION_TIME_MS = 3000;

    // Current reading for display
    float currentReading_;

    // Pump test state
    uint8_t currentPump_;
    uint32_t pumpTestStartTime_;
    static const uint32_t PUMP_TEST_DURATION_MS = 2000;

    // State handlers
    void handleMainMenu(char input);
    void handlePhMenu(char input);
    void handlePhNeutralWait(char input);
    void handlePhNeutralSample();
    void handlePhAcidWait(char input);
    void handlePhAcidSample();

    void handleTdsMenu(char input);
    void handleTdsSolutionWait(char input);
    void handleTdsSolutionSample();

    void handleLevelMenu(char input);
    void handleLevelEmptyWait(char input);
    void handleLevelEmptySample();
    void handleLevelFullWait(char input);
    void handleLevelFullSample();

    void handlePumpMenu(char input);
    void handlePumpTesting();

    void handleSaveConfirm(char input);

    // Helper functions
    void startSampling();
    bool continueSampling(float value);
    float getAverageSample();
    void resetSampling();

    uint16_t calculateChecksum(const CalibrationData& data);

    // Pump control
    void startPumpTest(uint8_t pumpIndex);
    void stopAllPumps();
};

// ============================================================================
// SERIAL UI HELPERS
// ============================================================================

#ifdef ARDUINO
class CalibrationUI {
public:
    static void printHeader();
    static void printMainMenu();
    static void printPhMenu();
    static void printTdsMenu();
    static void printLevelMenu();
    static void printPumpMenu();
    static void printProgress(uint8_t percent);
    static void printReading(const char* label, float value, const char* unit);
    static void printSuccess(const char* message);
    static void printError(const char* message);
    static void printInstruction(const char* message);
    static void clearScreen();
};
#endif

#endif // CALIBRATION_H
