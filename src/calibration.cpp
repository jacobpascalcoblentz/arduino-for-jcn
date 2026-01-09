/**
 * @file calibration.cpp
 * @brief Implementation of interactive calibration wizard
 */

#include "calibration.h"
#include "config.h"

#ifndef ARDUINO
#include "mock_arduino.h"
#endif

// ============================================================================
// CALIBRATION WIZARD IMPLEMENTATION
// ============================================================================

CalibrationWizard::CalibrationWizard(SensorManager* sensors)
    : sensors_(sensors)
    , state_(CalibrationState::IDLE)
    , sampleStartTime_(0)
    , lastSampleTime_(0)
    , sampleSum_(0.0f)
    , sampleCount_(0)
    , currentReading_(0.0f)
    , currentPump_(0)
    , pumpTestStartTime_(0) {

    // Initialize calibration data with defaults
    data_.phNeutralVoltage = PH_CALIBRATION_VOLTAGE_NEUTRAL;
    data_.phAcidVoltage = PH_CALIBRATION_VOLTAGE_ACID;
    data_.tdsCalibrationFactor = TDS_CALIBRATION_FACTOR;
    data_.tdsReferenceValue = 1000.0f;
    data_.levelEmptyDistance = WATER_LEVEL_TANK_HEIGHT_CM;
    data_.levelFullDistance = WATER_LEVEL_SENSOR_OFFSET_CM;
    data_.levelTankHeight = WATER_LEVEL_TANK_HEIGHT_CM;
    data_.phCalibrated = false;
    data_.tdsCalibrated = false;
    data_.levelCalibrated = false;
    data_.checksum = 0;
}

void CalibrationWizard::begin() {
    state_ = CalibrationState::MAIN_MENU;
    resetSampling();

#ifdef ARDUINO
    CalibrationUI::clearScreen();
    CalibrationUI::printHeader();
    CalibrationUI::printMainMenu();
#endif
}

void CalibrationWizard::end() {
    stopAllPumps();
    state_ = CalibrationState::IDLE;
}

void CalibrationWizard::processInput(char input) {
    if (input == 'q' || input == 'Q') {
        if (state_ != CalibrationState::IDLE) {
            state_ = CalibrationState::SAVE_CONFIRM;
#ifdef ARDUINO
            CalibrationUI::printInstruction("Save calibration before exit? (y/n)");
#endif
            return;
        }
    }

    switch (state_) {
        case CalibrationState::MAIN_MENU:
            handleMainMenu(input);
            break;
        case CalibrationState::PH_MENU:
            handlePhMenu(input);
            break;
        case CalibrationState::PH_NEUTRAL_WAIT:
            handlePhNeutralWait(input);
            break;
        case CalibrationState::PH_ACID_WAIT:
            handlePhAcidWait(input);
            break;
        case CalibrationState::TDS_MENU:
            handleTdsMenu(input);
            break;
        case CalibrationState::TDS_SOLUTION_WAIT:
            handleTdsSolutionWait(input);
            break;
        case CalibrationState::LEVEL_MENU:
            handleLevelMenu(input);
            break;
        case CalibrationState::LEVEL_EMPTY_WAIT:
            handleLevelEmptyWait(input);
            break;
        case CalibrationState::LEVEL_FULL_WAIT:
            handleLevelFullWait(input);
            break;
        case CalibrationState::PUMP_MENU:
            handlePumpMenu(input);
            break;
        case CalibrationState::SAVE_CONFIRM:
            handleSaveConfirm(input);
            break;
        default:
            break;
    }
}

void CalibrationWizard::update() {
    // Update current sensor reading for display
    if (sensors_) {
        sensors_->readAll();
    }

    switch (state_) {
        case CalibrationState::PH_NEUTRAL_SAMPLE:
            handlePhNeutralSample();
            break;
        case CalibrationState::PH_ACID_SAMPLE:
            handlePhAcidSample();
            break;
        case CalibrationState::TDS_SOLUTION_SAMPLE:
            handleTdsSolutionSample();
            break;
        case CalibrationState::LEVEL_EMPTY_SAMPLE:
            handleLevelEmptySample();
            break;
        case CalibrationState::LEVEL_FULL_SAMPLE:
            handleLevelFullSample();
            break;
        case CalibrationState::PUMP_TESTING:
            handlePumpTesting();
            break;
        default:
            break;
    }
}

// ============================================================================
// MAIN MENU HANDLER
// ============================================================================

void CalibrationWizard::handleMainMenu(char input) {
    switch (input) {
        case '1':
            state_ = CalibrationState::PH_MENU;
#ifdef ARDUINO
            CalibrationUI::printPhMenu();
#endif
            break;
        case '2':
            state_ = CalibrationState::TDS_MENU;
#ifdef ARDUINO
            CalibrationUI::printTdsMenu();
#endif
            break;
        case '3':
            state_ = CalibrationState::LEVEL_MENU;
#ifdef ARDUINO
            CalibrationUI::printLevelMenu();
#endif
            break;
        case '4':
            state_ = CalibrationState::PUMP_MENU;
#ifdef ARDUINO
            CalibrationUI::printPumpMenu();
#endif
            break;
        case '5':
            applyCalibration();
            saveToEeprom();
#ifdef ARDUINO
            CalibrationUI::printSuccess("Calibration saved!");
            CalibrationUI::printMainMenu();
#endif
            break;
        case '6':
            loadFromEeprom();
            applyCalibration();
#ifdef ARDUINO
            CalibrationUI::printSuccess("Calibration loaded!");
            CalibrationUI::printMainMenu();
#endif
            break;
        default:
            break;
    }
}

// ============================================================================
// pH CALIBRATION HANDLERS
// ============================================================================

void CalibrationWizard::handlePhMenu(char input) {
    switch (input) {
        case '1':
            state_ = CalibrationState::PH_NEUTRAL_WAIT;
#ifdef ARDUINO
            CalibrationUI::printInstruction(
                "Place pH probe in pH 7.0 buffer solution.\n"
                "Wait for reading to stabilize, then press ENTER.");
#endif
            break;
        case '2':
            state_ = CalibrationState::PH_ACID_WAIT;
#ifdef ARDUINO
            CalibrationUI::printInstruction(
                "Place pH probe in pH 4.0 buffer solution.\n"
                "Wait for reading to stabilize, then press ENTER.");
#endif
            break;
        case 'b':
        case 'B':
            state_ = CalibrationState::MAIN_MENU;
#ifdef ARDUINO
            CalibrationUI::printMainMenu();
#endif
            break;
        default:
            break;
    }
}

void CalibrationWizard::handlePhNeutralWait(char input) {
    if (sensors_ && sensors_->getPhSensor()) {
        currentReading_ = sensors_->getPhSensor()->getRawVoltage();
#ifdef ARDUINO
        CalibrationUI::printReading("Voltage", currentReading_, "mV");
#endif
    }

    if (input == '\n' || input == '\r' || input == 's' || input == 'S') {
        startSampling();
        state_ = CalibrationState::PH_NEUTRAL_SAMPLE;
#ifdef ARDUINO
        CalibrationUI::printInstruction("Sampling pH 7.0 point...");
#endif
    }
}

void CalibrationWizard::handlePhNeutralSample() {
    if (sensors_ && sensors_->getPhSensor()) {
        float voltage = sensors_->getPhSensor()->getRawVoltage();
        currentReading_ = voltage;

        if (continueSampling(voltage)) {
            data_.phNeutralVoltage = getAverageSample();
#ifdef ARDUINO
            Serial.print("pH 7.0 voltage recorded: ");
            Serial.print(data_.phNeutralVoltage);
            Serial.println(" mV");
#endif
            state_ = CalibrationState::PH_MENU;
#ifdef ARDUINO
            CalibrationUI::printSuccess("pH 7.0 calibration point saved!");
            CalibrationUI::printPhMenu();
#endif
        } else {
#ifdef ARDUINO
            CalibrationUI::printProgress(getSamplingProgress());
#endif
        }
    }
}

void CalibrationWizard::handlePhAcidWait(char input) {
    if (sensors_ && sensors_->getPhSensor()) {
        currentReading_ = sensors_->getPhSensor()->getRawVoltage();
#ifdef ARDUINO
        CalibrationUI::printReading("Voltage", currentReading_, "mV");
#endif
    }

    if (input == '\n' || input == '\r' || input == 's' || input == 'S') {
        startSampling();
        state_ = CalibrationState::PH_ACID_SAMPLE;
#ifdef ARDUINO
        CalibrationUI::printInstruction("Sampling pH 4.0 point...");
#endif
    }
}

void CalibrationWizard::handlePhAcidSample() {
    if (sensors_ && sensors_->getPhSensor()) {
        float voltage = sensors_->getPhSensor()->getRawVoltage();
        currentReading_ = voltage;

        if (continueSampling(voltage)) {
            data_.phAcidVoltage = getAverageSample();
            data_.phCalibrated = true;
#ifdef ARDUINO
            Serial.print("pH 4.0 voltage recorded: ");
            Serial.print(data_.phAcidVoltage);
            Serial.println(" mV");
#endif
            state_ = CalibrationState::PH_COMPLETE;
            applyCalibration();
#ifdef ARDUINO
            CalibrationUI::printSuccess("pH calibration complete!");
            Serial.println("Calculated slope: ");
            Serial.print((7.0f - 4.0f) / (data_.phNeutralVoltage - data_.phAcidVoltage) * 1000.0f);
            Serial.println(" pH/V");
#endif
            state_ = CalibrationState::PH_MENU;
#ifdef ARDUINO
            CalibrationUI::printPhMenu();
#endif
        } else {
#ifdef ARDUINO
            CalibrationUI::printProgress(getSamplingProgress());
#endif
        }
    }
}

// ============================================================================
// TDS CALIBRATION HANDLERS
// ============================================================================

void CalibrationWizard::handleTdsMenu(char input) {
    switch (input) {
        case '1':
            state_ = CalibrationState::TDS_SOLUTION_WAIT;
            data_.tdsReferenceValue = 1000.0f;
#ifdef ARDUINO
            CalibrationUI::printInstruction(
                "Place TDS probe in 1000 ppm calibration solution.\n"
                "Wait for reading to stabilize, then press ENTER.\n"
                "(Or press 'v' to enter a different TDS value)");
#endif
            break;
        case 'b':
        case 'B':
            state_ = CalibrationState::MAIN_MENU;
#ifdef ARDUINO
            CalibrationUI::printMainMenu();
#endif
            break;
        default:
            break;
    }
}

void CalibrationWizard::handleTdsSolutionWait(char input) {
    if (sensors_ && sensors_->getTdsSensor()) {
        currentReading_ = sensors_->getTdsSensor()->getValue();
#ifdef ARDUINO
        CalibrationUI::printReading("TDS", currentReading_, "ppm");
#endif
    }

    if (input == '\n' || input == '\r' || input == 's' || input == 'S') {
        startSampling();
        state_ = CalibrationState::TDS_SOLUTION_SAMPLE;
#ifdef ARDUINO
        CalibrationUI::printInstruction("Sampling TDS...");
#endif
    }
}

void CalibrationWizard::handleTdsSolutionSample() {
    if (sensors_ && sensors_->getTdsSensor()) {
        float tds = sensors_->getTdsSensor()->getValue();
        currentReading_ = tds;

        if (continueSampling(tds)) {
            float measuredTds = getAverageSample();
            if (measuredTds > 0) {
                data_.tdsCalibrationFactor = data_.tdsReferenceValue / measuredTds * TDS_CALIBRATION_FACTOR;
                data_.tdsCalibrated = true;
#ifdef ARDUINO
                Serial.print("TDS calibration factor: ");
                Serial.println(data_.tdsCalibrationFactor, 4);
#endif
            }
            state_ = CalibrationState::TDS_COMPLETE;
            applyCalibration();
#ifdef ARDUINO
            CalibrationUI::printSuccess("TDS calibration complete!");
#endif
            state_ = CalibrationState::TDS_MENU;
#ifdef ARDUINO
            CalibrationUI::printTdsMenu();
#endif
        } else {
#ifdef ARDUINO
            CalibrationUI::printProgress(getSamplingProgress());
#endif
        }
    }
}

// ============================================================================
// WATER LEVEL CALIBRATION HANDLERS
// ============================================================================

void CalibrationWizard::handleLevelMenu(char input) {
    switch (input) {
        case '1':
            state_ = CalibrationState::LEVEL_EMPTY_WAIT;
#ifdef ARDUINO
            CalibrationUI::printInstruction(
                "Empty the tank or point sensor at maximum distance.\n"
                "Press ENTER when ready.");
#endif
            break;
        case '2':
            state_ = CalibrationState::LEVEL_FULL_WAIT;
#ifdef ARDUINO
            CalibrationUI::printInstruction(
                "Fill tank to maximum level.\n"
                "Press ENTER when ready.");
#endif
            break;
        case 'b':
        case 'B':
            state_ = CalibrationState::MAIN_MENU;
#ifdef ARDUINO
            CalibrationUI::printMainMenu();
#endif
            break;
        default:
            break;
    }
}

void CalibrationWizard::handleLevelEmptyWait(char input) {
    if (sensors_ && sensors_->getLevelSensor()) {
        currentReading_ = sensors_->getLevelSensor()->getDistanceCm();
#ifdef ARDUINO
        CalibrationUI::printReading("Distance", currentReading_, "cm");
#endif
    }

    if (input == '\n' || input == '\r' || input == 's' || input == 'S') {
        startSampling();
        state_ = CalibrationState::LEVEL_EMPTY_SAMPLE;
#ifdef ARDUINO
        CalibrationUI::printInstruction("Sampling empty distance...");
#endif
    }
}

void CalibrationWizard::handleLevelEmptySample() {
    if (sensors_ && sensors_->getLevelSensor()) {
        float distance = sensors_->getLevelSensor()->getDistanceCm();
        currentReading_ = distance;

        if (continueSampling(distance)) {
            data_.levelEmptyDistance = getAverageSample();
#ifdef ARDUINO
            Serial.print("Empty distance recorded: ");
            Serial.print(data_.levelEmptyDistance);
            Serial.println(" cm");
#endif
            state_ = CalibrationState::LEVEL_MENU;
#ifdef ARDUINO
            CalibrationUI::printSuccess("Empty point saved!");
            CalibrationUI::printLevelMenu();
#endif
        } else {
#ifdef ARDUINO
            CalibrationUI::printProgress(getSamplingProgress());
#endif
        }
    }
}

void CalibrationWizard::handleLevelFullWait(char input) {
    if (sensors_ && sensors_->getLevelSensor()) {
        currentReading_ = sensors_->getLevelSensor()->getDistanceCm();
#ifdef ARDUINO
        CalibrationUI::printReading("Distance", currentReading_, "cm");
#endif
    }

    if (input == '\n' || input == '\r' || input == 's' || input == 'S') {
        startSampling();
        state_ = CalibrationState::LEVEL_FULL_SAMPLE;
#ifdef ARDUINO
        CalibrationUI::printInstruction("Sampling full distance...");
#endif
    }
}

void CalibrationWizard::handleLevelFullSample() {
    if (sensors_ && sensors_->getLevelSensor()) {
        float distance = sensors_->getLevelSensor()->getDistanceCm();
        currentReading_ = distance;

        if (continueSampling(distance)) {
            data_.levelFullDistance = getAverageSample();
            data_.levelTankHeight = data_.levelEmptyDistance - data_.levelFullDistance;
            data_.levelCalibrated = true;
#ifdef ARDUINO
            Serial.print("Full distance recorded: ");
            Serial.print(data_.levelFullDistance);
            Serial.println(" cm");
            Serial.print("Tank height: ");
            Serial.print(data_.levelTankHeight);
            Serial.println(" cm");
#endif
            state_ = CalibrationState::LEVEL_COMPLETE;
            applyCalibration();
#ifdef ARDUINO
            CalibrationUI::printSuccess("Water level calibration complete!");
#endif
            state_ = CalibrationState::LEVEL_MENU;
#ifdef ARDUINO
            CalibrationUI::printLevelMenu();
#endif
        } else {
#ifdef ARDUINO
            CalibrationUI::printProgress(getSamplingProgress());
#endif
        }
    }
}

// ============================================================================
// PUMP TEST HANDLERS
// ============================================================================

void CalibrationWizard::handlePumpMenu(char input) {
    switch (input) {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
            currentPump_ = input - '0';
            startPumpTest(currentPump_ - 1);
            state_ = CalibrationState::PUMP_TESTING;
#ifdef ARDUINO
            Serial.print("Testing pump ");
            Serial.println(currentPump_);
#endif
            break;
        case 'b':
        case 'B':
            state_ = CalibrationState::MAIN_MENU;
#ifdef ARDUINO
            CalibrationUI::printMainMenu();
#endif
            break;
        default:
            break;
    }
}

void CalibrationWizard::handlePumpTesting() {
    uint32_t elapsed = millis() - pumpTestStartTime_;
    if (elapsed >= PUMP_TEST_DURATION_MS) {
        stopAllPumps();
        state_ = CalibrationState::PUMP_MENU;
#ifdef ARDUINO
        CalibrationUI::printSuccess("Pump test complete!");
        CalibrationUI::printPumpMenu();
#endif
    }
}

void CalibrationWizard::startPumpTest(uint8_t pumpIndex) {
    stopAllPumps();
    pumpTestStartTime_ = millis();

    const uint8_t pumpPins[] = {
        PIN_PUMP_PH_DOWN,
        PIN_PUMP_PH_UP,
        PIN_PUMP_NUTRIENT_A,
        PIN_PUMP_NUTRIENT_B,
        PIN_VALVE_FRESH_WATER
    };

    if (pumpIndex < 5) {
#ifdef ARDUINO
        digitalWrite(pumpPins[pumpIndex], HIGH);
#endif
    }
}

void CalibrationWizard::stopAllPumps() {
#ifdef ARDUINO
    digitalWrite(PIN_PUMP_PH_DOWN, LOW);
    digitalWrite(PIN_PUMP_PH_UP, LOW);
    digitalWrite(PIN_PUMP_NUTRIENT_A, LOW);
    digitalWrite(PIN_PUMP_NUTRIENT_B, LOW);
    digitalWrite(PIN_VALVE_FRESH_WATER, LOW);
#endif
}

// ============================================================================
// SAVE/CONFIRM HANDLER
// ============================================================================

void CalibrationWizard::handleSaveConfirm(char input) {
    if (input == 'y' || input == 'Y') {
        saveToEeprom();
#ifdef ARDUINO
        CalibrationUI::printSuccess("Calibration saved!");
#endif
    }
    end();
}

// ============================================================================
// SAMPLING HELPERS
// ============================================================================

void CalibrationWizard::startSampling() {
    sampleStartTime_ = millis();
    lastSampleTime_ = 0;
    sampleSum_ = 0.0f;
    sampleCount_ = 0;
}

bool CalibrationWizard::continueSampling(float value) {
    uint32_t now = millis();

    // Wait for stabilization
    if (now - sampleStartTime_ < STABILIZATION_TIME_MS) {
        return false;
    }

    // Take samples at interval
    if (now - lastSampleTime_ >= SAMPLE_INTERVAL_MS) {
        sampleSum_ += value;
        sampleCount_++;
        lastSampleTime_ = now;
    }

    return sampleCount_ >= SAMPLES_REQUIRED;
}

float CalibrationWizard::getAverageSample() {
    if (sampleCount_ == 0) return 0.0f;
    return sampleSum_ / static_cast<float>(sampleCount_);
}

void CalibrationWizard::resetSampling() {
    sampleStartTime_ = 0;
    lastSampleTime_ = 0;
    sampleSum_ = 0.0f;
    sampleCount_ = 0;
}

uint8_t CalibrationWizard::getSamplingProgress() const {
    if (sampleCount_ >= SAMPLES_REQUIRED) return 100;
    return static_cast<uint8_t>((sampleCount_ * 100) / SAMPLES_REQUIRED);
}

// ============================================================================
// EEPROM STORAGE
// ============================================================================

uint16_t CalibrationWizard::calculateChecksum(const CalibrationData& data) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);
    uint16_t sum = 0;
    // Calculate checksum over all bytes except the checksum field itself
    size_t len = sizeof(CalibrationData) - sizeof(uint16_t);
    for (size_t i = 0; i < len; i++) {
        sum += ptr[i];
    }
    return sum ^ 0xA5A5;
}

bool CalibrationWizard::saveToEeprom() {
#ifdef ARDUINO
    data_.checksum = calculateChecksum(data_);

    // Write magic number
    EEPROM.put(EEPROM_MAGIC_ADDR, static_cast<uint16_t>(EEPROM_MAGIC_VALUE));

    // Write calibration data
    EEPROM.put(EEPROM_PH_ADDR, data_);

    return true;
#else
    return false;
#endif
}

bool CalibrationWizard::loadFromEeprom() {
#ifdef ARDUINO
    // Check magic number
    uint16_t magic;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    if (magic != EEPROM_MAGIC_VALUE) {
        return false;
    }

    // Read calibration data
    CalibrationData loadedData;
    EEPROM.get(EEPROM_PH_ADDR, loadedData);

    // Verify checksum
    uint16_t expectedChecksum = calculateChecksum(loadedData);
    if (loadedData.checksum != expectedChecksum) {
        return false;
    }

    data_ = loadedData;
    return true;
#else
    return false;
#endif
}

void CalibrationWizard::applyCalibration() {
    if (sensors_) {
        if (data_.phCalibrated && sensors_->getPhSensor()) {
            sensors_->getPhSensor()->setCalibration(
                data_.phNeutralVoltage,
                data_.phAcidVoltage
            );
        }

        if (data_.tdsCalibrated && sensors_->getTdsSensor()) {
            sensors_->getTdsSensor()->setCalibrationFactor(data_.tdsCalibrationFactor);
        }

        if (data_.levelCalibrated && sensors_->getLevelSensor()) {
            sensors_->getLevelSensor()->setTankHeight(data_.levelTankHeight);
            sensors_->getLevelSensor()->setSensorOffset(data_.levelFullDistance);
        }
    }
}

// ============================================================================
// UI PROMPTS
// ============================================================================

const char* CalibrationWizard::getCurrentPrompt() const {
    switch (state_) {
        case CalibrationState::MAIN_MENU:
            return "CALIBRATION MENU";
        case CalibrationState::PH_MENU:
            return "pH CALIBRATION";
        case CalibrationState::TDS_MENU:
            return "TDS CALIBRATION";
        case CalibrationState::LEVEL_MENU:
            return "WATER LEVEL CALIBRATION";
        case CalibrationState::PUMP_MENU:
            return "PUMP TEST";
        default:
            return "";
    }
}

const char* CalibrationWizard::getCurrentInstructions() const {
    switch (state_) {
        case CalibrationState::PH_NEUTRAL_WAIT:
            return "Place probe in pH 7.0 solution, press ENTER";
        case CalibrationState::PH_ACID_WAIT:
            return "Place probe in pH 4.0 solution, press ENTER";
        case CalibrationState::TDS_SOLUTION_WAIT:
            return "Place probe in calibration solution, press ENTER";
        case CalibrationState::LEVEL_EMPTY_WAIT:
            return "Empty tank, press ENTER";
        case CalibrationState::LEVEL_FULL_WAIT:
            return "Fill tank, press ENTER";
        default:
            return "";
    }
}

// ============================================================================
// SERIAL UI IMPLEMENTATION
// ============================================================================

#ifdef ARDUINO
void CalibrationUI::printHeader() {
    Serial.println(F("\n========================================"));
    Serial.println(F("   HYDROPONICS CALIBRATION WIZARD"));
    Serial.println(F("========================================"));
    Serial.println(F("Press 'q' at any time to exit\n"));
}

void CalibrationUI::printMainMenu() {
    Serial.println(F("\n--- Main Menu ---"));
    Serial.println(F("1. Calibrate pH Sensor"));
    Serial.println(F("2. Calibrate TDS Sensor"));
    Serial.println(F("3. Calibrate Water Level Sensor"));
    Serial.println(F("4. Test Pumps"));
    Serial.println(F("5. Save Calibration to EEPROM"));
    Serial.println(F("6. Load Calibration from EEPROM"));
    Serial.println(F("\nEnter choice: "));
}

void CalibrationUI::printPhMenu() {
    Serial.println(F("\n--- pH Calibration ---"));
    Serial.println(F("1. Calibrate pH 7.0 (neutral point)"));
    Serial.println(F("2. Calibrate pH 4.0 (acid point)"));
    Serial.println(F("b. Back to main menu"));
    Serial.println(F("\nEnter choice: "));
}

void CalibrationUI::printTdsMenu() {
    Serial.println(F("\n--- TDS Calibration ---"));
    Serial.println(F("1. Calibrate with 1000 ppm solution"));
    Serial.println(F("b. Back to main menu"));
    Serial.println(F("\nEnter choice: "));
}

void CalibrationUI::printLevelMenu() {
    Serial.println(F("\n--- Water Level Calibration ---"));
    Serial.println(F("1. Set empty point (tank empty)"));
    Serial.println(F("2. Set full point (tank full)"));
    Serial.println(F("b. Back to main menu"));
    Serial.println(F("\nEnter choice: "));
}

void CalibrationUI::printPumpMenu() {
    Serial.println(F("\n--- Pump Test ---"));
    Serial.println(F("1. Test pH Down pump"));
    Serial.println(F("2. Test pH Up pump"));
    Serial.println(F("3. Test Nutrient A pump"));
    Serial.println(F("4. Test Nutrient B pump"));
    Serial.println(F("5. Test Fresh Water valve"));
    Serial.println(F("b. Back to main menu"));
    Serial.println(F("\nEnter choice: "));
}

void CalibrationUI::printProgress(uint8_t percent) {
    Serial.print(F("\rSampling: ["));
    uint8_t bars = percent / 5;
    for (uint8_t i = 0; i < 20; i++) {
        Serial.print(i < bars ? '#' : '-');
    }
    Serial.print(F("] "));
    Serial.print(percent);
    Serial.print(F("%"));
}

void CalibrationUI::printReading(const char* label, float value, const char* unit) {
    Serial.print(F("\r"));
    Serial.print(label);
    Serial.print(F(": "));
    Serial.print(value, 2);
    Serial.print(F(" "));
    Serial.print(unit);
    Serial.print(F("    "));  // Clear any remaining chars
}

void CalibrationUI::printSuccess(const char* message) {
    Serial.println();
    Serial.print(F("[OK] "));
    Serial.println(message);
}

void CalibrationUI::printError(const char* message) {
    Serial.println();
    Serial.print(F("[ERROR] "));
    Serial.println(message);
}

void CalibrationUI::printInstruction(const char* message) {
    Serial.println();
    Serial.println(message);
}

void CalibrationUI::clearScreen() {
    // Send ANSI clear screen codes
    Serial.print(F("\033[2J\033[H"));
}
#endif
