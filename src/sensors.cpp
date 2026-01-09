/**
 * @file sensors.cpp
 * @brief Sensor implementation for aquaponics monitoring system
 */

#include "sensors.h"
#include "config.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "mock_arduino.h"
#endif

// ============================================================================
// pH SENSOR IMPLEMENTATION
// ============================================================================

PhSensor::PhSensor(uint8_t analogPin, uint8_t tempCompPin)
    : analogPin_(analogPin), tempCompPin_(tempCompPin) {
    calibration_.neutralVoltage = PH_CALIBRATION_VOLTAGE_NEUTRAL;
    calibration_.acidVoltage = PH_CALIBRATION_VOLTAGE_ACID;
    calibration_.slope = PH_CALIBRATION_SLOPE;
    calibration_.offset = 7.0f - (PH_CALIBRATION_VOLTAGE_NEUTRAL / 1000.0f) * calibration_.slope;
    calibration_.valid = true;
}

bool PhSensor::begin() {
#ifdef ARDUINO
    pinMode(analogPin_, INPUT);
    if (tempCompPin_ != 255) {
        pinMode(tempCompPin_, INPUT);
    }
#endif
    status_ = SensorStatus::OK;
    return true;
}

int PhSensor::readAnalog() {
    if (injectedValue_ >= 0) {
        return injectedValue_;
    }
#ifdef ARDUINO
    return analogRead(analogPin_);
#else
    return 512; // Default mid-range for testing
#endif
}

bool PhSensor::read() {
    if (!calibration_.valid) {
        setError(SensorStatus::CALIBRATION_ERROR);
        return false;
    }

    // Average multiple samples
    float voltageSum = 0.0f;
    for (int i = 0; i < PH_SAMPLE_COUNT; i++) {
        int raw = readAnalog();
        voltageSum += (raw * ADC_VREF / ADC_RESOLUTION) * 1000.0f; // mV

#ifdef ARDUINO
        delay(10);
#endif
    }
    rawVoltage_ = voltageSum / PH_SAMPLE_COUNT;

    // Convert to pH with temperature compensation
    ph_ = voltageToPhtemperatureCompensated(rawVoltage_, temperature_);

    // Validate range
    if (ph_ < 0.0f || ph_ > 14.0f) {
        setError(SensorStatus::OUT_OF_RANGE);
        return false;
    }

    clearError();
    lastReadTime_ = millis();
    return true;
}

float PhSensor::voltageToPhtemperatureCompensated(float voltage, float tempC) {
    // Nernst equation temperature compensation
    // At 25C, slope is -59.16 mV/pH
    // Temperature coefficient: slope = -59.16 * (273.15 + T) / 298.15

    float tempFactor = (273.15f + tempC) / 298.15f;

    // Two-point linear calibration
    float phPerMv = (4.0f - 7.0f) / (calibration_.acidVoltage - calibration_.neutralVoltage);
    float ph = 7.0f + (voltage - calibration_.neutralVoltage) * phPerMv;

    // Apply temperature compensation
    float deviation = ph - 7.0f;
    float compensatedPh = 7.0f + deviation / tempFactor;

    return compensatedPh;
}

void PhSensor::setCalibration(float neutralVoltage, float acidVoltage) {
    calibration_.neutralVoltage = neutralVoltage;
    calibration_.acidVoltage = acidVoltage;
    calibration_.slope = (4.0f - 7.0f) / ((acidVoltage - neutralVoltage) / 1000.0f);
    calibration_.offset = 7.0f - (neutralVoltage / 1000.0f) * calibration_.slope;
    calibration_.valid = true;
}

void PhSensor::calibratePoint(float knownPh, float measuredVoltage) {
    if (knownPh == 7.0f) {
        calibration_.neutralVoltage = measuredVoltage;
    } else if (knownPh == 4.0f) {
        calibration_.acidVoltage = measuredVoltage;
    }

    if (calibration_.neutralVoltage > 0 && calibration_.acidVoltage > 0) {
        calibration_.slope = (4.0f - 7.0f) /
            ((calibration_.acidVoltage - calibration_.neutralVoltage) / 1000.0f);
        calibration_.valid = true;
    }
}

void PhSensor::setTemperature(float tempC) {
    temperature_ = tempC;
}

void PhSensor::injectRawValue(int rawAdc) {
    injectedValue_ = rawAdc;
}

// ============================================================================
// TEMPERATURE SENSOR IMPLEMENTATION
// ============================================================================

TemperatureSensor::TemperatureSensor(uint8_t oneWirePin) : pin_(oneWirePin) {
}

bool TemperatureSensor::begin() {
#ifdef ARDUINO
    oneWire_ = new OneWire(pin_);
    sensors_ = new DallasTemperature(oneWire_);
    sensors_->begin();

    if (sensors_->getDeviceCount() == 0) {
        setError(SensorStatus::NOT_INITIALIZED);
        return false;
    }

    sensors_->setResolution(12);  // 12-bit resolution
#endif
    status_ = SensorStatus::OK;
    return true;
}

bool TemperatureSensor::read() {
    if (injectedTemp_ > -999.0f) {
        temperatureC_ = injectedTemp_;
        clearError();
        lastReadTime_ = millis();
        return true;
    }

#ifdef ARDUINO
    if (sensors_ == nullptr) {
        setError(SensorStatus::NOT_INITIALIZED);
        return false;
    }

    sensors_->requestTemperatures();
    float temp = sensors_->getTempCByIndex(0);

    if (temp == DEVICE_DISCONNECTED_C) {
        setError(SensorStatus::READ_ERROR);
        return false;
    }

    temperatureC_ = temp;
#endif

    // Validate range
    if (temperatureC_ < -10.0f || temperatureC_ > 50.0f) {
        setError(SensorStatus::OUT_OF_RANGE);
        return false;
    }

    clearError();
    lastReadTime_ = millis();
    return true;
}

void TemperatureSensor::injectValue(float tempC) {
    injectedTemp_ = tempC;
}

// ============================================================================
// TDS SENSOR IMPLEMENTATION
// ============================================================================

TdsSensor::TdsSensor(uint8_t analogPin, float vref)
    : analogPin_(analogPin), vref_(vref) {
    calibration_.factor = TDS_CALIBRATION_FACTOR;
    calibration_.tempCoefficient = TDS_TEMPERATURE_COEFFICIENT;
    calibration_.valid = true;
}

bool TdsSensor::begin() {
#ifdef ARDUINO
    pinMode(analogPin_, INPUT);
#endif
    status_ = SensorStatus::OK;
    return true;
}

int TdsSensor::readAnalog() {
    if (injectedValue_ >= 0) {
        return injectedValue_;
    }
#ifdef ARDUINO
    return analogRead(analogPin_);
#else
    return 512;
#endif
}

bool TdsSensor::read() {
    if (!calibration_.valid) {
        setError(SensorStatus::CALIBRATION_ERROR);
        return false;
    }

    // Average multiple samples
    float voltageSum = 0.0f;
    for (int i = 0; i < TDS_SAMPLE_COUNT; i++) {
        int raw = readAnalog();
        voltageSum += raw * vref_ / ADC_RESOLUTION;

#ifdef ARDUINO
        delay(10);
#endif
    }
    rawVoltage_ = voltageSum / TDS_SAMPLE_COUNT;

    // Convert to TDS with temperature compensation
    tdsPpm_ = voltageToTds(rawVoltage_, temperature_);

    // Validate range
    if (tdsPpm_ < 0.0f || tdsPpm_ > 5000.0f) {
        setError(SensorStatus::OUT_OF_RANGE);
        return false;
    }

    clearError();
    lastReadTime_ = millis();
    return true;
}

float TdsSensor::voltageToTds(float voltage, float tempC) {
    // Temperature compensation for TDS
    // EC increases ~2% per degree C
    float tempCompensation = 1.0f + calibration_.tempCoefficient * (tempC - 25.0f);

    // Compensated voltage
    float compensatedVoltage = voltage / tempCompensation;

    // TDS calculation (empirical formula for common TDS meters)
    // TDS = (133.42 * V^3 - 255.86 * V^2 + 857.39 * V) * factor
    float tds = (133.42f * compensatedVoltage * compensatedVoltage * compensatedVoltage
                - 255.86f * compensatedVoltage * compensatedVoltage
                + 857.39f * compensatedVoltage) * calibration_.factor;

    return tds > 0.0f ? tds : 0.0f;
}

void TdsSensor::setCalibrationFactor(float factor) {
    calibration_.factor = factor;
}

void TdsSensor::setTemperatureCoefficient(float coeff) {
    calibration_.tempCoefficient = coeff;
}

void TdsSensor::setTemperature(float tempC) {
    temperature_ = tempC;
}

void TdsSensor::injectRawValue(int rawAdc) {
    injectedValue_ = rawAdc;
}

// ============================================================================
// WATER LEVEL SENSOR IMPLEMENTATION
// ============================================================================

WaterLevelSensor::WaterLevelSensor(uint8_t trigPin, uint8_t echoPin)
    : trigPin_(trigPin), echoPin_(echoPin) {
    calibration_.tankHeightCm = WATER_LEVEL_TANK_HEIGHT_CM;
    calibration_.sensorOffsetCm = WATER_LEVEL_SENSOR_OFFSET_CM;
    calibration_.minLevelCm = WATER_LEVEL_MIN_CM;
    calibration_.maxLevelCm = WATER_LEVEL_MAX_CM;
    calibration_.valid = true;
}

bool WaterLevelSensor::begin() {
#ifdef ARDUINO
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
#endif
    status_ = SensorStatus::OK;
    return true;
}

bool WaterLevelSensor::read() {
    if (!calibration_.valid) {
        setError(SensorStatus::CALIBRATION_ERROR);
        return false;
    }

    // Average multiple samples
    float distanceSum = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < LEVEL_SAMPLE_COUNT; i++) {
        float d = measureDistance();
        if (d > 0.0f && d < 400.0f) {  // HC-SR04 max range ~400cm
            distanceSum += d;
            validSamples++;
        }
#ifdef ARDUINO
        delay(50);  // Ultrasonic needs time between readings
#endif
    }

    if (validSamples == 0) {
        setError(SensorStatus::READ_ERROR);
        return false;
    }

    distanceCm_ = distanceSum / validSamples;
    levelCm_ = distanceToLevel(distanceCm_);
    levelPercent_ = levelToPercent(levelCm_);

    // Validate range
    if (levelPercent_ < 0.0f || levelPercent_ > 100.0f) {
        // Clamp but don't error
        levelPercent_ = levelPercent_ < 0.0f ? 0.0f : 100.0f;
    }

    clearError();
    lastReadTime_ = millis();
    return true;
}

float WaterLevelSensor::measureDistance() {
    if (injectedDistance_ >= 0.0f) {
        return injectedDistance_;
    }

#ifdef ARDUINO
    // Send ultrasonic pulse
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    // Measure echo time
    unsigned long duration = pulseIn(echoPin_, HIGH, 30000);  // 30ms timeout

    if (duration == 0) {
        return -1.0f;  // No echo received
    }

    // Convert to distance (speed of sound = 343 m/s at 20C)
    // distance = duration * 0.0343 / 2
    return duration * 0.0343f / 2.0f;
#else
    return 20.0f;  // Default for testing
#endif
}

float WaterLevelSensor::distanceToLevel(float distanceCm) {
    // Level = tank height - (distance - sensor offset)
    return calibration_.tankHeightCm - (distanceCm - calibration_.sensorOffsetCm);
}

float WaterLevelSensor::levelToPercent(float levelCm) {
    float range = calibration_.maxLevelCm - calibration_.minLevelCm;
    if (range <= 0.0f) return 0.0f;

    float percent = (levelCm - calibration_.minLevelCm) / range * 100.0f;
    return percent;
}

void WaterLevelSensor::setTankHeight(float heightCm) {
    calibration_.tankHeightCm = heightCm;
}

void WaterLevelSensor::setSensorOffset(float offsetCm) {
    calibration_.sensorOffsetCm = offsetCm;
}

void WaterLevelSensor::setLevelLimits(float minCm, float maxCm) {
    calibration_.minLevelCm = minCm;
    calibration_.maxLevelCm = maxCm;
}

void WaterLevelSensor::injectDistance(float distanceCm) {
    injectedDistance_ = distanceCm;
}

// ============================================================================
// SENSOR MANAGER IMPLEMENTATION
// ============================================================================

SensorManager::SensorManager() {
    phSensor_ = new PhSensor(PIN_PH_SENSOR, PIN_PH_TEMP_COMP);
    tempSensor_ = new TemperatureSensor(PIN_TEMP_ONEWIRE);
    tdsSensor_ = new TdsSensor(PIN_TDS_SENSOR);
    levelSensor_ = new WaterLevelSensor(PIN_ULTRASONIC_TRIG, PIN_ULTRASONIC_ECHO);

    readings_.ph = 7.0f;
    readings_.temperatureC = 25.0f;
    readings_.tdsPpm = 0.0f;
    readings_.waterLevelPercent = 0.0f;
    readings_.timestamp = 0;
    readings_.allValid = false;
}

SensorManager::~SensorManager() {
    delete phSensor_;
    delete tempSensor_;
    delete tdsSensor_;
    delete levelSensor_;
}

bool SensorManager::begin() {
    bool success = true;

    success &= phSensor_->begin();
    success &= tempSensor_->begin();
    success &= tdsSensor_->begin();
    success &= levelSensor_->begin();

    return success;
}

bool SensorManager::readAll() {
    bool allValid = true;

    // Read temperature first for compensation
    if (tempSensor_->read()) {
        readings_.temperatureC = tempSensor_->getValue();
        applyTemperatureCompensation();
    } else {
        allValid = false;
    }

    // Read other sensors
    if (phSensor_->read()) {
        readings_.ph = phSensor_->getValue();
    } else {
        allValid = false;
    }

    if (tdsSensor_->read()) {
        readings_.tdsPpm = tdsSensor_->getValue();
    } else {
        allValid = false;
    }

    if (levelSensor_->read()) {
        readings_.waterLevelPercent = levelSensor_->getValue();
    } else {
        allValid = false;
    }

    readings_.timestamp = millis();
    readings_.allValid = allValid;

    return allValid;
}

void SensorManager::applyTemperatureCompensation() {
    float temp = tempSensor_->getValue();
    phSensor_->setTemperature(temp);
    tdsSensor_->setTemperature(temp);
}
