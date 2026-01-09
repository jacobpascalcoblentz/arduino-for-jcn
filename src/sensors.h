/**
 * @file sensors.h
 * @brief Sensor interface classes for aquaponics monitoring system
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#else
// Mock types for testing
typedef uint8_t byte;
#endif

// ============================================================================
// SENSOR STATUS ENUM
// ============================================================================

enum class SensorStatus : uint8_t {
    OK = 0,
    NOT_INITIALIZED,
    READ_ERROR,
    OUT_OF_RANGE,
    CALIBRATION_ERROR,
    TIMEOUT
};

// ============================================================================
// BASE SENSOR CLASS
// ============================================================================

class SensorBase {
public:
    virtual ~SensorBase() = default;

    virtual bool begin() = 0;
    virtual bool read() = 0;
    virtual float getValue() const = 0;
    virtual SensorStatus getStatus() const { return status_; }
    virtual const char* getName() const = 0;

    bool isValid() const { return status_ == SensorStatus::OK; }

protected:
    SensorStatus status_ = SensorStatus::NOT_INITIALIZED;
    float value_ = 0.0f;
    uint32_t lastReadTime_ = 0;
    uint8_t errorCount_ = 0;

    void setError(SensorStatus status) {
        status_ = status;
        errorCount_++;
    }

    void clearError() {
        status_ = SensorStatus::OK;
        errorCount_ = 0;
    }
};

// ============================================================================
// pH SENSOR CLASS
// ============================================================================

struct PhCalibration {
    float neutralVoltage;   // Voltage at pH 7.0
    float acidVoltage;      // Voltage at pH 4.0
    float slope;            // Calculated slope
    float offset;           // Calculated offset
    bool valid;
};

class PhSensor : public SensorBase {
public:
    PhSensor(uint8_t analogPin, uint8_t tempCompPin = 255);

    bool begin() override;
    bool read() override;
    float getValue() const override { return ph_; }
    const char* getName() const override { return "pH"; }

    // Calibration
    void setCalibration(float neutralVoltage, float acidVoltage);
    void calibratePoint(float knownPh, float measuredVoltage);
    PhCalibration getCalibration() const { return calibration_; }

    // Temperature compensation
    void setTemperature(float tempC);
    float getRawVoltage() const { return rawVoltage_; }

    // For testing
    void injectRawValue(int rawAdc);

private:
    uint8_t analogPin_;
    uint8_t tempCompPin_;
    float ph_ = 7.0f;
    float rawVoltage_ = 0.0f;
    float temperature_ = 25.0f;
    PhCalibration calibration_;
    int injectedValue_ = -1;

    float voltageToPhtemperatureCompensated(float voltage, float tempC);
    int readAnalog();
};

// ============================================================================
// TEMPERATURE SENSOR CLASS (DS18B20)
// ============================================================================

class TemperatureSensor : public SensorBase {
public:
    explicit TemperatureSensor(uint8_t oneWirePin);

    bool begin() override;
    bool read() override;
    float getValue() const override { return temperatureC_; }
    const char* getName() const override { return "Temperature"; }

    float getCelsius() const { return temperatureC_; }
    float getFahrenheit() const { return temperatureC_ * 9.0f / 5.0f + 32.0f; }

    // For testing
    void injectValue(float tempC);

private:
    uint8_t pin_;
    float temperatureC_ = 25.0f;
    float injectedTemp_ = -1000.0f;

#ifdef ARDUINO
    OneWire* oneWire_ = nullptr;
    DallasTemperature* sensors_ = nullptr;
#endif
};

// ============================================================================
// TDS SENSOR CLASS
// ============================================================================

struct TdsCalibration {
    float factor;           // Calibration factor
    float tempCoefficient;  // Temperature compensation coefficient
    bool valid;
};

class TdsSensor : public SensorBase {
public:
    TdsSensor(uint8_t analogPin, float vref = 5.0f);

    bool begin() override;
    bool read() override;
    float getValue() const override { return tdsPpm_; }
    const char* getName() const override { return "TDS"; }

    // Calibration
    void setCalibrationFactor(float factor);
    void setTemperatureCoefficient(float coeff);
    TdsCalibration getCalibration() const { return calibration_; }

    // Temperature compensation
    void setTemperature(float tempC);

    // Get EC (Electrical Conductivity) in mS/cm
    float getEc() const { return tdsPpm_ / 500.0f; }
    float getRawVoltage() const { return rawVoltage_; }

    // For testing
    void injectRawValue(int rawAdc);

private:
    uint8_t analogPin_;
    float vref_;
    float tdsPpm_ = 0.0f;
    float rawVoltage_ = 0.0f;
    float temperature_ = 25.0f;
    TdsCalibration calibration_;
    int injectedValue_ = -1;

    float voltageToTds(float voltage, float tempC);
    int readAnalog();
};

// ============================================================================
// WATER LEVEL SENSOR CLASS (Ultrasonic)
// ============================================================================

struct LevelCalibration {
    float tankHeightCm;     // Total tank height
    float sensorOffsetCm;   // Distance from sensor to tank top
    float minLevelCm;       // Minimum safe level
    float maxLevelCm;       // Maximum level
    bool valid;
};

class WaterLevelSensor : public SensorBase {
public:
    WaterLevelSensor(uint8_t trigPin, uint8_t echoPin);

    bool begin() override;
    bool read() override;
    float getValue() const override { return levelPercent_; }
    const char* getName() const override { return "Water Level"; }

    // Calibration
    void setTankHeight(float heightCm);
    void setSensorOffset(float offsetCm);
    void setLevelLimits(float minCm, float maxCm);
    LevelCalibration getCalibration() const { return calibration_; }

    float getLevelCm() const { return levelCm_; }
    float getLevelPercent() const { return levelPercent_; }
    float getDistanceCm() const { return distanceCm_; }

    // For testing
    void injectDistance(float distanceCm);

private:
    uint8_t trigPin_;
    uint8_t echoPin_;
    float distanceCm_ = 0.0f;
    float levelCm_ = 0.0f;
    float levelPercent_ = 0.0f;
    LevelCalibration calibration_;
    float injectedDistance_ = -1.0f;

    float measureDistance();
    float distanceToLevel(float distanceCm);
    float levelToPercent(float levelCm);
};

// ============================================================================
// SENSOR MANAGER CLASS
// ============================================================================

struct SensorReadings {
    float ph;
    float temperatureC;
    float tdsPpm;
    float waterLevelPercent;
    uint32_t timestamp;
    bool allValid;
};

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    bool begin();
    bool readAll();

    SensorReadings getReadings() const { return readings_; }

    PhSensor* getPhSensor() { return phSensor_; }
    TemperatureSensor* getTempSensor() { return tempSensor_; }
    TdsSensor* getTdsSensor() { return tdsSensor_; }
    WaterLevelSensor* getLevelSensor() { return levelSensor_; }

    // Cross-sensor compensation
    void applyTemperatureCompensation();

private:
    PhSensor* phSensor_;
    TemperatureSensor* tempSensor_;
    TdsSensor* tdsSensor_;
    WaterLevelSensor* levelSensor_;
    SensorReadings readings_;
};

#endif // SENSORS_H
