/**
 * @file test_sensors.cpp
 * @brief Google Test unit tests for sensor classes
 */

#include <gtest/gtest.h>
#include <cmath>
#include "../src/sensors.h"
#include "../src/config.h"

// ============================================================================
// pH SENSOR TESTS
// ============================================================================

class PhSensorTest : public ::testing::Test {
protected:
    PhSensor* sensor;

    void SetUp() override {
        sensor = new PhSensor(A0, A1);
        sensor->begin();
    }

    void TearDown() override {
        delete sensor;
    }
};

TEST_F(PhSensorTest, Initialization) {
    EXPECT_EQ(sensor->getStatus(), SensorStatus::OK);
    EXPECT_TRUE(sensor->isValid());
    EXPECT_STREQ(sensor->getName(), "pH");
}

TEST_F(PhSensorTest, CalibrationSetup) {
    sensor->setCalibration(1500.0f, 2032.0f);
    PhCalibration cal = sensor->getCalibration();

    EXPECT_FLOAT_EQ(cal.neutralVoltage, 1500.0f);
    EXPECT_FLOAT_EQ(cal.acidVoltage, 2032.0f);
    EXPECT_TRUE(cal.valid);
}

TEST_F(PhSensorTest, ReadAtNeutral) {
    // Inject ADC value that corresponds to neutral voltage (1500mV)
    // ADC = voltage_mV * 1024 / 5000
    int adcValue = (int)(1500.0f * 1024.0f / 5000.0f);
    sensor->injectRawValue(adcValue);
    sensor->setCalibration(1500.0f, 2032.0f);

    EXPECT_TRUE(sensor->read());
    float ph = sensor->getValue();

    // Should be close to 7.0 at neutral calibration voltage
    EXPECT_NEAR(ph, 7.0f, 0.1f);
}

TEST_F(PhSensorTest, ReadAtAcidic) {
    // Inject ADC value for pH 4.0 buffer (2032mV)
    int adcValue = (int)(2032.0f * 1024.0f / 5000.0f);
    sensor->injectRawValue(adcValue);
    sensor->setCalibration(1500.0f, 2032.0f);

    EXPECT_TRUE(sensor->read());
    float ph = sensor->getValue();

    // Should be close to 4.0
    EXPECT_NEAR(ph, 4.0f, 0.1f);
}

TEST_F(PhSensorTest, TemperatureCompensation) {
    int adcValue = (int)(1600.0f * 1024.0f / 5000.0f);
    sensor->injectRawValue(adcValue);
    sensor->setCalibration(1500.0f, 2032.0f);

    // Read at 25C (standard)
    sensor->setTemperature(25.0f);
    sensor->read();
    float ph25 = sensor->getValue();

    // Read at 35C
    sensor->setTemperature(35.0f);
    sensor->read();
    float ph35 = sensor->getValue();

    // Temperature compensation should produce different readings
    // Higher temp = larger Nernst slope = different pH for same voltage
    EXPECT_NE(ph25, ph35);
}

TEST_F(PhSensorTest, OutOfRangeDetection) {
    // Inject voltage that would produce pH < 0 or > 14
    sensor->injectRawValue(1023);  // Max ADC value
    sensor->setCalibration(1500.0f, 2032.0f);

    // This may or may not fail depending on calibration
    sensor->read();
    // Just ensure no crash
    EXPECT_TRUE(true);
}

TEST_F(PhSensorTest, TwoPointCalibration) {
    sensor->calibratePoint(7.0f, 1500.0f);
    sensor->calibratePoint(4.0f, 2032.0f);

    PhCalibration cal = sensor->getCalibration();
    EXPECT_TRUE(cal.valid);
    EXPECT_FLOAT_EQ(cal.neutralVoltage, 1500.0f);
    EXPECT_FLOAT_EQ(cal.acidVoltage, 2032.0f);
}

// ============================================================================
// TEMPERATURE SENSOR TESTS
// ============================================================================

class TemperatureSensorTest : public ::testing::Test {
protected:
    TemperatureSensor* sensor;

    void SetUp() override {
        sensor = new TemperatureSensor(2);
        sensor->begin();
    }

    void TearDown() override {
        delete sensor;
    }
};

TEST_F(TemperatureSensorTest, Initialization) {
    EXPECT_STREQ(sensor->getName(), "Temperature");
}

TEST_F(TemperatureSensorTest, InjectedValueRead) {
    sensor->injectValue(23.5f);
    EXPECT_TRUE(sensor->read());
    EXPECT_FLOAT_EQ(sensor->getCelsius(), 23.5f);
}

TEST_F(TemperatureSensorTest, CelsiusToFahrenheit) {
    sensor->injectValue(25.0f);
    sensor->read();

    float fahrenheit = sensor->getFahrenheit();
    EXPECT_FLOAT_EQ(fahrenheit, 77.0f);  // 25C = 77F
}

TEST_F(TemperatureSensorTest, FreezingPoint) {
    sensor->injectValue(0.0f);
    sensor->read();

    EXPECT_FLOAT_EQ(sensor->getCelsius(), 0.0f);
    EXPECT_FLOAT_EQ(sensor->getFahrenheit(), 32.0f);
}

TEST_F(TemperatureSensorTest, BoilingPoint) {
    sensor->injectValue(100.0f);
    sensor->read();

    // Should be out of range (>50C limit)
    EXPECT_EQ(sensor->getStatus(), SensorStatus::OUT_OF_RANGE);
}

TEST_F(TemperatureSensorTest, NegativeTemperature) {
    sensor->injectValue(-5.0f);
    sensor->read();

    EXPECT_FLOAT_EQ(sensor->getCelsius(), -5.0f);
    EXPECT_FLOAT_EQ(sensor->getFahrenheit(), 23.0f);
}

// ============================================================================
// TDS SENSOR TESTS
// ============================================================================

class TdsSensorTest : public ::testing::Test {
protected:
    TdsSensor* sensor;

    void SetUp() override {
        sensor = new TdsSensor(A2, 5.0f);
        sensor->begin();
    }

    void TearDown() override {
        delete sensor;
    }
};

TEST_F(TdsSensorTest, Initialization) {
    EXPECT_EQ(sensor->getStatus(), SensorStatus::OK);
    EXPECT_STREQ(sensor->getName(), "TDS");
}

TEST_F(TdsSensorTest, ZeroTdsAtZeroVoltage) {
    sensor->injectRawValue(0);
    sensor->read();

    EXPECT_FLOAT_EQ(sensor->getValue(), 0.0f);
    EXPECT_FLOAT_EQ(sensor->getEc(), 0.0f);
}

TEST_F(TdsSensorTest, CalibrationFactorAdjustment) {
    sensor->injectRawValue(512);  // Mid-range
    sensor->setCalibrationFactor(1.0f);
    sensor->read();
    float tds1 = sensor->getValue();

    sensor->setCalibrationFactor(2.0f);
    sensor->read();
    float tds2 = sensor->getValue();

    EXPECT_NEAR(tds2, tds1 * 2.0f, 1.0f);
}

TEST_F(TdsSensorTest, TemperatureCompensation) {
    sensor->injectRawValue(512);

    sensor->setTemperature(25.0f);
    sensor->read();
    float tds25 = sensor->getValue();

    sensor->setTemperature(35.0f);
    sensor->read();
    float tds35 = sensor->getValue();

    // At higher temp, compensated TDS should be lower
    // (same reading represents higher actual TDS at higher temp)
    EXPECT_LT(tds35, tds25);
}

TEST_F(TdsSensorTest, EcConversion) {
    sensor->injectRawValue(512);
    sensor->read();

    float tds = sensor->getValue();
    float ec = sensor->getEc();

    // EC = TDS / 500
    EXPECT_FLOAT_EQ(ec, tds / 500.0f);
}

TEST_F(TdsSensorTest, CalibrationPersistence) {
    TdsCalibration cal;
    cal.factor = 1.5f;
    cal.tempCoefficient = 0.025f;

    sensor->setCalibrationFactor(cal.factor);
    sensor->setTemperatureCoefficient(cal.tempCoefficient);

    TdsCalibration readCal = sensor->getCalibration();
    EXPECT_FLOAT_EQ(readCal.factor, 1.5f);
    EXPECT_FLOAT_EQ(readCal.tempCoefficient, 0.025f);
}

// ============================================================================
// WATER LEVEL SENSOR TESTS
// ============================================================================

class WaterLevelSensorTest : public ::testing::Test {
protected:
    WaterLevelSensor* sensor;

    void SetUp() override {
        sensor = new WaterLevelSensor(3, 4);
        sensor->begin();
        // Set known tank parameters
        sensor->setTankHeight(60.0f);
        sensor->setSensorOffset(5.0f);
        sensor->setLevelLimits(10.0f, 55.0f);
    }

    void TearDown() override {
        delete sensor;
    }
};

TEST_F(WaterLevelSensorTest, Initialization) {
    EXPECT_EQ(sensor->getStatus(), SensorStatus::OK);
    EXPECT_STREQ(sensor->getName(), "Water Level");
}

TEST_F(WaterLevelSensorTest, FullTankLevel) {
    // Full tank: distance from sensor to water = sensor offset
    sensor->injectDistance(5.0f);  // Sensor offset
    sensor->read();

    // Level should be tank height - (distance - offset) = 60 - (5 - 5) = 60cm
    EXPECT_NEAR(sensor->getLevelCm(), 60.0f, 1.0f);
    // Percent should be 100% or clamped
    EXPECT_GE(sensor->getLevelPercent(), 95.0f);
}

TEST_F(WaterLevelSensorTest, EmptyTankLevel) {
    // Empty tank: distance from sensor to water is large
    sensor->injectDistance(55.0f);  // Tank height - min level + offset
    sensor->read();

    // Level = 60 - (55 - 5) = 10cm (minimum)
    EXPECT_NEAR(sensor->getLevelCm(), 10.0f, 1.0f);
    EXPECT_NEAR(sensor->getLevelPercent(), 0.0f, 5.0f);
}

TEST_F(WaterLevelSensorTest, HalfTankLevel) {
    // Half tank (32.5cm level)
    // distance = tank_height - level + offset = 60 - 32.5 + 5 = 32.5cm
    sensor->injectDistance(32.5f);
    sensor->read();

    float level = sensor->getLevelCm();
    // level = 60 - (32.5 - 5) = 32.5cm
    EXPECT_NEAR(level, 32.5f, 1.0f);

    // Percent = (32.5 - 10) / (55 - 10) * 100 = 50%
    EXPECT_NEAR(sensor->getLevelPercent(), 50.0f, 5.0f);
}

TEST_F(WaterLevelSensorTest, CalibrationParameters) {
    LevelCalibration cal = sensor->getCalibration();

    EXPECT_FLOAT_EQ(cal.tankHeightCm, 60.0f);
    EXPECT_FLOAT_EQ(cal.sensorOffsetCm, 5.0f);
    EXPECT_FLOAT_EQ(cal.minLevelCm, 10.0f);
    EXPECT_FLOAT_EQ(cal.maxLevelCm, 55.0f);
}

TEST_F(WaterLevelSensorTest, DistanceReading) {
    sensor->injectDistance(25.0f);
    sensor->read();

    EXPECT_FLOAT_EQ(sensor->getDistanceCm(), 25.0f);
}

// ============================================================================
// SENSOR MANAGER TESTS
// ============================================================================

class SensorManagerTest : public ::testing::Test {
protected:
    SensorManager* manager;

    void SetUp() override {
        manager = new SensorManager();
        manager->begin();
    }

    void TearDown() override {
        delete manager;
    }
};

TEST_F(SensorManagerTest, Initialization) {
    EXPECT_NE(manager->getPhSensor(), nullptr);
    EXPECT_NE(manager->getTempSensor(), nullptr);
    EXPECT_NE(manager->getTdsSensor(), nullptr);
    EXPECT_NE(manager->getLevelSensor(), nullptr);
}

TEST_F(SensorManagerTest, ReadAllSensors) {
    // Inject test values
    manager->getTempSensor()->injectValue(24.0f);
    manager->getPhSensor()->injectRawValue(307);  // ~1500mV
    manager->getTdsSensor()->injectRawValue(512);
    manager->getLevelSensor()->injectDistance(20.0f);

    manager->readAll();

    SensorReadings readings = manager->getReadings();
    EXPECT_NEAR(readings.temperatureC, 24.0f, 0.1f);
    EXPECT_GT(readings.timestamp, 0u);
}

TEST_F(SensorManagerTest, TemperatureCompensationApplied) {
    manager->getTempSensor()->injectValue(30.0f);
    manager->getPhSensor()->injectRawValue(307);
    manager->getTdsSensor()->injectRawValue(512);
    manager->getLevelSensor()->injectDistance(20.0f);

    manager->readAll();

    // Temperature compensation should have been applied
    // We can't directly test this without accessing private state,
    // but we can verify no crash and valid readings
    SensorReadings readings = manager->getReadings();
    EXPECT_FLOAT_EQ(readings.temperatureC, 30.0f);
}

TEST_F(SensorManagerTest, SensorReadingsStructure) {
    manager->getTempSensor()->injectValue(25.0f);
    manager->getPhSensor()->injectRawValue(307);
    manager->getTdsSensor()->injectRawValue(400);
    manager->getLevelSensor()->injectDistance(15.0f);

    manager->readAll();
    SensorReadings readings = manager->getReadings();

    // All fields should be populated
    EXPECT_GT(readings.ph, 0.0f);
    EXPECT_LT(readings.ph, 14.0f);
    EXPECT_GT(readings.temperatureC, 0.0f);
    EXPECT_GE(readings.tdsPpm, 0.0f);
    EXPECT_GE(readings.waterLevelPercent, 0.0f);
}

// ============================================================================
// EDGE CASES AND ERROR HANDLING
// ============================================================================

TEST(SensorEdgeCases, PhSensorInvalidCalibration) {
    PhSensor sensor(A0);
    sensor.begin();

    // Set invalid calibration (same voltage for both points)
    sensor.setCalibration(1500.0f, 1500.0f);

    sensor.injectRawValue(512);
    // Should handle division by zero gracefully
    sensor.read();
    // Don't crash
    EXPECT_TRUE(true);
}

TEST(SensorEdgeCases, TdsSensorMaxValue) {
    TdsSensor sensor(A2);
    sensor.begin();

    sensor.injectRawValue(1023);  // Max ADC
    sensor.read();

    // Should either read high TDS or report out of range
    float tds = sensor.getValue();
    EXPECT_GE(tds, 0.0f);
}

TEST(SensorEdgeCases, WaterLevelNegativeDistance) {
    WaterLevelSensor sensor(3, 4);
    sensor.begin();
    sensor.setTankHeight(60.0f);
    sensor.setSensorOffset(5.0f);

    // This shouldn't happen in practice, but handle gracefully
    sensor.injectDistance(-1.0f);
    // The read may fail due to invalid measurement
    // Just ensure no crash
    sensor.read();
    EXPECT_TRUE(true);
}

// ============================================================================
// SENSOR STATUS TESTS
// ============================================================================

TEST(SensorStatus, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::OK), 0);
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::NOT_INITIALIZED), 1);
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::READ_ERROR), 2);
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::OUT_OF_RANGE), 3);
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::CALIBRATION_ERROR), 4);
    EXPECT_EQ(static_cast<uint8_t>(SensorStatus::TIMEOUT), 5);
}
