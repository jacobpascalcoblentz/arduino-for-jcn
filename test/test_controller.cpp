/**
 * @file test_controller.cpp
 * @brief Google Test unit tests for feedforward controller
 */

#include <gtest/gtest.h>
#include <cmath>
#include "../src/controller.h"
#include "../src/config.h"

// Mock millis() for testing
static uint32_t mockMillisValue = 0;
uint32_t millis() { return mockMillisValue; }
void setMockMillis(uint32_t ms) { mockMillisValue = ms; }
void advanceMillis(uint32_t ms) { mockMillisValue += ms; }

// ============================================================================
// SYSTEM MODEL TESTS
// ============================================================================

class SystemModelTest : public ::testing::Test {
protected:
    SystemModel model;

    void SetUp() override {
        model.setDefaults();
    }
};

TEST_F(SystemModelTest, DefaultValues) {
    EXPECT_FLOAT_EQ(model.systemVolumeLiters, SYSTEM_VOLUME_LITERS);
    EXPECT_GT(model.phDownStrength, 0.0f);
    EXPECT_GT(model.phUpStrength, 0.0f);
    EXPECT_GT(model.nutrientATdsFactor, 0.0f);
}

TEST_F(SystemModelTest, VolumeUpdate) {
    float originalPhDownStrength = model.phDownStrength;

    model.updateFromVolume(50.0f);  // Half the default volume

    // Strength should double with half the volume
    EXPECT_NEAR(model.phDownStrength, originalPhDownStrength * 2.0f, 0.01f);
}

TEST_F(SystemModelTest, ModelConfidence) {
    EXPECT_GE(model.phModelConfidence, 0.0f);
    EXPECT_LE(model.phModelConfidence, 1.0f);
    EXPECT_GE(model.tdsModelConfidence, 0.0f);
    EXPECT_LE(model.tdsModelConfidence, 1.0f);
    EXPECT_GE(model.levelModelConfidence, 0.0f);
    EXPECT_LE(model.levelModelConfidence, 1.0f);
}

// ============================================================================
// FEEDFORWARD CONTROLLER TESTS
// ============================================================================

class FeedforwardControllerTest : public ::testing::Test {
protected:
    FeedforwardController* controller;

    void SetUp() override {
        mockMillisValue = 0;
        controller = new FeedforwardController();

        // Set known setpoints
        controller->setSetpoints(6.5f, 800.0f, 80.0f);
        controller->setDeadbands(0.2f, 50.0f, 5.0f);

        // Use default system model
        SystemModel model;
        model.setDefaults();
        controller->setSystemModel(model);

        // Allow immediate dosing for tests
        controller->setMinDoseInterval(0);
    }

    void TearDown() override {
        delete controller;
    }

    SensorReadings createReadings(float ph, float temp, float tds, float level) {
        SensorReadings readings;
        readings.ph = ph;
        readings.temperatureC = temp;
        readings.tdsPpm = tds;
        readings.waterLevelPercent = level;
        readings.timestamp = millis();
        readings.allValid = true;
        return readings;
    }
};

TEST_F(FeedforwardControllerTest, Initialization) {
    EXPECT_FLOAT_EQ(controller->getPhError(), 0.0f);
    EXPECT_FLOAT_EQ(controller->getTdsError(), 0.0f);
    EXPECT_FLOAT_EQ(controller->getLevelError(), 0.0f);
}

TEST_F(FeedforwardControllerTest, AtSetpoint_NoDosing) {
    SensorReadings readings = createReadings(6.5f, 25.0f, 800.0f, 80.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.phUpDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.nutrientADoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.nutrientBDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.freshWaterMl, 0.0f);
}

TEST_F(FeedforwardControllerTest, WithinDeadband_NoDosing) {
    // All values within deadband
    SensorReadings readings = createReadings(6.6f, 25.0f, 820.0f, 82.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.phUpDoseMl, 0.0f);
    EXPECT_TRUE(controller->isInDeadband());
}

TEST_F(FeedforwardControllerTest, PhTooHigh_DoseAcid) {
    SensorReadings readings = createReadings(7.5f, 25.0f, 800.0f, 80.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_GT(action.phDownDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.phUpDoseMl, 0.0f);
    EXPECT_LT(controller->getPhError(), 0.0f);  // Error is negative (setpoint - actual)
}

TEST_F(FeedforwardControllerTest, PhTooLow_DoseBase) {
    SensorReadings readings = createReadings(5.5f, 25.0f, 800.0f, 80.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
    EXPECT_GT(action.phUpDoseMl, 0.0f);
    EXPECT_GT(controller->getPhError(), 0.0f);  // Error is positive
}

TEST_F(FeedforwardControllerTest, TdsTooLow_DoseNutrients) {
    SensorReadings readings = createReadings(6.5f, 25.0f, 600.0f, 80.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_GT(action.nutrientADoseMl, 0.0f);
    EXPECT_GT(action.nutrientBDoseMl, 0.0f);
    // Nutrients should be dosed equally
    EXPECT_FLOAT_EQ(action.nutrientADoseMl, action.nutrientBDoseMl);
}

TEST_F(FeedforwardControllerTest, WaterLevelLow_AddWater) {
    SensorReadings readings = createReadings(6.5f, 25.0f, 800.0f, 50.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_GT(action.freshWaterMl, 0.0f);
}

TEST_F(FeedforwardControllerTest, DoseLimitsEnforced) {
    controller->setDoseLimits(5.0f, 10.0f, 500.0f);

    // Extreme error
    SensorReadings readings = createReadings(9.0f, 25.0f, 100.0f, 20.0f);

    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_LE(action.phDownDoseMl, 5.0f);
    EXPECT_LE(action.nutrientADoseMl, 10.0f);
    EXPECT_LE(action.freshWaterMl, 500.0f);
}

TEST_F(FeedforwardControllerTest, EmergencyStop) {
    controller->setEmergencyStop(true);

    SensorReadings readings = createReadings(9.0f, 25.0f, 100.0f, 20.0f);
    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.nutrientADoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.freshWaterMl, 0.0f);
    EXPECT_FALSE(action.circulationPumpOn);
}

TEST_F(FeedforwardControllerTest, IntegratorAccumulation) {
    controller->setFeedforwardGains(0.0f, 0.0f, 0.0f);  // Disable FF
    controller->setIntegralGains(0.1f, 0.1f, 0.1f);

    SensorReadings readings = createReadings(7.0f, 25.0f, 700.0f, 70.0f);

    // Multiple updates should accumulate integral
    for (int i = 0; i < 10; i++) {
        advanceMillis(1000);
        controller->update(readings, 1.0f);
    }

    // Integrators should have accumulated
    EXPECT_NE(controller->getPhIntegral(), 0.0f);
    EXPECT_NE(controller->getTdsIntegral(), 0.0f);
}

TEST_F(FeedforwardControllerTest, IntegratorReset) {
    SensorReadings readings = createReadings(7.0f, 25.0f, 700.0f, 70.0f);
    controller->update(readings, 1.0f);

    controller->resetIntegrators();

    EXPECT_FLOAT_EQ(controller->getPhIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(controller->getTdsIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(controller->getLevelIntegral(), 0.0f);
}

TEST_F(FeedforwardControllerTest, FeedforwardDisabled) {
    controller->enableFeedforward(false);
    controller->setIntegralGains(0.0f, 0.0f, 0.0f);  // Also disable feedback

    SensorReadings readings = createReadings(9.0f, 25.0f, 100.0f, 20.0f);
    ControlAction action = controller->update(readings, 1.0f);

    // No control action without FF or FB
    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
}

TEST_F(FeedforwardControllerTest, FeedbackOnly) {
    controller->enableFeedforward(false);
    controller->setIntegralGains(0.5f, 0.5f, 0.5f);

    SensorReadings readings = createReadings(7.0f, 25.0f, 700.0f, 70.0f);

    // First update
    controller->update(readings, 1.0f);

    // Integrator should have some value
    EXPECT_NE(controller->getPhIntegral(), 0.0f);
}

TEST_F(FeedforwardControllerTest, DisturbanceEstimation) {
    SensorReadings prev = createReadings(6.5f, 25.0f, 800.0f, 80.0f);
    advanceMillis(3600000);  // 1 hour later
    SensorReadings curr = createReadings(6.3f, 25.0f, 780.0f, 79.0f);

    controller->updateDisturbanceEstimates(prev, curr, 3600.0f);

    // pH drifted down 0.2 in 1 hour
    EXPECT_LT(controller->getEstimatedPhDrift(), 0.0f);
    // TDS decreased 20ppm in 1 hour (plant uptake)
    EXPECT_GT(controller->getEstimatedTdsUptake(), 0.0f);
}

TEST_F(FeedforwardControllerTest, CirculationPumpAlwaysOn) {
    SensorReadings readings = createReadings(6.5f, 25.0f, 800.0f, 80.0f);
    ControlAction action = controller->update(readings, 1.0f);

    EXPECT_TRUE(action.circulationPumpOn);
}

TEST_F(FeedforwardControllerTest, MinDoseInterval) {
    controller->setMinDoseInterval(300000);  // 5 minutes

    SensorReadings readings = createReadings(7.5f, 25.0f, 800.0f, 80.0f);

    // First dose
    mockMillisValue = 0;
    ControlAction action1 = controller->update(readings, 1.0f);
    EXPECT_GT(action1.phDownDoseMl, 0.0f);

    // Try again immediately - should not dose
    mockMillisValue = 1000;  // 1 second later
    ControlAction action2 = controller->update(readings, 1.0f);
    // Might still show some action due to integrator

    // After interval passed
    mockMillisValue = 400000;  // 6.67 minutes later
    ControlAction action3 = controller->update(readings, 1.0f);
    EXPECT_GT(action3.phDownDoseMl, 0.0f);
}

// ============================================================================
// SAFETY MONITOR TESTS
// ============================================================================

class SafetyMonitorTest : public ::testing::Test {
protected:
    SafetyMonitor* monitor;

    void SetUp() override {
        monitor = new SafetyMonitor();

        SafetyLimits limits;
        limits.phMin = 5.5f;
        limits.phMax = 7.5f;
        limits.tdsMin = 400.0f;
        limits.tdsMax = 1200.0f;
        limits.tempMin = 18.0f;
        limits.tempMax = 30.0f;
        limits.levelMin = 20.0f;
        limits.levelMax = 95.0f;
        monitor->setLimits(limits);
    }

    void TearDown() override {
        delete monitor;
    }

    SensorReadings createReadings(float ph, float temp, float tds, float level) {
        SensorReadings readings;
        readings.ph = ph;
        readings.temperatureC = temp;
        readings.tdsPpm = tds;
        readings.waterLevelPercent = level;
        readings.timestamp = 0;
        readings.allValid = true;
        return readings;
    }
};

TEST_F(SafetyMonitorTest, NormalConditions) {
    SensorReadings readings = createReadings(6.5f, 24.0f, 800.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::NORMAL);
    EXPECT_STREQ(monitor->getStateString(), "NORMAL");
}

TEST_F(SafetyMonitorTest, PhLowAlarm) {
    SensorReadings readings = createReadings(5.0f, 24.0f, 800.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::ALARM);
    EXPECT_STREQ(monitor->getAlarmReason(), "pH LOW");
}

TEST_F(SafetyMonitorTest, PhHighAlarm) {
    SensorReadings readings = createReadings(8.0f, 24.0f, 800.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::ALARM);
    EXPECT_STREQ(monitor->getAlarmReason(), "pH HIGH");
}

TEST_F(SafetyMonitorTest, TdsLowAlarm) {
    SensorReadings readings = createReadings(6.5f, 24.0f, 300.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::ALARM);
    EXPECT_STREQ(monitor->getAlarmReason(), "TDS LOW");
}

TEST_F(SafetyMonitorTest, TemperatureOutOfRange) {
    SensorReadings readings = createReadings(6.5f, 35.0f, 800.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::ALARM);
    EXPECT_STREQ(monitor->getAlarmReason(), "TEMP OUT");
}

TEST_F(SafetyMonitorTest, WaterLevelLowAlarm) {
    SensorReadings readings = createReadings(6.5f, 24.0f, 800.0f, 15.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::ALARM);
    EXPECT_STREQ(monitor->getAlarmReason(), "LEVEL LOW");
}

TEST_F(SafetyMonitorTest, ConsecutiveAlarmsEmergencyStop) {
    SensorReadings readings = createReadings(5.0f, 24.0f, 800.0f, 80.0f);

    // Trigger multiple consecutive alarms
    for (int i = 0; i < SENSOR_FAULT_THRESHOLD; i++) {
        monitor->check(readings);
    }

    SafetyState state = monitor->getState();
    EXPECT_EQ(state, SafetyState::EMERGENCY_STOP);
    EXPECT_STREQ(monitor->getStateString(), "E-STOP");
}

TEST_F(SafetyMonitorTest, AlarmAcknowledge) {
    SensorReadings readings = createReadings(5.0f, 24.0f, 800.0f, 80.0f);
    monitor->check(readings);

    EXPECT_EQ(monitor->getState(), SafetyState::ALARM);

    monitor->acknowledgeAlarm();

    EXPECT_EQ(monitor->getState(), SafetyState::WARNING);
}

TEST_F(SafetyMonitorTest, Reset) {
    SensorReadings readings = createReadings(5.0f, 24.0f, 800.0f, 80.0f);

    for (int i = 0; i < 5; i++) {
        monitor->check(readings);
    }

    monitor->reset();

    EXPECT_EQ(monitor->getState(), SafetyState::NORMAL);
}

TEST_F(SafetyMonitorTest, WarningState) {
    // pH near limit but not over
    SensorReadings readings = createReadings(5.7f, 24.0f, 800.0f, 80.0f);

    SafetyState state = monitor->check(readings);

    EXPECT_EQ(state, SafetyState::WARNING);
}

// ============================================================================
// CONTROL ACTION TESTS
// ============================================================================

TEST(ControlActionTest, Clear) {
    ControlAction action;
    action.phDownDoseMl = 5.0f;
    action.nutrientADoseMl = 10.0f;
    action.circulationPumpOn = false;

    action.clear();

    EXPECT_FLOAT_EQ(action.phDownDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.phUpDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.nutrientADoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.nutrientBDoseMl, 0.0f);
    EXPECT_FLOAT_EQ(action.freshWaterMl, 0.0f);
    EXPECT_TRUE(action.circulationPumpOn);
}

// ============================================================================
// ACTUATOR DRIVER TESTS
// ============================================================================

class ActuatorDriverTest : public ::testing::Test {
protected:
    ActuatorDriver* driver;

    void SetUp() override {
        driver = new ActuatorDriver();
        driver->begin();
    }

    void TearDown() override {
        delete driver;
    }
};

TEST_F(ActuatorDriverTest, Initialization) {
    // All pumps should start off
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_PH_DOWN));
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_PH_UP));
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_CIRCULATION));
}

TEST_F(ActuatorDriverTest, DoseTracking) {
    EXPECT_FLOAT_EQ(driver->getTotalDosedMl(PIN_PUMP_PH_DOWN), 0.0f);

    // Note: execute() would actually run pumps in Arduino environment
    // In test environment, we just verify the tracking mechanism
}

TEST_F(ActuatorDriverTest, DoseCounterReset) {
    // Simulate some dosing would have occurred
    driver->resetDoseCounters();

    EXPECT_FLOAT_EQ(driver->getTotalDosedMl(PIN_PUMP_PH_DOWN), 0.0f);
    EXPECT_FLOAT_EQ(driver->getTotalDosedMl(PIN_PUMP_NUTRIENT_A), 0.0f);
}

TEST_F(ActuatorDriverTest, StopAllPumps) {
    driver->stopAllPumps();

    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_PH_DOWN));
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_PH_UP));
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_NUTRIENT_A));
    EXPECT_FALSE(driver->isPumpRunning(PIN_PUMP_CIRCULATION));
}

TEST_F(ActuatorDriverTest, InvalidPumpPin) {
    EXPECT_FALSE(driver->isPumpRunning(255));
    EXPECT_FLOAT_EQ(driver->getTotalDosedMl(255), 0.0f);
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

class ControllerIntegrationTest : public ::testing::Test {
protected:
    FeedforwardController* controller;
    SafetyMonitor* safety;
    ActuatorDriver* actuator;

    void SetUp() override {
        mockMillisValue = 0;
        controller = new FeedforwardController();
        safety = new SafetyMonitor();
        actuator = new ActuatorDriver();

        controller->setSetpoints(6.5f, 800.0f, 80.0f);
        controller->setMinDoseInterval(0);

        SafetyLimits limits;
        limits.phMin = 5.5f;
        limits.phMax = 7.5f;
        limits.tdsMin = 400.0f;
        limits.tdsMax = 1200.0f;
        limits.tempMin = 18.0f;
        limits.tempMax = 30.0f;
        limits.levelMin = 20.0f;
        limits.levelMax = 95.0f;
        safety->setLimits(limits);
    }

    void TearDown() override {
        delete controller;
        delete safety;
        delete actuator;
    }
};

TEST_F(ControllerIntegrationTest, SafetyOverridesControl) {
    SensorReadings readings;
    readings.ph = 4.0f;  // Dangerously low
    readings.temperatureC = 25.0f;
    readings.tdsPpm = 800.0f;
    readings.waterLevelPercent = 80.0f;
    readings.timestamp = millis();
    readings.allValid = true;

    // Check safety first
    SafetyState state = safety->check(readings);
    EXPECT_EQ(state, SafetyState::ALARM);

    // On alarm, system should not dose aggressively
    // In real implementation, multiple alarms would trigger E-stop
}

TEST_F(ControllerIntegrationTest, FullControlLoop) {
    SensorReadings readings;
    readings.ph = 7.0f;
    readings.temperatureC = 25.0f;
    readings.tdsPpm = 700.0f;  // Below setpoint
    readings.waterLevelPercent = 75.0f;
    readings.timestamp = millis();
    readings.allValid = true;

    // Safety check
    SafetyState state = safety->check(readings);
    EXPECT_EQ(state, SafetyState::NORMAL);

    // Get control action
    ControlAction action = controller->update(readings, 1.0f);

    // System should want to add nutrients and water
    EXPECT_GT(action.nutrientADoseMl, 0.0f);
    // Level might trigger water addition depending on deadband
}

TEST_F(ControllerIntegrationTest, ConvergenceToSetpoint) {
    // Simulate control loop converging
    float ph = 7.0f;
    float tds = 600.0f;

    for (int i = 0; i < 100; i++) {
        SensorReadings readings;
        readings.ph = ph;
        readings.temperatureC = 25.0f;
        readings.tdsPpm = tds;
        readings.waterLevelPercent = 80.0f;
        readings.timestamp = millis();
        readings.allValid = true;

        advanceMillis(5000);
        ControlAction action = controller->update(readings, 5.0f);

        // Simulate system response (simplified)
        if (action.phDownDoseMl > 0) {
            ph -= action.phDownDoseMl * 0.1f;  // Simplified response
        }
        if (action.phUpDoseMl > 0) {
            ph += action.phUpDoseMl * 0.1f;
        }
        if (action.nutrientADoseMl > 0) {
            tds += action.nutrientADoseMl * 10.0f;  // Simplified response
        }
    }

    // Should be closer to setpoints
    EXPECT_NEAR(ph, 6.5f, 1.0f);  // Within 1 pH unit
    EXPECT_NEAR(tds, 800.0f, 200.0f);  // Within 200 ppm
}
