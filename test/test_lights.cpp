/**
 * @file test_lights.cpp
 * @brief Unit tests for light controller and RTC
 */

#include <gtest/gtest.h>
#include "../src/rtc.h"
#include "../src/lights.h"
#include "../src/power_monitor.h"

// ============================================================================
// RTC TESTS
// ============================================================================

class RtcTest : public ::testing::Test {
protected:
    RealTimeClock rtc;

    void SetUp() override {
        rtc.begin();
    }
};

TEST_F(RtcTest, InjectTimeWorks) {
    rtc.injectTime(14, 30, 45);
    TimeInfo t = rtc.getTime();

    EXPECT_EQ(t.hour, 14);
    EXPECT_EQ(t.minute, 30);
    EXPECT_EQ(t.second, 45);
}

TEST_F(RtcTest, MinutesSinceMidnight) {
    rtc.injectTime(6, 0, 0);
    TimeInfo t = rtc.getTime();

    EXPECT_EQ(t.minutesSinceMidnight(), 360);  // 6 * 60
}

TEST_F(RtcTest, SecondsSinceMidnight) {
    rtc.injectTime(1, 30, 45);
    TimeInfo t = rtc.getTime();

    EXPECT_EQ(t.secondsSinceMidnight(), 5445);  // 1*3600 + 30*60 + 45
}

TEST_F(RtcTest, ClearInjectedTime) {
    rtc.injectTime(12, 0, 0);
    rtc.clearInjectedTime();

    // After clearing, should return default time (0:0:0 in test mode)
    TimeInfo t = rtc.getTime();
    EXPECT_EQ(t.hour, 0);
}

// ============================================================================
// LIGHT SCHEDULE TESTS
// ============================================================================

class LightScheduleTest : public ::testing::Test {
protected:
    RealTimeClock rtc;
    LightController* lights;

    void SetUp() override {
        rtc.begin();
        lights = new LightController(&rtc);
        lights->begin(11, 12, 255, 255);  // No dimming
    }

    void TearDown() override {
        delete lights;
    }
};

TEST_F(LightScheduleTest, DefaultScheduleZone1Is18_6) {
    LightSchedule s = lights->getSchedule(0);

    EXPECT_EQ(s.onHour, 6);
    EXPECT_EQ(s.offHour, 24);  // Midnight
    EXPECT_FLOAT_EQ(s.getPhotoperiod(), 18.0f);
}

TEST_F(LightScheduleTest, DefaultScheduleZone2Is12_12) {
    LightSchedule s = lights->getSchedule(1);

    EXPECT_EQ(s.onHour, 6);
    EXPECT_EQ(s.offHour, 18);
    EXPECT_FLOAT_EQ(s.getPhotoperiod(), 12.0f);
}

TEST_F(LightScheduleTest, LightOnDuringPhotoperiod) {
    // Set time to middle of light period (12:00)
    rtc.injectTime(12, 0, 0);
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::ON);
    EXPECT_TRUE(status.relayOn);
    EXPECT_EQ(status.brightness, 255);
}

TEST_F(LightScheduleTest, LightOffBeforeOnTime) {
    // Set time to before lights on (5:00)
    rtc.injectTime(5, 0, 0);
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::OFF);
    EXPECT_FALSE(status.relayOn);
    EXPECT_EQ(status.brightness, 0);
}

TEST_F(LightScheduleTest, SunriseStateAtOnTime) {
    // Set custom schedule with sunrise
    LightSchedule s = {
        .onHour = 6, .onMinute = 0,
        .offHour = 18, .offMinute = 0,
        .sunriseMins = 30, .sunsetMins = 30,
        .enabled = true
    };
    lights->setSchedule(0, s);

    // Set time to 15 minutes after on time (in sunrise)
    rtc.injectTime(6, 15, 0);
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::SUNRISE);
    EXPECT_TRUE(status.relayOn);
    // Brightness should be ~50% (15/30 minutes)
    EXPECT_GT(status.brightness, 100);
    EXPECT_LT(status.brightness, 200);
}

TEST_F(LightScheduleTest, SunsetStateBeforeOffTime) {
    LightSchedule s = {
        .onHour = 6, .onMinute = 0,
        .offHour = 18, .offMinute = 0,
        .sunriseMins = 30, .sunsetMins = 30,
        .enabled = true
    };
    lights->setSchedule(0, s);

    // Set time to 15 minutes before off time (in sunset)
    rtc.injectTime(17, 45, 0);
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::SUNSET);
    EXPECT_TRUE(status.relayOn);
    // Brightness should be ~50%
    EXPECT_GT(status.brightness, 100);
    EXPECT_LT(status.brightness, 200);
}

TEST_F(LightScheduleTest, ManualOverrideOn) {
    rtc.injectTime(3, 0, 0);  // Night time
    lights->update();

    // Should be off
    EXPECT_EQ(lights->getStatus(0).state, LightState::OFF);

    // Manual override on
    lights->setManualOverride(0, true);

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::MANUAL_ON);
    EXPECT_TRUE(status.relayOn);
    EXPECT_EQ(status.brightness, 255);
}

TEST_F(LightScheduleTest, ManualOverrideOff) {
    rtc.injectTime(12, 0, 0);  // Day time
    lights->update();

    // Should be on
    EXPECT_EQ(lights->getStatus(0).state, LightState::ON);

    // Manual override off
    lights->setManualOverride(0, false);

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::MANUAL_OFF);
    EXPECT_FALSE(status.relayOn);
    EXPECT_EQ(status.brightness, 0);
}

TEST_F(LightScheduleTest, ClearManualOverride) {
    rtc.injectTime(12, 0, 0);
    lights->setManualOverride(0, false);  // Force off

    EXPECT_TRUE(lights->isManualOverride(0));

    lights->clearManualOverride(0);
    lights->update();

    // Should return to scheduled state (ON at noon)
    EXPECT_FALSE(lights->isManualOverride(0));
    EXPECT_EQ(lights->getStatus(0).state, LightState::ON);
}

TEST_F(LightScheduleTest, DisabledZoneStaysOff) {
    lights->setZoneEnabled(0, false);

    rtc.injectTime(12, 0, 0);  // Day time
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.state, LightState::OFF);
    EXPECT_FALSE(status.relayOn);
}

TEST_F(LightScheduleTest, MinutesUntilChange) {
    LightSchedule s = {
        .onHour = 6, .onMinute = 0,
        .offHour = 18, .offMinute = 0,
        .sunriseMins = 0, .sunsetMins = 0,  // No sunrise/sunset
        .enabled = true
    };
    lights->setSchedule(0, s);

    // At 12:00, should be ON, 6 hours until off
    rtc.injectTime(12, 0, 0);
    lights->update();

    LightZoneStatus status = lights->getStatus(0);
    EXPECT_EQ(status.minutesUntilChange, 360);  // 6 hours
}

// ============================================================================
// PRESET TESTS
// ============================================================================

TEST(LightPresetsTest, VegSchedule) {
    EXPECT_EQ(LightPresets::VEG_18_6.onHour, 6);
    EXPECT_EQ(LightPresets::VEG_18_6.offHour, 24);
    EXPECT_FLOAT_EQ(LightPresets::VEG_18_6.getPhotoperiod(), 18.0f);
}

TEST(LightPresetsTest, FlowerSchedule) {
    EXPECT_EQ(LightPresets::FLOWER_12_12.onHour, 6);
    EXPECT_EQ(LightPresets::FLOWER_12_12.offHour, 18);
    EXPECT_FLOAT_EQ(LightPresets::FLOWER_12_12.getPhotoperiod(), 12.0f);
}

TEST(LightPresetsTest, SeedlingSchedule) {
    EXPECT_FLOAT_EQ(LightPresets::SEEDLING_16_8.getPhotoperiod(), 16.0f);
}

TEST(LightPresetsTest, LettuceSchedule) {
    EXPECT_FLOAT_EQ(LightPresets::LETTUCE_14_10.getPhotoperiod(), 14.0f);
}

// ============================================================================
// POWER MONITOR TESTS
// ============================================================================

class PowerMonitorTest : public ::testing::Test {
protected:
    PowerMonitor monitor;

    PowerMonitorTest() : monitor(2) {}

    void SetUp() override {
        monitor.begin(0, A4, ACS712Variant::ACS712_5A, 120.0f);
        monitor.begin(1, A5, ACS712Variant::ACS712_5A, 120.0f);
    }
};

TEST_F(PowerMonitorTest, InjectReadingWorks) {
    monitor.injectReading(0, 3.5f);  // 3.5 amps
    PowerReading r = monitor.read(0);

    EXPECT_TRUE(r.valid);
    EXPECT_FLOAT_EQ(r.currentAmps, 3.5f);
    EXPECT_FLOAT_EQ(r.powerWatts, 420.0f);  // 3.5 * 120
}

TEST_F(PowerMonitorTest, LightDetectionThreshold) {
    monitor.setDetectionThreshold(0, 0.5f);  // 0.5A threshold

    monitor.injectReading(0, 0.3f);
    monitor.read(0);
    EXPECT_FALSE(monitor.isLightOn(0));

    monitor.injectReading(0, 1.0f);
    monitor.read(0);
    EXPECT_TRUE(monitor.isLightOn(0));
}

TEST_F(PowerMonitorTest, TotalPower) {
    monitor.injectReading(0, 2.0f);  // 240W
    monitor.injectReading(1, 3.0f);  // 360W
    monitor.readAll();

    EXPECT_FLOAT_EQ(monitor.getTotalPower(), 600.0f);
}

TEST_F(PowerMonitorTest, IsAnyLightOn) {
    monitor.setDetectionThreshold(0, 0.5f);
    monitor.setDetectionThreshold(1, 0.5f);

    monitor.injectReading(0, 0.1f);
    monitor.injectReading(1, 0.1f);
    monitor.readAll();
    EXPECT_FALSE(monitor.isAnyLightOn());

    monitor.injectReading(1, 2.0f);  // Turn on zone 2
    monitor.readAll();
    EXPECT_TRUE(monitor.isAnyLightOn());
}

// ============================================================================
// LIGHT POWER VERIFIER TESTS
// ============================================================================

class LightPowerVerifierTest : public ::testing::Test {
protected:
    LightPowerVerifier verifier;

    void SetUp() override {
        verifier.begin(A4, A5);
    }
};

TEST(LightPowerVerifierTest, VerifyMatchesExpected) {
    // This test would need injection support in LightPowerVerifier
    // For now, just verify the class compiles and initializes
    LightPowerVerifier v;
    EXPECT_TRUE(v.begin(A4, A5));
}

// ============================================================================
// INTEGRATION TESTS
// ============================================================================

class LightPowerIntegrationTest : public ::testing::Test {
protected:
    RealTimeClock rtc;
    LightController* lights;
    PowerMonitor monitor;

    LightPowerIntegrationTest() : monitor(2) {}

    void SetUp() override {
        rtc.begin();
        lights = new LightController(&rtc);
        lights->begin(11, 12);

        monitor.begin(0, A4, ACS712Variant::ACS712_5A, 120.0f);
        monitor.begin(1, A5, ACS712Variant::ACS712_5A, 120.0f);

        // Set expected power
        lights->setExpectedPower(0, 400.0f, 20.0f);
        lights->setExpectedPower(1, 600.0f, 20.0f);
    }

    void TearDown() override {
        delete lights;
    }
};

TEST_F(LightPowerIntegrationTest, PowerVerificationWhenOn) {
    rtc.injectTime(12, 0, 0);  // Lights should be on
    lights->update();

    // Simulate power reading
    monitor.injectReading(0, 3.5f);  // ~420W
    monitor.read(0);

    // Report power to light controller
    lights->reportPower(0, monitor.getReading(0).powerWatts);
    lights->update();

    // Power should be verified (420W is within 20% of 400W)
    EXPECT_TRUE(lights->getStatus(0).powerVerified);
}

TEST_F(LightPowerIntegrationTest, PowerAnomalyDetection) {
    rtc.injectTime(12, 0, 0);  // Lights should be on
    lights->update();

    // Simulate low power (bulb burned out)
    lights->reportPower(0, 50.0f);  // Only 50W
    lights->update();

    // Should detect anomaly
    EXPECT_FALSE(lights->getStatus(0).powerVerified);
    EXPECT_TRUE(lights->hasPowerAnomaly());
}
