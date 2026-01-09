/**
 * @file config.h
 * @brief System configuration and pin definitions for aquaponics controller
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// MOCK ARDUINO DEFINITIONS FOR NATIVE BUILDS
// ============================================================================

#ifndef ARDUINO
// Mock analog pin definitions for testing
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#endif

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Analog Sensor Pins
#define PIN_PH_SENSOR        A0
#define PIN_PH_TEMP_COMP     A1
#define PIN_TDS_SENSOR       A2
#define PIN_WATER_LEVEL_ANALOG A3

// Digital Sensor Pins
#define PIN_TEMP_ONEWIRE     2
#define PIN_ULTRASONIC_TRIG  3
#define PIN_ULTRASONIC_ECHO  4

// Actuator Pins (PWM capable)
#define PIN_PUMP_PH_DOWN     5
#define PIN_PUMP_PH_UP       6
#define PIN_PUMP_NUTRIENT_A  7
#define PIN_PUMP_NUTRIENT_B  8
#define PIN_VALVE_FRESH_WATER 9
#define PIN_PUMP_CIRCULATION 10

// SPI Pins (SD Card)
#define PIN_SD_CS            53

// I2C Pins (RTC DS3231, LCD)
// SDA = 20, SCL = 21 on Arduino Mega (hardware I2C)
#define PIN_I2C_SDA          20
#define PIN_I2C_SCL          21

// Light Control Pins
#define PIN_LIGHT_ZONE_1     11   // Relay for light zone 1 (veg)
#define PIN_LIGHT_ZONE_2     12   // Relay for light zone 2 (flower)
#define PIN_LIGHT_DIM_1      44   // PWM for dimming zone 1 (optional)
#define PIN_LIGHT_DIM_2      45   // PWM for dimming zone 2 (optional)

// Power Monitoring Pins
#define PIN_CURRENT_SENSOR_1 A4   // ACS712 for zone 1
#define PIN_CURRENT_SENSOR_2 A5   // ACS712 for zone 2

// ============================================================================
// SENSOR CALIBRATION DEFAULTS
// ============================================================================

// pH Sensor Calibration (two-point calibration)
#define PH_CALIBRATION_VOLTAGE_NEUTRAL  1500.0f  // mV at pH 7.0
#define PH_CALIBRATION_VOLTAGE_ACID     2032.0f  // mV at pH 4.0
#define PH_CALIBRATION_SLOPE           -5.70f    // pH units per volt

// TDS Sensor Calibration
#define TDS_CALIBRATION_FACTOR         0.5f      // Adjust based on calibration
#define TDS_TEMPERATURE_COEFFICIENT    0.02f     // 2% per degree C

// Water Level Calibration (ultrasonic)
#define WATER_LEVEL_TANK_HEIGHT_CM     60.0f     // Total tank height
#define WATER_LEVEL_SENSOR_OFFSET_CM   5.0f      // Sensor distance from tank top
#define WATER_LEVEL_MIN_CM             10.0f     // Minimum safe level
#define WATER_LEVEL_MAX_CM             55.0f     // Maximum level

// ============================================================================
// CONTROL SETPOINTS
// ============================================================================

// pH Control
#define PH_SETPOINT           6.5f
#define PH_DEADBAND           0.2f      // No action within +/- deadband
#define PH_MIN_SAFE           5.5f
#define PH_MAX_SAFE           7.5f

// Temperature Control (Celsius)
#define TEMP_SETPOINT         24.0f
#define TEMP_DEADBAND         1.0f
#define TEMP_MIN_SAFE         18.0f
#define TEMP_MAX_SAFE         30.0f

// TDS/EC Control (ppm)
#define TDS_SETPOINT          800.0f
#define TDS_DEADBAND          50.0f
#define TDS_MIN_SAFE          400.0f
#define TDS_MAX_SAFE          1200.0f

// Water Level Control (%)
#define LEVEL_SETPOINT        80.0f
#define LEVEL_DEADBAND        5.0f
#define LEVEL_MIN_SAFE        20.0f
#define LEVEL_MAX_SAFE        95.0f

// ============================================================================
// FEEDFORWARD CONTROLLER PARAMETERS
// ============================================================================

// Dosing pump flow rates (mL/sec at 100% duty)
#define PUMP_PH_DOWN_FLOW_RATE    1.0f
#define PUMP_PH_UP_FLOW_RATE      1.0f
#define PUMP_NUTRIENT_A_FLOW_RATE 2.0f
#define PUMP_NUTRIENT_B_FLOW_RATE 2.0f
#define VALVE_WATER_FLOW_RATE     50.0f  // mL/sec when open

// System volume (liters)
#define SYSTEM_VOLUME_LITERS      100.0f

// Chemical concentrations
#define PH_DOWN_CONCENTRATION     10.0f   // Effective pH units per liter
#define PH_UP_CONCENTRATION       10.0f
#define NUTRIENT_A_TDS_PER_ML     50.0f   // ppm TDS increase per mL
#define NUTRIENT_B_TDS_PER_ML     50.0f

// Feedforward model parameters
#define FF_PH_GAIN               0.8f     // Feedforward gain (0-1, conservative)
#define FF_TDS_GAIN              0.7f
#define FF_LEVEL_GAIN            0.9f

// Rate limiters (prevent overdosing)
#define MAX_PH_DOSE_ML           10.0f    // Max single dose
#define MAX_NUTRIENT_DOSE_ML     20.0f
#define MIN_DOSE_INTERVAL_SEC    300UL    // 5 minutes between doses

// ============================================================================
// TIMING PARAMETERS
// ============================================================================

#define SENSOR_READ_INTERVAL_MS      1000    // 1 second
#define CONTROL_UPDATE_INTERVAL_MS   5000    // 5 seconds
#define LOG_INTERVAL_MS              60000   // 1 minute
#define DISPLAY_UPDATE_INTERVAL_MS   500     // 0.5 seconds

// Sensor averaging
#define PH_SAMPLE_COUNT          10
#define TDS_SAMPLE_COUNT         10
#define LEVEL_SAMPLE_COUNT       5

// ============================================================================
// SAFETY PARAMETERS
// ============================================================================

#define SENSOR_FAULT_THRESHOLD   3        // Consecutive bad reads before fault
#define WATCHDOG_TIMEOUT_MS      8000     // Hardware watchdog
#define EMERGENCY_STOP_ENABLED   true

// ADC parameters
#define ADC_RESOLUTION           1024
#define ADC_VREF                 5.0f

// ============================================================================
// LIGHT CONTROL PARAMETERS
// ============================================================================

#define NUM_LIGHT_ZONES          2
#define LIGHT_CHECK_INTERVAL_MS  1000     // Check schedule every second

// Default schedules (24h format)
#define LIGHT_ZONE1_ON_HOUR      6        // 6 AM
#define LIGHT_ZONE1_OFF_HOUR     24       // Midnight (18h photoperiod - veg)
#define LIGHT_ZONE2_ON_HOUR      6        // 6 AM
#define LIGHT_ZONE2_OFF_HOUR     18       // 6 PM (12h photoperiod - flower)

// Sunrise/sunset simulation
#define SUNRISE_DURATION_MIN     30       // 30 minute ramp up
#define SUNSET_DURATION_MIN      30       // 30 minute ramp down

// ============================================================================
// POWER MONITORING PARAMETERS
// ============================================================================

// ACS712 current sensor (5A version)
#define ACS712_SENSITIVITY       0.185f   // V/A for 5A version (0.185), 20A (0.100), 30A (0.066)
#define ACS712_ZERO_POINT        2.5f     // Voltage at 0 amps (Vcc/2)
#define POWER_VOLTAGE            120.0f   // Mains voltage (120V US, 230V EU)

// Expected power draw per zone (watts) - for verification
#define LIGHT_ZONE1_EXPECTED_WATTS  400.0f
#define LIGHT_ZONE2_EXPECTED_WATTS  600.0f
#define POWER_TOLERANCE_PERCENT     20.0f  // Alert if power differs by more than 20%

#endif // CONFIG_H
