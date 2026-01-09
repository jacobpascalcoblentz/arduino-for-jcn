/**
 * @file controller.cpp
 * @brief Feedforward + Feedback Controller Implementation
 */

#include "controller.h"
#include "config.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
extern uint32_t millis();
extern void setMockMillis(uint32_t ms);
#endif

// ============================================================================
// SYSTEM MODEL IMPLEMENTATION
// ============================================================================

void SystemModel::setDefaults() {
    systemVolumeLiters = SYSTEM_VOLUME_LITERS;
    evaporationRateMlPerHour = 50.0f;  // Depends on environment

    // pH model
    phBufferCapacity = 0.01f;           // Typical for nutrient solution
    phDownStrength = PH_DOWN_CONCENTRATION / systemVolumeLiters;
    phUpStrength = PH_UP_CONCENTRATION / systemVolumeLiters;
    phTimeConstantSec = 60.0f;          // 1 minute mixing

    // TDS model
    nutrientATdsFactor = NUTRIENT_A_TDS_PER_ML;
    nutrientBTdsFactor = NUTRIENT_B_TDS_PER_ML;
    tdsUptakeRatePpmPerHour = 20.0f;    // Depends on plant load
    tdsTimeConstantSec = 30.0f;

    // Level model
    tankAreaCm2 = 2500.0f;              // 50cm x 50cm tank

    // Model confidence
    phModelConfidence = 0.8f;
    tdsModelConfidence = 0.7f;
    levelModelConfidence = 0.9f;
}

void SystemModel::updateFromVolume(float volumeLiters) {
    systemVolumeLiters = volumeLiters;
    phDownStrength = PH_DOWN_CONCENTRATION / volumeLiters;
    phUpStrength = PH_UP_CONCENTRATION / volumeLiters;
}

// ============================================================================
// FEEDFORWARD CONTROLLER IMPLEMENTATION
// ============================================================================

FeedforwardController::FeedforwardController() {
    // Initialize setpoints from config
    phSetpoint_ = PH_SETPOINT;
    tdsSetpoint_ = TDS_SETPOINT;
    levelSetpoint_ = LEVEL_SETPOINT;

    // Initialize deadbands
    phDeadband_ = PH_DEADBAND;
    tdsDeadband_ = TDS_DEADBAND;
    levelDeadband_ = LEVEL_DEADBAND;

    // Initialize system model
    model_.setDefaults();

    // Feedforward gains (conservative)
    ffPhGain_ = FF_PH_GAIN;
    ffTdsGain_ = FF_TDS_GAIN;
    ffLevelGain_ = FF_LEVEL_GAIN;

    // Integral gains (very slow for stability)
    kiPh_ = 0.001f;
    kiTds_ = 0.01f;
    kiLevel_ = 0.05f;

    // Reset integrators
    phIntegral_ = 0.0f;
    tdsIntegral_ = 0.0f;
    levelIntegral_ = 0.0f;

    // Reset errors
    phError_ = 0.0f;
    tdsError_ = 0.0f;
    levelError_ = 0.0f;

    // Deadband flags
    inPhDeadband_ = true;
    inTdsDeadband_ = true;
    inLevelDeadband_ = true;

    // Dose limits
    maxPhDoseMl_ = MAX_PH_DOSE_ML;
    maxNutrientDoseMl_ = MAX_NUTRIENT_DOSE_ML;
    maxWaterMl_ = 1000.0f;

    // Timing
    lastDoseTime_ = 0;
    minDoseIntervalMs_ = MIN_DOSE_INTERVAL_SEC * 1000;

    // Control enables
    feedforwardEnabled_ = true;
    feedbackEnabled_ = true;
    emergencyStop_ = false;

    // Disturbance estimates
    estimatedPhDrift_ = 0.0f;
    estimatedTdsUptake_ = 0.0f;
}

void FeedforwardController::setSetpoints(float ph, float tds, float levelPercent) {
    phSetpoint_ = ph;
    tdsSetpoint_ = tds;
    levelSetpoint_ = levelPercent;
}

void FeedforwardController::setDeadbands(float phDb, float tdsDb, float levelDb) {
    phDeadband_ = phDb;
    tdsDeadband_ = tdsDb;
    levelDeadband_ = levelDb;
}

void FeedforwardController::setSystemModel(const SystemModel& model) {
    model_ = model;
}

void FeedforwardController::setFeedforwardGains(float phGain, float tdsGain, float levelGain) {
    ffPhGain_ = phGain;
    ffTdsGain_ = tdsGain;
    ffLevelGain_ = levelGain;
}

void FeedforwardController::setIntegralGains(float phKi, float tdsKi, float levelKi) {
    kiPh_ = phKi;
    kiTds_ = tdsKi;
    kiLevel_ = levelKi;
}

void FeedforwardController::setDoseLimits(float maxPhDose, float maxNutrientDose, float maxWaterMl) {
    maxPhDoseMl_ = maxPhDose;
    maxNutrientDoseMl_ = maxNutrientDose;
    maxWaterMl_ = maxWaterMl;
}

void FeedforwardController::setMinDoseInterval(uint32_t intervalMs) {
    minDoseIntervalMs_ = intervalMs;
}

ControlAction FeedforwardController::update(const SensorReadings& readings, float dtSeconds) {
    ControlAction action;
    action.clear();
    action.timestamp = readings.timestamp;

    // Emergency stop check
    if (emergencyStop_) {
        action.circulationPumpOn = false;
        return action;
    }

    // Calculate errors
    phError_ = phSetpoint_ - readings.ph;
    tdsError_ = tdsSetpoint_ - readings.tdsPpm;
    levelError_ = levelSetpoint_ - readings.waterLevelPercent;

    // Check deadbands
    inPhDeadband_ = fabs(phError_) < phDeadband_;
    inTdsDeadband_ = fabs(tdsError_) < tdsDeadband_;
    inLevelDeadband_ = fabs(levelError_) < levelDeadband_;

    // Check dose interval
    bool canDose = checkDoseInterval();

    // =========================================================================
    // pH CONTROL
    // =========================================================================
    if (!inPhDeadband_ && canDose) {
        float phDose = 0.0f;

        // Feedforward component
        if (feedforwardEnabled_) {
            phDose += calculatePhFeedforward(phError_);
        }

        // Feedback (integral) component
        if (feedbackEnabled_) {
            phDose += calculatePhFeedback(phError_, dtSeconds);
        }

        // Apply rate limiter
        phDose = applyRateLimiter(phDose, maxPhDoseMl_);

        // Assign to appropriate pump
        if (phDose > 0) {
            action.phDownDoseMl = phDose;  // pH too high, add acid
        } else {
            action.phUpDoseMl = -phDose;   // pH too low, add base
        }
    } else if (inPhDeadband_) {
        // Reset integrator when in deadband to prevent windup
        phIntegral_ *= 0.95f;  // Slow decay
    }

    // =========================================================================
    // TDS/EC CONTROL
    // =========================================================================
    if (!inTdsDeadband_ && canDose) {
        float tdsDose = 0.0f;

        // Feedforward component
        if (feedforwardEnabled_) {
            tdsDose += calculateTdsFeedforward(tdsError_);
        }

        // Feedback component
        if (feedbackEnabled_) {
            tdsDose += calculateTdsFeedback(tdsError_, dtSeconds);
        }

        // Apply rate limiter
        tdsDose = applyRateLimiter(tdsDose, maxNutrientDoseMl_);

        // Only add nutrients, can't remove them (except dilution)
        if (tdsDose > 0 && tdsError_ > 0) {
            // TDS too low, need to add nutrients
            action.nutrientADoseMl = tdsDose / 2.0f;
            action.nutrientBDoseMl = tdsDose / 2.0f;
        } else if (tdsError_ < -tdsDeadband_ * 2) {
            // TDS too high, need dilution - handled by level control adding fresh water
            // Set flag for level controller to prefer fresh water
        }
    } else if (inTdsDeadband_) {
        tdsIntegral_ *= 0.95f;
    }

    // =========================================================================
    // WATER LEVEL CONTROL
    // =========================================================================
    if (!inLevelDeadband_ && canDose) {
        float levelDose = 0.0f;

        // Feedforward component
        if (feedforwardEnabled_) {
            levelDose += calculateLevelFeedforward(levelError_);
        }

        // Feedback component
        if (feedbackEnabled_) {
            levelDose += calculateLevelFeedback(levelError_, dtSeconds);
        }

        // Apply rate limiter
        levelDose = applyRateLimiter(levelDose, maxWaterMl_);

        // Only add water, can't remove (evaporation will reduce)
        if (levelDose > 0) {
            action.freshWaterMl = levelDose;
        }
    } else if (inLevelDeadband_) {
        levelIntegral_ *= 0.95f;
    }

    // Update dose timestamp if any dosing occurred
    if (action.phDownDoseMl > 0 || action.phUpDoseMl > 0 ||
        action.nutrientADoseMl > 0 || action.freshWaterMl > 0) {
        lastDoseTime_ = millis();
    }

    // Circulation pump always on unless emergency
    action.circulationPumpOn = true;

    return action;
}

float FeedforwardController::calculatePhFeedforward(float error) {
    // Inverse model: dose = error * volume / concentration
    // pH change = dose * concentration / (volume * buffer_capacity)
    // dose = error * volume * buffer_capacity / concentration

    float dose;
    if (error > 0) {
        // pH too low, need to raise (add base)
        dose = -error * model_.systemVolumeLiters / model_.phUpStrength;
    } else {
        // pH too high, need to lower (add acid)
        dose = -error * model_.systemVolumeLiters / model_.phDownStrength;
    }

    // Apply feedforward gain (conservative)
    dose *= ffPhGain_ * model_.phModelConfidence;

    // Add disturbance compensation
    // If pH is drifting, pre-compensate
    float driftCompensation = estimatedPhDrift_ * (model_.phTimeConstantSec / 3600.0f);
    dose += driftCompensation * model_.systemVolumeLiters / model_.phDownStrength;

    return dose;
}

float FeedforwardController::calculateTdsFeedforward(float error) {
    // dose = error * volume / tds_per_ml

    float dose = 0.0f;

    if (error > 0) {
        // TDS too low, need to add nutrients
        float avgTdsFactor = (model_.nutrientATdsFactor + model_.nutrientBTdsFactor) / 2.0f;
        dose = error * model_.systemVolumeLiters / (avgTdsFactor * 1000.0f);  // Convert L to mL
    }

    // Apply feedforward gain
    dose *= ffTdsGain_ * model_.tdsModelConfidence;

    // Add uptake compensation
    float uptakeCompensation = estimatedTdsUptake_ * (model_.tdsTimeConstantSec / 3600.0f);
    float avgTdsFactor = (model_.nutrientATdsFactor + model_.nutrientBTdsFactor) / 2.0f;
    dose += uptakeCompensation * model_.systemVolumeLiters / (avgTdsFactor * 1000.0f);

    return dose;
}

float FeedforwardController::calculateLevelFeedforward(float error) {
    // Convert percent error to volume
    // volume = error% * tank_area * (max_level - min_level) / 100

    float levelRange = WATER_LEVEL_MAX_CM - WATER_LEVEL_MIN_CM;
    float volumeMl = error * model_.tankAreaCm2 * levelRange / 100.0f;

    // Apply feedforward gain
    volumeMl *= ffLevelGain_ * model_.levelModelConfidence;

    // Add evaporation compensation
    float evapCompensation = model_.evaporationRateMlPerHour * (model_.tdsTimeConstantSec / 3600.0f);
    volumeMl += evapCompensation;

    return volumeMl;
}

float FeedforwardController::calculatePhFeedback(float error, float dt) {
    // Only accumulate integral outside deadband
    updateIntegrator(phIntegral_, error, dt, kiPh_, maxPhDoseMl_);
    return phIntegral_;
}

float FeedforwardController::calculateTdsFeedback(float error, float dt) {
    updateIntegrator(tdsIntegral_, error, dt, kiTds_, maxNutrientDoseMl_);
    return tdsIntegral_;
}

float FeedforwardController::calculateLevelFeedback(float error, float dt) {
    updateIntegrator(levelIntegral_, error, dt, kiLevel_, maxWaterMl_);
    return levelIntegral_;
}

void FeedforwardController::updateIntegrator(float& integrator, float error, float dt, float ki, float limit) {
    integrator += error * ki * dt;
    antiWindup(integrator, limit);
}

void FeedforwardController::antiWindup(float& integrator, float limit) {
    if (integrator > limit) {
        integrator = limit;
    } else if (integrator < -limit) {
        integrator = -limit;
    }
}

float FeedforwardController::applyRateLimiter(float dose, float maxDose) {
    if (dose > maxDose) return maxDose;
    if (dose < -maxDose) return -maxDose;
    return dose;
}

bool FeedforwardController::checkDoseInterval() {
    uint32_t now = millis();
    return (now - lastDoseTime_) >= minDoseIntervalMs_;
}

void FeedforwardController::resetIntegrators() {
    phIntegral_ = 0.0f;
    tdsIntegral_ = 0.0f;
    levelIntegral_ = 0.0f;
}

void FeedforwardController::updateDisturbanceEstimates(const SensorReadings& prev,
                                                        const SensorReadings& curr,
                                                        float dt) {
    if (dt <= 0) return;

    // Estimate pH drift rate (pH/hour)
    float phChange = curr.ph - prev.ph;
    float hoursElapsed = dt / 3600.0f;
    float instantDrift = phChange / hoursElapsed;

    // Low-pass filter the estimate
    estimatedPhDrift_ = 0.9f * estimatedPhDrift_ + 0.1f * instantDrift;

    // Estimate TDS uptake rate (ppm/hour)
    float tdsChange = prev.tdsPpm - curr.tdsPpm;  // Positive = uptake
    float instantUptake = tdsChange / hoursElapsed;

    // Low-pass filter
    estimatedTdsUptake_ = 0.9f * estimatedTdsUptake_ + 0.1f * instantUptake;
}

// ============================================================================
// ACTUATOR DRIVER IMPLEMENTATION
// ============================================================================

ActuatorDriver::ActuatorDriver() {
    // Set default flow rates (mL/sec)
    pumpFlowRates_[0] = PUMP_PH_DOWN_FLOW_RATE;
    pumpFlowRates_[1] = PUMP_PH_UP_FLOW_RATE;
    pumpFlowRates_[2] = PUMP_NUTRIENT_A_FLOW_RATE;
    pumpFlowRates_[3] = PUMP_NUTRIENT_B_FLOW_RATE;
    pumpFlowRates_[4] = VALVE_WATER_FLOW_RATE;
    pumpFlowRates_[5] = 0.0f;  // Circulation pump (not a dosing pump)

    for (int i = 0; i < 6; i++) {
        totalDosed_[i] = 0.0f;
        pumpStates_[i] = false;
    }
}

void ActuatorDriver::begin() {
#ifdef ARDUINO
    pinMode(PIN_PUMP_PH_DOWN, OUTPUT);
    pinMode(PIN_PUMP_PH_UP, OUTPUT);
    pinMode(PIN_PUMP_NUTRIENT_A, OUTPUT);
    pinMode(PIN_PUMP_NUTRIENT_B, OUTPUT);
    pinMode(PIN_VALVE_FRESH_WATER, OUTPUT);
    pinMode(PIN_PUMP_CIRCULATION, OUTPUT);

    stopAllPumps();
#endif
}

void ActuatorDriver::execute(const ControlAction& action) {
    if (action.phDownDoseMl > 0.01f) {
        doseVolume(PIN_PUMP_PH_DOWN, action.phDownDoseMl, pumpFlowRates_[0]);
        totalDosed_[0] += action.phDownDoseMl;
    }

    if (action.phUpDoseMl > 0.01f) {
        doseVolume(PIN_PUMP_PH_UP, action.phUpDoseMl, pumpFlowRates_[1]);
        totalDosed_[1] += action.phUpDoseMl;
    }

    if (action.nutrientADoseMl > 0.01f) {
        doseVolume(PIN_PUMP_NUTRIENT_A, action.nutrientADoseMl, pumpFlowRates_[2]);
        totalDosed_[2] += action.nutrientADoseMl;
    }

    if (action.nutrientBDoseMl > 0.01f) {
        doseVolume(PIN_PUMP_NUTRIENT_B, action.nutrientBDoseMl, pumpFlowRates_[3]);
        totalDosed_[3] += action.nutrientBDoseMl;
    }

    if (action.freshWaterMl > 0.01f) {
        doseVolume(PIN_VALVE_FRESH_WATER, action.freshWaterMl, pumpFlowRates_[4]);
        totalDosed_[4] += action.freshWaterMl;
    }

#ifdef ARDUINO
    digitalWrite(PIN_PUMP_CIRCULATION, action.circulationPumpOn ? HIGH : LOW);
#endif
    pumpStates_[5] = action.circulationPumpOn;
}

void ActuatorDriver::doseVolume(uint8_t pumpPin, float volumeMl, float flowRate) {
    if (flowRate <= 0 || volumeMl <= 0) return;

    float durationSec = volumeMl / flowRate;

#ifdef ARDUINO
    digitalWrite(pumpPin, HIGH);
    delay((unsigned long)(durationSec * 1000));
    digitalWrite(pumpPin, LOW);
#endif
}

void ActuatorDriver::runPump(uint8_t pumpPin, float durationSec) {
#ifdef ARDUINO
    digitalWrite(pumpPin, HIGH);
    delay((unsigned long)(durationSec * 1000));
    digitalWrite(pumpPin, LOW);
#endif
}

void ActuatorDriver::stopAllPumps() {
#ifdef ARDUINO
    digitalWrite(PIN_PUMP_PH_DOWN, LOW);
    digitalWrite(PIN_PUMP_PH_UP, LOW);
    digitalWrite(PIN_PUMP_NUTRIENT_A, LOW);
    digitalWrite(PIN_PUMP_NUTRIENT_B, LOW);
    digitalWrite(PIN_VALVE_FRESH_WATER, LOW);
    digitalWrite(PIN_PUMP_CIRCULATION, LOW);
#endif

    for (int i = 0; i < 6; i++) {
        pumpStates_[i] = false;
    }
}

void ActuatorDriver::setCirculationSpeed(uint8_t speed) {
#ifdef ARDUINO
    analogWrite(PIN_PUMP_CIRCULATION, speed);
#endif
}

bool ActuatorDriver::isPumpRunning(uint8_t pumpPin) const {
    // Map pin to index
    int idx = -1;
    if (pumpPin == PIN_PUMP_PH_DOWN) idx = 0;
    else if (pumpPin == PIN_PUMP_PH_UP) idx = 1;
    else if (pumpPin == PIN_PUMP_NUTRIENT_A) idx = 2;
    else if (pumpPin == PIN_PUMP_NUTRIENT_B) idx = 3;
    else if (pumpPin == PIN_VALVE_FRESH_WATER) idx = 4;
    else if (pumpPin == PIN_PUMP_CIRCULATION) idx = 5;

    if (idx >= 0) return pumpStates_[idx];
    return false;
}

float ActuatorDriver::getTotalDosedMl(uint8_t pumpPin) const {
    int idx = -1;
    if (pumpPin == PIN_PUMP_PH_DOWN) idx = 0;
    else if (pumpPin == PIN_PUMP_PH_UP) idx = 1;
    else if (pumpPin == PIN_PUMP_NUTRIENT_A) idx = 2;
    else if (pumpPin == PIN_PUMP_NUTRIENT_B) idx = 3;
    else if (pumpPin == PIN_VALVE_FRESH_WATER) idx = 4;

    if (idx >= 0) return totalDosed_[idx];
    return 0.0f;
}

void ActuatorDriver::resetDoseCounters() {
    for (int i = 0; i < 6; i++) {
        totalDosed_[i] = 0.0f;
    }
}

// ============================================================================
// SAFETY MONITOR IMPLEMENTATION
// ============================================================================

SafetyMonitor::SafetyMonitor() {
    limits_.phMin = PH_MIN_SAFE;
    limits_.phMax = PH_MAX_SAFE;
    limits_.tdsMin = TDS_MIN_SAFE;
    limits_.tdsMax = TDS_MAX_SAFE;
    limits_.tempMin = TEMP_MIN_SAFE;
    limits_.tempMax = TEMP_MAX_SAFE;
    limits_.levelMin = LEVEL_MIN_SAFE;
    limits_.levelMax = LEVEL_MAX_SAFE;

    state_ = SafetyState::NORMAL;
    alarmReason_[0] = '\0';
    consecutiveAlarms_ = 0;
}

void SafetyMonitor::setLimits(const SafetyLimits& limits) {
    limits_ = limits;
}

SafetyState SafetyMonitor::check(const SensorReadings& readings) {
    SafetyState newState = SafetyState::NORMAL;
    alarmReason_[0] = '\0';

    // Check pH
    if (readings.ph < limits_.phMin) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "pH LOW");
    } else if (readings.ph > limits_.phMax) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "pH HIGH");
    } else if (readings.ph < limits_.phMin + 0.5f || readings.ph > limits_.phMax - 0.5f) {
        if (newState < SafetyState::WARNING) {
            newState = SafetyState::WARNING;
            strcpy(alarmReason_, "pH WARN");
        }
    }

    // Check TDS
    if (readings.tdsPpm < limits_.tdsMin) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "TDS LOW");
    } else if (readings.tdsPpm > limits_.tdsMax) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "TDS HIGH");
    }

    // Check temperature
    if (readings.temperatureC < limits_.tempMin || readings.temperatureC > limits_.tempMax) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "TEMP OUT");
    }

    // Check water level
    if (readings.waterLevelPercent < limits_.levelMin) {
        newState = SafetyState::ALARM;
        strcpy(alarmReason_, "LEVEL LOW");
    } else if (readings.waterLevelPercent > limits_.levelMax) {
        if (newState < SafetyState::WARNING) {
            newState = SafetyState::WARNING;
            strcpy(alarmReason_, "LEVEL HIGH");
        }
    }

    // Track consecutive alarms for emergency stop
    if (newState >= SafetyState::ALARM) {
        consecutiveAlarms_++;
        if (consecutiveAlarms_ >= SENSOR_FAULT_THRESHOLD) {
            newState = SafetyState::EMERGENCY_STOP;
        }
    } else {
        consecutiveAlarms_ = 0;
    }

    state_ = newState;
    return state_;
}

const char* SafetyMonitor::getStateString() const {
    switch (state_) {
        case SafetyState::NORMAL: return "NORMAL";
        case SafetyState::WARNING: return "WARNING";
        case SafetyState::ALARM: return "ALARM";
        case SafetyState::EMERGENCY_STOP: return "E-STOP";
        default: return "UNKNOWN";
    }
}

void SafetyMonitor::acknowledgeAlarm() {
    if (state_ == SafetyState::ALARM) {
        state_ = SafetyState::WARNING;
    }
    consecutiveAlarms_ = 0;
}

void SafetyMonitor::reset() {
    state_ = SafetyState::NORMAL;
    alarmReason_[0] = '\0';
    consecutiveAlarms_ = 0;
}
