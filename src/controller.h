/**
 * @file controller.h
 * @brief Advanced Feedforward + Feedback Controller for aquaponics system
 *
 * This controller implements a model-based feedforward control strategy
 * combined with integral feedback for steady-state error correction.
 * The feedforward component predicts the control action needed based on
 * system dynamics, while the feedback corrects for model errors.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

// ============================================================================
// CONTROL ACTION STRUCTURE
// ============================================================================

struct ControlAction {
    float phDownDoseMl;      // Acid to add (mL)
    float phUpDoseMl;        // Base to add (mL)
    float nutrientADoseMl;   // Nutrient A to add (mL)
    float nutrientBDoseMl;   // Nutrient B to add (mL)
    float freshWaterMl;      // Fresh water to add (mL)
    bool circulationPumpOn;  // Main pump state
    uint32_t timestamp;

    void clear() {
        phDownDoseMl = 0.0f;
        phUpDoseMl = 0.0f;
        nutrientADoseMl = 0.0f;
        nutrientBDoseMl = 0.0f;
        freshWaterMl = 0.0f;
        circulationPumpOn = true;
        timestamp = 0;
    }
};

// ============================================================================
// SYSTEM MODEL FOR FEEDFORWARD CONTROL
// ============================================================================

/**
 * @brief System dynamics model for feedforward predictions
 *
 * Models the chemical/physical relationships in the system:
 * - pH response to acid/base addition
 * - TDS response to nutrient addition
 * - Volume response to water addition
 * - First-order dynamics with time constants
 */
struct SystemModel {
    // Volume parameters
    float systemVolumeLiters;        // Current system volume
    float evaporationRateMlPerHour;  // Estimated evaporation

    // pH dynamics
    float phBufferCapacity;          // mol/L/pH unit (higher = more resistant to change)
    float phDownStrength;            // Effective pH change per mL at current volume
    float phUpStrength;              // Effective pH change per mL at current volume
    float phTimeConstantSec;         // First-order response time

    // TDS/EC dynamics
    float nutrientATdsFactor;        // ppm increase per mL
    float nutrientBTdsFactor;        // ppm increase per mL
    float tdsUptakeRatePpmPerHour;   // Plant nutrient uptake
    float tdsTimeConstantSec;        // Mixing time constant

    // Level dynamics
    float tankAreaCm2;               // Tank cross-sectional area

    // Model uncertainty factors (0-1, 1 = perfect model)
    float phModelConfidence;
    float tdsModelConfidence;
    float levelModelConfidence;

    void setDefaults();
    void updateFromVolume(float volumeLiters);
};

// ============================================================================
// FEEDFORWARD CONTROLLER
// ============================================================================

/**
 * @brief Feedforward controller with integral feedback correction
 *
 * Control law: u = u_ff + u_fb
 *
 * Feedforward (u_ff): Based on inverse system model
 *   - Calculates exact dose needed to achieve setpoint
 *   - Accounts for known disturbances (evaporation, uptake)
 *
 * Feedback (u_fb): Integral action for steady-state correction
 *   - Corrects for model errors
 *   - Slow integral to avoid oscillation
 */
class FeedforwardController {
public:
    FeedforwardController();

    // Configuration
    void setSetpoints(float ph, float tds, float levelPercent);
    void setDeadbands(float phDb, float tdsDb, float levelDb);
    void setSystemModel(const SystemModel& model);
    void setFeedforwardGains(float phGain, float tdsGain, float levelGain);
    void setIntegralGains(float phKi, float tdsKi, float levelKi);
    void setDoseLimits(float maxPhDose, float maxNutrientDose, float maxWaterMl);
    void setMinDoseInterval(uint32_t intervalMs);

    // Main control update
    ControlAction update(const SensorReadings& readings, float dtSeconds);

    // Manual overrides
    void resetIntegrators();
    void enableFeedforward(bool enable) { feedforwardEnabled_ = enable; }
    void enableFeedback(bool enable) { feedbackEnabled_ = enable; }
    void setEmergencyStop(bool stop) { emergencyStop_ = stop; }

    // State inspection
    float getPhError() const { return phError_; }
    float getTdsError() const { return tdsError_; }
    float getLevelError() const { return levelError_; }
    float getPhIntegral() const { return phIntegral_; }
    float getTdsIntegral() const { return tdsIntegral_; }
    float getLevelIntegral() const { return levelIntegral_; }
    bool isInDeadband() const { return inPhDeadband_ && inTdsDeadband_ && inLevelDeadband_; }

    // Disturbance estimation (for adaptive feedforward)
    void updateDisturbanceEstimates(const SensorReadings& prev, const SensorReadings& curr, float dt);
    float getEstimatedPhDrift() const { return estimatedPhDrift_; }
    float getEstimatedTdsUptake() const { return estimatedTdsUptake_; }

private:
    // Setpoints
    float phSetpoint_;
    float tdsSetpoint_;
    float levelSetpoint_;

    // Deadbands
    float phDeadband_;
    float tdsDeadband_;
    float levelDeadband_;

    // System model
    SystemModel model_;

    // Feedforward gains (0-1, conservative tuning)
    float ffPhGain_;
    float ffTdsGain_;
    float ffLevelGain_;

    // Integral gains (small for slow correction)
    float kiPh_;
    float kiTds_;
    float kiLevel_;

    // Integrator states
    float phIntegral_;
    float tdsIntegral_;
    float levelIntegral_;

    // Error states
    float phError_;
    float tdsError_;
    float levelError_;

    // Deadband flags
    bool inPhDeadband_;
    bool inTdsDeadband_;
    bool inLevelDeadband_;

    // Dose limits
    float maxPhDoseMl_;
    float maxNutrientDoseMl_;
    float maxWaterMl_;

    // Timing
    uint32_t lastDoseTime_;
    uint32_t minDoseIntervalMs_;

    // Control enables
    bool feedforwardEnabled_;
    bool feedbackEnabled_;
    bool emergencyStop_;

    // Disturbance estimates (adaptive feedforward)
    float estimatedPhDrift_;      // pH/hour from biological activity
    float estimatedTdsUptake_;    // ppm/hour plant uptake

    // Internal methods
    float calculatePhFeedforward(float error);
    float calculateTdsFeedforward(float error);
    float calculateLevelFeedforward(float error);

    float calculatePhFeedback(float error, float dt);
    float calculateTdsFeedback(float error, float dt);
    float calculateLevelFeedback(float error, float dt);

    float applyRateLimiter(float dose, float maxDose);
    bool checkDoseInterval();

    void updateIntegrator(float& integrator, float error, float dt, float ki, float limit);
    void antiWindup(float& integrator, float limit);
};

// ============================================================================
// ACTUATOR DRIVER
// ============================================================================

/**
 * @brief Non-blocking pump state for timer-based control
 */
struct PumpTimer {
    uint8_t pin;
    uint32_t startTime;
    uint32_t durationMs;
    bool active;
    float volumeMl;  // For tracking total dosed

    void clear() {
        pin = 0;
        startTime = 0;
        durationMs = 0;
        active = false;
        volumeMl = 0.0f;
    }
};

/**
 * @brief Converts control actions to hardware outputs (non-blocking)
 *
 * Uses timer-based approach instead of blocking delay() calls.
 * Call update() frequently from main loop to process pump timers.
 */
class ActuatorDriver {
public:
    ActuatorDriver();

    void begin();

    // IMPORTANT: Call this frequently from main loop!
    // Processes pump timers and turns off pumps when done
    void update();

    // Execute control action (non-blocking - schedules pump runs)
    void execute(const ControlAction& action);

    // Direct pump control (for manual/calibration)
    // Non-blocking - schedules the pump to run
    void runPump(uint8_t pumpPin, float durationSec);

    // Immediately stop all pumps
    void stopAllPumps();

    // PWM speed control for main pump
    void setCirculationSpeed(uint8_t speed);  // 0-255

    // Status
    bool isPumpRunning(uint8_t pumpPin) const;
    bool isAnyPumpRunning() const;
    bool isDosingInProgress() const;
    float getTotalDosedMl(uint8_t pumpPin) const;
    void resetDoseCounters();

    // Get remaining time for active pump (ms)
    uint32_t getRemainingTime(uint8_t pumpPin) const;

private:
    static const int NUM_PUMPS = 6;

    float pumpFlowRates_[NUM_PUMPS];    // mL/sec for each pump
    float totalDosed_[NUM_PUMPS];       // Running total
    PumpTimer pumpTimers_[NUM_PUMPS];   // Non-blocking timers

    // Map pin to index
    int pinToIndex(uint8_t pin) const;
    uint8_t indexToPin(int index) const;

    // Internal: start pump with timer (non-blocking)
    void startPumpTimer(uint8_t pumpPin, float durationSec, float volumeMl);

    // Internal: check and process expired timers
    void processTimers();
};

// ============================================================================
// SAFETY MONITOR
// ============================================================================

/**
 * @brief Monitors system for unsafe conditions
 */
struct SafetyLimits {
    float phMin, phMax;
    float tdsMin, tdsMax;
    float tempMin, tempMax;
    float levelMin, levelMax;
};

enum class SafetyState : uint8_t {
    NORMAL = 0,
    WARNING,
    ALARM,
    EMERGENCY_STOP
};

class SafetyMonitor {
public:
    SafetyMonitor();

    void setLimits(const SafetyLimits& limits);
    SafetyState check(const SensorReadings& readings);
    SafetyState getState() const { return state_; }
    const char* getStateString() const;
    const char* getAlarmReason() const { return alarmReason_; }

    void acknowledgeAlarm();
    void reset();

private:
    SafetyLimits limits_;
    SafetyState state_;
    char alarmReason_[64];
    uint8_t consecutiveAlarms_;
};

#endif // CONTROLLER_H
