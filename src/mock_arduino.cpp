/**
 * @file mock_arduino.cpp
 * @brief Mock Arduino functions implementation for native testing
 */

#include "mock_arduino.h"

#ifndef ARDUINO

static uint32_t mockMillisValue = 0;

uint32_t millis() {
    return mockMillisValue;
}

void setMockMillis(uint32_t ms) {
    mockMillisValue = ms;
}

void advanceMockMillis(uint32_t ms) {
    mockMillisValue += ms;
}

#endif // ARDUINO
