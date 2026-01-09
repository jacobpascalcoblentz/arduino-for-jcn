/**
 * @file mock_arduino.h
 * @brief Mock Arduino functions for native testing
 */

#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>

#ifndef ARDUINO

// Mock millis() function
uint32_t millis();
void setMockMillis(uint32_t ms);
void advanceMockMillis(uint32_t ms);

#endif // ARDUINO

#endif // MOCK_ARDUINO_H
