// header file for ttc.cpp

// if ttc.h is not defined, define it. avoids multiple inclusion of the same file.
#ifndef TTC_H
#define TTC_H

#include <stdint.h>

// Threat level
enum AEBStatus : uint8_t {
  OK, BRAKE, AEB
};

// closing speed can be derived from successive distance readings.
struct TTCState {
  uint8_t  prevDistance; // cm
  uint32_t prevTime;     // millis()
  bool     hasPrev;      // false until a real reading comes in
};

void ttcInit(TTCState *state);
AEBStatus ttcUpdate(TTCState *state, uint8_t currentDistance);

#endif
