
#include "ttc.h"
#include <Arduino.h> // for millis()

// Thresholds based on Highway Code: ~0.67s reaction time,
// ~0.25s system latency budget = ~0.92s minimum safety margin.
// Thresholds below give staged warnings
#define TTC_EMERGENCY_SEC 0.5
#define TTC_BRAKE_SEC     1.0
#define TTC_WARNING_SEC   2.0


void ttcInit(TTCState *state) {
  state->prevDistance = 255;
  state->prevTime = 0;
  state->hasPrev = false;
}

AEBStatus ttcUpdate(TTCState *state, uint8_t currentDistance) {
  uint32_t now = millis();

  // no valid reading right now
  if (currentDistance == 255) {
    state->hasPrev = false;
    return OK;
  }

  // first real reading, or resuming after a gap
  if (!state->hasPrev) {
    state->prevDistance = currentDistance;
    state->prevTime = now;
    state->hasPrev = true;
    return OK;
  }

  float dt = (now - state->prevTime) / 1000.0; // seconds
  if (dt <= 0) {
    return OK; // guard against a zero/negative gap
  }

  float distanceChange = (float)state->prevDistance - (float)currentDistance; // positive = closing
  float closingSpeed = (distanceChange / 100.0) / dt; // convert to m/s

  // update state for next call, regardless of outcome below
  state->prevDistance = currentDistance;
  state->prevTime = now;

  if (closingSpeed <= 0) {
    // not approaching (steady or moving away)
    return OK;
  }

  float ttcSeconds = (currentDistance / 100.0) / closingSpeed; // distance in meters / speed

  if (ttcSeconds < TTC_EMERGENCY_SEC) {
    return AEB;
  } else if (ttcSeconds < TTC_BRAKE_SEC) {
    return BRAKE;
  } 
  return OK;
}