#include <Arduino.h>
#include <SoftwareSerial.h>
#include "protocol.h"

const int numSensors = NUM_SENSORS; // number of sensors, from protocol.h

// trig pin first, followed by echo pin
const int trigPins[numSensors] = {2,4,6,8,10};
const int echoPins[numSensors] = {3,5,7,9,11};

// firing order 1,3,5,2,4 - reduces cross-talk
const int fireOrder[numSensors] = {0,2,4,1,3};

const int minSlotMs = 12; // minimum time per sensor before firing the next
const uint8_t earlySendCm = 100; // Threshold where AEB triggers immediately instead of waiting for sweep to finish

SoftwareSerial relay(12, 13); // RX, TX

Frame frame; // global so older readings persist between sweeps

long getDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW); // HC-SR04 suggests a short LOW for a clean HIGH pulse
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // pulseIn is blocking.
  long duration = pulseIn(echoPin, HIGH, 30000); // time how long the echo pin recieves HIGH

  if (duration == 0) {
    return 255; // error value
  }

  long cm = duration * 0.0343 / 2; // cm per microseconds, divided by 2 (accounting for both ways)
  if (cm > 200) {
    return 255; // out of range
  }
  return (uint8_t)cm;
}

void setup() {
  relay.begin(9600);

  // initialise pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    frame.distances[i] = 255; // no reading yet
  }
}

void sendFrame() {
  uint8_t buffer[FRAME_SIZE]; // 8 bytes
  encodeFrame(&frame, buffer); // buffer is automatically passed as a pointer

  for (int i = 0; i < FRAME_SIZE; i++){
    relay.write(buffer[i]);
  }
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    int s = fireOrder[i];
    uint32_t start = millis();

    frame.distances[s] = getDistance(trigPins[s], echoPins[s]);

    // don't wait for the rest of the sweep
    if (frame.distances[s] < earlySendCm) {
      sendFrame();
    }

    // only pad if the reading was quick
    uint32_t elapsed = millis() - start;
    if (elapsed < minSlotMs) {
      delay(minSlotMs - elapsed);
    }
  }

  sendFrame();
